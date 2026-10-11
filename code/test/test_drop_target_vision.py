"""Offline CH3 acceptance: road-only X, one side-line approach, straight return."""

from contextlib import ExitStack
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, PropertyMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import competition_task as task
import payload_detour
from components import drop_target_vision as dvision
from components.pose_fusion import FusedPoseEstimate, PoseFusion, PoseFusionState
from components.relay_lcus import FakeLCUSRelay
from components.payload_task import prepare_payload
from config.v2_runtime import RuntimeMode
from core.types import Pose2D
from robocup_runtime import build_runtime, load_runtime_config

class ChassisModel:
    """Continuous fused side-line model; rotations include real translation."""
    def __init__(self, *, visible=True):
        self.pose = [0., 0., math.pi/2]
        self.visible = visible
        self.now = 0.
        self.stages, self.lines = [], []

    def arc(self):
        return (340., 300.+600.*(self.pose[1]-.43), 450.) if self.visible else None

    def motion(self, label, method, *args, **kwargs):
        self.stages.append((label, method))
        if method == "rotate_to":
            self.pose[0] += .08
            self.pose[2] = args[0]
        elif method == "track_global_line":
            start,end = args
            self.lines.append((start,end,kwargs.get("reverse",False)))
            n = max(1, math.ceil(math.dist(start,end)/.01))
            for index in range(1,n+1):
                self.pose[:2] = [start[i]+(end[i]-start[i])*index/n for i in (0,1)]
                self.now += .2
                pose = SimpleNamespace(x_m=self.pose[0],y_m=self.pose[1],yaw_rad=self.pose[2])
                if kwargs.get("on_tick",lambda _:False)(pose):
                    break
        else:
            raise AssertionError(method)
        return SimpleNamespace(x_m=self.pose[0],y_m=self.pose[1],yaw_rad=self.pose[2])


class DropAlignLoopTests(unittest.TestCase):
    def setUp(self):
        self.config = load_runtime_config()
        self.events = []
        self.chassis = ChassisModel()
        self.runtime = SimpleNamespace(motion=SimpleNamespace(drive=self.config.drive,
            navigation=self.config.navigation, stop=Mock()), drive=Mock(),
            clock=lambda:self.chassis.now,record_event=lambda e,**v:self.events.append((e,v)))
        self.camera = Mock()
        self.camera.read.return_value = (True,object())
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.stack.enter_context(patch.object(task,"DROP_REFERENCE",
            Path(__file__).resolve().parents[2]/"assets/ch3_yellow_servo0_reference.jpg"))
        self.stack.enter_context(patch.object(dvision,"read_reference",return_value=(300.,300.,450.)))
        self.extract = self.stack.enter_context(patch.object(dvision,"extract_target_arc",
            side_effect=lambda *_:self.chassis.arc()))
        self.opener = self.stack.enter_context(patch.object(task,"_open_yellow_camera",
            return_value=(self.camera,False)))
        self.stack.enter_context(patch.object(task, "_step", return_value=Mock()))
        self.stack.enter_context(patch.object(task.time, "sleep", side_effect=self.advance))
        self.stack.enter_context(patch.object(task, "wait_for_fused_localization",
            side_effect=lambda _:SimpleNamespace(estimate=SimpleNamespace(pose=SimpleNamespace(
                x_m=self.chassis.pose[0], y_m=self.chassis.pose[1], yaw_rad=self.chassis.pose[2])))))

    def advance(self, dt):
        self.chassis.now += dt

    def align(self, **kwargs):
        return task.align_drop_position(self.runtime,self.camera,
            SimpleNamespace(x_m=0.,y_m=0.,yaw_rad=math.pi/2),0.,math.pi/2,
            self.chassis.motion,safe_distance_m=.47,**kwargs)

    def test_reference_y_cannot_stop_blind_fused_approach_early(self):
        pose = self.align()
        self.assertEqual(self.chassis.stages,[("safe_side_approach","track_global_line")])
        self.assertEqual(len(self.chassis.lines),1)
        self.assertAlmostEqual(pose.x_m,0.)
        self.assertAlmostEqual(pose.y_m,.47)
        done = [v for e,v in self.events if e=="drop_align_done"][-1]
        self.assertTrue(done["success"])
        self.assertEqual(done["source"], "blind_fused_endpoint")
        self.opener.assert_not_called()
        self.extract.assert_not_called()

    def test_lost_arc_goes_to_safe_endpoint_without_turning(self):
        self.chassis.visible = False
        pose = self.align()
        self.assertAlmostEqual(pose.y_m,.47)
        self.assertEqual(len(self.chassis.lines),1)
        self.assertTrue(all(m=="track_global_line" for _,m in self.chassis.stages))

    def test_recorded_width_and_x_jumps_never_drive_sideways_or_flip_direction(self):
        widths=[278,312,401,350,352,408,378,322,354,242,180,248]
        xs=[209,295,302,380,582,221,435,276,580,209,505,222]
        self.extract.side_effect = [(x,300.,w) for x,w in zip(xs,widths)]*5
        pose = self.align()
        self.assertAlmostEqual(pose.y_m,.47)
        self.assertAlmostEqual(pose.x_m,0.)
        self.assertEqual(len(self.chassis.lines),1)
        self.assertFalse(any('flip' in e for e,_ in self.events))

    def test_unavailable_camera_and_reference_and_other_color_use_same_safe_line(self):
        for reason in ("camera","reference","red"):
            with self.subTest(reason=reason):
                self.chassis.pose=[0.,0.,math.pi/2]
                self.chassis.lines.clear()
                with ExitStack() as stack:
                    if reason=="camera":
                        stack.enter_context(patch.object(task,"_open_yellow_camera",side_effect=OSError("camera")))
                    if reason=="reference":
                        stack.enter_context(patch.object(dvision,"read_reference",return_value=None))
                    pose=self.align(color="red" if reason=="red" else "yellow")
                self.assertAlmostEqual(pose.y_m,.47)
                self.assertEqual(len(self.chassis.lines),1)

    def test_speed_cap_restored_and_backend_errors_propagate(self):
        def failing(*_a,**_k):
            self.assertEqual(self.runtime.motion.drive.max_linear_speed_m_s,task.DROP_FINE_SPEED_M_S)
            raise RuntimeError("drive backend failed")
        with self.assertRaisesRegex(RuntimeError,"backend"):
            task.align_drop_position(self.runtime,self.camera,
                SimpleNamespace(x_m=0.,y_m=0.,yaw_rad=math.pi/2),0.,math.pi/2,failing,
                safe_distance_m=.47)
        self.assertIs(self.runtime.motion.drive,self.config.drive)

    def test_terminal_speed_reduces_before_fixed_endpoint(self):
        with patch.object(task,"DROP_REFERENCE",Path("not-a-field-reference.jpg")):
            pose=self.align()
        self.assertAlmostEqual(pose.y_m,.47)
        caps=[v["speed_cap_m_s"] for e,v in self.events if e=="drop_fused_tracking"]
        self.assertEqual(set(caps), {.012, task.DROP_FINE_SPEED_M_S})
        self.assertIs(self.runtime.motion.navigation, self.config.navigation)

    def test_stop_coast_is_measured_and_corrected_parallel_to_saved_axis(self):
        original = self.chassis.motion
        def coast(label, method, *args, **kwargs):
            pose = original(label, method, *args, **kwargs)
            if label == "safe_side_approach":
                self.chassis.pose[0] += .015  # Cross-track residual is not chased at the edge.
                self.chassis.pose[1] += .020  # Coast visible only in stopped fusion.
            return pose
        self.chassis.motion = coast
        pose = self.align()
        self.assertAlmostEqual(pose.y_m, .47)
        self.assertAlmostEqual(pose.x_m, .015)
        self.assertEqual(self.chassis.stages[-1], ("drop_fused_distance_adjust", "track_global_line"))
        start, end, reverse = self.chassis.lines[-1]
        self.assertTrue(reverse)
        self.assertAlmostEqual(start[0], end[0])

    def test_repeated_fused_coast_prevents_release_at_wrong_reach(self):
        original = self.chassis.motion
        def coast(label, method, *args, **kwargs):
            pose = original(label, method, *args, **kwargs)
            self.chassis.pose[1] += .020
            return pose
        self.chassis.motion = coast
        with self.assertRaisesRegex(RuntimeError, "within 3 mm"):
            self.align()
        self.assertIs(self.runtime.motion.navigation, self.config.navigation)
        self.assertIs(self.runtime.motion.drive, self.config.drive)

    def test_stationary_drop_align_never_calls_motion(self):
        runtime=Mock()
        with patch("config.v2_factory.build_servo",return_value=Mock()), \
             patch.object(task,"_open_yellow_camera",return_value=(self.camera,False)):
            task.run_drop_align(runtime,self.camera)
        runtime.motion.track_global_line.assert_not_called()
        runtime.motion.rotate_to.assert_not_called()


class ReferencePhotoTests(unittest.TestCase):
    def setUp(self):
        import cv2
        import numpy as np
        self.cv2,self.np=cv2,np
        self.image=np.zeros((480,640,3),dtype=np.uint8)
        xs=np.arange(90,551)
        points=np.stack((xs,300+.0018*(xs-320)**2),axis=1).astype(np.int32)
        cv2.polylines(self.image,[points],False,(0,255,255),4)

    def test_whole_model_survives_local_occlusion_and_baffle_changes(self):
        base=dvision.extract_target_arc(self.image)
        self.assertIsNotNone(base)
        for baffle in (False,True):
            image=self.image.copy()
            image[:,300:340]=0
            if baffle:
                image[442:,:]=(0,255,255)
            arc=dvision.extract_target_arc(image)
            self.assertIsNotNone(arc)
            self.assertAlmostEqual(arc[0],base[0],delta=3)
            self.assertAlmostEqual(arc[1],base[1],delta=3)

    def test_changing_visible_span_invalidates_temporal_observation(self):
        features=[]
        for margin in (90,130,180):
            image=self.image.copy()
            image[:,:margin]=0
            image[:,640-margin:]=0
            features.append(dvision.extract_target_arc(image))
        self.assertIsNone(dvision.stable_feature(features))

    def test_pixel_translation_moves_the_fitted_feature(self):
        moved=self.cv2.warpAffine(self.image,self.np.float32([[1,0,15],[0,1,10]]),(640,480))
        base=dvision.extract_target_arc(self.image)
        shifted=dvision.extract_target_arc(moved)
        self.assertAlmostEqual(shifted[0]-base[0],15,delta=2)
        self.assertAlmostEqual(shifted[1]-base[1],10,delta=2)

    def test_packaged_photo_is_comparison_only_and_uses_identical_extractor(self):
        path=Path(__file__).resolve().parents[2]/"assets/ch3_yellow_servo0_reference.jpg"
        image=self.cv2.imread(str(path))
        self.assertEqual(dvision.read_reference(path),dvision.extract_target_arc(image))
        self.assertNotEqual(path,task.DROP_REFERENCE)

    def test_unusable_and_other_colors_reject_without_movement_signal(self):
        self.assertIsNone(dvision.extract_target_arc(None))
        for color in ("red","blue","green","magenta"):
            self.assertIsNone(dvision.extract_target_arc(self.image,color))
        self.assertIsNone(dvision.stable_feature([(300.,300.,450.),None,(300.,300.,450.)]))


class RoadAlignmentTests(unittest.TestCase):
    def fixture(self, gain=-800., *, fail_after_moves=None, same_timestamp=False):
        self.now, self.x = 10., 0.
        self.moves, self.events = [], []
        config=load_runtime_config()
        self.runtime=SimpleNamespace(motion=SimpleNamespace(navigation=config.navigation,
            drive=config.drive, stop=Mock()), drive=Mock(), clock=lambda:self.now,
            record_event=lambda e,**v:self.events.append((e,v)))
        self.initial=(330.,0.,370.,40.,.95,3)
        self.vision=SimpleNamespace(frame_size=(640,480))

        def observation(_):
            cx=350.+gain*self.x
            box=(cx-20,0,cx+20,40,.95,3)
            if fail_after_moves is not None and len(self.moves)>=fail_after_moves:
                box=None
            return (10. if same_timestamp else self.now),box,box

        self.vision.observe_drop=observation
        self.stack=ExitStack()
        self.addCleanup(self.stack.close)
        self.stack.enter_context(patch.object(task,'_step',return_value=Mock()))
        self.stack.enter_context(patch.object(task,'wait_for_fused_localization',
            side_effect=lambda _:SimpleNamespace(estimate=SimpleNamespace(pose=SimpleNamespace(
                x_m=self.x,y_m=0.,yaw_rad=0.)))))
        self.stack.enter_context(patch.object(task.time,'sleep',side_effect=self.advance))
        self.stack.enter_context(patch.object(task,'_fused_motion',return_value=self.motion))

    def advance(self,dt):
        self.now+=dt

    def motion(self,label,method,start,end,**kwargs):
        self.assertEqual(method,'track_global_line')
        self.assertAlmostEqual(start[1],end[1])
        if label != 'search_restore_stop':
            self.assertLessEqual(math.dist(start,end),.021)
        self.moves.append((label,start,end,kwargs))
        self.x=end[0]
        self.now+=.5
        return SimpleNamespace(x_m=self.x,y_m=0.,yaw_rad=0.)

    def test_one_probe_fixes_gain_sign_and_centres_before_any_turn(self):
        for gain in (-800.,800.):
            with self.subTest(gain=gain):
                self.fixture(gain)
                box=task.fine_center_on_road(self.runtime,self.vision,self.initial)
                self.assertAlmostEqual((box[0]+box[2])/2,task.SEARCH_REFERENCE_CX,delta=4)
                gains=[v for e,v in self.events if e=='search_road_gain']
                self.assertEqual(len(gains),1)
                self.assertAlmostEqual(gains[0]['gain_px_per_m'],gain)
                self.assertEqual(self.moves[0][0],'search_gain_probe')
                self.assertFalse(any('flip' in e for e,_ in self.events))

    def test_unidentifiable_gain_and_lost_new_frames_skip_uncentred_drop(self):
        for gain,fail in ((0.,None),(-800.,1),(-800.,2)):
            with self.subTest(gain=gain,fail=fail):
                self.fixture(gain,fail_after_moves=fail)
                box=task.fine_center_on_road(self.runtime,self.vision,self.initial)
                self.assertIsNone(box)
                self.assertTrue(any(e=='search_road_align_done' and not v['success']
                                    for e,v in self.events))

    def test_repeated_timestamp_is_not_three_fresh_frames(self):
        self.fixture(same_timestamp=True)
        self.assertIsNone(task.fine_center_on_road(self.runtime,self.vision,self.initial))
        self.assertFalse(self.moves)

    def test_search_coast_does_not_consume_gain_probe(self):
        self.fixture()
        initial_observation=self.vision.observe_drop
        def observation(now):
            if not self.moves:
                self.x=.018  # Search has coasted while fresh images arrive.
            return initial_observation(now)
        self.vision.observe_drop=observation
        box=task.fine_center_on_road(self.runtime,self.vision,self.initial)
        self.assertIsNotNone(box)
        probe=self.moves[0]
        self.assertAlmostEqual(probe[1][0], .018)
        self.assertAlmostEqual(probe[2][0]-probe[1][0], .02)
        self.assertAlmostEqual((box[0]+box[2])/2, 320, delta=4)


class DetourFineAlignTests(unittest.TestCase):
    """The detour returns from the real drop pose, never from a fixed 47 cm."""

    def fixture(self, pose=(5., 7., math.pi / 2)):
        self.now, self.steps = 10., 0
        self.wheel_bias = 0.
        self.pose = list(pose)
        self.releasing = False
        self.stage_poses, self.events, self.releases = {}, [], []
        self.runtime = build_runtime(load_runtime_config(), RuntimeMode.DRY_RUN,
                                     clock=lambda: self.now)
        self.addCleanup(self.runtime.close)
        self.relay = FakeLCUSRelay(4)
        self.runtime.relay = self.relay
        self.runtime._consume_t265 = lambda *_a, **_k: None
        self.runtime._consume_d500 = lambda *_a, **_k: None
        self.runtime.fusion.estimate = lambda now: FusedPoseEstimate(
            Pose2D(*self.pose, now), PoseFusionState.OK, 0., ("test",), 0., 0.,
            None, None, True, anchor_initialized=True, t265_confidence=1.)
        local = patch.object(PoseFusion, "continuous_t265_pose", new_callable=PropertyMock)
        prop = local.start()
        self.addCleanup(local.stop)
        prop.side_effect = lambda: Pose2D(*self.pose, self.now)
        self.runtime.record_event = self.record
        self.runtime.start()
        self.runtime.motion.stop()

    def record(self, event, **values):
        self.events.append((event, values))
        if event == "payload_release_start":
            self.releases.append((values["channel"], tuple(self.pose)))
        if event == "payload_detour_stage_done":
            self.stage_poses[values["stage"]] = values["pose"]

    def advance(self, dt):
        self.steps += 1
        self.assertLess(self.steps, 15000)
        twist = self.runtime.drive.last_limited_twist
        omega = twist.angular_z_rad_s + self.wheel_bias * max(0., twist.linear_x_m_s)
        yaw = self.pose[2] + omega * dt / 2
        self.pose[0] += twist.linear_x_m_s * math.cos(yaw) * dt
        self.pose[1] += twist.linear_x_m_s * math.sin(yaw) * dt
        self.pose[2] += omega * dt
        self.now += dt

    def fine_align(self, side_pose, road_yaw, side_yaw, motion):
        start=(side_pose.x_m,side_pose.y_m)
        far=(start[0]+.40*math.cos(side_yaw),start[1]+.40*math.sin(side_yaw))
        return motion("safe_side_approach","track_global_line",start,far)

    def run_detour(self, fine_align):
        return payload_detour.run_payload_detour(
            self.runtime, payload_detour.DetourSettings(payload_slot=2),
            clock=lambda: self.now, sleep=self.advance, fine_align=fine_align)

    def test_unequal_wheels_correct_cross_track_without_extending_47cm_reach(self):
        self.fixture(pose=(0., 0., 0.))
        self.wheel_bias = .9  # Without steering, 47 cm curves by about 24 degrees.
        def blind(side_pose, road_yaw, side_yaw, motion):
            return task.align_drop_position(self.runtime, None, side_pose, road_yaw,
                                            side_yaw, motion, safe_distance_m=.47)
        with patch.object(task.time, "sleep", side_effect=self.advance), \
             patch.object(task, "_open_yellow_camera", side_effect=AssertionError("blind drop")):
            self.run_detour(blind)
        drop=[v for e,v in self.events if e=="payload_drop_pose"][-1]
        self.assertAlmostEqual(drop['raw_progress_m'], .47, delta=.003)
        self.assertLess(abs(drop['cross_track_m']), .015)
        stages=[v['stage'] for e,v in self.events if e=='payload_detour_stage_start']
        self.assertEqual(sum(s in ('left_90deg', 'right_90deg') for s in stages), 2)
        self.assertEqual(len(self.releases), 1)
        self.assertEqual(self.releases[0][0], 3)

    def test_turn_translation_3_to_8cm_uses_actual_pivot_and_only_two_turns(self):
        for drift in (.03,.05,.08):
            with self.subTest(drift=drift):
                self.fixture()
                rotate=self.runtime.motion.rotate_to
                locations=[]
                def drifting_turn(yaw):
                    locations.append(tuple(self.pose))
                    if len(locations)==1:
                        self.pose[1]+=drift
                    rotate(yaw)
                self.runtime.motion.rotate_to=Mock(side_effect=drifting_turn)
                self.run_detour(self.fine_align)
                self.assertEqual(len(locations),2)
                pivot=self.stage_poses['left_90deg']
                road=next(v['pose'] for e,v in self.events if e=='payload_detour_road_pose')
                self.assertAlmostEqual(pivot.y_m-road.y_m,drift,delta=.001)
                starts={v['stage']:v for e,v in self.events if e=='payload_detour_stage_start'}
                self.assertEqual(starts['safe_side_approach']['args'][0],(pivot.x_m,pivot.y_m))
                self.assertLess(abs(locations[1][0]-pivot.x_m),.01)
                a,b=starts['return_from_drop']['args']
                self.assertAlmostEqual(a[1],b[1])
                self.assertTrue(starts['return_from_drop']['kwargs']['reverse'])
                self.assertEqual([channel for channel,_ in self.releases],[3])

    def test_old_edge_rotation_callback_is_rejected_before_motor_call(self):
        self.fixture()
        rotate=self.runtime.motion.rotate_to=Mock(wraps=self.runtime.motion.rotate_to)
        def illegal(side_pose,road_yaw,side_yaw,motion):
            return motion('illegal_edge_turn','rotate_to',road_yaw)
        with self.assertRaisesRegex(ValueError,'only straight'):
            self.run_detour(illegal)
        self.assertEqual(rotate.call_count,1)
        self.assertFalse(self.releases)

    def test_stable_visual_stop_interrupts_single_approach_then_reverses_inside(self):
        self.fixture((0.,0.,0.))
        def stop_early(side_pose,road_yaw,side_yaw,motion):
            start=(side_pose.x_m,side_pose.y_m)
            return motion('safe_side_approach','track_global_line',start,(start[0],start[1]+.47),
                          on_tick=lambda pose:pose.y_m-start[1]>=.30)
        returned=self.run_detour(stop_early)
        self.assertAlmostEqual(self.releases[0][1][1],.30,delta=.01)
        self.assertLess(abs(returned.y_m),.01)
        self.assertTrue(any(e=='payload_detour_stage_done' and v.get('early_stop') for e,v in self.events))

    def test_return_leg_reuses_the_real_drop_pose_and_restores_road_yaw(self):
        self.fixture()
        self.assertTrue(prepare_payload(self.relay, 2))
        returned = self.run_detour(self.fine_align)
        starts = {values["stage"]: values for event, values in self.events
                  if event == "payload_detour_stage_start"}
        self.assertNotIn("advance_7cm", starts)
        self.assertIn("safe_side_approach", starts)
        self.assertNotIn("forward_47cm", starts)
        turn_xy = starts["safe_side_approach"]["args"][0]
        drop_xy = starts["return_from_drop"]["args"][0]
        self.assertLess(math.dist(starts["return_from_drop"]["args"][1], turn_xy), .01)
        self.assertTrue(starts["return_from_drop"]["kwargs"]["reverse"])
        self.assertNotAlmostEqual(math.dist(drop_xy, turn_xy), .47, delta=.05)
        self.assertAlmostEqual(math.dist(drop_xy, turn_xy), .40, delta=.01)
        self.assertAlmostEqual(returned.yaw_rad, math.pi / 2, delta=.06)
        self.assertEqual([channel for channel, _ in self.releases], [3])
        self.assertFalse(any(self.relay.query_status().values()))

    def test_zero_length_return_is_skipped_without_a_line_error(self):
        self.fixture()
        self.assertTrue(prepare_payload(self.relay, 2))

        def back_to_turn(side_pose, road_yaw, side_yaw, motion):
            return side_pose

        self.run_detour(back_to_turn)
        self.assertTrue(any(event == "payload_detour_stage_skipped" for event, _ in self.events))
        self.assertEqual([channel for channel, _ in self.releases], [3])
        self.assertEqual(self.runtime.drive.last_limited_twist.linear_x_m_s, 0)


if __name__ == "__main__":
    unittest.main()
