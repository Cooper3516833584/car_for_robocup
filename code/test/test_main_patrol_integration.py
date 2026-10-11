"""Production mission acceptance on its own route using fake drive/vision/relay."""

from contextlib import ExitStack
from dataclasses import replace
import math
from pathlib import Path
import signal
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, PropertyMock, call, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import competition_task as task
import target_patrol
from components.pose_fusion import FusedPoseEstimate, PoseFusion, PoseFusionState
from components.task_board_reader import TaskCounts
from config.v2_runtime import RuntimeMode
from core.types import Pose2D, Twist2D
from main_robocup import main
from mission_control import step_runtime
from robocup_runtime import RobocupMissionState, build_runtime, load_runtime_config


class MainPatrolIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.now = 10.
        self.pose = [0., 0., 0.]
        self.events = []
        self.steps = 0
        self.detour_caps = {}
        self.center_available = True
        self.stop_during_detour = False
        self.config = load_runtime_config()
        self.config = replace(self.config, servo=replace(self.config.servo, enabled=True))
        self.logger = Mock()
        self.servo = Mock(is_running=True, pulse_us=2500)
        # The 0 deg refinement camera is faked: no test may open a real device.
        self.capture = Mock()
        self.capture.read.return_value = (False, None)
        self.stack.enter_context(patch.object(task, "_open_yellow_camera",
                                              return_value=(self.capture, False)))
        self.vision = Mock(ready=True, frame_size=(640,480))
        self.vision.observe.side_effect = lambda now: (now, self.boxes()[0])
        self.vision.observe_drop.side_effect = lambda now: (now, *self.boxes())
        self.vision_factory = self.stack.enter_context(patch.object(target_patrol, "YoloVision", return_value=self.vision))
        self.stack.enter_context(patch("config.v2_factory.build_servo", return_value=self.servo))
        self.stack.enter_context(patch("main_robocup.load_runtime_config", return_value=self.config))
        self.stack.enter_context(patch("main_robocup.JsonlEventLogger", return_value=self.logger))
        self.builder = self.stack.enter_context(patch("main_robocup.build_runtime", side_effect=self.make_runtime))
        self.stack.enter_context(patch.object(task.time, "sleep", self.advance))
        self.counts = TaskCounts(2,1,1)
        self.stack.enter_context(patch.object(task, "read_task_board", return_value=self.counts))
        self.send = self.stack.enter_context(patch.object(task, "send_task_to_drone_once", return_value=True))
        self.stack.enter_context(patch.object(task, "load_detector", return_value=Mock()))
        # A different field path, not the test fixture's 280/420/250 cm rectangle.
        for name,value in {"LANE_ENTRY_X":.15,"TASK_BOARD_X":.25,"CORNER_1_X":1.05,
                           "CROSS_LANE_YAW_DEG":90,"YELLOW_SEARCH_START_X":1.05,
                           "YELLOW_SEARCH_START_Y":.10,"YELLOW_SEARCH_END_X":1.05,
                           "YELLOW_SEARCH_END_Y":.65,"CORNER_2_X":1.05,
                           "CORNER_2_Y":.85,"FINISH_X":1.40,"FINISH_Y":.85}.items():
            self.stack.enter_context(patch.object(task,name,value))
        self.stack.enter_context(patch.object(target_patrol, "route_from_pose",
                                  side_effect=AssertionError("test route must not enter production")))
        self.directory = self.stack.enter_context(tempfile.TemporaryDirectory())
        self.stop_file = Path(self.directory)/"STOP"

    def make_runtime(self, config, _mode, **_kwargs):
        self.used_config = config
        runtime = build_runtime(config, RuntimeMode.DRY_RUN, clock=lambda:self.now, fake_sample_count=0)
        self.runtime = runtime
        self.addCleanup(runtime.close)
        runtime._consume_t265 = lambda *_a,**_k:None
        runtime._consume_d500 = lambda *_a,**_k:None
        runtime.fusion.estimate = lambda now:FusedPoseEstimate(
            Pose2D(*self.pose,now),PoseFusionState.OK,0.,("test",),0.,0.,None,None,True,
            anchor_initialized=True,t265_confidence=1.)
        # T265 translation is frozen in a different frame; yaw rate remains real.
        local = self.stack.enter_context(patch.object(PoseFusion,"continuous_t265_pose",new_callable=PropertyMock))
        local.side_effect = lambda:Pose2D(50.,-20.,self.pose[2]-.4,self.now)
        runtime.record_event = lambda event,**values:self.events.append((event,values))
        runtime.motion.track_global_line = Mock(wraps=runtime.motion.track_global_line)
        runtime.motion.track_local_line = Mock(side_effect=AssertionError("raw T265 translation used"))
        return runtime

    def boxes(self):
        width,_ = self.vision.frame_size
        if self.pose[1] >= .27 and self.center_available:
            box=(width*.46,0,width*.54,40,.95,3)
            return box,box
        return (width*.1,0,width*.2,40,.95,3),None

    def advance(self, dt):
        self.steps += 1
        self.assertLess(self.steps,10000)
        phase = next((v["stage"] for e,v in reversed(self.events)
                      if e=="payload_detour_stage_start"),None)
        if phase is not None:
            self.detour_caps.setdefault(phase,set()).add(self.runtime.motion.drive.max_linear_speed_m_s)
        command = self.runtime.drive.last_limited_twist
        heading = self.pose[2]+command.angular_z_rad_s*dt/2
        self.pose[0] += command.linear_x_m_s*math.cos(heading)*dt
        self.pose[1] += command.linear_x_m_s*math.sin(heading)*dt
        self.pose[2] += command.angular_z_rad_s*dt
        self.now += dt
        if self.stop_during_detour and any(e=="payload_detour_stage_start" and v["stage"]=="safe_side_approach"
                                          for e,v in self.events):
            self.stop_file.touch()

    def run_main(self, *extra):
        return main(["--mode","hardware-mission","--competition","--release-mode","simulate",
                     "--speed-scale","2","--payload-slot","3","--log-dir",self.directory,*extra])

    def test_full_entry_preserves_field_route_and_uses_shared_fused_motion(self):
        self.assertEqual(self.run_main(),0)
        self.assertEqual(self.runtime.mission.state,RobocupMissionState.FINISHED)
        self.assertEqual(self.runtime.mission.task_counts,self.counts)
        self.send.assert_called_once_with(self.counts,runtime=self.runtime)
        self.assertFalse(self.used_config.relay.enabled)
        self.assertEqual(self.used_config.navigation.translation_speed_scale,2)
        self.assertEqual(self.used_config.drive.max_linear_accel_m_s2,self.config.drive.max_linear_accel_m_s2)
        kwargs=self.vision_factory.call_args.kwargs
        self.assertEqual(kwargs["confidence"],.8)
        self.assertEqual(kwargs["target_class_name"],"yellow")
        self.assertAlmostEqual(kwargs["drop_center_width_ratio"],.1)
        # +90 search; blind drop never moves the camera servo.
        self.assertEqual(self.servo.set_angle.call_args_list,
                         [call(90, settle=True)])
        self.servo.close.assert_called_once_with(hold=True)
        self.vision.close.assert_called_once()
        calls=self.runtime.motion.track_global_line.call_args_list
        self.assertTrue(any(c.args==((.25,0),(1.05,0)) for c in calls))
        self.assertTrue(any(c.args==((1.05,.10),(1.05,.65)) for c in calls))
        self.assertTrue(any(c.args==((1.05,.10),(1.05,.85)) for c in calls))
        starts={v["stage"]:v for e,v in self.events if e=="payload_detour_stage_start"}
        self.assertLess(math.dist(starts["safe_side_approach"]["args"][0],
                         starts["return_from_drop"]["args"][1]), .01)
        self.assertTrue(starts["return_from_drop"]["kwargs"]["reverse"])
        self.assertTrue(any(c.kwargs.get("reverse") for c in calls))
        self.assertLess(math.dist(self.pose[:2],(1.40,.85)),.035)
        self.assertEqual(self.runtime.drive.last_limited_twist,Twist2D(0,0))
        self.assertFalse(any(self.runtime.relay._states.values()))
        self.assertFalse(self.runtime.relay.connected)
        self.assertEqual(self.detour_caps["advance_7cm"],{.16})
        cruise_cap = self.config.drive.max_linear_speed_m_s * 2
        self.assertEqual(self.detour_caps["safe_side_approach"],{task.DROP_FINE_SPEED_M_S,.012,cruise_cap})
        # 1-2 cm refinements run at the dedicated fine speed; the release hold
        # afterwards is already back on the route drive.
        self.assertNotIn("creep_to_fallback",self.detour_caps)
        self.assertEqual(self.detour_caps["return_from_drop"],{cruise_cap})
        self.assertEqual(self.runtime.motion.drive.max_linear_speed_m_s,cruise_cap)
        actions=[c.args[0] for c in self.logger.emit.call_args_list if c.args[0]["type"]=="motion_action"]
        self.assertTrue(any(abs(e["diagnostics"].get("lookahead_m",0)-.35)<1e-6 for e in actions))
        self.assertTrue(all(e["pose_reference"]=="fused" for e in actions))

    def test_native_camera_size_does_not_reenter_pixel_alignment_after_centering(self):
        self.vision.frame_size=(1280,720)
        with patch.object(task,"_detect_stopped",side_effect=AssertionError("already centered")):
            self.assertEqual(self.run_main(),0)
        self.assertTrue(any(e=="yellow_align" and v.get("source")=="continuous_search" for e,v in self.events))

    def test_standalone_search_uses_the_same_continuous_field_search(self):
        self.assertEqual(main(["--mode","hardware-mission","--competition-stage","yellow-search",
                               "--log-dir",self.directory]),0)
        self.assertTrue(any(e=="target_search_centered" for e,v in self.events))
        self.assertFalse(any(e=="payload_release_start" for e,v in self.events))
        self.servo.close.assert_called_once_with(hold=True)
        self.assertEqual(self.runtime.drive.last_limited_twist,Twist2D(0,0))

    def test_search_exhaustion_skips_detour_and_finishes_original_route(self):
        self.center_available=False
        self.assertEqual(self.run_main(),0)
        self.assertTrue(any(e=="target_search_exhausted" for e,v in self.events))
        self.assertFalse(any(e=="payload_release_start" for e,v in self.events))
        self.assertEqual(self.runtime.mission.state,RobocupMissionState.FINISHED)

    def test_zero_degree_camera_failure_releases_and_resumes_without_edge_turns(self):
        self.stack.enter_context(patch.object(task,'_open_yellow_camera',side_effect=OSError('camera lost')))
        self.assertEqual(self.run_main('--payload-slot','2'),0)
        stages=[v['stage'] for e,v in self.events if e=='payload_detour_stage_start']
        self.assertEqual(stages,['advance_7cm','left_90deg','safe_side_approach','return_from_drop','right_90deg'])
        release=[v for e,v in self.events if e=='payload_release_done']
        self.assertEqual(len(release),1)
        self.assertEqual(release[0]['channel'],3)
        self.assertTrue(any(e=='drop_align_done' and v['source']=='blind_fused_endpoint' for e,v in self.events))

    def test_unstable_zero_degree_arc_uses_fused_endpoint_and_continues_track(self):
        from components import drop_target_vision as vision
        self.capture.read.return_value=(True,object())
        self.stack.enter_context(patch.object(task,'DROP_REFERENCE',
            Path(__file__).resolve().parents[2]/'assets/ch3_yellow_servo0_reference.jpg'))
        self.stack.enter_context(patch.object(vision,'read_reference',return_value=(300.,300.,450.)))
        widths=[278,312,401,350,352,408,378,322,354,242,180,248]
        self.stack.enter_context(patch.object(vision,'extract_target_arc',
            side_effect=[(209. if i%2 else 582.,300.,w) for i,w in enumerate(widths)]*100))
        self.assertEqual(self.run_main('--payload-slot','2'),0)
        self.assertTrue(any(e=='drop_align_done' and v['source']=='blind_fused_endpoint' for e,v in self.events))
        starts=[v['stage'] for e,v in self.events if e=='payload_detour_stage_start']
        self.assertEqual(starts,['advance_7cm','left_90deg','safe_side_approach','return_from_drop','right_90deg'])
        self.assertEqual(self.runtime.mission.state,RobocupMissionState.FINISHED)

    def test_worker_failure_stops_and_closes_vision_and_servo(self):
        self.vision.observe.side_effect=RuntimeError("stale inference")
        self.assertEqual(self.run_main(),1)
        self.assertEqual(self.runtime.mission.state,RobocupMissionState.SAFE_STOP)
        self.vision.close.assert_called_once()
        self.servo.close.assert_called_once_with(hold=True)
        self.assertEqual(self.runtime.drive.last_limited_twist,Twist2D(0,0))

    def test_release_failure_stops_before_second_corner(self):
        with patch("payload_detour.drop_payload",return_value=False):
            self.assertEqual(self.run_main(),1)
        self.assertFalse(any(e=="competition_stage_start" and v["stage"]=="corner2" for e,v in self.events))
        self.assertEqual(self.runtime.mission.state,RobocupMissionState.SAFE_STOP)
        self.assertFalse(any(self.runtime.relay._states.values()))

    def test_stop_file_interrupts_nested_payload_motion_and_guard_is_scoped(self):
        self.stop_during_detour=True
        self.assertEqual(self.run_main(),1)
        self.assertFalse(any(e=="payload_release_start" for e,v in self.events))
        self.assertEqual(self.runtime.drive.last_limited_twist,Twist2D(0,0))
        # An independent task must not inherit this mission's pending STOP file.
        fresh=self.make_runtime(self.config,RuntimeMode.DRY_RUN)
        fresh.start();fresh.motion.stop()
        step_runtime(fresh)
        self.assertNotEqual(fresh.mission.state,RobocupMissionState.SAFE_STOP)

    def test_task_deadline_interrupts_original_lane_motion(self):
        self.assertEqual(self.run_main("--max-seconds","1"),1)
        self.send.assert_not_called()
        self.assertEqual(self.runtime.mission.state,RobocupMissionState.SAFE_STOP)
        self.assertEqual(self.runtime.drive.last_limited_twist,Twist2D(0,0))

    def test_sigterm_uses_interrupt_cleanup_and_restores_signal_handler(self):
        previous=signal.getsignal(signal.SIGTERM)
        def terminate(*_a,**_k):
            signal.getsignal(signal.SIGTERM)(signal.SIGTERM,None)
        with patch.object(task,"run_competition_stage",side_effect=terminate):
            self.assertEqual(self.run_main(),130)
        self.assertEqual(signal.getsignal(signal.SIGTERM),previous)
        self.assertEqual(self.runtime.drive.last_limited_twist,Twist2D(0,0))

    def test_invalid_speed_and_conflicting_simulation_port_fail_before_runtime(self):
        for value in ("0","-1","nan","inf"):
            self.assertEqual(self.run_main("--speed-scale",value),2)
        self.assertEqual(self.run_main("--relay-port","/dev/physical-relay"),2)
        self.builder.assert_not_called()


class SharedVisionWorkerTests(unittest.TestCase):
    def test_preloaded_detector_accepts_borrowed_read_only_camera_and_native_size(self):
        frame = SimpleNamespace(shape=(720,1280,3))
        camera = SimpleNamespace(read=Mock(return_value=(True,frame)))
        detector = Mock()
        vision = target_patrol.YoloVision(camera=camera, detector=detector,
                    target_region="frame", target_class_name="yellow")
        yellow = (590,0,690,40,.85,1)
        result = Mock(names={0:"red",1:"yellow"})
        result.boxes.data.cpu.return_value.tolist.return_value = [(590,0,690,40,.99,0),yellow]

        def predict(*_args,**_kwargs):
            if detector.predict.call_count == 2:
                vision._quit.set()
            return [result]

        detector.predict.side_effect = predict
        with patch.dict(sys.modules,{"cv2":SimpleNamespace(),
                                     "torch":SimpleNamespace(get_num_threads=lambda:1)}), \
             patch.object(target_patrol,"pin_vision_worker",return_value=set()), \
             patch("components.yellow_yolo_adapter.load_detector") as load:
            vision._worker()
        self.assertTrue(vision.ready)
        self.assertEqual(vision.frame_size,(1280,720))
        self.assertEqual(vision.observe_drop(time.monotonic())[1:],(yellow,yellow))
        self.assertEqual(camera.read.call_count,2)
        load.assert_not_called()
        detector.predict.assert_called_with(frame,imgsz=320,conf=.8,device="cpu",verbose=False)


if __name__=="__main__":
    unittest.main()
