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
import competition_boot as boot
import target_patrol
from components.pose_fusion import FusedPoseEstimate, PoseFusion, PoseFusionState
from components.task_board_reader import TaskCounts
from config.v2_runtime import RuntimeMode
from core.types import Pose2D, Twist2D
from main_robocup import main, stop_marker_signature
from mission_control import step_runtime
from robocup_runtime import RobocupMissionState, build_runtime, load_runtime_config


class MainPatrolIntegrationTests(unittest.TestCase):
    def test_four_targets_five_releases_and_right_mount_resume_original_route(self):
        self.vision.observe_colours.side_effect = lambda now: (
            now, tuple((color, self.boxes()[0]) for color in ('green', 'red', 'yellow', 'blue')))
        self.assertEqual(self.run_main(), 0)
        self.assertEqual([c.args for c in self.open.call_args_list], [(1,), (4,), (2,), (3,), (0,)])
        drops = [v for e, v in self.events if e == 'payload_drop' and v['detour_completed']]
        self.assertEqual(len(drops), 4)
        special = [v['stage'] for e, v in self.events if e == 'payload_detour_stage_start'
                   and v['behavior'] == 'right_mount']
        self.assertEqual(special, ['left_90deg', 'safe_side_approach', 'right_mount_left_90deg',
                                   'right_mount_right_90deg', 'return_from_drop', 'right_90deg'])
        for endpoints in (((0., 0.), (1.05, 0.)), ((1.05, 0.), (1.05, .85)),
                           ((1.05, .85), (0., .85))):
            self.assertTrue(any(c.args == endpoints for c in self.runtime.motion.track_global_line.call_args_list))
        self.assertLess(math.dist(self.pose[:2], (0., .85)), .035)
        self.servo.set_angle.assert_called_once_with(90, settle=True)
        self.assertEqual(self.config.servo.chip_device_match, ('febd0030.pwm',))
        self.assertEqual(self.runtime.mission.state, RobocupMissionState.FINISHED)

    def setUp(self):
        self.stack = ExitStack()
        self.stack.enter_context(patch.object(task,"payload_configured",return_value=True))
        self.addCleanup(self.stack.close)
        self.open=self.stack.enter_context(patch("payload_detour.open_payload"))
        self.final_open=self.stack.enter_context(patch("main_robocup.open_payload"))
        self.now = 10.
        self.pose = [0., 0., 0.]
        self.events = []
        self.steps = 0
        self.detour_caps = {}
        self.center_available = True
        self.stop_during_detour = False
        self.config = load_runtime_config()
        self.config = replace(self.config, servo=replace(self.config.servo, enabled=True),
                              d500_mount=replace(self.config.d500_mount, x_m=0., y_m=0.))
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
        self.vision.observe_colours.side_effect = lambda now: (
            now, tuple((color, self.boxes()[0]) for color in ('yellow', 'blue')))
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
        original_full = task.run_full_mission
        def with_fake_detector(runtime, **kwargs):
            kwargs['detector'] = Mock()
            return original_full(runtime, **kwargs)
        self.stack.enter_context(patch.object(task,'run_full_mission',side_effect=with_fake_detector))
        # A different field path, not the test fixture's 280/420/250 cm rectangle.
        for name,value in {"ROUTE_POINTS":((0.,0.),(1.05,0.),(1.05,.85),(0.,.85)),
                           "LANE_ENTRY_X":0.,"TASK_BOARD_X":0.,"CORNER_1_X":1.05,
                           "CROSS_LANE_YAW_DEG":90,"YELLOW_SEARCH_START_X":1.05,
                           "YELLOW_SEARCH_START_Y":.10,"YELLOW_SEARCH_END_X":1.05,
                           "YELLOW_SEARCH_END_Y":.65,"CORNER_2_X":1.05,
                           "CORNER_2_Y":.85,"FINISH_X":0.,"FINISH_Y":.85}.items():
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
            self.stop_file.write_text('STOP issued during this run')

    def run_main(self, *extra, **kwargs):
        return main(["--mode","hardware-mission","--competition",
                     "--speed-scale","2","--log-dir",self.directory,*extra], **kwargs)

    def test_full_entry_preserves_field_route_and_uses_shared_fused_motion(self):
        self.assertEqual(self.run_main(),0)
        self.assertEqual(self.runtime.mission.state,RobocupMissionState.FINISHED)
        self.assertEqual(self.runtime.mission.task_counts,self.counts)
        self.send.assert_called_once_with(task.DroneAllocation(0,0,0),runtime=self.runtime)
        self.assertEqual(self.used_config.navigation.translation_speed_scale,2)
        self.assertEqual(self.used_config.drive.max_linear_accel_m_s2,self.config.drive.max_linear_accel_m_s2)
        kwargs=self.vision_factory.call_args.kwargs
        self.assertEqual(kwargs["confidence"],.8)
        self.assertNotIn("target_class_name",kwargs)  # One worker supplies both named colours.
        self.assertAlmostEqual(kwargs["drop_center_width_ratio"],.1)
        # +90 search; blind drop never moves the camera servo.
        self.assertEqual(self.servo.set_angle.call_args_list,
                         [call(90, settle=True)])
        self.servo.close.assert_called_once_with(hold=True)
        self.assertEqual(self.vision.close.call_count,1)
        calls=self.runtime.motion.track_global_line.call_args_list
        self.assertTrue(any(c.args==((0.,0.),(1.05,0.)) for c in calls))
        self.assertTrue(any(c.args==((1.05,0.),(1.05,.85)) for c in calls))
        self.assertTrue(any(c.args==((1.05,.85),(0.,.85)) for c in calls))
        starts={v["stage"]:v for e,v in self.events if e=="payload_detour_stage_start"}
        self.assertLess(math.dist(starts["safe_side_approach"]["args"][0],
                         starts["return_from_drop"]["args"][1]), .01)
        self.assertTrue(starts["return_from_drop"]["kwargs"]["reverse"])
        self.assertTrue(any(c.kwargs.get("reverse") for c in calls))
        self.assertLess(math.dist(self.pose[:2],(0.,.85)),.035)
        self.assertEqual(self.runtime.drive.last_limited_twist,Twist2D(0,0))
        self.final_open.assert_called()
        self.assertNotIn("advance_7cm", self.detour_caps)
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
        centered=[v for e,v in self.events if e=='competition_target_centered']
        self.assertTrue(centered)
        self.assertTrue(all(v['cx_px']==320 and v['tolerance_px']==32 for v in centered))
        for index,(event,values) in enumerate(self.events):
            if event=='competition_target_centered':
                self.assertEqual(self.events[index+1][0],'competition_stage_start')
                self.assertEqual(self.events[index+2][0],'payload_detour_road_pose')
                first=next(v for e,v in self.events[index+1:] if e=='payload_detour_stage_start')
                self.assertEqual(first['stage'],'left_90deg')
        forbidden={'search_gain_probe','search_road_gain','search_road_adjust',
                   'competition_target_blind_drop'}
        self.assertFalse(forbidden.intersection(e for e,v in self.events))

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
        self.assertEqual(self.run_main(),0)
        stages=[v['stage'] for e,v in self.events if e=='payload_detour_stage_start']
        self.assertEqual(stages,['left_90deg','safe_side_approach','return_from_drop','right_90deg']*2)
        release=[v for e,v in self.events if e=='payload_release_done']
        self.assertEqual(len(release),2)
        self.assertEqual(release[0]['actuator'],'payload_servo')
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
        self.assertEqual(self.run_main(),0)
        self.assertTrue(any(e=='drop_align_done' and v['source']=='blind_fused_endpoint' for e,v in self.events))
        starts=[v['stage'] for e,v in self.events if e=='payload_detour_stage_start']
        self.assertEqual(starts,['left_90deg','safe_side_approach','return_from_drop','right_90deg']*2)
        self.assertEqual(self.runtime.mission.state,RobocupMissionState.FINISHED)

    def test_worker_failure_stops_and_closes_vision_and_servo(self):
        self.vision.observe_colours.side_effect=RuntimeError("stale inference")
        self.assertEqual(self.run_main(),1)
        self.assertEqual(self.runtime.mission.state,RobocupMissionState.SAFE_STOP)
        self.vision.close.assert_called_once()
        self.servo.close.assert_called_once_with(hold=True)
        self.assertEqual(self.runtime.drive.last_limited_twist,Twist2D(0,0))

    def test_release_failure_stops_before_second_corner(self):
        with patch("payload_detour.open_payload",side_effect=RuntimeError("release failed")):
            self.assertEqual(self.run_main(),1)
        self.assertFalse(any(e=="competition_route_leg_start" and v["leg"]==3 for e,v in self.events))
        self.assertEqual(self.runtime.mission.state,RobocupMissionState.SAFE_STOP)

    def test_stop_file_interrupts_nested_payload_motion_and_guard_is_scoped(self):
        self.stop_during_detour=True
        self.assertIsNone(stop_marker_signature(self.stop_file))
        self.assertEqual(self.run_main(stop_file_baseline=None),1)
        self.assertFalse(any(e=="payload_release_start" for e,v in self.events))
        self.assertEqual(self.runtime.mission.state, RobocupMissionState.SAFE_STOP)
        self.assertEqual(self.runtime.drive.last_limited_twist,Twist2D(0,0))
        # An independent task must not inherit this mission's pending STOP file.
        fresh=self.make_runtime(self.config,RuntimeMode.DRY_RUN)
        fresh.start();fresh.motion.stop()
        step_runtime(fresh)
        self.assertNotEqual(fresh.mission.state,RobocupMissionState.SAFE_STOP)

    def test_unchanged_old_stop_allows_new_run_without_deleting_marker(self):
        self.stop_file.write_text('previous round STOP')
        baseline = stop_marker_signature(self.stop_file)
        self.assertEqual(self.run_main(stop_file_baseline=baseline), 0)
        self.assertEqual(self.runtime.mission.state, RobocupMissionState.FINISHED)
        self.assertEqual(stop_marker_signature(self.stop_file), baseline)
        self.assertEqual(self.stop_file.read_text(), 'previous round STOP')

    def test_rewritten_old_stop_interrupts_nested_payload_motion(self):
        self.stop_file.write_text('old STOP')
        baseline = stop_marker_signature(self.stop_file)
        self.stop_during_detour = True
        self.assertEqual(self.run_main(stop_file_baseline=baseline), 1)
        self.assertNotEqual(stop_marker_signature(self.stop_file), baseline)
        self.assertFalse(any(e == 'payload_release_start' for e, v in self.events))
        self.assertEqual(self.runtime.mission.state, RobocupMissionState.SAFE_STOP)
        self.assertEqual(self.runtime.drive.last_limited_twist, Twist2D(0, 0))

    def test_direct_main_with_old_stop_still_rejects_before_motion(self):
        self.stop_file.write_text('old STOP')
        self.assertEqual(self.run_main(), 1)
        self.send.assert_not_called()
        self.assertEqual(self.steps, 0)
        self.assertEqual(self.runtime.mission.state, RobocupMissionState.SAFE_STOP)
        self.assertEqual(self.runtime.drive.last_limited_twist, Twist2D(0, 0))

    def test_removed_old_stop_is_not_a_new_stop_event(self):
        self.stop_file.write_text('old STOP')
        baseline = stop_marker_signature(self.stop_file)
        self.stop_file.unlink()
        self.assertEqual(self.run_main(stop_file_baseline=baseline), 0)
        self.assertEqual(self.runtime.mission.state, RobocupMissionState.FINISHED)

    def test_stop_updated_during_boot_countdown_rejects_first_main_guard(self):
        self.stop_file.write_text('old STOP')
        baseline = stop_marker_signature(self.stop_file)
        stop = Mock()
        stop.is_set.return_value = False
        stop.set.side_effect = lambda: setattr(stop.is_set, 'return_value', True)
        def wait(duration):
            self.now += duration
            self.stop_file.write_text('new STOP during countdown')
            return False
        stop.wait.side_effect = wait
        sender = Mock()
        mission = Mock(side_effect=lambda argv, **kwargs: self.run_main(**kwargs))
        listener = boot.CompetitionBoot(driver_factory=Mock(), mission_main=mission,
                                       alarm=Mock(), clock=lambda: self.now)
        with patch.object(boot, 'Path', return_value=self.stop_file), \
             patch.object(boot.threading, 'Event', return_value=stop), \
             patch.object(boot.threading, 'Thread') as thread, \
             patch.object(boot, 'FleetTaskSender', return_value=sender):
            self.assertEqual(listener._start_mission()[0], boot.AckStatus.ACCEPTED)
            args = thread.call_args.kwargs['args']
        self.assertEqual(args[4], baseline)
        with patch.object(boot, 'close_payload'), patch.object(boot, 'open_payload'), \
             patch.object(boot, 'restart_car_slam_sidecar'):
            listener._run_mission(*args)
        self.assertEqual(mission.call_args.kwargs['stop_file_baseline'], baseline)
        self.send.assert_not_called()
        sender.send_task_and_start.assert_not_called()
        self.assertEqual(self.steps, 0)
        self.assertEqual(self.runtime.mission.state, RobocupMissionState.SAFE_STOP)
        self.assertEqual(self.runtime.drive.last_limited_twist, Twist2D(0, 0))
        listener._alarm.off.assert_called()

    def test_stop_stat_error_during_motion_fails_safely(self):
        def signature(path):
            if self.steps > 0:
                raise PermissionError('STOP stat denied')
            return stop_marker_signature(path)
        with patch('main_robocup.stop_marker_signature', side_effect=signature):
            self.assertEqual(self.run_main(), 1)
        self.assertGreater(self.steps, 0)
        self.assertFalse(any(e == 'payload_release_start' for e, v in self.events))
        self.assertEqual(self.runtime.mission.state, RobocupMissionState.SAFE_STOP)
        self.assertEqual(self.runtime.drive.last_limited_twist, Twist2D(0, 0))

    def test_stop_event_is_checked_before_marker_stat(self):
        stop = Mock()
        stop.is_set.return_value = True
        with patch('main_robocup.stop_marker_signature') as signature:
            self.assertEqual(self.run_main(stop_event=stop), 1)
        signature.assert_not_called()
        self.assertEqual(self.steps, 0)
        self.assertEqual(self.runtime.mission.state, RobocupMissionState.SAFE_STOP)

    def test_task_deadline_interrupts_original_lane_motion(self):
        self.assertEqual(self.run_main("--max-seconds","1"),1)
        self.send.assert_called_once()  # Handoff precedes the three-leg patrol.
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
        self.final_open.assert_called_once_with()

    def test_invalid_speed_and_conflicting_simulation_port_fail_before_runtime(self):
        for value in ("0","-1","nan","inf"):
            self.assertEqual(self.run_main("--speed-scale",value),2)
        with self.assertRaises(SystemExit):
            self.run_main("--relay-port","/dev/physical-relay")
        self.builder.assert_not_called()


class StopMarkerSignatureTests(unittest.TestCase):
    def test_signature_covers_file_identity_size_and_nanosecond_timestamps(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'STOP'
            self.assertIsNone(stop_marker_signature(path))
            path.write_text('STOP')
            st = path.stat()
            baseline = stop_marker_signature(path)
            self.assertEqual(baseline, (st.st_dev, st.st_ino, st.st_size,
                                        st.st_mtime_ns, st.st_ctime_ns))
            path.write_text('new STOP with different length')
            self.assertNotEqual(stop_marker_signature(path), baseline)

    def test_stat_errors_are_not_treated_as_absence(self):
        for error in (PermissionError('denied'), OSError('stat failed')):
            with self.subTest(error=error):
                path = Mock()
                path.stat.side_effect = error
                with self.assertRaises(type(error)):
                    stop_marker_signature(path)


class SharedVisionWorkerTests(unittest.TestCase):
    def test_colour_snapshot_preserves_freshness_and_worker_error_gates(self):
        vision = target_patrol.YoloVision()
        vision._frame_at = 10.
        vision._colours = (('blue',(300,0,340,40,.9,99)),)
        self.assertEqual(vision.observe_colours(11.)[1],vision._colours)
        with self.assertRaisesRegex(RuntimeError,'stale'):
            vision.observe_colours(13.)
        vision._error = 'camera failed'
        with self.assertRaisesRegex(RuntimeError,'camera failed'):
            vision.observe_colours(10.)

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
        stamp, colours = vision.observe_colours(time.monotonic())
        self.assertEqual(colours, (('red',(590,0,690,40,.99,0)),('yellow',yellow)))
        self.assertIsInstance(colours, tuple)
        self.assertEqual(camera.read.call_count,2)
        load.assert_not_called()
        detector.predict.assert_called_with(frame,imgsz=320,conf=.8,device="cpu",verbose=False)


if __name__=="__main__":
    unittest.main()
