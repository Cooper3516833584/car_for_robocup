"""Hardware-free contracts for unified motion actions."""
from pathlib import Path
import sys
import math
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from components.basic_motion_controller import BasicMotionController, MotionActionState, MotionBusyError, MotionActionType
from components.differential_navigation import DifferentialNavigator
from config.v2_loader import load_v2_config
from core.types import Pose2D, Twist2D

class BasicMotionTests(unittest.TestCase):
    def setUp(self):
        c = load_v2_config()
        self.motion = BasicMotionController(DifferentialNavigator(c.drive, c.navigation), c.navigation, c.drive)

    def step(self, x=0., y=0., yaw=0., t=1.):
        p = Pose2D(x,y,yaw,t)
        return self.motion.step(p, now_s=t, t265_pose=p)

    def test_busy_cancel_and_safe_stop(self):
        self.motion.track_global_line((0,0),(1,0))
        with self.assertRaises(MotionBusyError): self.motion.rotate(1)
        self.motion.stop()
        self.assertEqual(self.step().state, MotionActionState.CANCELLED)
        self.motion.safe_stop('operator')
        self.assertEqual(self.step().command, Twist2D(0,0))
        with self.assertRaises(RuntimeError): self.motion.rotate(1)

    def test_invalid_points(self):
        for a,b in [((0,0),(0,0)), ((float('nan'),0),(1,0)), ((0,),(1,0))]:
            with self.assertRaises(ValueError): self.motion.track_local_line(a,b)

    def test_lateral_offsets_continue_forward(self):
        for y in (.01,.03,.05,.10,-.10):
            self.motion.stop()
            self.motion.track_global_line((0,0),(2.8,0))
            out = self.step(.1,y)
            self.assertEqual(out.state, MotionActionState.RUNNING)
            self.assertGreater(out.command.linear_x_m_s,0)
            self.assertLess(out.command.angular_z_rad_s*y,0)

    def test_progress_monotonic_and_resume_projects_original_line(self):
        self.motion.track_global_line((0,0),(2.8,0))
        self.assertAlmostEqual(self.step(.8).diagnostics['progress_m'],.8)
        out = self.step(.7,t=1.05)
        self.assertAlmostEqual(out.diagnostics['progress_m'],.8)
        self.assertGreater(out.diagnostics['carrot_x_m'],.8)
        self.motion.stop()
        self.motion.track_global_line((0,0),(2.8,0))
        self.assertAlmostEqual(self.step(1.1,t=1.1).diagnostics['progress_m'],1.1)

    def test_endpoint_no_pivot_or_reverse(self):
        self.motion.track_local_line((0,0),(.47,0))
        out = self.step(.47,.02)
        self.assertEqual(out.state, MotionActionState.SUCCEEDED)
        self.assertEqual(out.command,Twist2D(0,0))
        self.motion.stop()
        self.motion.track_local_line((0,0),(.47,0))
        out = self.step(.47,.04)
        self.assertEqual(out.state, MotionActionState.SUCCEEDED)
        self.assertEqual(out.command,Twist2D(0,0))
        self.assertAlmostEqual(out.diagnostics['cross_track_m'],.04)

    def test_pose_loss_zero_and_no_old_commands(self):
        self.motion.track_local_line((0,0),(1,0))
        self.step()
        out=self.motion.step(None,now_s=1.05)
        self.assertEqual(out.state,MotionActionState.POSE_LOST)
        self.assertEqual(out.command,Twist2D(0,0))

    def test_navigation_uses_same_kernel(self):
        self.motion.navigate_to(1,0)
        out=self.step(.1,.05)
        self.assertIn('curvature_inv_m',out.diagnostics)
        self.assertIsNotNone(self.motion.last_navigation_output)

    def test_reverse_correct_sign_and_reuses_endpoints(self):
        self.motion.track_local_line((.47,0),(0,0),reverse=True)
        out=self.step(.45,.05)
        self.assertLess(out.command.linear_x_m_s,0)
        self.assertGreater(out.command.angular_z_rad_s,0)
        self.assertTrue(out.diagnostics['reverse'])

    def test_all_line_actions_forward_t265_and_wait_for_alignment(self):
        for method,args in (("track_global_line",((0,0),(1,0))),
                            ("track_local_line",((0,0),(1,0))),
                            ("navigate_to",(1,0)), ("navigate_to_pose",(1,0,0))):
            with self.subTest(method=method):
                self.motion.stop()
                getattr(self.motion,method)(*args)
                self.step(yaw=math.pi/2,t=1.)
                self.assertEqual(self.step(yaw=math.radians(55),t=1.05).command.linear_x_m_s,0)
                self.step(t=1.10)
                for i in range(1,5): out=self.step(t=1.10+.05*i)
                self.assertEqual(out.diagnostics["line_state"],"align_settled")
                self.assertEqual(out.state,MotionActionState.RUNNING)
                self.assertGreater(self.step(t=1.35).command.linear_x_m_s,0)

    def test_cancel_restart_clears_line_alignment_target(self):
        self.motion.track_global_line((0,0),(1,0))
        self.step(yaw=math.pi/2)
        self.motion.stop()
        self.assertIsNone(self.motion.navigator.controller._line_align_yaw)
        self.motion.track_local_line((0,0),(0,1))
        out=self.step(t=1.05)
        self.assertGreater(out.command.angular_z_rad_s,0)
        self.assertEqual(out.command.linear_x_m_s,0)

    def test_line_alignment_never_uses_fused_pose_as_settle_sample(self):
        self.motion.track_global_line((0,0),(1,0))
        out=self.motion.step(Pose2D(0,0,math.pi/2,1.),now_s=1.)
        self.assertEqual(out.state,MotionActionState.POSE_LOST)
        self.assertEqual(out.command,Twist2D(0,0))

    def test_legacy_actions_removed(self):
        for name in ('drive_distance','follow_segment','drive_to','face_point'):
            self.assertFalse(hasattr(self.motion,name))
        self.assertEqual({x.value for x in MotionActionType}, {'track_global_line','track_local_line',
            'rotate_relative','rotate_to','rotate_local_to','navigate_to','navigate_to_pose'})

if __name__ == '__main__': unittest.main()
