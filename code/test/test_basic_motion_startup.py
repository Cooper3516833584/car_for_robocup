"""Measured turn settling replaces retired fixed startup yaw compensation."""
import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from components.basic_motion_controller import BasicMotionController, MotionActionState
from components.differential_navigation import DifferentialNavigator
from config.v2_loader import load_v2_config
from core.types import Pose2D, Twist2D

class TurnSettlingTests(unittest.TestCase):
    def setUp(self):
        c=load_v2_config()
        self.motion=BasicMotionController(DifferentialNavigator(c.drive,c.navigation),c.navigation,c.drive)
        self.motion.rotate_to(0)
    def step(self,t,yaw=0.,local_yaw=0.,sample_t=None):
        return self.motion.step(Pose2D(0,0,yaw,t),now_s=t,
            t265_pose=Pose2D(0,0,local_yaw,t if sample_t is None else sample_t))
    def test_first_angle_entry_and_repeated_sample_do_not_complete(self):
        self.assertEqual(self.step(1.).state,MotionActionState.RUNNING)
        for t in (1.01,1.02,1.03):
            out=self.step(t,sample_t=1.)
            self.assertEqual(out.diagnostics['stable_frames'],0)
        for i in range(1,5): out=self.step(1.+.05*i)
        self.assertEqual(out.state,MotionActionState.SUCCEEDED)
    def test_fused_yaw_stable_but_t265_still_rotating_cannot_complete(self):
        for i in range(6):
            out=self.step(1.+i*.05,local_yaw=i*.01)
            self.assertEqual(out.state,MotionActionState.RUNNING)
        self.assertGreater(out.diagnostics['t265_yaw_rate_rad_s'],math.radians(3))
    def test_coast_past_fixed_target_requires_correction(self):
        self.step(1.,yaw=.02)
        out=self.step(1.05,yaw=.1,local_yaw=.1)
        self.assertEqual(out.state,MotionActionState.RUNNING)
        self.assertLess(out.command.angular_z_rad_s,0)
        self.assertEqual(out.diagnostics['stable_frames'],0)
        self.step(1.1)
        for i in range(1,5): out=self.step(1.1+i*.05)
        self.assertEqual(out.state,MotionActionState.SUCCEEDED)
    def test_missing_and_stale_t265_stop(self):
        out=self.motion.step(Pose2D(0,0,1,1),now_s=1)
        self.assertEqual(out.command,Twist2D(0,0))
        self.assertEqual(out.state,MotionActionState.POSE_LOST)
        out=self.step(2.,yaw=1.,sample_t=1.)
        self.assertEqual(out.command,Twist2D(0,0))
    def test_relative_turn_wrap_and_full_turn(self):
        self.motion.stop()
        self.motion.rotate(2*math.pi)
        start=math.radians(170)
        self.step(1.,yaw=start,local_yaw=start)
        for i in range(1,17):
            yaw=(start+i*2*math.pi/16+math.pi)%(2*math.pi)-math.pi
            out=self.step(1.+i*.05,yaw=yaw,local_yaw=yaw)
        self.assertEqual(out.state,MotionActionState.RUNNING)
        for i in range(1,5): out=self.step(1.8+i*.05,yaw=start,local_yaw=start)
        self.assertEqual(out.state,MotionActionState.SUCCEEDED)
    def test_navigation_final_yaw_uses_same_measured_settle(self):
        self.motion.stop()
        self.motion.navigate_to_pose(0,0,0)
        self.step(1.)
        for i in range(6): out=self.step(1.05+i*.05)
        self.assertEqual(out.state,MotionActionState.SUCCEEDED)

if __name__=='__main__': unittest.main()
