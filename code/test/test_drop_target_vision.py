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
import target_patrol as patrol
from components.basic_motion_controller import MotionActionState
from components import drop_target_vision as dvision
from components.pose_fusion import FusedPoseEstimate, PoseFusion, PoseFusionState
from config.v2_runtime import RuntimeMode
from core.types import Pose2D
from robocup_runtime import build_runtime, load_runtime_config





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


class RoadTriggerTests(unittest.TestCase):
    def test_native_width_inclusive_boundaries_and_no_vertical_gate(self):
        for width, low, high in ((640,288,352),(1280,576,704)):
            for cx, expected in ((low-1,False),(low,True),(width/2,True),
                                 (high,True),(high+1,False)):
                for y in (0, 680):
                    with self.subTest(width=width,cx=cx,y=y):
                        box=(cx-10,y,cx+10,y+20,.8,3)
                        result=patrol.horizontal_target([box],width,min_conf=.8)
                        self.assertEqual(result is not None,expected)
                        if expected:
                            self.assertEqual(result,box)

    def test_invalid_or_low_confidence_boxes_never_trigger(self):
        for box in ((300,0,340,40,.79,3),(340,0,300,40,.95,3),
                    (300,40,340,0,.95,3),(300,0,math.nan,40,.95,3)):
            self.assertIsNone(patrol.horizontal_target([box],640,min_conf=.8))

    def fixture(self, centres):
        self.tick=0
        self.events=[]
        config=load_runtime_config()
        self.runtime=SimpleNamespace(motion=SimpleNamespace(drive=config.drive,
            state=MotionActionState.RUNNING,stop=Mock(),track_global_line=Mock()),
            drive=Mock(),mission=Mock(),record_event=lambda e,**v:self.events.append((e,v)))
        self.original_drive=config.drive
        def observe(now):
            cx=centres[min(self.tick,len(centres)-1)]
            box=None if cx is None else (cx-20,0,cx+20,40,.95,3)
            centered=patrol.horizontal_target([box] if box else [],640,min_conf=.8)
            return now,box,centered
        self.vision=SimpleNamespace(ready=True,observe_drop=observe)
        def step(_):
            self.tick+=1
            if self.tick>=len(centres):
                self.runtime.motion.state=MotionActionState.SUCCEEDED
        self.step=step

    def test_visibility_disappearance_restores_speed_then_first_center_stops(self):
        self.fixture([250,None,320])
        with patch.object(patrol,'_step',side_effect=self.step):
            box=patrol.find_centered_target(self.runtime,self.vision,(0,0),(1,0),
                clock=lambda:10.+self.tick*.05,sleep=lambda _:None)
        self.assertEqual((box[0]+box[2])/2,320)
        speeds=[v for e,v in self.events if e=='target_search_speed']
        self.assertEqual([v['slowing'] for v in speeds],[True,False,True])
        self.assertEqual([v['max_speed_m_s'] for v in speeds],
                         [.08,self.original_drive.max_linear_speed_m_s,.08])
        self.assertEqual(self.tick,2)  # A single central frame; no confirmation step.
        self.runtime.motion.track_global_line.assert_called_once_with((0,0),(1,0))
        self.runtime.motion.stop.assert_called()
        self.runtime.drive.stop.assert_called()
        self.assertIs(self.runtime.motion.drive,self.original_drive)

    def test_visible_uncentered_target_only_slowly_finishes_original_line(self):
        self.fixture([250,250,250])
        with patch.object(patrol,'_step',side_effect=self.step):
            result=patrol.find_centered_target(self.runtime,self.vision,(0,0),(1,0),
                clock=lambda:10.+self.tick*.05,sleep=lambda _:None)
        self.assertIsNone(result)
        self.runtime.motion.track_global_line.assert_called_once_with((0,0),(1,0))
        self.assertFalse(any(e=='target_search_centered' for e,v in self.events))
        self.assertTrue(any(e=='target_search_exhausted' for e,v in self.events))
        self.assertIs(self.runtime.motion.drive,self.original_drive)




if __name__ == "__main__":
    unittest.main()
