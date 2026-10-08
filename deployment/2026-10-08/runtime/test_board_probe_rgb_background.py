"""Pure RGB correspondence/mask and fixed-X invariant regressions."""
import importlib.util
from pathlib import Path
import unittest

import cv2
import numpy as np

PATH=Path(__file__).parent/'gen4_root_cause/review_board_probe_rgb_background.py'
SPEC=importlib.util.spec_from_file_location('background_review',PATH)
M=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(M)


class BackgroundReviewTests(unittest.TestCase):
    def test_hands_keyboard_and_board_are_excluded_from_mousemat_mask(self):
        image=np.full((480,640,3),[30,50,220],np.uint8);board=np.zeros((480,640),np.uint8);board[180:320,230:490]=255
        masks=M.diagnostic_masks(image,board)
        self.assertFalse(masks['mousemat_texture'][:,:180].any());self.assertFalse(masks['mousemat_texture'][board>0].any())
        self.assertFalse(masks['laptop_shell'][:,:545].any());self.assertFalse(masks['phone_whitebox'][:,:205].any())

    def test_bidirectional_RGB_tracks_known_pixel_shift_and_static_control(self):
        random=np.random.RandomState(17);gray=np.uint8(random.randint(0,256,(160,200)))
        A=cv2.cvtColor(gray,cv2.COLOR_GRAY2BGR);B=cv2.warpAffine(A,np.float32([[1,0,-2],[0,1,-7]]),(200,160))
        mask=np.zeros((160,200),np.uint8);mask[20:140,20:180]=255
        x,y,quality=M.track_region(A,B,mask);self.assertGreater(quality['accepted'],100)
        np.testing.assert_allclose(np.median(y-x,axis=0),[-2,-7],atol=.05)
        x,y,quality=M.track_region(A,A,mask);self.assertGreater(quality['accepted'],100)
        self.assertLess(np.median(np.linalg.norm(y-x,axis=1)),.01)

    def test_fixed_handeye_conjugation_cannot_change_rotation_angle(self):
        R=cv2.Rodrigues(np.array([.05,-.02,.01]))[0];X=cv2.Rodrigues(np.array([.7,.3,-.6]))[0]
        self.assertAlmostEqual(M.CORE.rotation_angle_deg(R),M.CORE.rotation_angle_deg(X.T@R@X),places=10)


if __name__=='__main__':unittest.main()
