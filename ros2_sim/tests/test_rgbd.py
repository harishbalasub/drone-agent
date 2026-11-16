import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from rgbd.geometry import localize_package
from rgbd.data import Synchronizer,decode_image
from rgbd.world import camera_transform,sample_scene,write_world
from rgbd.train_segmentation import validate_splits

class RGBDContracts(unittest.TestCase):
    def test_downward_camera_metric_center_and_height_correction(self):
        result=localize_package(np.ones((5,5)),np.ones((5,5)),{'fx':100,'fy':100,'cx':2,'cy':2},camera_transform([3,4,1.24],0))
        np.testing.assert_allclose(result['position'],[3,4,.12],atol=1e-6)
        self.assertFalse(result['uncertainty_calibrated'])
    def test_yaw_preserves_center(self):
        result=localize_package(np.ones((5,5)),np.ones((5,5)),{'fx':100,'fy':100,'cx':2,'cy':2},camera_transform([3,4,1.24],np.pi/2))
        np.testing.assert_allclose(result['position'],[3,4,.12],atol=1e-6)
    def test_missing_depth_and_bad_alignment_rejected(self):
        args=({'fx':100,'fy':100,'cx':2,'cy':2},camera_transform([3,4,1.24],0))
        with self.assertRaisesRegex(ValueError,'INSUFFICIENT'):localize_package(np.full((5,5),np.nan),np.ones((5,5)),*args)
        with self.assertRaisesRegex(ValueError,'matching'):localize_package(np.ones((5,5)),np.ones((4,4)),*args)
    def test_synchronizer_never_pairs_arbitrary_latest_frames(self):
        s=Synchronizer()
        def msg(n):return NS(header=NS(stamp=NS(sec=1,nsec=n)))
        s.add('rgb',msg(1));s.add('depth',msg(2));s.add('mask',msg(1));self.assertIsNone(s.matched())
        s.add('depth',msg(1));self.assertEqual(s.matched()[0],1000000001)
        self.assertIsNone(s.matched(after_ns=1000000001))
    def test_depth_rows_decode_with_padding(self):
        raw=np.array([1,2],dtype='<f4').tobytes()+b'xxxx'+np.array([3,4],dtype='<f4').tobytes()+b'xxxx'
        a=decode_image(NS(width=2,height=2,step=12,data=raw),1,np.dtype('<f4'))
        np.testing.assert_allclose(a,[[1,2],[3,4]])
    def test_episode_leakage_is_rejected(self):
        rows=[{'episode_seed':0,'split':'train'},{'episode_seed':14,'split':'validation'},{'episode_seed':17,'split':'test'}]
        validate_splits(rows)
        rows[-1]['episode_seed']=0
        with self.assertRaisesRegex(ValueError,'leakage'):validate_splits(rows)
    def test_generated_masks_are_labels_not_package_color_thresholds(self):
        import xml.etree.ElementTree as E
        with tempfile.TemporaryDirectory() as folder:
            scene=sample_scene(0);p=write_world(scene,Path(folder)/'scene.sdf');root=E.parse(p).getroot()
            self.assertIsNone(root.find('world/model[@name="drone"]'))
            self.assertEqual(root.find('world/model[@name="red_package"]/plugin/label').text,'1')
            self.assertIsNotNone(root.find('.//sensor[@type="segmentation"]'))
            negative=sample_scene(1);self.assertFalse(negative['target_present']);write_world(negative,p)
            self.assertIsNone(E.parse(p).getroot().find('world/model[@name="red_package"]'))

if __name__=='__main__':unittest.main()
