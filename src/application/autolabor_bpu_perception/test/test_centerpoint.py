import unittest
from types import SimpleNamespace as NS
import numpy as np
from autolabor_bpu_perception.centerpoint import decode,SHAPES
from autolabor_bpu_perception.cloud import pointcloud_xyzi,model_points,supported_box


class CenterPointTest(unittest.TestCase):
    def outputs(self):
        outputs=[np.zeros(s,np.float32) for s in SHAPES]
        for i in range(5,36,6):outputs[i].fill(-20)
        return outputs

    def test_known_head_location_dimensions_and_yaw(self):
        outputs=self.outputs();head=4*6;y,x=60,70
        outputs[head+5][y,x,0]=10
        outputs[head][y,x]=[.25,.5];outputs[head+1][y,x]=.2
        outputs[head+2][y,x]=np.log([1,2,3]);outputs[head+3][y,x]=[1,0]
        result=decode(outputs)
        self.assertEqual(len(result),1);d=result[0]
        self.assertEqual(d['class_name'],'motorcycle')
        np.testing.assert_allclose(d['position'],[5,-2.8,.2],atol=1e-6)
        np.testing.assert_allclose(d['dimensions'],[1,2,3],atol=1e-6)
        self.assertAlmostEqual(d['yaw'],np.pi/2);self.assertFalse(d['velocity_valid'])

    def test_empty_and_malformed_outputs(self):
        outputs=self.outputs();self.assertEqual(decode(outputs),[])
        outputs[0][0,0,0]=np.nan
        with self.assertRaises(ValueError):decode(outputs)

    def test_organized_big_endian_padded_rows(self):
        data=bytearray(2*80)
        dtype=np.dtype(dict(names=['x','y','z','intensity'],formats=['>f4']*4,offsets=[0,4,8,16],itemsize=32))
        view=np.ndarray((2,2),dtype=dtype,buffer=data,strides=(80,32))
        for i,k in enumerate(dtype.names):view[k]=np.arange(4).reshape(2,2)+i
        fields=[NS(name=k,datatype=7,count=1,offset=o) for k,o in zip(dtype.names,[0,4,8,16])]
        msg=NS(fields=fields,width=2,height=2,point_step=32,row_step=80,data=data,is_bigendian=True)
        np.testing.assert_array_equal(pointcloud_xyzi(msg),np.arange(4)[:,None]+np.arange(4))
        msg.data=data[:20]
        with self.assertRaises(ValueError):pointcloud_xyzi(msg)

    def test_deskewed_frame_conversion_and_self_crop(self):
        lidar=np.array([[2,0,0,20],[0,0,0,20],[50,0,0,20]],np.float32)
        body=lidar.copy();body[:,:3]+=[-.011,-.02329,.04412]
        result=model_points(body);self.assertEqual(result.shape,(1,5))
        np.testing.assert_allclose(result[0],[2,0,0,20,0],atol=1e-6)

    def test_boxes_require_actual_point_support(self):
        d=dict(position=[2,0,0],dimensions=[1,1,1],yaw=0)
        points=np.tile([2,0,0,1,0],(8,1))
        self.assertTrue(supported_box(d,points));self.assertFalse(supported_box(d,points[:7]))


if __name__=='__main__':unittest.main()
