import ctypes as C
import numpy as np
from autolabor_bpu_perception.centerpoint import SHAPES,OUTPUT_FLOATS


class ResidentCenterPoint:
    def __init__(self,lab,model):
        self.context=None
        self.library=C.CDLL(str(lab/'bin/libcenterpoint_resident.so'))
        self.library.centerpoint_create.argtypes=[C.c_char_p,C.c_void_p,C.c_size_t]
        self.library.centerpoint_create.restype=C.c_void_p
        self.library.centerpoint_infer.argtypes=[C.c_void_p,C.c_void_p,C.c_size_t,C.c_void_p,
                                                C.c_size_t,C.c_void_p,C.c_void_p,C.c_size_t]
        self.library.centerpoint_infer.restype=C.c_int
        self.library.centerpoint_destroy.argtypes=[C.c_void_p]
        self.library.centerpoint_destroy.restype=None
        self.error=C.create_string_buffer(512);self.timing=(C.c_double*5)()
        self.buffer=np.empty(OUTPUT_FLOATS,dtype=np.float32)
        self.outputs=[];offset=0
        for shape in SHAPES:
            size=int(np.prod(shape));self.outputs.append(self.buffer[offset:offset+size].reshape(shape));offset+=size
        self.context=self.library.centerpoint_create(str(model).encode(),self.error,len(self.error))
        if not self.context:raise RuntimeError(self.error.value.decode())

    def infer(self,points):
        points=np.ascontiguousarray(points,dtype=np.float32)
        if not self.context or points.ndim!=2 or points.shape[1]!=5 or not 0<len(points)<=300000:
            raise ValueError('Invalid point tensor or closed model')
        if self.library.centerpoint_infer(self.context,points.ctypes.data,len(points),self.buffer.ctypes.data,
                self.buffer.size,self.timing,self.error,len(self.error)):
            raise RuntimeError(self.error.value.decode())
        return self.outputs,dict(preprocess_ms=self.timing[0],vendor_infer_ms=self.timing[1],
            dequantize_ms=self.timing[2],pillars=int(self.timing[3]),kept_points=int(self.timing[4]))

    def close(self):
        if self.context:self.library.centerpoint_destroy(self.context);self.context=None
