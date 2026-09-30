"""Equivalent query work elimination; no retained support is deleted.

For any footprint F, d(F,c)-R <= d(F,p)-r if ||c-p||+r <= R.
Every term has the same future speed slope. Thus a point term strictly inside a
retained track/cell circle cannot change the minimum at any query horizon.
Keep every persistent circle; skip only such redundant current point terms.
A1e-9m interior guard separates this numeric broad phase from equality cases.
This is not learned contraction, support eviction or a physical certificate.
"""
import hashlib
import json
import numpy as np
from sensor_tracks_v2 import BatchHardOccupancy


class DominanceBatchHardOccupancy(BatchHardOccupancy):
    def __init__(self,snapshot,now_ns):
        super().__init__(snapshot,now_ns)
        self.all_centers=self.centers;self.all_radii=self.radii
        persistent=len(snapshot.tracks)+len(snapshot.sparse_supports)
        count=len(self.centers);keep=np.ones(count,dtype=bool);certificates=[]
        cell_index={key:len(snapshot.tracks)+i for i,(key,_) in enumerate(snapshot.sparse_supports)}
        ntracks=len(snapshot.tracks)
        indices=np.arange(persistent,count);points=self.centers[persistent:]
        keys=np.floor(points/self.c.sparse_cell_m).astype(int)
        witness=np.asarray([cell_index.get(tuple(key),-1) for key in keys],dtype=int)
        slacks=np.full(len(points),-np.inf)
        present=witness>=0
        if np.any(present):
            jj=witness[present];ii=indices[present]
            slacks[present]=self.radii[jj]-self.radii[ii]-np.linalg.norm(self.centers[jj]-self.centers[ii],axis=1)
        remaining=slacks<=1e-9
        if ntracks and np.any(remaining):
            ii=indices[remaining]
            margins=self.radii[None,:ntracks]-self.radii[ii,None]-np.linalg.norm(self.centers[None,:ntracks,:]-self.centers[ii,None,:],axis=2)
            best=np.argmax(margins,axis=1);slacks[remaining]=margins[np.arange(len(ii)),best];witness[remaining]=best
        omit=slacks>1e-9;keep[indices[omit]]=False
        certificates=[(int(i),int(j),float(slack)) for i,j,slack in zip(indices[omit],witness[omit],slacks[omit])]
        self.query_indices=np.flatnonzero(keep);self.centers=self.all_centers[keep];self.radii=self.all_radii[keep]
        self.centers.flags.writeable=False;self.radii.flags.writeable=False
        self.dominance_certificates=tuple(certificates)
        self.identity.update(query_schema='astra_m08_strict_circle_dominance_v1',support_count=count,
            persistent_support_count=persistent,evaluated_support_count=len(self.centers),
            algebraically_redundant_point_terms=len(certificates),occupancy_removed=False,
            dominance_certificate_sha256=hashlib.sha256(json.dumps(certificates,separators=(',',':')).encode()).hexdigest())
