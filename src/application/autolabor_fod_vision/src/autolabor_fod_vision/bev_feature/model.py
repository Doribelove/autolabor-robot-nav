"""Executable float reference, NOT a claimed port of unavailable HEAL sources.
ResNet50/FPN -> depth & context -> sparse LSS feature fusion -> BEV head.
All image encoders, GridSample/Mul/Sum, BEV encoder and heads are in one graph.
BPU placement remains subject to actual Horizon compilation/profiling.
"""
import torch
from torch import nn
from torch.nn import functional as F
from torchvision.models import resnet50

class SparseLSS(nn.Module):
    def __init__(self,shape):
        super().__init__();self.p,self.h,self.w=map(int,shape)
    def forward(self,context,depth,feature_grid,depth_grid,valid):
        # Batch is one synchronized four-view set. Export ABI keeps 4 explicit.
        f=F.grid_sample(context,feature_grid,align_corners=False)
        d=F.grid_sample(depth.flatten(1,2).unsqueeze(1),depth_grid,align_corners=False)
        lifted=f*d*valid
        return lifted.reshape(4,context.shape[1],self.p,self.h,self.w).sum(dim=(0,2)).unsqueeze(0)

class TrashBEV(nn.Module):
    def __init__(self,shape,depth_bins=40,channels=64):
        super().__init__();r=resnet50(weights=None)
        self.stem=nn.Sequential(r.conv1,r.bn1,r.relu,r.maxpool)
        self.l1,self.l2,self.l3,self.l4=r.layer1,r.layer2,r.layer3,r.layer4
        self.lat3=nn.Conv2d(1024,channels,1);self.lat4=nn.Conv2d(2048,channels,1)
        self.fpn=nn.Conv2d(channels,channels,3,padding=1)
        self.context=nn.Conv2d(channels,channels,1);self.depth=nn.Conv2d(channels,depth_bins,1)
        self.lss=SparseLSS(shape)
        # Small ground-target BEV encoder; not a substitute claimed identical to HEAL ResNet50 BEV encoder.
        self.bev=nn.Sequential(nn.Conv2d(channels,channels,3,padding=1),nn.ReLU(),nn.Conv2d(channels,channels,3,padding=1),nn.ReLU())
        self.heatmap=nn.Conv2d(channels,1,1);self.offset=nn.Conv2d(channels,2,1)
        nn.init.constant_(self.heatmap.bias,-2.19)
    def forward(self,images,feature_grid,depth_grid,valid):
        x=self.l2(self.l1(self.stem(images)));a=self.l3(x);b=self.l4(a)
        x=self.fpn(self.lat3(a)+F.interpolate(self.lat4(b),size=a.shape[-2:],mode='nearest'))
        x=self.lss(self.context(x),self.depth(x).softmax(1),feature_grid,depth_grid,valid)
        x=self.bev(x)
        return self.heatmap(x),self.offset(x).sigmoid()
