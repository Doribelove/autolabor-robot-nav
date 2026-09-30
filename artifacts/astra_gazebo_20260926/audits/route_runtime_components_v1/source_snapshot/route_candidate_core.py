"""New route-aware candidate/predictor/verifier core shared by training and ROS.

No ROS, simulator truth, actor IDs, or checkpoint loading. Pure sensor state,
the shared static prior, and actual expanded robot geometry are the inputs.
Backup masks qualify this prediction model only; they are not physical safety
certificates. The physical actuator and continuous-sweep qualification is separate.
"""
from dataclasses import dataclass,asdict
import hashlib
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET
import numpy as np
from shapely.geometry import Polygon,Point,MultiPoint,box
from shapely.ops import unary_union

from geometry import RobotCollisionModel,Pose3
from route_world_adapter import wrap_angle
from axle_reference_adapter import base_pose_to_rear,rear_pose_to_base,base_twist_to_rear

OBSERVATION_SCHEMA='astra_route_observation_v1'
ACTION_NAMES=('backup_stop','route_v040','route_v080','route_v120','left_v060','right_v060','left_v120','right_v120')
SPEEDS=(0.,.4,.8,1.2,.6,.6,1.2,1.2)
OFFSETS=(0.,0.,0.,0.,1.35,-1.35,1.35,-1.35)


@dataclass(frozen=True)
class RouteConfig:
    wheelbase_m:float=.65
    track_m:float=.60
    maximum_steering_rad:float=.4964
    maximum_steering_rate_radps:float=.75
    acceleration_mps2:float=.6
    deceleration_mps2:float=.9
    margin_m:float=.33
    prediction_step_s:float=.10
    horizon_s:float=3.
    maximum_episode_duration_s:float=22.
    response_delay_s:float=.35
    stopped_dwell_s:float=1.5
    lookahead_m:float=.7
    goal_tolerance_m:float=.25
    goal_yaw_tolerance_rad:float=.25
    maximum_roll_pitch_sum_rad:float=.05
    static_association_tolerance_m:float=.15
    dynamic_speed_bound_mps:float=1.35
    control_period_ns:int=50_000_000
    decision_period_ns:int=250_000_000
    inference_deadline_ms:float=20.
    transaction_deadline_ms:float=50.

    def __post_init__(self):
        if any(not math.isfinite(v) or v<=0 for v in asdict(self).values()):raise ValueError('positive finite runtime configuration required')
        if (self.wheelbase_m,self.track_m,self.maximum_steering_rad,self.maximum_steering_rate_radps,self.margin_m)!=(.65,.60,.4964,.75,.33):
            raise ValueError('M2 geometry/reference limits and0.33m margin frozen in this version')
        if type(self.control_period_ns)is not int or type(self.decision_period_ns)is not int:raise ValueError('integer scheduling required')


def actual_center_steering(left,right,L=.65,track=.6):
    if not all(math.isfinite(v) for v in (left,right)) or max(abs(left),abs(right))>.7446:raise ValueError('actual front steering outside URDF support')
    kl=math.tan(left)/(L+track/2*math.tan(left));kr=math.tan(right)/(L-track/2*math.tan(right))
    return math.atan(L*(kl+kr)/2),abs(kl-kr)


class FullFootprintEnvelope:
    """Conservative hull of every URDF collision, all front steering angles.

    Front rotating parts are enclosed by pivot-centred disks; full wheel rotation
    leaves the circular wheel cylinder invariant. A declared roll+pitch bound
    adds a geometric tilt envelope. The hull fills gaps, conservatively.
    """
    def __init__(self,urdf_xml,config):
        self.model=RobotCollisionModel(urdf_xml)
        if self.model.root_link!='base_link':raise ValueError('explicit M2 base_link root required')
        moving={name for name,j in self.model.joints.items() if j.kind!='fixed'}
        expected={side+'_'+part+'_joint' for side in ('front_left','front_right','rear_left','rear_right') for part in ('wheel',)}|{'front_left_steer_joint','front_right_steer_joint'}
        if moving!=expected:raise ValueError('unqualified articulated geometry')
        neutral=self.model.project(Pose3(),dict.fromkeys(moving,0.))
        parts=[];front={'front_left_steer_link','front_left_wheel_link','front_right_steer_link','front_right_wheel_link'}
        for projection in neutral:
            if projection.link_name not in front:parts.append(Polygon(projection.polygon_xy))
        self.front_envelope_radii={}
        for side,sign in (('left',1),('right',-1)):
            joint=self.model.joints['front_'+side+'_steer_joint'];pivot=joint.origin.xyz[:2]
            if math.dist(pivot,(config.wheelbase_m/2,sign*config.track_m/2))>1e-9:raise ValueError('URDF axle geometry differs from declared M2')
            selected=[p for p in neutral if p.link_name in ('front_'+side+'_steer_link','front_'+side+'_wheel_link')]
            radius=max(math.dist(vertex,pivot) for p in selected for vertex in p.polygon_xy)
            self.front_envelope_radii[side]=radius
            parts.append(Point(pivot).buffer(radius/math.cos(math.pi/48),resolution=12))
        radius3=max(math.sqrt(x*x+y*y+max(abs(p.z_min_m),abs(p.z_max_m))**2) for p in neutral for x,y in p.polygon_xy)
        self.tilt_pad=2*radius3*math.sin(config.maximum_roll_pitch_sum_rad/2)
        hull=unary_union(parts).convex_hull.buffer(self.tilt_pad/math.cos(math.pi/64),resolution=16).convex_hull
        self.vertices=np.asarray(hull.exterior.coords)[:-1];self.radius=float(np.linalg.norm(self.vertices,axis=1).max())
        self.urdf_sha256=hashlib.sha256(urdf_xml.encode()).hexdigest();self.component_count=len(neutral)
        self.sha256=hashlib.sha256(self.vertices.tobytes()).hexdigest()

    def at(self,pose):
        x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw)
        return Polygon(self.vertices@np.asarray([[c,s],[-s,c]])+np.asarray([x,y]))


def occupied_union(grid):
    """Coalesce each occupied/unknown row before GEOS union; no actor inputs."""
    rectangles=[];r=grid.resolution;ox,oy=grid.origin[:2]
    for row,values in enumerate(grid.pixels!=254):
        padded=np.r_[False,values,False].astype(np.int8);transitions=np.diff(padded)
        for start,end in zip(np.flatnonzero(transitions==1),np.flatnonzero(transitions==-1)):
            rectangles.append(box(ox+start*r,oy+row*r,ox+end*r,oy+(row+1)*r))
    # Exactly remove redundant collinear cell edges; no positive-tolerance
    # simplification that could carve occupied pixels or alter clearance.
    return unary_union(rectangles).simplify(0.,preserve_topology=True)


class RouteAwareRuntime:
    def __init__(self,grid,route,urdf_xml,config=None,actor=None,checkpoint_metadata=None):
        self.grid,self.route=grid,route;self.config=config or RouteConfig()
        self.route_start=np.asarray(route.points[:-1]);self.route_delta=np.diff(np.asarray(route.points),axis=0)
        self.route_length=np.asarray(route.lengths);self.route_unit=self.route_delta/self.route_length[:,None]
        self.route_arcs=np.asarray(route.arcs[:-1])
        self.footprint=FullFootprintEnvelope(urdf_xml,self.config);self.static=occupied_union(grid)
        self.domain=box(grid.origin[0],grid.origin[1],grid.origin[0]+grid.width*grid.resolution,grid.origin[1]+grid.height*grid.resolution)
        self.previous_action=0;self.stop_count=0;self.actor=actor
        physical=[p for p in ET.fromstring(urdf_xml).findall('.//plugin') if p.get('filename')=='libastra_physical_ackermann_p02_plugin.so']
        if len(physical)!=1:raise ValueError('one declared shared physical Ackermann plugin required')
        parameters={child.tag:(child.text or '').strip() for child in physical[0] if child.tag not in ('dynamicsSeed','robotNamespace')}
        identity={'schema':OBSERVATION_SCHEMA,'config':asdict(self.config),'actions':ACTION_NAMES,'speeds':SPEEDS,'offsets':OFFSETS,
                  'footprint_sha256':self.footprint.sha256,'physical_parameters_without_seed':parameters,
                  'prediction':'rear_bicycle_with_measured_initial_yawrate_and_constant_measured_rear_vy_v1',
                  'dynamic_predictor':'fresh_scan_stationary_nominal_with_isotropic_speed_bound_v1',
                  'core_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  'input_helper_sha256':{name:hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest() for name in ('route_sensor_buffer.py','geometry.py','axle_reference_adapter.py','common_route.py','route_world_adapter.py')},
                  'history':'six unique real scans at-or-before integer200ms buckets; separate latest exact-time transformed scan',
                  'feature_shapes':{'scans':[6,72],'robot':[10],'candidates':[8,7],'analytic_prior':[8],'action_mask':[8]},
                  'learned_candidate_conditioned_prediction_implemented':False,'physical_safety_certified':False}
        self.identity=identity;self.semantics_sha256=hashlib.sha256(json.dumps(identity,sort_keys=True,separators=(',',':')).encode()).hexdigest()
        if actor is not None:
            if not checkpoint_metadata or checkpoint_metadata.get('observation_schema')!=OBSERVATION_SCHEMA or checkpoint_metadata.get('candidate_semantics_sha256')!=self.semantics_sha256 or checkpoint_metadata.get('same_generator_predictor_verifier_used_in_training')is not True:
                raise ValueError('new route-observation checkpoint provenance required; frozen R2 checkpoint cannot be relabelled')

    def project(self,xy):
        # Same earliest-segment tie/projection as CommonRoute, vectorized over
        # its immutable segments for20Hz backup qualification.
        delta=np.asarray(xy)-self.route_start
        along=np.clip(np.sum(delta*self.route_unit,axis=1),0,self.route_length)
        points=self.route_start+along[:,None]*self.route_unit
        error=np.asarray(xy)-points;distances=np.sum(error*error,axis=1);index=int(np.argmin(distances))
        unit=self.route_unit[index]
        return {'arc_m':float(self.route_arcs[index]+along[index]),'lateral_m':float(-error[index,0]*unit[1]+error[index,1]*unit[0]),
                'heading_rad':math.atan2(unit[1],unit[0]),'distance_m':math.sqrt(distances[index]),'segment':index}

    def validate_inputs(self,actual,fresh_scan,now_ns):
        if type(now_ns)is not int or now_ns<=0:raise ValueError('integer current source time required')
        for stamp in (actual['stamp_ns'],fresh_scan.stamp_ns):
            if type(stamp)is not int or not 0<=now_ns-stamp<=200_000_000:raise ValueError('stale/future sensor state')

    def state(self,actual):
        numeric=tuple(actual['pose_map'])+tuple(actual['body_twist'])+tuple(actual['roll_pitch'])
        if len(numeric)!=8 or not all(math.isfinite(v) for v in numeric):raise ValueError('nonfinite or malformed actual state')
        if sum(abs(v) for v in actual['roll_pitch'])>self.config.maximum_roll_pitch_sum_rad:raise ValueError('base tilt outside declared footprint envelope')
        if not -.05<=actual['body_twist'][0]<=2.778:raise ValueError('actual forward speed outside qualified interface support')
        steering,disagreement=actual_center_steering(actual['joints']['front_left_steer_joint'],actual['joints']['front_right_steer_joint'])
        rear=base_pose_to_rear(actual['pose_map'],self.config.wheelbase_m)
        vx,vy,w=base_twist_to_rear(actual['body_twist'],self.config.wheelbase_m)
        return {'rear':rear,'vx':vx,'vy':vy,'w':w,'steer':steering,'t':0.,'base':tuple(actual['pose_map']),'front_curvature_disagreement':disagreement}

    def step(self,state,action,dt,braking=False,hold=False):
        c=self.config;x,y,yaw=state['rear'];projection=self.project(state['base'][:2])
        remaining=max(0.,self.route.length-projection['arc_m']-c.goal_tolerance_m)
        target_speed=state['vx'] if hold else (0. if braking or action==0 else min(SPEEDS[action],math.sqrt(2*c.deceleration_mps2*remaining)))
        if hold:desired=state['steer']
        elif braking or action==0:desired=0.
        else:
            sample=self.route.sample(projection['arc_m']+c.lookahead_m)
            offset=OFFSETS[action]*math.sin(math.pi*min(1.,projection['arc_m']/self.route.length))
            target=(sample[0]-math.sin(sample[2])*offset,sample[1]+math.cos(sample[2])*offset,sample[2])
            target_rear=base_pose_to_rear(target,c.wheelbase_m)
            bearing=wrap_angle(math.atan2(target_rear[1]-y,target_rear[0]-x)-yaw)
            desired=math.atan2(2*c.wheelbase_m*math.sin(bearing),max(.05,math.dist((x,y),target_rear[:2])))
            if remaining<1.0:
                goal_error=wrap_angle(self.route.goal_yaw-state['base'][2])
                # Keep terminal cross-track correction; heading alone could
                # stop a laterally displaced candidate outside the base goal.
                desired=math.atan(c.wheelbase_m*(2*goal_error-2*projection['lateral_m']))
            desired=max(-c.maximum_steering_rad,min(c.maximum_steering_rad,desired))
        speed=state['vx']+max(-c.deceleration_mps2*dt,min(c.acceleration_mps2*dt,target_speed-state['vx']))
        steer=state['steer']+max(-c.maximum_steering_rate_radps*dt,min(c.maximum_steering_rate_radps*dt,desired-state['steer']))
        w=speed*math.tan(steer)/c.wheelbase_m;angle=wrap_angle(yaw+.5*(state['w']+w)*dt);middle=wrap_angle(yaw+.25*(state['w']+w)*dt)
        mean_v=.5*(state['vx']+speed);vy=state['vy']
        rear=(x+(mean_v*math.cos(middle)-vy*math.sin(middle))*dt,y+(mean_v*math.sin(middle)+vy*math.cos(middle))*dt,angle)
        return dict(state,rear=rear,base=rear_pose_to_base(rear,c.wheelbase_m),vx=speed,w=w,steer=steer,t=state['t']+dt)

    def rollout(self,state,action,backup=False):
        states=[dict(state)];c=self.config
        def advance(duration,**flags):
            remaining=duration
            while remaining>1e-9:
                dt=min(c.prediction_step_s,remaining);states.append(self.step(states[-1],action,dt,**flags));remaining-=dt
        if backup:
            advance(c.response_delay_s,hold=True)
            advance(c.control_period_ns/1e9)
            advance(abs(states[-1]['vx'])/c.deceleration_mps2+c.stopped_dwell_s,braking=True)
        else:advance(c.horizon_s)
        return states

    def predict(self,fresh_scan):
        cloud=np.asarray(fresh_scan.points_map,dtype=float)
        known=np.asarray([self.grid.clearance(p)<=self.config.static_association_tolerance_m for p in cloud],dtype=bool)
        dynamic=cloud[~known]
        return {'static_associated_count':int(known.sum()),'dynamic_count':len(dynamic),
                'dynamic_shape':MultiPoint(dynamic) if len(dynamic) else None,'stamp_ns':fresh_scan.stamp_ns,
                'model':'stationary nominal plus1.35m/s isotropic reachable expansion; map association is an explicit sensing assumption'}

    def verify(self,states,prediction,now_ns):
        c=self.config;values=[]
        age=max(0.,(now_ns-prediction['stamp_ns'])/1e9)
        for state in states:
            shape=self.footprint.at(state['base'])
            static=shape.distance(self.static) if self.domain.contains(shape) else -1.
            dynamic=math.inf if prediction['dynamic_shape'] is None else shape.distance(prediction['dynamic_shape'])-c.dynamic_speed_bound_mps*(state['t']+age)
            values.append(min(static,dynamic)-c.margin_m)
        # Covers the explicitly interpolated candidate pose segments, not ODE.
        for index,(a,b) in enumerate(zip(states,states[1:])):
            motion=math.dist(a['base'][:2],b['base'][:2])+self.footprint.radius*abs(wrap_angle(b['base'][2]-a['base'][2]))
            moving_obstacle=0. if prediction['dynamic_shape'] is None else c.dynamic_speed_bound_mps*(b['t']-a['t'])
            values[index]=min(values[index],values[index+1])-.5*(motion+moving_obstacle)
        return min(values)

    def build(self,actual,fresh_scan,history,elapsed_s,now_ns):
        self.validate_inputs(actual,fresh_scan,now_ns)
        if np.shape(history)!=(6,72) or not np.all(np.isfinite(history)):raise ValueError('six real finite model-history rows required')
        state=self.state(actual);prediction=self.predict(fresh_scan);previews=[];backups=[];clearances=[];mask=[];features=[];prior=[]
        current=self.project(state['base'][:2]);progress=current['arc_m']/self.route.length
        for action in range(8):
            preview=self.rollout(state,action);backup=self.rollout(state,action,True)
            full=self.verify(preview,prediction,now_ns);safe=self.verify(backup,prediction,now_ns)
            end=self.project(preview[-1]['base'][:2]);gain=end['arc_m']-current['arc_m'];switch=float(action!=self.previous_action)
            features.append([SPEEDS[action]/1.2,OFFSETS[action]/1.35,gain/3.6,np.clip(full,-3,3)/3,abs(SPEEDS[action]-state['vx'])/1.2,abs(end['lateral_m'])/1.35,switch])
            prior.append(gain+1.5*np.clip(full,-1,2)-.15*switch-.05*abs(end['lateral_m']))
            previews.append(preview);backups.append(backup);clearances.append(safe);mask.append(safe>=0.)
        observation={'scans':np.asarray(history,dtype=np.float32)/8.,'robot':np.asarray([progress,current['lateral_m']/2.35,wrap_angle(state['base'][2]-current['heading_rad'])/.7,state['vx']/1.2,SPEEDS[self.previous_action]/1.2,(self.route.length-current['arc_m'])/9.,self.previous_action/7.,min(elapsed_s/self.config.maximum_episode_duration_s,1.),min(self.stop_count,12)/12.,float(np.min(fresh_scan.model_ranges))/8.],dtype=np.float32),
                     'candidates':np.asarray(features,dtype=np.float32),'analytic_prior':np.asarray(prior,dtype=np.float32),'action_mask':np.asarray(mask,dtype=bool)}
        return {'observation':observation,'previews':previews,'backup_clearances':clearances,'prediction_audit':{k:v for k,v in prediction.items() if k!='dynamic_shape'},'mask':mask}

    def select(self,built,clock_ns=time.perf_counter_ns):
        observation=built['observation'];mask=observation['action_mask']
        if not np.any(mask):return 0,None,'no_prediction_model_backup_pass',0.
        analytic=int(np.argmax(np.where(mask,observation['analytic_prior'],-np.inf)))
        if self.actor is None:return analytic,analytic,None,0.
        begin=clock_ns()
        try:proposed=int(self.actor(observation));reason=None
        except Exception as exc:return analytic,None,'inference_error:'+type(exc).__name__,(clock_ns()-begin)/1e6
        latency=(clock_ns()-begin)/1e6
        if latency>self.config.inference_deadline_ms:return analytic,proposed,'inference_deadline_drop',latency
        if not 0<=proposed<8 or not mask[proposed]:return analytic,proposed,'masked_action_drop',latency
        return proposed,proposed,reason,latency

    def command(self,actual,fresh_scan,action,now_ns):
        """Revalidate the committed action every20Hz from current actual state."""
        self.validate_inputs(actual,fresh_scan,now_ns)
        state=self.state(actual);prediction=self.predict(fresh_scan);backup=self.rollout(state,action,True)
        clearance=self.verify(backup,prediction,now_ns)
        if clearance<0:return 0.,0.,0,'backup_revalidation_failed',clearance
        if action==0:return 0.,0.,0,'backup_stop_requested',clearance
        target=self.step(state,action,self.config.control_period_ns/1e9)
        # vx command is body-forward; yawrate maps to steer in the shared driver.
        return max(0.,target['vx']),max(0.,target['vx'])*math.tan(target['steer'])/self.config.wheelbase_m,action,None,clearance

    def goal_reached(self,actual):
        pose=actual['pose_map']
        return math.dist(pose[:2],self.route.points[-1])<=self.config.goal_tolerance_m and abs(wrap_angle(pose[2]-self.route.goal_yaw))<=self.config.goal_yaw_tolerance_rad


def prepublish_guard(command,source_now_ns,actual_stamp_ns,scan_stamp_ns,transaction_ms,config):
    """Pure transaction end gate: never publish a stale or late candidate.

    Caller takes a fresh ROS timestamp immediately before this check. This
    preserves a rejected proposal in traces while returning a zero command.
    """
    v,w,executed,reason,clearance=command
    if type(source_now_ns)is not int or source_now_ns<=0 or any(type(stamp)is not int or not 0<=source_now_ns-stamp<=200_000_000 for stamp in (actual_stamp_ns,scan_stamp_ns)):
        return 0.,0.,0,'prepublish_input_health_drop',clearance
    if not math.isfinite(transaction_ms) or transaction_ms>config.transaction_deadline_ms:
        return 0.,0.,0,'transaction_deadline_drop',clearance
    if not all(math.isfinite(value) for value in (v,w)):
        return 0.,0.,0,'nonfinite_command_drop',clearance
    return command
