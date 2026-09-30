"""Pure route/world and gate-time adapters; no ROS side effects.

These component functions are not a claim that the existing live node/plugin
already applies this bridge.  Callers must explicitly establish the integration.
"""
from dataclasses import dataclass
import math


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


@dataclass(frozen=True)
class RouteFrame:
    translation_x_m: float = 0.0
    translation_y_m: float = 0.0
    yaw_rad: float = 0.0

    def __post_init__(self):
        if not all(math.isfinite(v) for v in (self.translation_x_m,self.translation_y_m,self.yaw_rad)):
            raise ValueError('non-finite route transform')

    def point_to_world(self, point):
        x,y=point;c,s=math.cos(self.yaw_rad),math.sin(self.yaw_rad)
        return (self.translation_x_m+c*x-s*y,self.translation_y_m+s*x+c*y)

    def point_to_route(self, point):
        x,y=point[0]-self.translation_x_m,point[1]-self.translation_y_m
        c,s=math.cos(self.yaw_rad),math.sin(self.yaw_rad)
        return (c*x+s*y,-s*x+c*y)

    def pose_to_world(self, pose):
        return (*self.point_to_world(pose[:2]),wrap_angle(pose[2]+self.yaw_rad))

    def pose_to_route(self, pose):
        return (*self.point_to_route(pose[:2]),wrap_angle(pose[2]-self.yaw_rad))

    def box_to_world(self, box):
        output=dict(box);output['x'],output['y']=self.point_to_world((box['x'],box['y']))
        output['yaw_rad']=wrap_angle(box.get('yaw_rad',0.0)+self.yaw_rad)
        return output

    def actor_path_to_world(self, actor):
        output=dict(actor)
        for key in ('start','end','conflict'):output[key]=list(self.point_to_world(actor[key]))
        return output

    @staticmethod
    def body_twist_to_world_command(linear_x_mps,yaw_rate_radps):
        # geometry_msgs/Twist body command is invariant under a world SE(2)
        # change; rotating these scalars would incorrectly change execution.
        if not all(math.isfinite(v) for v in (linear_x_mps,yaw_rate_radps)):
            raise ValueError('non-finite command')
        return linear_x_mps,yaw_rate_radps


class GateRelativeActivation:
    """Component semantics for a future actor plugin, distinct from start_delay."""
    def __init__(self,delay_s,gate_route_x_m=-4.02):
        if not math.isfinite(delay_s) or delay_s<0:raise ValueError('invalid gate delay')
        self.delay_s=delay_s;self.gate_route_x_m=gate_route_x_m
        self.gate_time_s=None;self.last_time_s=None

    def update(self,sim_time_s,ego_route_x_m):
        if not math.isfinite(sim_time_s) or not math.isfinite(ego_route_x_m):
            raise ValueError('non-finite gate input')
        if self.last_time_s is not None and sim_time_s<self.last_time_s:
            raise RuntimeError('clock reset requires an explicit new episode')
        self.last_time_s=sim_time_s
        if self.gate_time_s is None and ego_route_x_m>=self.gate_route_x_m:
            self.gate_time_s=sim_time_s
        return self.gate_time_s is not None and sim_time_s-self.gate_time_s>=self.delay_s-1e-12


LIVE_INTEGRATION_STATUS = {
    'route_world_transform': 'component_only_not_wired_to_legacy_episode_node',
    'gate_relative_delay': 'component_only_not_wired_to_legacy_actor_plugin',
    'yaw_zero_legacy_bridge': 'identity_transform_supported_by_existing_scene_generator',
}
