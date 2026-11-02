#!/usr/bin/env python3
"""Plan the saved garage extent through existing coverage services; execute explicitly."""
import argparse
import io
import json
import math
from pathlib import Path
import time

import numpy as np
from PIL import Image
import yaml


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def main():
    import rospy
    from autolabor_coverage.msg import CoveragePlanningParameters, CoverageStatus
    from autolabor_coverage.srv import (PlanCoverage, PlanCoverageRequest,
        SetCoveragePlanningDefaults, SetCoveragePlanningDefaultsRequest,
        StartCoverage, StartCoverageRequest)
    from geometry_msgs.msg import Point32
    from visualization_msgs.msg import MarkerArray
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args(rospy.myargv()[1:])
    args.output.mkdir(parents=True, exist_ok=True)
    rospy.init_node('garage_coverage_mission', anonymous=True)
    if args.execute:
        saved = json.loads((args.output/'plan.json').read_text())
        request = StartCoverageRequest(plan_id=saved['plan_id'])
        for key, value in saved['execution_parameters'].items():
            setattr(request, key, value)
        print('Submitting the user-requested coverage start', flush=True)
        response = rospy.ServiceProxy('/coverage/start', StartCoverage)(request)
        write_json(args.output/'start_result.json', dict(accepted=response.accepted,
            message=response.message, time=time.time()))
        print(response, flush=True)
        if not response.accepted:
            raise RuntimeError(response.message)
        return

    source = args.dataset/'online_map_at_recording_end/map_fused_2d'
    config = yaml.safe_load((source/'map.yaml').read_text())
    raster = np.asarray(Image.open(source/config['image']))
    height, width = raster.shape
    resolution = float(config['resolution'])
    ox, oy, angle = config['origin']
    if abs(angle) > 1e-8:
        raise ValueError('This saved garage map is expected to have axis-aligned origin')
    vertices = [[ox, oy], [ox+width*resolution, oy],
                [ox+width*resolution, oy+height*resolution], [ox, oy+height*resolution]]
    status = rospy.wait_for_message('/coverage/status', CoverageStatus, timeout=5)
    defaults = CoveragePlanningParameters(operation_width_m=1.2,
        overlap_ratio=rospy.get_param('/coverage_manager/overlap_ratio'),
        allow_reverse=status.allow_reverse_transit,
        max_forward_speed_mps=status.max_forward_speed_mps,
        max_reverse_speed_mps=status.max_reverse_speed_mps,
        max_angular_speed_rps=status.max_angular_speed_rps,
        linear_accel_mps2=status.linear_accel_mps2,
        angular_accel_rps2=status.angular_accel_rps2,
        direction_change_penalty_sec=status.direction_change_penalty_sec,
        segment_handoff_penalty_sec=status.segment_handoff_penalty_sec,
        transit_replan_period_sec=status.transit_replan_period_sec)
    # All execution values above came from the backend. Only this task's
    # width may need changing. An identical defaults transaction is rejected
    # during an existing preview and should not prevent reloading that plan.
    if abs(status.operation_width_m-defaults.operation_width_m) > 1e-6:
        changed = rospy.ServiceProxy('/coverage/set_planning_defaults', SetCoveragePlanningDefaults)(
            SetCoveragePlanningDefaultsRequest(restore_factory_defaults=False, parameters=defaults))
        if not changed.success:
            raise RuntimeError(changed.message)
        defaults = changed.effective
    execution = dict(max_speed_mps=defaults.max_forward_speed_mps,
        allow_reverse_transit=defaults.allow_reverse,
        reverse_speed_mps=defaults.max_reverse_speed_mps,
        max_angular_speed_rps=defaults.max_angular_speed_rps,
        linear_accel_mps2=defaults.linear_accel_mps2,
        angular_accel_rps2=defaults.angular_accel_rps2,
        direction_change_penalty_sec=defaults.direction_change_penalty_sec,
        segment_handoff_penalty_sec=defaults.segment_handoff_penalty_sec,
        transit_replan_period_sec=defaults.transit_replan_period_sec)
    request = PlanCoverageRequest(operation_width_m=defaults.operation_width_m,
        overlap_ratio=defaults.overlap_ratio, map_digest=status.map_digest, **execution)
    request.region.header.frame_id = 'map'
    request.region.header.stamp = rospy.Time.now()
    request.region.polygon.points = [Point32(x=x, y=y, z=0) for x,y in vertices]
    write_json(args.output/'request.json', dict(dataset=str(args.dataset.resolve()),
        vertices_map=vertices, operation_width_m=defaults.operation_width_m,
        overlap_ratio=defaults.overlap_ratio, execution_parameters=execution,
        map_digest=status.map_digest, starts_motion=False))
    print('Planning saved garage extent, width=1.2 m; no motion requested', flush=True)
    began = time.monotonic()
    response = rospy.ServiceProxy('/coverage/plan', PlanCoverage)(request)
    buffer = io.BytesIO(); response.serialize(buffer)
    (args.output/'plan_response.rosmsg').write_bytes(buffer.getvalue())
    summary = {key:getattr(response,key) for key in response.__slots__ if key!='planned_path'}
    summary.update(execution_parameters=execution, operation_width_m=defaults.operation_width_m,
        overlap_ratio=defaults.overlap_ratio, planning_seconds=time.monotonic()-began)
    write_json(args.output/'plan.json', summary)
    print(json.dumps(summary, ensure_ascii=False), flush=True)
    if not response.success:
        raise RuntimeError(response.message)
    markers = rospy.wait_for_message('/coverage/markers', MarkerArray, timeout=5)
    points = next(m.points for m in markers.markers if m.ns=='coverage_swaths')
    lines = [[[points[i].x,points[i].y],[points[i+1].x,points[i+1].y]]
             for i in range(0,len(points),2)]
    write_json(args.output/'ordered_swaths.json', lines)
    with (args.output/'ordered_swaths.csv').open('w') as stream:
        stream.write('order,start_x_m,start_y_m,end_x_m,end_y_m\n')
        for i,line in enumerate(lines,1):
            stream.write(','.join(map(str,[i,*line[0],*line[1]]))+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    figure, ax = plt.subplots(figsize=(12,13))
    ax.imshow(raster, cmap='gray', vmin=0, vmax=255,
              extent=[ox,ox+width*resolution,oy,oy+height*resolution], origin='upper')
    ax.add_collection(LineCollection(lines, cmap='turbo', array=np.arange(len(lines)), linewidths=1))
    for i,line in enumerate(lines,1):
        if i==1 or i%10==0:
            ax.text(*line[0],str(i),fontsize=6,color='darkred')
    ax.plot(*lines[0][0], 'go', label='First sweep entry')
    ax.set_aspect('equal'); ax.set_xlabel('map X (m)'); ax.set_ylabel('map Y (m)')
    ax.set_title('Garage coverage: 1.2 m width, %.2f m lane spacing\n%d ordered sweep lines; connecting turns planned by live Hybrid A*' %
        (defaults.operation_width_m*(1-defaults.overlap_ratio), len(lines)))
    ax.legend(); figure.tight_layout(); figure.savefig(args.output/'coverage_route.png',dpi=160)
    plt.close(figure)


if __name__ == '__main__':
    main()
