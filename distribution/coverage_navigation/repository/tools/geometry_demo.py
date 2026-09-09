#!/usr/bin/env python3
"""Generate real coverage geometry on a synthetic map without ROS or hardware."""
from dataclasses import asdict
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/application/autolabor_coverage/src'))
from autolabor_coverage.coverage_geometry import CoveragePlanner, GridMap, Point

grid=GridMap(160,120,.1,0.,0.,[0]*(160*120))
polygon=[Point(2,2),Point(12,2),Point(12,8),Point(2,8)]
plan=CoveragePlanner(grid).plan(polygon,operation_width=1.,overlap_ratio=.15,
    route_origin=Point(2,2),route_yaw=0.)
out=ROOT/'validation/geometry_demo'
out.mkdir(parents=True,exist_ok=True)
data=asdict(plan)
(out/'plan.json').write_text(json.dumps(data,indent=2)+'\n')
lines=['<svg xmlns="http://www.w3.org/2000/svg" width="800" height="600" viewBox="0 0 800 600">',
       '<rect width="800" height="600" fill="white"/>',
       '<text x="25" y="35" font-size="20">Synthetic coverage swaths (no transition execution)</text>',
       '<g transform="translate(0,600) scale(50,-50)">',
       '<polygon points="2,2 12,2 12,8 2,8" fill="#eef4f8" stroke="#455a64" stroke-width="0.04"/>']
for s in plan.swaths:
    lines.append(f'<line x1="{s.start.x}" y1="{s.start.y}" x2="{s.end.x}" y2="{s.end.y}" stroke="#1565c0" stroke-width="0.05"/>')
lines+=['</g></svg>']
(out/'swaths.svg').write_text('\n'.join(lines)+'\n')
print(json.dumps(dict(swaths=len(plan.swaths),spacing_m=plan.spacing,
    requested_area_m2=plan.requested_area,output=str(out)),indent=2))
