#!/usr/bin/env python3
"""Plot saved ROS map/path evidence; this is not a Qt/RViz screenshot."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

root = Path(__file__).resolve().parents[4] / 'validation/first_entry_simulation_20260907'
run = Path((root / 'latest_run.txt').read_text().strip())
fig, axes = plt.subplots(2, 2, figsize=(13, 8), constrained_layout=True)
cases = [('baseline_path_is_erased', 'Before fix: generated, then erased', True),
         ('fixed_open_2', 'After fix: retained after batch failure', False),
         ('fixed_wall_detour', 'After fix: real Navfn detour around wall', False),
         ('new_batch_replaces_preview', 'New batch: new entrance replaces old preview', False)]
for ax,(name,title,baseline) in zip(axes.flat,cases):
    evidence = json.loads((run / (name + '_path.json')).read_text())
    grid = np.array(evidence['map']['data']).reshape(200,320)
    ax.imshow(grid,origin='lower',extent=(0,32,0,20),cmap='Greys',vmin=0,vmax=100)
    points = np.array([[p['pose']['position']['x'],p['pose']['position']['y']]
                       for p in evidence['path']['poses']])
    region = evidence['request']['regions'][0]['region']['polygon']['points']
    region = np.array([[p['x'],p['y']] for p in region] + [[region[0]['x'],region[0]['y']]])
    ax.plot(region[:,0],region[:,1],color='#be8700',lw=1.5,label='Requested cleaning region')
    swaths = evidence.get('swaths',{}).get('points',[])
    for a,b in zip(swaths[::2],swaths[1::2]):
        ax.plot([a['x'],b['x']],[a['y'],b['y']],color='#15aaa2',lw=1)
    ax.plot(points[:,0],points[:,1],color='#d17a19' if baseline else '#2878ff',
            linestyle='--' if baseline else '-',lw=2.4,
            label='Generated path (erased)' if baseline else 'Retained entry preview')
    ax.plot(4,4,'o',color='#163a59',ms=8,label='Synthetic current pose')
    ax.plot(*points[-1],'*',color='#d94040',ms=13,label='First swath entrance')
    length = np.linalg.norm(np.diff(points,axis=0),axis=1).sum()
    count = 0 if baseline else len(points)
    ax.text(.03,.95,f'Latched points after failure: {count}\nGenerated length: {length:.2f} m',
            transform=ax.transAxes,va='top',fontsize=10,
            bbox=dict(facecolor='white',alpha=.9,edgecolor='#dddddd'))
    ax.set(title=title,xlabel='map x (m)',ylabel='map y (m)',xlim=(0,32),ylim=(0,20))
    ax.set_aspect('equal')
    ax.legend(loc='lower right',fontsize=7,framealpha=.9)
fig.suptitle('First-entry preview regression — isolated ROS simulation, motion gate disabled',fontsize=14)
fig.savefig(run/'first_entry_comparison.png',dpi=160)
fig.savefig(run/'first_entry_comparison.svg')
print(run/'first_entry_comparison.png')
