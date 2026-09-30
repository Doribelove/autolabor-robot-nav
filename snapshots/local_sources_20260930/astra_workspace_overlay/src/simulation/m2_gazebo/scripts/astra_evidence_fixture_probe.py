#!/usr/bin/env python3
"""Isolated positive-contact fixture; never controls a robot."""
import json
import math
import os
from pathlib import Path
import threading
import time

import rospy
from gazebo_msgs.msg import ContactsState, WorldState
from std_msgs.msg import String
from std_srvs.srv import Empty


def main():
    rospy.init_node('astra_evidence_fixture_probe')
    root = Path(rospy.get_param('~output'))
    lock = threading.Lock()
    records = {'models': [], 'links': [], 'contacts': [], 'metadata': None}

    def state(message, name):
        with lock:
            records[name].append({'seq': message.header.seq, 'stamp': message.header.stamp.to_sec(),
                'frame': message.header.frame_id, 'names': list(message.name),
                'poses': [[p.position.x, p.position.y, p.position.z,
                           p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w] for p in message.pose],
                'twists': [[t.linear.x,t.linear.y,t.linear.z] for t in message.twist],
                'wrench_count': len(message.wrench)})

    def contacts(message):
        with lock:
            records['contacts'].append({'seq': message.header.seq, 'stamp': message.header.stamp.to_sec(),
                'frame': message.header.frame_id,
                'pairs': [{'first':s.collision1_name,'second':s.collision2_name,'depths':list(s.depths),
                           'info':s.info, 'normal_z':[n.z for n in s.contact_normals],
                           'normals':[[n.x,n.y,n.z] for n in s.contact_normals],
                           'positions':[[p.x,p.y,p.z] for p in s.contact_positions]} for s in message.states]})

    def metadata(message):
        with lock: records['metadata'] = json.loads(message.data)

    subscribers = [rospy.Subscriber('/astra/world_models', WorldState, state, 'models', queue_size=1000),
        rospy.Subscriber('/astra/world_links', WorldState, state, 'links', queue_size=1000),
        rospy.Subscriber('/astra/physics_contacts', ContactsState, contacts, queue_size=10000),
        rospy.Subscriber('/astra/physics_metadata', String, metadata, queue_size=1)]
    rospy.wait_for_service('/gazebo/unpause_physics', timeout=30)
    deadline = time.monotonic()+30
    while time.monotonic()<deadline:
        if all(s.get_num_connections()>0 for s in subscribers) and records['metadata'] is not None: break
        time.sleep(.02)
    else: raise RuntimeError('evidence topics failed to connect while paused')
    rospy.ServiceProxy('/gazebo/unpause_physics', Empty)()
    deadline = time.monotonic()+30
    while time.monotonic()<deadline and not rospy.is_shutdown():
        if records['models'] and records['models'][-1]['stamp']>=4.0: break
        time.sleep(.02)
    else: raise RuntimeError('fixture clock did not complete four seconds')
    rospy.ServiceProxy('/gazebo/pause_physics', Empty)()
    time.sleep(.2)
    with lock:
        raw = json.loads(json.dumps(records))
    # Write all observations before indexing or assessing: malformed/missing
    # evidence must remain recoverable even when a gate cannot be calculated.
    (root/'probe_raw.json').write_text(json.dumps(raw,indent=2)+'\n')
    contacts_rows = raw['contacts']; model_rows = raw['models']; link_rows = raw['links']
    # ROS publishers each assign Header.seq; only source timestamps pair these
    # topics. Store integer nanoseconds to avoid a tolerance-based fuzzy join.
    stamp_key = lambda r: int(round(r['stamp']*1e9))
    same = {stamp_key(r):r for r in link_rows}
    pairs = [p for r in contacts_rows for p in r['pairs']]
    contact_offsets = [r['stamp']-float(p['info'].split(';',1)[0].split('=',1)[1])
                       for r in contacts_rows for p in r['pairs']]
    cube_z = [r['poses'][r['names'].index('falling_cube')][2] for r in model_rows]
    dt = [b['stamp']-a['stamp'] for a,b in zip(contacts_rows,contacts_rows[1:])]
    checks = {
      'expected_ode_step': raw['metadata']['physics_type']=='ode' and abs(raw['metadata']['max_step_s']-.002)<1e-12,
      'all_collisions_collection_enabled': raw['metadata']['all_collision_pairs_retained'],
      'source_stamps_monotonic': bool(dt) and min(dt)>0,
      'physics_contact_packets_500hz_no_gaps': len(dt)>1800 and max(abs(d-.002) for d in dt)<1e-9,
      'state_packets_50hz': len(model_rows)>=195 and all(abs(b['stamp']-a['stamp']-.02)<1e-9 for a,b in zip(model_rows,model_rows[1:])),
      'paired_models_links_have_identical_source_stamp': len(same)==len(link_rows)==len(model_rows) and len({stamp_key(r) for r in model_rows})==len(model_rows) and all(stamp_key(r) in same for r in model_rows),
      'model_link_cube_positions_identical': all(max(abs(a-b) for a,b in zip(r['poses'][r['names'].index('falling_cube')],same[stamp_key(r)]['poses'][same[stamp_key(r)]['names'].index('falling_cube::body')]))<1e-10 for r in model_rows if stamp_key(r) in same),
      'positive_contact_detected': len(pairs)>100 and any('falling_cube' in p['first']+p['second'] and 'ground_plane' in p['first']+p['second'] for p in pairs),
      'contact_free_fall_before_touch': any(not r['pairs'] for r in contacts_rows if r['stamp']<.2),
      'cube_really_falls_and_settles': cube_z[0]>.9 and abs(cube_z[-1]-.2)<.005,
      'contact_normals_are_vertical': bool(pairs) and all(abs(abs(v)-1.)<1e-5 for p in pairs for v in p['normal_z']),
      'contact_source_time_within_one_physics_step': bool(contact_offsets) and all(-1e-9<=d<=.002+1e-9 for d in contact_offsets),
      'frame_is_world': all(r['frame']=='world' for name in ('models','links','contacts') for r in raw[name]),
      'unavailable_wrenches_not_silently_filled': all(r['wrench_count']==0 for r in model_rows+link_rows),
      'all_float_data_finite': all(math.isfinite(x) for z in model_rows+link_rows for p in z['poses'] for x in p),
      'body_array_lengths_and_twists_finite': all(len(r['names'])==len(r['poses'])==len(r['twists'])>0 and all(math.isfinite(x) for t in r['twists'] for x in t) for r in model_rows+link_rows),
      'contact_arrays_nonempty_equal_and_finite': bool(pairs) and all(len(p['positions'])==len(p['normals'])==len(p['depths'])>0 and all(math.isfinite(x) for vec in p['positions']+p['normals'] for x in vec) and all(math.isfinite(x) for x in p['depths']) for p in pairs) and all(math.isfinite(x) for x in contact_offsets),
    }
    report={'checks':checks,'pass':all(checks.values()),'model_packets':len(model_rows),
       'link_packets':len(link_rows),'contact_packets':len(contacts_rows),'contact_pairs':len(pairs),
       'first_cube_z_m':cube_z[0],'last_cube_z_m':cube_z[-1],
       'contact_packet_minus_source_time_min_s':min(contact_offsets) if contact_offsets else None,
       'contact_packet_minus_source_time_max_s':max(contact_offsets) if contact_offsets else None,
       'purpose':'positive and negative measurement fixture; no navigation performance evidence',
       'limits':['Does not validate collision response of pose-set M2 driver.',
                 'ROS header sequence is per publisher; source correspondence is verified by simulation stamps.',
                 'Contact source times are recorded separately; a bounded one-step offset is allowed and reported, never assumed zero.',
                 'No robot or planner is launched.']}
    temp=root/'probe_report.json.tmp';temp.write_text(json.dumps(report,indent=2)+'\n');os.replace(str(temp),str(root/'probe_report.json'))
    return 0 if report['pass'] else 3


if __name__=='__main__':
    raise SystemExit(main())
