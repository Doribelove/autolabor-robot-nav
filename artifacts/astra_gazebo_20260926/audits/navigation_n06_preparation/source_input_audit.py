#!/usr/bin/env python3
"""Read-only source/paired semantics audit; no runtime launch or budget write."""
import ast
import copy
import json
import os
from pathlib import Path
import sys

A=Path(__file__).resolve().parent;S=A.parent.parent;W=S/'workspace'
sys.path.insert(0,str(W/'scripts/astra_gazebo'))
import prepare_navigation_n06_static as p
from route_candidate_core import RouteAwareRuntime,RouteConfig
from common_route import load_bundle
import roslib.packages

def main():
    result=p.read(p.PREPARED/'prepared.json');pairs=[];roots=[]
    for deriv in p.read(A/'identity_clone_derivations.json'):
        source=Path(deriv['parent']).read_text();expected=source
        assert p.sha(deriv['parent'])==deriv['parent_sha256']
        for old,new in deriv['replacements']:
            assert expected.count(old)==1;expected=expected.replace(old,new)
        assert expected==Path(deriv['child']).read_text()
        assert p.sha(deriv['child'])==deriv['child_sha256']
        roots.extend((Path(deriv['parent']),Path(deriv['child'])))
    for block in result['blocks']:
        pair=[u for u in result['units'] if u['block_index']==block['block_index']]
        grid,route,_=load_bundle(Path(block['bundle']['path']).parent)
        urdf=Path(pair[0]['robot_description']['path']).read_text()
        a=RouteAwareRuntime(grid,route,urdf,RouteConfig());b=RouteAwareRuntime(grid,route,urdf,RouteConfig(lookahead_m=1.2))
        x,y=copy.deepcopy(a.identity),copy.deepcopy(b.identity)
        assert x['config'].pop('lookahead_m')==.7 and y['config'].pop('lookahead_m')==1.2
        assert x==y and a.semantics_sha256!=b.semantics_sha256
        assert a.actor is None and b.actor is None
        pairs.append({'block_index':block['block_index'],'parent_geometry_id':block['parent_geometry_id'],
            'control_semantics_sha256':a.semantics_sha256,'treatment_semantics_sha256':b.semantics_sha256,
            'identity_difference_only_lookahead':True,'shared_static_geometry_sha256':block['world']['sha256'],
            'shared_bundle_sha256':block['bundle']['sha256'],'learned_actor_loaded':False,
            'same_observation_schema':a.identity['schema'],'same_actions':list(a.identity['actions'])})
    resolved=[]
    report=p.read(A/'launch_dump_report.json')
    for row in report['units']:
        for node in row['nodes']:
            paths=roslib.packages.find_node(node['package'],node['type'])
            assert paths, node
            for path in paths:
                assert os.access(path,os.X_OK),path
            if node['package']=='m2_gazebo':
                expected=p.G/'scripts'/node['type']
                assert [str(Path(v).resolve()) for v in paths]==[str(expected.resolve())],paths
                roots.append(expected)
            resolved.append({'package':node['package'],'type':node['type'],'resolved_paths':paths})
    roots.extend([Path(p.__file__),p.G/'scripts/astra_route_lookahead_runtime_n06.py',p.G/'scripts/astra_route_coordinated_runtime.py'])
    # Static local import graph, including conditional branches; never execute
    # optional policy/checkpoint loading. External package versions/ELFs need a
    # separate exact future execution closure, not inferred from this graph.
    sources={};queue=list(roots)
    search=[W/'scripts/astra_gazebo',p.G/'scripts']
    while queue:
        path=queue.pop().resolve()
        if str(path) in sources:continue
        sources[str(path)]=p.binding(path)
        tree=ast.parse(path.read_text())
        for n in ast.walk(tree):
            names=[n.module] if isinstance(n,ast.ImportFrom) else [a.name for a in n.names] if isinstance(n,ast.Import) else []
            for name in names:
                if not name:continue
                for root in search:
                    candidate=root/(name.replace('.','/')+'.py')
                    if candidate.is_file():queue.append(candidate)
    assert not any('route_preview_selection_n05' in name or 'astra_route_preview_runtime_n05' in name for name in sources)
    p.write_new(A/'source_input_audit.json',{'status':'author_paired_semantics_and_source_inputs_pass',
        'paired_core_identities':pairs,'runtime_node_paths_resolved':resolved,
        'static_local_source_graph':sorted(sources.values(),key=lambda x:x['path']),
        'N05_preview_code_in_active_or_conditional_local_import_graph':False,
        'source_equivalence':p.treatment_equivalence(),'scope':'read-only source/constructor/launch-path audit; no live qualification',
        'ROS_started':False,'seed_registry_modified':False,'execution_authorized':False})
    print(json.dumps({'status':'author_paired_semantics_and_source_inputs_pass','pairs':len(pairs),'local_source_files':len(sources)}))

if __name__=='__main__':main()
