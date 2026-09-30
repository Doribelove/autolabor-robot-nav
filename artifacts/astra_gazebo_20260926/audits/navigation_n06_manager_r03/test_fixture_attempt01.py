import contextlib
import copy
import csv
import io
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch,Mock

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import navigation_n06_integrity_r03 as q
import navigation_n06_process_r03 as process
import navigation_n06_admin_plan_r03 as admin
import run_navigation_n06_manager_r03 as manager

def csv_document(fields,rows=()):
    out=io.StringIO(newline='');writer=csv.DictWriter(out,fieldnames=fields,lineterminator='\r\n');writer.writeheader();writer.writerows(rows);return out.getvalue().encode()

def receipt(row,ok=True,outcome='timeout'):
    root=q.STAGE/'runs'/row['run_id'];root.mkdir(parents=True,exist_ok=True);report=None
    if ok:
        identity=dict(stage=q.NAME,run_id=row['run_id'],seed=row['seed'],method=row['method'])
        for name in ('fixture.bag','world.sdf','robot_description_runtime.xml'):(root/name).write_bytes(b'synthetic fixture bytes, never replayed as evidence')
        (root/'collector_raw.json').write_bytes(q.json_bytes(identity));(root/'collector_report.json').write_bytes(q.json_bytes(dict(identity,**{'pass':True})))
        (root/'run_closure.json').write_bytes(q.json_bytes(dict(schema='astra_p04_run_closure_v1',stage=q.NAME,run_id=row['run_id'])))
        (root/'process.json').write_bytes(q.json_bytes({'pid':987654,'pgid':987654,'start_ticks':'synthetic'}))
        (root/'sweep_evaluation').mkdir(exist_ok=True);sweep=root/'sweep_evaluation/report.json';sweep.write_bytes(q.json_bytes({'schema':'astra_p04_engineering_sweep_v1','integrity_pass':True}))
        report=dict(identity,schema='astra_navigation_assessment_v1',capture_integrity_pass=True,task={'task_phase':outcome},
            integrity_checks={key:True for key in q.ASSESSMENT_CHECKS},source_bindings={name:q.binding(root/name) for name in q.ASSESSMENT_SOURCES},sweep_report=q.binding(sweep))
        (root/'assessment.json').write_bytes(q.json_bytes(report))
    unit=dict(row,contract=q.binding(q.prepared_api.PREPARED/row['run_id']/'contract.json'))
    return manager.terminal_receipt(unit,'1'*64,report=report,error=None if ok else 'injected transport failure',exit_code=0 if ok else None,root=root,started='synthetic',elapsed=0.)

class TempStageCase(unittest.TestCase):
    def setUp(self):
        self.stage_temp=tempfile.TemporaryDirectory();self.stage_patch=patch.object(q,'STAGE',Path(self.stage_temp.name));self.stage_patch.start()
    def tearDown(self):self.stage_patch.stop();self.stage_temp.cleanup()

class IntegrityTests(TempStageCase):
    def test_typed_bool_not_integer_and_nested_types(self):
        self.assertFalse(q.same_typed(True,1));self.assertFalse(q.same_typed({'x':[1]},{'x':[True]}));self.assertFalse(q.same_typed(6,6.))
        self.assertTrue(q.same_typed({'x':[1,False]},{'x':[1,False]}))
    def test_duplicate_and_nonfinite_json_rejected(self):
        for data in ('{"a":1,"a":2}','{"a":NaN}','{"a":Infinity}'):
            with self.subTest(data=data),self.assertRaises(ValueError):q.strict_json(data)
    def test_bound_regular_file_hash_and_symlink(self):
        with tempfile.TemporaryDirectory() as temp:
            p=Path(temp)/'input';p.write_bytes(b'one');item=q.binding(p);self.assertEqual(q.read_bound(item),b'one')
            p.write_bytes(b'two')
            with self.assertRaises(ValueError):q.read_bound(item)
            link=Path(temp)/'link';link.symlink_to(p)
            with self.assertRaises(ValueError):q.read_bound({'path':str(link),'sha256':q.digest(b'two')})
    def test_binding_closed_schema_and_noncanonical_path(self):
        for item in ({'path':'relative','sha256':'0'*64},{'path':'/tmp/x','sha256':'wrong'},{'path':'/tmp/x','sha256':'0'*64,'role':'extra'}):
            with self.subTest(item=item),self.assertRaises((ValueError,FileNotFoundError)):q.read_bound(item)
    def test_registry_exact_append_no_historical_rewrite(self):
        base=csv_document(q.REGISTRY_FIELDS,[dict(namespace='old',split='old',purpose='quoted, value',seed='1',run_id='old',status='consumed',source_binding='')])
        rows=q.prepared_api.schedule();expected=q.reserved_registry_bytes(base,'1'*64,rows)
        self.assertTrue(expected.startswith(base));q.check_registry(expected,base,'1'*64,rows)
        for wrong in (expected+b'\n',expected.replace(b'reserved_exact_n06_release',b'consumed',1),expected.replace(b'263003001',b'263003099',1)):
            with self.assertRaises(ValueError):q.check_registry(wrong,base,'1'*64,rows)
    def test_registry_foreign_seed_or_run_rejected(self):
        rows=q.prepared_api.schedule()
        for seed,name in [('263003001','foreign'),('2',rows[0]['run_id'])]:
            base=csv_document(q.REGISTRY_FIELDS,[dict(namespace='old',split='old',purpose='old',seed=seed,run_id=name,status='old',source_binding='')])
            with self.assertRaises(ValueError):q.reserved_registry_bytes(base,'1'*64,rows)
    def test_successful_task_failure_does_not_stop_matrix(self):
        s=q.initial_state('1'*64,q.prepared_api.schedule());q.consume(s,0)
        q.close_unit(s,0,{'path':'/fixture/receipt','sha256':'0'*64},receipt(s['units'][0],True,'timeout'))
        q.consume(s,1);self.assertEqual(s['units'][1]['status'],'consumed')
    def test_integrity_failure_stops_and_retires_tail(self):
        s=q.initial_state('1'*64,q.prepared_api.schedule());q.consume(s,0);q.close_unit(s,0,{},receipt(s['units'][0],False))
        with self.assertRaises(ValueError):q.consume(s,1)
        q.terminal(s);self.assertEqual([r['status'] for r in s['units']],['consumed']+['retired_unrun']*7)
    def test_wrong_receipt_identity_and_repeated_consume_rejected(self):
        s=q.initial_state('1'*64,q.prepared_api.schedule());q.consume(s,0)
        with self.assertRaises(ValueError):q.consume(s,0)
        r=receipt(s['units'][0]);r['seed']=True
        with self.assertRaises(ValueError):q.close_unit(s,0,{},r)
    def test_nonboolean_integrity_rejected(self):
        s=q.initial_state('1'*64,q.prepared_api.schedule());q.consume(s,0);r=receipt(s['units'][0]);r['integrity_pass']=1
        with self.assertRaises(ValueError):q.close_unit(s,0,{},r)
    def test_release_missing_or_wrong_closed_authority_fails_before_allocation(self):
        with tempfile.TemporaryDirectory() as temp:
            p=Path(temp)/'synthetic_document.json'
            for value in ({},{'schema':'astra_navigation_n06_exact_release_v1','execution_authorized':True}):
                p.write_bytes(q.json_bytes(value))
                with self.assertRaises(ValueError):q.verify_release(p,q.digest(p.read_bytes()))
            self.assertEqual(sorted(x.name for x in Path(temp).iterdir()),['synthetic_document.json'])

class SyntheticClosureTests(unittest.TestCase):
    """Actual frozen preparation plus temp dependency stand-ins, never executed.

    Native facts and converter authentication are explicitly stubbed here;
    separate tests exercise real saved converter records and rejected fakes.
    These tests prove the structural
    verifier, not native/runtime qualification. Runtime stand-ins are labelled
    synthetic and have no executable permission.
    """
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name);self.resources={}
        def add(path):
            item=q.binding(path);self.resources[item['path']]=item;return item
        self.add=add
        self.prep=q.prepared_api.STAGE/'audits/navigation_n06_preparation/source_evidence_manifest.json'
        for group in q.strict_json(self.prep.read_bytes()).values():
            if isinstance(group,list):
                for item in group:add(item['path'])
        for p in q.required_manager_paths():add(p)
        self.documents={}
        self.base={'schema':'astra_navigation_n06_exact_release_v1','stage':q.NAME,'status':'root_reviewed_release',
            'execution_authorized':True,'training_authorized':False,'real_vehicle_authorized':False,'maximum_episodes':8,
            'retry_allowed':False,'resume_allowed':False,'prepared':add(q.prepared_api.PREPARED/'prepared.json'),
            'schedule':add(q.prepared_api.PREPARED/'schedule.json'),'preparation_manifest':add(self.prep)}
        self.documents['d04_review']={'status':'independent_offline_joint_split_dedup_and_four_static_raster_review_pass','author_report':{'sha256':q.prepared_api.D04_REPORT_SHA}}
        self.documents['predecessor_ledger']={'stage':'astra_navigation_n05_preview_selection','status':'terminal'}
        native=self.root/'synthetic_native_fact.bin';native.write_bytes(b'synthetic native stand-in, not qualified');native=add(native)
        self.documents['passive_geometry_qualification']={'qualified_for_navigation_evidence':True,'resolved_steering_kd':6.,'evidence_bindings':[native]}
        self.documents['source_review']={'schema':'astra_navigation_source_review_v1','stage':q.NAME,'status':'independently_reviewed',
            'navigation_lifecycle_authority':'collector_only_spawn_unpause_pause_fixed_gateway','robot_pose_velocity_writes':[],
            'engine_module_bindings':[native],'engine_source_bindings':[native],
            'native_engine_report':native,'native_engine_records':native,'upstream_source_manifest':native}
        runtime=[add(Path('/usr/bin/gzserver').resolve())]
        for name in ('libastra_physical_ackermann_p05_plugin.so','libastra_dense_evidence_plugin.so','libastra_physics_metadata_p03_plugin.so',
            'libgazebo_ros_api_plugin.so','libgazebo_ros_laser.so','libgazebo_ros_bumper.so','libgazebo_ros_imu_sensor.so','libgazebo_ros_camera.so','record','relay','robot_state_publisher','roslaunch'):
            path=self.root/name;path.write_bytes(b'synthetic non-executable runtime stand-in');runtime.append(add(path))
        self.documents['runtime_closure']={'schema':'astra_navigation_n06_runtime_closure_v1','stage':q.NAME,'status':'reviewed_disk_dependency_closure',
            'python_executable':str((q.prepared_api.WORKSPACE/'.venv/bin/python').resolve()),'python_prefix':str((q.prepared_api.WORKSPACE/'.venv').resolve()),'python_no_user_site':True,'resources':runtime}
        units=q.json_bound(self.base['prepared'])['units'];converted=[]
        import dense_sweep_n05 as dense
        for i,u in enumerate(units):
            sdf=self.root/('synthetic_sdf%d.bin'%i);sdf.write_bytes(b'not native conversion evidence');sdf=add(sdf)
            value={'input_urdf':u['robot_description'],'converted_sdf':sdf,'resources':[u['robot_description'],sdf]}
            path=self.root/('synthetic_conversion%d.json'%i);path.write_bytes(q.json_bytes(value));bound=add(path)
            converted.append({'run_id':u['run_id'],'input_urdf':u['robot_description'],'converted_sdf':sdf,'conversion_manifest':bound,
                'conversion_resources':dense.conversion_resources(path)})
        self.documents['conversion_bundle']={'schema':'astra_navigation_n06_conversion_bundle_v1','stage':q.NAME,'units':converted}
        base=self.root/'baseline.csv';base.write_bytes(csv_document(q.REGISTRY_FIELDS));self.base['registry_baseline']=add(base)
        self.native=patch('run_passive_sweep_q05.native_review_facts',return_value={'native_reference_pass':True,'upstream_source_review_pass':True});self.native.start()
        self.converter=patch('dense_sweep_n05.authenticate_conversion',return_value={'pass':True,'diagnostic_only':True,'runtime_converter_correspondence_qualified':False});self.converter_mock=self.converter.start()
    def tearDown(self):self.converter.stop();self.native.stop();self.temp.cleanup()
    def document(self,change=None):
        value=copy.deepcopy(self.base);docs=copy.deepcopy(self.documents)
        if change:change(value,docs)
        # Do not create a source-review self-reference or bind older revisions
        # of the temp control documents across mutation cases.
        control_paths={str(self.root/(key+'.json')) for key in docs}
        docs['source_review']['resources']=[item for path,item in self.resources.items() if path not in control_paths]
        for key,doc in docs.items():
            p=self.root/(key+'.json');p.write_bytes(q.json_bytes(doc));value[key]=self.add(p)
        value['resources']=list(self.resources.values())
        path=self.root/'synthetic_document.json';path.write_bytes(q.json_bytes(value));return path,q.digest(path.read_bytes())
    def test_complete_actual_prepared_structural_roster(self):
        path,sha=self.document();out=q.verify_release(path,sha)
        self.assertEqual(out['rows'],q.prepared_api.schedule());self.assertEqual(len(out['prepared']['units']),8)
        self.assertFalse((self.root/'queue').exists());self.assertFalse((self.root/'runs').exists())
        self.assertEqual(self.converter_mock.call_count,8)
        self.assertTrue(all(call.kwargs=={'diagnostic':True} for call in self.converter_mock.call_args_list))
    def test_real_authenticator_rejects_three_field_conversion_before_allocation(self):
        self.converter.stop();path,sha=self.document()
        with self.assertRaisesRegex(ValueError,'converter provenance failed'):q.verify_release(path,sha)
        self.assertFalse((self.root/'queue').exists());self.assertFalse((self.root/'runs').exists())
    def test_positive_roster_mutations_rejected(self):
        mutations=[
            ('bool_budget',lambda v,d:v.update(maximum_episodes=True),'authority/type'),
            ('retry',lambda v,d:v.update(retry_allowed=True),'authority/type'),
            ('extra_authority',lambda v,d:v.update(extra=True),'closed exact'),
            ('wrong_d04',lambda v,d:d['d04_review'].update(status='pending'),'D04 review'),
            ('unclosed_predecessor',lambda v,d:d['predecessor_ledger'].update(status='active'),'predecessor not terminal'),
            ('boolean_gain',lambda v,d:d['passive_geometry_qualification'].update(resolved_steering_kd=True),'passive qualification'),
            ('unreviewed_source',lambda v,d:d['source_review'].update(status='pending'),'source/lifecycle'),
            ('writer',lambda v,d:d['source_review'].update(robot_pose_velocity_writes=['new']),'source/lifecycle'),
            ('missing_runtime',lambda v,d:d['runtime_closure']['resources'].pop(),'runtime root omitted'),
            ('missing_canonical_gzserver',lambda v,d:d['runtime_closure']['resources'].pop(0),'canonical gzserver'),
            ('wrong_interpreter',lambda v,d:d['runtime_closure'].update(python_executable='/usr/bin/false'),'runtime interpreter'),
            ('wrong_prefix_same_binary',lambda v,d:d['runtime_closure'].update(python_prefix='/usr'),'runtime virtualenv'),
            ('conversion_input',lambda v,d:d['conversion_bundle']['units'][0].update(input_urdf=d['conversion_bundle']['units'][1]['converted_sdf']),'conversion sidecar'),
        ]
        for name,change,reason in mutations:
            with self.subTest(name=name):
                path,sha=self.document(change)
                with self.assertRaisesRegex(ValueError,reason):q.verify_release(path,sha)
    def test_missing_and_duplicate_resource_rejected(self):
        for mode in ('missing','duplicate'):
            path,_=self.document();value=q.strict_json(path.read_bytes())
            if mode=='missing':value['resources']=[x for x in value['resources'] if x!=value['prepared']]
            else:value['resources'].append(value['resources'][0])
            path.write_bytes(q.json_bytes(value))
            with self.subTest(mode=mode),self.assertRaises(ValueError):q.verify_release(path,q.digest(path.read_bytes()))
    def test_session_closes_permanently_after_bound_drift(self):
        path,sha=self.document();session=q.VerifiedSession(path,sha);session.recheck()
        (self.root/'relay').write_bytes(b'changed')
        with self.assertRaises(ValueError):session.recheck()
        (self.root/'relay').write_bytes(b'synthetic non-executable runtime stand-in')
        with self.assertRaises(RuntimeError):session.recheck()
    def test_real_installed_gzserver_alias_resolves_to_required_canonical_resource(self):
        alias=Path('/usr/bin/gzserver');target=alias.resolve();self.assertTrue(alias.is_symlink());self.assertNotEqual(alias.name,target.name)
        path,sha=self.document();out=q.verify_release(path,sha)
        self.assertIn(q.binding(target),q.json_bound(out['release']['runtime_closure'])['resources'])
        with self.assertRaisesRegex(ValueError,'canonical absolute'):q.regular_bytes(alias)

class ProcessTests(unittest.TestCase):
    def test_assessment_child_inherits_nonce_and_environment_restores_on_fault(self):
        previous={key:os.environ.get(key) for key in (process.NONCE_KEY,'ROS_LOG_DIR')}
        with tempfile.TemporaryDirectory() as temp:
            with self.assertRaisesRegex(RuntimeError,'synthetic'):
                with process.assessment_child_environment(Path(temp),'synthetic-assessment'):
                    child=subprocess.Popen([sys.executable,'-c','import time;time.sleep(20)'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
                    try:self.assertTrue(process.child_marker(child.pid,'synthetic-assessment',Path(temp)))
                    finally:child.terminate();child.wait(timeout=5)
                    raise RuntimeError('synthetic')
        self.assertEqual(previous,{key:os.environ.get(key) for key in previous})
    def test_actual_ancestry_nonce_and_ticks_all_required(self):
        rows={21:dict(pid=21,ppid=20,start_ticks='a',state='S'),20:dict(pid=20,ppid=10,start_ticks='b',state='S')}
        reader=rows.get
        self.assertTrue(process.eligible(rows[21],10,True,reader));self.assertFalse(process.eligible(rows[21],10,False,reader))
        self.assertFalse(process.eligible(dict(rows[21],start_ticks='old'),10,True,reader))
        self.assertFalse(process.eligible(rows[21],99,True,reader))
    def test_real_pidfd_refuses_wrong_start_tick_without_signal(self):
        child=subprocess.Popen([sys.executable,'-c','import time;time.sleep(20)'],start_new_session=True)
        try:
            self.assertFalse(process.signal_pinned(child.pid,'not_actual',signal.SIGTERM,lambda _:True));self.assertIsNone(child.poll())
        finally:child.terminate();child.wait(timeout=5)
    def test_real_owned_child_cleanup_and_supervisor_survival(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);nonce='synthetic-nonce';env=dict(os.environ,ASTRA_N06_CHILD_NONCE=nonce,ROS_LOG_DIR=str(root/'roslog'))
            child=subprocess.Popen([sys.executable,'-c','import time;time.sleep(20)'],env=env,start_new_session=True)
            try:
                with patch.object(process,'assert_ports_free'):
                    result=process.cleanup(child,root,nonce,grace=(.2,.2,.2))
                self.assertTrue(result['pass']);self.assertFalse(result['killpg_used']);self.assertIsNotNone(child.poll())
                self.assertTrue(process.checked_parent(os.getpid(),process.proc_row(os.getpid())['start_ticks']))
            finally:
                if child.poll() is None:child.kill();child.wait(timeout=5)
    def test_child_vanishes_between_eligible_failure_and_recheck_is_benign(self):
        with tempfile.TemporaryDirectory() as temp:
            row=dict(pid=987654,ppid=os.getpid(),start_ticks='t',state='S',ancestry=[],marker_match=True)
            reads=iter([dict(row),None]);scans=iter([[row],[],[],[]])
            def reader(pid):return next(reads,None) if pid==row['pid'] else None
            with patch.object(process,'scan',side_effect=lambda *_:next(scans,[])),patch.object(process,'proc_row',side_effect=reader),patch.object(process,'eligible',return_value=False),patch.object(process,'child_marker',return_value=False),patch.object(process,'assert_ports_free'),patch.object(process,'signal_pinned') as sent:
                result=process.cleanup(None,Path(temp),'synthetic',grace=(0.,0.,0.))
            self.assertTrue(result['pass']);self.assertEqual(len(result['benign_vanished_or_reaped']),1);sent.assert_not_called()
    def test_live_unverifiable_identity_fails_without_signal(self):
        with tempfile.TemporaryDirectory() as temp:
            row=dict(pid=987654,ppid=os.getpid(),start_ticks='t',state='S',ancestry=[],marker_match=False)
            with patch.object(process,'scan',return_value=[row]),patch.object(process,'proc_row',side_effect=lambda pid:dict(row) if pid==987654 else None),patch.object(process,'eligible',return_value=False),patch.object(process,'child_marker',return_value=False),patch.object(process,'assert_ports_free'),patch.object(process,'signal_pinned') as sent:
                with self.assertRaises(RuntimeError):process.cleanup(None,Path(temp),'synthetic',grace=(0.,0.,0.))
            sent.assert_not_called();self.assertFalse(q.strict_json((Path(temp)/'cleanup.json').read_bytes())['pass'])
    def test_no_process_group_signal_call(self):
        import ast
        tree=ast.parse(Path(process.__file__).read_text())
        self.assertFalse(any(isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='killpg' for n in ast.walk(tree)))

class RealRetainedConversionTests(unittest.TestCase):
    def test_saved_q05_conversion_authenticates_diagnostically_without_new_process(self):
        import dense_sweep_n05 as dense
        path=q.prepared_api.STAGE/'contracts/passive_sweep_q05_prepared_r02/passive_sweep_q05_static/conversion/manifest.json'
        value=q.strict_json(path.read_bytes())
        with patch.object(subprocess,'Popen',side_effect=AssertionError('conversion authentication must not spawn a process')):
            result=dense.authenticate_conversion(path,value['input_urdf']['path'],value['converted_sdf']['path'],{},diagnostic=True)
        self.assertTrue(result['pass'],result['issues']);self.assertTrue(result['diagnostic_only']);self.assertFalse(result['runtime_converter_correspondence_qualified'])

class ReceiptSemanticsTests(TempStageCase):
    def test_negative_assessment_is_valid_infrastructure_failure_without_error(self):
        row=q.prepared_api.schedule()[0];r=receipt(row);path=Path(r['assessment']['path']);report=q.json_bound(r['assessment'])
        report['capture_integrity_pass']=False;report['integrity_checks']['passive_source_geometry_complete']=False;path.write_bytes(q.json_bytes(report))
        r.update(integrity_pass=False,assessment=q.binding(path))
        self.assertTrue(q.validate_terminal_receipt(r,row,'1'*64))
    def test_positive_assessment_with_later_error_retains_negative_overall_integrity(self):
        row=q.prepared_api.schedule()[0];r=receipt(row);r.update(integrity_pass=False,error='post-assessment resource drift')
        self.assertTrue(q.validate_terminal_receipt(r,row,'1'*64))
    def test_false_without_error_or_negative_assessment_rejected(self):
        row=q.prepared_api.schedule()[0];r=receipt(row);r.update(integrity_pass=False)
        with self.assertRaisesRegex(ValueError,'failed integrity'):q.validate_terminal_receipt(r,row,'1'*64)

class JournalTests(unittest.TestCase):
    @contextlib.contextmanager
    def journal(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);queue=root/'queue';queue.mkdir();events=queue/'events';events.mkdir()
            with patch.multiple(manager,STAGE=root,QUEUE=queue,LEDGER=queue/'ledger.json',EVENTS=events),patch.object(q,'STAGE',root):yield root
    def test_event_survives_ledger_pointer_write_failure(self):
        with self.journal():
            state=q.initial_state('1'*64,q.prepared_api.schedule());original=q.publish
            def fail(path,*args,**kwargs):
                if Path(path)==manager.LEDGER:raise OSError('injected pointer replace failure')
                return original(path,*args,**kwargs)
            with patch.object(q,'publish',side_effect=fail):
                with self.assertRaises(OSError):manager.persist(state,'allocation')
            recovered=manager.replay_state({'release_binding':{'sha256':'1'*64},'rows':q.prepared_api.schedule()})
            self.assertEqual(recovered['event_sequence'],1);self.assertFalse(manager.LEDGER.exists())
    def test_broken_event_chain_rejected(self):
        with self.journal():
            state=q.initial_state('1'*64,q.prepared_api.schedule());manager.persist(state,'allocation');manager.persist(state,'second')
            file=manager.EVENTS/'0002.json';bad=q.strict_json(file.read_bytes());bad['previous_event']=None;file.write_bytes(q.json_bytes(bad))
            with self.assertRaises(ValueError):manager.replay_state({'release_binding':{'sha256':'1'*64},'rows':q.prepared_api.schedule()})
    def test_aborted_worker_before_ledger_retires_all_without_motion_claim(self):
        with self.journal() as root:
            data={'release_binding':{'sha256':'1'*64},'rows':q.prepared_api.schedule(),'prepared':{'units':[]}}
            result=manager.reconcile_worker_exit(data,-15)
            self.assertTrue(result['ledger_present']);s=q.strict_json(manager.LEDGER.read_bytes())
            self.assertEqual(s['status'],'terminal');self.assertEqual([r['status'] for r in s['units']],['retired_unrun']*8)
            self.assertFalse((root/'runs').exists())
    def test_failed_receipt_report_none_keeps_original_reason(self):
        with tempfile.TemporaryDirectory() as temp:
            unit=q.prepared_api.schedule()[0];unit['contract']={'sha256':'2'*64}
            result=manager.terminal_receipt(unit,'1'*64,report=None,error='injected',exit_code=-15,root=Path(temp),started='synthetic',elapsed=0.)
            self.assertFalse(result['integrity_pass']);self.assertEqual(result['error'],'injected');self.assertIsNone(result['task_outcome']);self.assertEqual(result['host_shell_exit_status'],'unknown')
    def test_default_cli_does_not_allocate(self):
        with patch.object(sys,'argv',['manager']),patch('sys.stdout',new_callable=io.StringIO) as out:
            self.assertEqual(manager.main(),0);self.assertFalse(json.loads(out.getvalue())['execution_authorized'])
    def test_system_prefix_cannot_alias_reviewed_venv_interpreter(self):
        with patch.object(sys,'prefix','/usr'),patch.dict(os.environ,{'THESIS_WS':str(manager.WORKSPACE)}):
            with self.assertRaisesRegex(ValueError,'virtualenv prefix'):manager.live_preflight({})
    def test_terminal_negative_worker_return_preserves_ledger_reason(self):
        with self.journal():
            state=q.initial_state('1'*64,q.prepared_api.schedule());q.terminal(state,'original infrastructure cause');manager.persist(state,'terminal')
            before=manager.LEDGER.read_bytes()
            manager.reconcile_worker_exit({'release_binding':{'sha256':'1'*64},'rows':q.prepared_api.schedule(),'prepared':{'units':[]}},3)
            self.assertEqual(manager.LEDGER.read_bytes(),before)
    def test_supervisor_spawn_failure_retires_without_invented_wait(self):
        with self.journal() as root:
            manager.EVENTS.rmdir();data={'release_binding':{'sha256':'1'*64},'rows':q.prepared_api.schedule(),'prepared':{'units':[]}}
            with patch.object(q,'verify_release',return_value=data),patch.object(manager,'live_preflight'),patch.object(process,'enable_subreaper'),patch.object(subprocess,'Popen',side_effect=OSError('injected spawn failure')):
                self.assertEqual(manager.supervise(root/'synthetic_document.json','1'*64),3)
            out=q.strict_json((manager.QUEUE/'navigation_n06_supervisor_wait.json').read_bytes())
            self.assertIsNone(out['worker_pid']);self.assertIsNone(out['observed_child_returncode']);self.assertEqual(out['authority'],'no child exit observed')
            self.assertEqual(out['host_shell_exit_status'],'unknown');self.assertEqual(q.strict_json(manager.LEDGER.read_bytes())['consumed_units'],0)
    def test_worker_crash_after_consume_closes_missing_receipt_and_retires_tail(self):
        with self.journal() as root:
            (root/'runs').mkdir();rows=q.prepared_api.schedule();units=[dict(r,contract=q.binding(q.prepared_api.PREPARED/r['run_id']/'contract.json')) for r in rows]
            state=q.initial_state('1'*64,rows);manager.persist(state,'allocate');q.consume(state,0);manager.persist(state,'consume')
            result=manager.reconcile_worker_exit({'release_binding':{'sha256':'1'*64},'rows':rows,'prepared':{'units':units}},-9)
            self.assertEqual(result['missing_receipts_closed'],[rows[0]['run_id']]);state=q.strict_json(manager.LEDGER.read_bytes())
            self.assertFalse(state['units'][0]['integrity_pass']);self.assertIsNone(state['units'][0]['task_outcome'])
            self.assertEqual([r['status'] for r in state['units']],['consumed']+['retired_unrun']*7)

class UnitSubprocessFaultTests(unittest.TestCase):
    """Call the actual execute_unit orchestration with fake subprocess only."""
    def run_case(self,kind):
        with tempfile.TemporaryDirectory() as temp,contextlib.ExitStack() as stack:
            root=Path(temp);(root/'runs').mkdir();inputs=root/'inputs';inputs.mkdir()
            def file(name,value):
                p=inputs/name;q.publish(p,value);return q.binding(p)
            unit=dict(q.prepared_api.schedule()[0]);unit['contract']=file('contract.json',{'scene_manifest':file('scene.yaml',b'synthetic'),
                'shared_bundle':{'synthetic':True},'wall_cap_s':30.})
            for key in ('launch','world','robot_description','gate_expectation'):unit[key]=file(key+'.bin',b'synthetic')
            side={key:file(key+'.json',{}) for key in ('conversion_manifest','converted_sdf')}
            data={'release':{'resources':[]},'review':{},'release_binding':{'path':'/synthetic/not-read','sha256':'1'*64},'registry_baseline':b'','rows':[]}
            session=Mock();stop=Mock(signal=None);parent={'pid':os.getpid(),'start_ticks':'synthetic'}
            fake=Mock(pid=987654,returncode=9 if kind=='nonzero' else 0);fake.poll.return_value=fake.returncode
            stack.enter_context(patch.object(manager,'STAGE',root));stack.enter_context(patch.object(q,'check_registry'))
            (root/'seed_registry.csv').write_bytes(b'synthetic')
            stack.enter_context(patch('run_passive_sweep_q05.archive'))
            stack.enter_context(patch.object(process,'assert_ports_free'));stack.enter_context(patch.object(process,'checked_parent',return_value=True))
            stack.enter_context(patch.object(process,'proc_row',return_value={'pid':987654,'pgid':987654,'start_ticks':'synthetic'}))
            cleanup=stack.enter_context(patch.object(process,'cleanup',side_effect=RuntimeError('injected cleanup failure') if kind=='cleanup' else None))
            spawn=stack.enter_context(patch.object(subprocess,'Popen',side_effect=OSError('injected spawn failure') if kind=='spawn' else None,return_value=fake))
            closure=stack.enter_context(patch.object(manager.evidence,'closure',side_effect=ValueError('injected closure failure') if kind=='closure' else None))
            assess=stack.enter_context(patch.object(manager.evidence,'assess',return_value={'capture_integrity_pass':True,'task':{'task_phase':'timeout'}}))
            bound,result=manager.execute_unit(unit,side,data,session,stop,parent)
            self.assertEqual(q.json_bound(bound),result);self.assertEqual(result['host_shell_exit_status'],'unknown')
            self.assertEqual(spawn.call_count,1);self.assertEqual(cleanup.call_count,2)
            self.assertEqual(cleanup.call_args.kwargs['audit_name'],'cleanup_after_assessment.json')
            self.assertTrue(spawn.call_args.kwargs['start_new_session']);self.assertNotIn('shell',spawn.call_args.kwargs)
            if kind=='timeout':
                self.assertTrue(result['integrity_pass']);self.assertEqual(result['task_outcome'],'timeout');self.assertEqual(assess.call_count,1)
            else:
                self.assertFalse(result['integrity_pass']);self.assertIsNone(result['task_outcome']);self.assertEqual(assess.call_count,0)
                self.assertIn({'spawn':'spawn failure','cleanup':'cleanup failure','closure':'closure failure','nonzero':'returncode=9'}[kind],result['error'])
            return result
    def test_spawn_failure_retained_and_no_assessment(self):self.run_case('spawn')
    def test_nonzero_launch_retained_and_no_assessment(self):self.run_case('nonzero')
    def test_cleanup_failure_retained_and_no_assessment(self):self.run_case('cleanup')
    def test_closure_failure_retained_and_no_assessment(self):self.run_case('closure')
    def test_integrity_valid_timeout_is_task_result(self):self.run_case('timeout')

class AdministrationTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.stage=Path(self.temp.name);self.rows=q.prepared_api.schedule();self.sha='1'*64
        self.stage_patch=patch.object(q,'STAGE',self.stage);self.stage_patch.start()
        self.baseline=csv_document(q.REGISTRY_FIELDS,[dict(namespace='old',split='old',purpose='quoted, retained',seed='42',run_id='historical_duplicate',status='consumed',source_binding='')]*2)
        (self.stage/'seed_registry.csv').write_bytes(q.reserved_registry_bytes(self.baseline,self.sha,self.rows))
        for name in ('run_manifest.csv','EXPERIMENTS.csv'):
            fields=admin.FIELDS[name];old={k:'' for k in fields};old['run_id']='historical_duplicate'
            (self.stage/name).write_bytes(csv_document(fields,[old,old]))
        self.before={name:(self.stage/name).read_bytes() for name in admin.FIELDS};self.state=q.initial_state(self.sha,self.rows);self.receipts={};self.bindings={}
        for index in range(2):
            q.consume(self.state,index);r=receipt(self.rows[index],index==0,'timeout');path=self.stage/('receipt%d.json'%index);q.publish(path,r)
            self.receipts[r['run_id']]=r;self.bindings[r['run_id']]=q.binding(path);q.close_unit(self.state,index,q.binding(path),r)
        q.terminal(self.state);q.publish(self.stage/'ledger.json',self.state)
    def tearDown(self):self.stage_patch.stop();self.temp.cleanup()
    def plan(self):
        binding=admin.publish_plan(self.stage,self.stage/'transaction',q.binding(self.stage/'ledger.json'),self.bindings,[])
        review={'schema':'astra_navigation_n06_admin_review_v1','stage':q.NAME,'status':'independently_reviewed','plan':binding,'terminal_evidence_review_pass':True,'administrative_apply_authorized':True}
        q.publish(self.stage/'synthetic_review.json',review);return binding,q.binding(self.stage/'synthetic_review.json')
    def test_plan_changes_no_original_table_and_preserves_history(self):
        plan,_=self.plan()
        self.assertEqual({n:(self.stage/n).read_bytes() for n in admin.FIELDS},self.before)
        for name in admin.FIELDS:
            _,old=admin.records(self.before[name],admin.FIELDS[name]);_,after=admin.records((self.stage/'transaction'/('after_'+name)).read_bytes(),admin.FIELDS[name])
            self.assertEqual([raw for r,raw in old if r['run_id']=='historical_duplicate'],[raw for r,raw in after if r['run_id']=='historical_duplicate'])
        self.assertFalse(q.json_bound(plan)['three_original_paths_jointly_atomic'])
    def test_partial_second_replace_failure_recovers_and_repeat_is_noop(self):
        plan,review=self.plan();original=q.publish;calls=[]
        def fail(path,*args,**kwargs):
            path=Path(path)
            if path.parent==self.stage and path.name in admin.FIELDS and kwargs.get('replace'):
                calls.append(path.name)
                if len(calls)==2:raise OSError('injected second table replacement failure')
            return original(path,*args,**kwargs)
        with patch.object(q,'publish',side_effect=fail):
            with self.assertRaises(OSError):admin.apply_reviewed_plan(plan,review)
        self.assertNotEqual((self.stage/'seed_registry.csv').read_bytes(),self.before['seed_registry.csv'])
        self.assertEqual((self.stage/'run_manifest.csv').read_bytes(),self.before['run_manifest.csv'])
        admin.apply_reviewed_plan(plan,review)
        stats={p:(p.read_bytes(),p.stat().st_ino,p.stat().st_mtime_ns) for p in self.stage.rglob('*') if p.is_file()}
        admin.apply_reviewed_plan(plan,review)
        self.assertEqual(stats,{p:(p.read_bytes(),p.stat().st_ino,p.stat().st_mtime_ns) for p in self.stage.rglob('*') if p.is_file()})
    def test_crash_after_replace_before_progress_rolls_forward(self):
        plan,review=self.plan();original=q.publish
        def fail(path,*args,**kwargs):
            if Path(path).name=='progress.json':raise OSError('injected journal update failure after durable table replace')
            return original(path,*args,**kwargs)
        with patch.object(q,'publish',side_effect=fail):
            with self.assertRaises(OSError):admin.apply_reviewed_plan(plan,review)
        self.assertNotEqual((self.stage/'seed_registry.csv').read_bytes(),self.before['seed_registry.csv'])
        admin.apply_reviewed_plan(plan,review)
        self.assertTrue((self.stage/'transaction/completion.json').is_file())
    def test_receipt_drift_after_first_replace_rejects_before_next_or_completion(self):
        plan,review=self.plan();original=q.publish;mutated=[False]
        def inject(path,*args,**kwargs):
            result=original(path,*args,**kwargs)
            if Path(path)==self.stage/'seed_registry.csv' and not mutated[0]:
                mutated[0]=True;(self.stage/'receipt0.json').write_bytes(b'{}')
            return result
        with patch.object(q,'publish',side_effect=inject):
            with self.assertRaisesRegex(ValueError,'bound bytes changed'):admin.apply_reviewed_plan(plan,review)
        self.assertNotEqual((self.stage/'seed_registry.csv').read_bytes(),self.before['seed_registry.csv'])
        self.assertEqual((self.stage/'run_manifest.csv').read_bytes(),self.before['run_manifest.csv'])
        self.assertFalse((self.stage/'transaction/completion.json').exists())
    def test_assessment_drift_after_last_replace_rejects_completion(self):
        plan,review=self.plan();original=q.publish;assessment=Path(self.receipts[self.rows[0]['run_id']]['assessment']['path'])
        def inject(path,*args,**kwargs):
            result=original(path,*args,**kwargs)
            if Path(path)==self.stage/'EXPERIMENTS.csv':assessment.write_bytes(b'{}')
            return result
        with patch.object(q,'publish',side_effect=inject):
            with self.assertRaisesRegex(ValueError,'bound bytes changed'):admin.apply_reviewed_plan(plan,review)
        self.assertFalse((self.stage/'transaction/completion.json').exists())
    def test_receipt_stage_error_missing_assessment_and_report_mismatch_rejected(self):
        original=self.receipts[self.rows[0]['run_id']]
        for key,value in (('stage','foreign'),('error','retained failure'),('assessment',None),('roslaunch_exit_code',-15),('task_outcome','complete')):
            rows=copy.deepcopy(self.receipts);rows[self.rows[0]['run_id']][key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):admin.projected_tables(self.before,self.state,rows,self.sha)
        report=q.json_bound(original['assessment']);report['seed']+=1
        path=self.stage/'wrong_assessment.json';q.publish(path,report);rows=copy.deepcopy(self.receipts);rows[self.rows[0]['run_id']]['assessment']=q.binding(path)
        with self.assertRaisesRegex(ValueError,'assessment identity'):admin.projected_tables(self.before,self.state,rows,self.sha)
    def test_unknown_third_table_version_rejects_before_other_write(self):
        plan,review=self.plan();(self.stage/'EXPERIMENTS.csv').write_bytes(self.before['EXPERIMENTS.csv']+b'foreign\n')
        with self.assertRaises(ValueError):admin.apply_reviewed_plan(plan,review)
        self.assertEqual((self.stage/'seed_registry.csv').read_bytes(),self.before['seed_registry.csv'])
    def test_mutated_snapshot_fails_before_csv_write(self):
        plan,review=self.plan();(self.stage/'transaction/after_seed_registry.csv').write_bytes(b'bad')
        with self.assertRaises(ValueError):admin.apply_reviewed_plan(plan,review)
        self.assertEqual((self.stage/'seed_registry.csv').read_bytes(),self.before['seed_registry.csv'])
    def test_changed_consumed_receipt_rejects_commit(self):
        plan,review=self.plan();(self.stage/'receipt1.json').write_bytes(b'{}')
        with self.assertRaises(ValueError):admin.apply_reviewed_plan(plan,review)
    def test_missing_receipt_and_reordered_budget_rejected(self):
        with self.assertRaises(ValueError):admin.projected_tables(self.before,self.state,{},self.sha)
        state=copy.deepcopy(self.state);state['units'][0],state['units'][1]=state['units'][1],state['units'][0]
        with self.assertRaises(ValueError):admin.projected_tables(self.before,state,self.receipts,self.sha)
    def test_budget_outcome_and_reservation_metadata_mismatch_rejected(self):
        for field,value in (('consumed_units',True),('maximum_units',9)):
            state=copy.deepcopy(self.state);state[field]=value
            with self.assertRaisesRegex(ValueError,'budget totals'):admin.projected_tables(self.before,state,self.receipts,self.sha)
        state=copy.deepcopy(self.state);state['units'][0]['integrity_pass']=False
        with self.assertRaisesRegex(ValueError,'outcomes disagree'):admin.projected_tables(self.before,state,self.receipts,self.sha)
        before=dict(self.before);before['seed_registry.csv']=before['seed_registry.csv'].replace(b'n06_static_lookahead_pilot',b'other_purpose',1)
        with self.assertRaisesRegex(ValueError,'reservation changed'):admin.projected_tables(before,self.state,self.receipts,self.sha)
    def test_unreviewed_plan_has_no_apply_authority(self):
        plan,review=self.plan();data=q.json_bound(review);data['administrative_apply_authorized']=False
        file=self.stage/'denied.json';q.publish(file,data)
        with self.assertRaises(ValueError):admin.apply_reviewed_plan(plan,q.binding(file))
    def test_same_named_csvs_under_wrong_root_rejected(self):
        plan,review=self.plan();value=q.json_bound(plan)
        other=self.stage/'wrong-root';other.mkdir()
        for row in value['tables']:
            name=Path(row['target']).name;(other/name).write_bytes(self.before[name]);row['target']=str(other/name)
        file=self.stage/'wrong_root_plan.json';q.publish(file,value);new_plan=q.binding(file)
        reviewed=q.json_bound(review);reviewed['plan']=new_plan;review_file=self.stage/'wrong_root_review.json';q.publish(review_file,reviewed)
        with self.assertRaisesRegex(ValueError,'canonical stage'):admin.apply_reviewed_plan(new_plan,q.binding(review_file))
        self.assertEqual({n:(other/n).read_bytes() for n in admin.FIELDS},self.before)
    def test_plan_cannot_be_republished_over_old_transaction(self):
        self.plan()
        with self.assertRaises(FileExistsError):admin.publish_plan(self.stage,self.stage/'transaction',q.binding(self.stage/'ledger.json'),self.bindings,[])

if __name__=='__main__':unittest.main()
