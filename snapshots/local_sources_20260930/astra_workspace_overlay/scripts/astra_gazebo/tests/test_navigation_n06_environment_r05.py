import contextlib
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

P=Path(__file__).resolve().parents[1];sys.path.insert(0,str(P))
import navigation_n06_environment_r05 as env
from test_navigation_n06_manager_r05 import environment_fixture

class EnvironmentTests(unittest.TestCase):
    def data(self):return {'runtime':{'execution_environment':environment_fixture()},'release':{'runtime_closure':{'sha256':'1'*64}}}
    def test_exact_keys_values_and_sensitive_absence(self):
        d=self.data();expected=env.expected_environment(d['runtime']['execution_environment'],'1'*64)
        self.assertEqual(env.validate_current(d,expected),expected)
        for key in sorted(env.FORBIDDEN|{'UNDECLARED_RENDER_SETTING'}):
            changed=dict(expected);changed[key]='synthetic_not_loaded'
            with self.subTest(key=key),self.assertRaises(ValueError):env.validate_current(d,changed)
        for key in expected:
            changed=dict(expected);changed[key]+='altered'
            with self.subTest(changed=key),self.assertRaises(ValueError):env.validate_current(d,changed)
    def test_absent_empty_distinction_is_enforced(self):
        d=self.data();expected=env.expected_environment(d['runtime']['execution_environment'],'1'*64);expected['LD_PRELOAD']=''
        with self.assertRaises(ValueError):env.validate_current(d,expected)
    def test_release_spec_cannot_admit_forbidden_key_or_remote_model_database(self):
        for key,value in [('LD_PRELOAD','synthetic'),('GAZEBO_MODEL_DATABASE_URI','http://models.gazebosim.org'),('ROS_IP','192.0.2.1')]:
            spec=environment_fixture();spec['values'][key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):env.validate_spec(spec)
    def test_child_environment_is_exact_with_only_run_fields(self):
        d=self.data();base=env.expected_environment(d['runtime']['execution_environment'],'1'*64);root=env.WORKSPACE.parent/'runs/synthetic_unallocated_n06'
        with patch.dict(os.environ,base,clear=True):
            self.assertEqual(env.child_environment(d),base);child=env.child_environment(d,root,'a'*48)
        self.assertEqual({k for k in child if child.get(k)!=base.get(k)},{'ROS_HOME','ROS_LOG_DIR','ASTRA_N06_CHILD_NONCE'})
        self.assertEqual(child['GAZEBO_MODEL_DATABASE_URI'],env.MODEL_DATABASE)
    def test_parent_mutation_before_next_child_fails(self):
        d=self.data();base=env.expected_environment(d['runtime']['execution_environment'],'1'*64)
        with patch.dict(os.environ,base,clear=True):
            env.child_environment(d);os.environ['LD_AUDIT']='not_loaded_fixture'
            with self.assertRaises(ValueError):env.child_environment(d)
    def test_assessment_context_replaces_and_restores_after_fault(self):
        d=self.data();base=env.expected_environment(d['runtime']['execution_environment'],'1'*64);root=env.WORKSPACE.parent/'runs/synthetic_unallocated_n06'
        with patch.dict(os.environ,base,clear=True):
            with self.assertRaisesRegex(RuntimeError,'fixture'):
                with env.assessment_environment(d,root,'a'*48):
                    self.assertEqual(os.environ['ASTRA_N06_CHILD_NONCE'],'a'*48);raise RuntimeError('fixture')
            self.assertEqual(dict(os.environ),base)
    def test_noncanonical_run_nonce_or_environment_rejected(self):
        d=self.data();base=env.expected_environment(d['runtime']['execution_environment'],'1'*64)
        with patch.dict(os.environ,base,clear=True):
            for root,nonce in [(Path('/tmp/other'),'a'*48),(env.WORKSPACE.parent/'runs/test','bad')]:
                with self.assertRaises(ValueError):env.child_environment(d,root,nonce)
    def test_actual_manager_preflight_rejects_ambient_injection_before_any_allocation(self):
        import run_navigation_n06_manager_r05 as manager
        d=self.data();base=env.expected_environment(d['runtime']['execution_environment'],'1'*64);base['LD_AUDIT']='not_loaded_dictionary_fixture'
        with patch.dict(os.environ,base,clear=True),patch.object(manager.q,'publish') as publish,patch.object(subprocess,'Popen') as spawn:
            with self.assertRaisesRegex(ValueError,'environment mismatch'):manager.live_preflight(d)
            publish.assert_not_called();spawn.assert_not_called()
    def test_actual_supervisor_spawn_uses_fixed_environment(self):
        from test_navigation_n06_manager_r05 import JournalTests,q,manager,process
        d=self.data();base=env.expected_environment(d['runtime']['execution_environment'],'1'*64)
        with JournalTests().journal() as root,patch.dict(os.environ,base,clear=True):
            manager.EVENTS.rmdir();d.update(release_binding={'sha256':'1'*64},rows=q.prepared_api.schedule(),prepared={'units':[]})
            with patch.object(q,'verify_release',return_value=d),patch.object(manager,'live_preflight'),patch.object(process,'enable_subreaper'),patch.object(subprocess,'Popen',side_effect=OSError('synthetic spawn boundary')) as spawn:
                self.assertEqual(manager.supervise(root/'synthetic.json','1'*64),3)
            self.assertEqual(spawn.call_args.kwargs['env'],base)

class BootstrapTests(unittest.TestCase):
    def fixture(self,root):
        def bound(path):return {'path':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
        runtime=root/'runtime.json';runtime.write_text(json.dumps({'execution_environment':environment_fixture(),'python_executable':str((env.WORKSPACE/'.venv/bin/python').resolve()),'python_prefix':str(env.WORKSPACE/'.venv'),'python_no_user_site':True}))
        items=[bound(P/name) for name in ('navigation_n06_bootstrap_r05.py','navigation_n06_environment_r05.py','navigation_n06_entry_r05.sh','run_navigation_n06_manager_r05.py')]+[bound(runtime)]
        release=root/'synthetic.json';release.write_text(json.dumps({'schema':'astra_navigation_n06_exact_release_v1','stage':'astra_navigation_n06_lookahead_static_pilot','runtime_closure':bound(runtime),'resources':items}))
        return release,bound(release)['sha256']
    def test_actual_clean_entry_inspects_without_importing_manager_or_ros(self):
        with tempfile.TemporaryDirectory() as temp:
            path,sha=self.fixture(Path(temp));cmd=['/usr/bin/env','-i',*[k+'='+v for k,v in env.BOOTSTRAP_ENV.items()],'/bin/bash','--noprofile','--norc',str(P/'navigation_n06_entry_r05.sh'),'--verify','--release',str(path),'--release-sha256',sha,'--inspect-bootstrap']
            r=subprocess.run(cmd,capture_output=True,text=True,timeout=10);self.assertEqual(r.returncode,0,r.stderr);out=json.loads(r.stdout)
            self.assertFalse(out['manager_imported']);self.assertFalse(out['execution_performed']);self.assertEqual(out['environment']['GAZEBO_MODEL_DATABASE_URI'],env.MODEL_DATABASE)
            self.assertEqual(out['argv'][0],str(env.WORKSPACE/'.venv/bin/python'));self.assertNotIn('LD_PRELOAD',out['environment'])
    def test_direct_bootstrap_rejects_extra_parent_environment_before_manager(self):
        with tempfile.TemporaryDirectory() as temp:
            path,sha=self.fixture(Path(temp));parent=dict(env.BOOTSTRAP_ENV,UNDECLARED='synthetic')
            r=subprocess.run(['/usr/bin/python3','-I','-S',str(P/'navigation_n06_bootstrap_r05.py'),'--verify','--release',str(path),'--release-sha256',sha,'--inspect-bootstrap'],env=parent,capture_output=True,text=True,timeout=10)
            self.assertNotEqual(r.returncode,0);self.assertIn('entry required',r.stderr)
    def test_wrong_release_sha_and_unbound_bootstrap_source_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            path,sha=self.fixture(Path(temp));base=['/usr/bin/python3','-I','-S',str(P/'navigation_n06_bootstrap_r05.py'),'--verify','--release',str(path),'--inspect-bootstrap']
            r=subprocess.run(base+['--release-sha256','0'*64],env=env.BOOTSTRAP_ENV,capture_output=True,text=True);self.assertNotEqual(r.returncode,0)
            value=json.loads(path.read_text());value['resources']=value['resources'][1:];path.write_text(json.dumps(value));sha=hashlib.sha256(path.read_bytes()).hexdigest()
            r=subprocess.run(base+['--release-sha256',sha],env=env.BOOTSTRAP_ENV,capture_output=True,text=True);self.assertNotEqual(r.returncode,0);self.assertIn('source omitted',r.stderr)

if __name__=='__main__':unittest.main()
