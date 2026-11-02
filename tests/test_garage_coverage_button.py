import importlib.util,json,tempfile,threading,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch

spec=importlib.util.spec_from_file_location('button','scripts/garage_coverage_button.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

class OneClickTest(unittest.TestCase):
    def test_one_click_submits_saved_plan_once_and_pending_click_does_not_repeat(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);plan=root/'plan.json'
            plan.write_text(json.dumps(dict(plan_id='reviewed-route',execution_parameters=dict(max_speed_mps=.8,allow_reverse_transit=True,reverse_speed_mps=.3))))
            fake=SimpleNamespace(pending=False,button=SimpleNamespace(get_sensitive=lambda:True),plan_file=plan,
                                 output=root/'status.json',refresh=Mock(),finished=Mock(),last_result='')
            called=threading.Event();release=threading.Event()
            def service(request):
                self.assertEqual(request.plan_id,'reviewed-route');self.assertAlmostEqual(request.max_speed_mps,.8)
                self.assertTrue(request.allow_reverse_transit);called.set();release.wait(3)
                return SimpleNamespace(accepted=True,message='started')
            with patch.object(module.rospy,'ServiceProxy',return_value=service) as proxy,patch.object(module.GLib,'idle_add') as idle:
                module.CoverageButton.start(fake,None);self.assertTrue(called.wait(2));self.assertTrue(fake.pending)
                module.CoverageButton.start(fake,None);proxy.assert_called_once_with('/coverage/start',module.StartCoverage)
                release.set()
                for _ in range(100):
                    if idle.called:break
                    threading.Event().wait(.01)
                self.assertTrue(idle.called)
                self.assertTrue(json.loads((root/'button_start_result.json').read_text())['accepted'])

    def test_disabled_button_never_calls_motion_service(self):
        fake=SimpleNamespace(pending=False,button=SimpleNamespace(get_sensitive=lambda:False))
        with patch.object(module.rospy,'ServiceProxy') as proxy:
            module.CoverageButton.start(fake,None);proxy.assert_not_called()

if __name__=='__main__':unittest.main()
