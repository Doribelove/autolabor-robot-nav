"""Run publisher integration cases in the same interpreter as the live node.

System/user OpenCV and the detector venv have different ABIs on this host.
Do not modify either installation to make the test suite use a different one.
"""
import os
from pathlib import Path
import subprocess
import unittest
import xml.etree.ElementTree as ET


class RuntimeInterpreterTest(unittest.TestCase):
    def test_real_publisher_in_detector_interpreter(self):
        package = Path(__file__).resolve().parents[1]
        launch = ET.parse(str(package/'launch/fcos_rgbd.launch')).getroot()
        executable = launch.find("arg[@name='detector_python']").get('default')
        result = subprocess.run([executable,str(package/'test/litter_runtime_cases.py'),'-v'],
            env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1'),
            stdout=subprocess.PIPE,stderr=subprocess.STDOUT,universal_newlines=True,timeout=60)
        self.assertEqual(result.returncode,0,result.stdout)


if __name__=='__main__':unittest.main()
