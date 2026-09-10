"""Use the production detector interpreter for the RGB-D node integration test."""
import os
from pathlib import Path
import subprocess
import unittest
import xml.etree.ElementTree as ET


class MapRuntimeInterpreterTest(unittest.TestCase):
    def test_current_map_publisher_in_detector_interpreter(self):
        package=Path(__file__).resolve().parents[1]
        launch=ET.parse(str(package/'launch/perception.launch')).getroot()
        configured=launch.find("arg[@name='detector_python']").get('default')
        executable=os.environ.get('FOD_YOLO_PYTHON') or configured.replace(
            '$(find autolabor_fod_vision)', str(package))
        result=subprocess.run([executable,str(package/'test/map_runtime_cases.py'),'-v'],
            env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1'),
            stdout=subprocess.PIPE,stderr=subprocess.STDOUT,universal_newlines=True,timeout=60)
        self.assertEqual(result.returncode,0,result.stdout)


if __name__=='__main__':unittest.main()
