#!/usr/bin/env python3
"""Count XML testcase nodes once, including nested GoogleTest suites."""
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

root=Path(sys.argv[1] if len(sys.argv)>1 else 'build/test_results')
files=sorted(root.rglob('*.xml'))
counts=dict(files=len(files),testcases=0,failures=0,errors=0,skipped=0)
for path in files:
    for case in ET.parse(path).getroot().iter('testcase'):
        counts['testcases']+=1
        for key,tag in (('failures','failure'),('errors','error'),('skipped','skipped')):
            counts[key]+=int(case.find(tag) is not None)
print(json.dumps(counts,sort_keys=True))
if not files or not counts['testcases'] or counts['failures'] or counts['errors']:
    raise SystemExit(1)
