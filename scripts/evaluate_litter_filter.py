#!/usr/bin/env python3
"""Read an acceptance rosbag/JSONL and annotations; never connect to live ROS."""
import argparse
import json
from pathlib import Path
from autolabor_fod_vision.litter_evaluation import evaluate


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--audit',type=Path,required=True,help='filter_audit JSONL, or a ROS1 .bag')
    parser.add_argument('--labels',type=Path,required=True,help='exhaustive source-frame trash annotations')
    parser.add_argument('--output',type=Path,required=True,help='new JSON report (refuses overwrite)')
    args=parser.parse_args()
    if args.audit.suffix=='.bag':
        import rosbag
        with rosbag.Bag(str(args.audit),'r') as bag:
            audits=[json.loads(msg.data) for _,msg,_ in bag.read_messages(topics=['/perception/fcos/filter_audit'])]
    else:
        audits=[json.loads(line) for line in args.audit.read_text().splitlines() if line.strip()]
    report=evaluate(audits,json.loads(args.labels.read_text()))
    with args.output.open('x') as stream:json.dump(report,stream,ensure_ascii=False,indent=2,allow_nan=False)
    print(json.dumps({key:value for key,value in report.items() if key!='per_frame'},ensure_ascii=False,indent=2))


if __name__=='__main__':main()
