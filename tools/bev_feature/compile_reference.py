#!/usr/bin/env python3
"""Recompile the official SIX-camera BC with OE 3.9.1; never a trash model.

This uses APIs read from the delivered hbdk4-compiler 4.11.11 wheel. Execution
must be verified on native x86; the ARM host can only prepare the inputs.
"""
import argparse
import json
from pathlib import Path
import sys
import time

from oe391_environment import runtime_report, sha256
from prepare_official import COMPILER_HASHES, REVISION

ROOT = Path(__file__).resolve().parents[2]
ARTIFACT = 'nash_m/compile/quantized_deploy.bc'


def describe_hbm(hbm):
    def tensor(value):
        return dict(name=value.name, shape=value.type.shape,
                    dtype=str(value.type.np_dtype) if value.type.np_dtype is not None else None,
                    children=[tensor(v) for v in value.children] if value.children else [])
    return dict(march=hbm.march_name, toolkit_version=str(hbm.toolkit_version),
                graphs=[dict(name=g.name, num_cores=g.num_cores,
                             inputs=[tensor(v) for v in g.flatten_inputs],
                             outputs=[tensor(v) for v in g.flatten_outputs],
                             nodes=[dict(name=n.name, device=str(n.device)) for n in g.nodes])
                        for g in hbm.graphs])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bc', type=Path, default=ROOT/'runtime/bev_feature_20261101/official'/ARTIFACT)
    parser.add_argument('--output', type=Path, required=True,
                        help='New directory for the six-camera HBM and compiler report')
    parser.add_argument('--opt', type=int, choices=(0, 1, 2), default=0)
    parser.add_argument('--jobs', type=int, default=2)
    args = parser.parse_args()
    if not 1 <= args.jobs <= 32:
        parser.error('--jobs must be 1..32')
    if not args.bc.is_file() or sha256(args.bc) != COMPILER_HASHES[ARTIFACT]:
        raise ValueError('Expected the pinned official six-camera deploy BC')
    environment = runtime_report('compiler')
    if environment['errors']:
        print(json.dumps(environment, indent=2))
        return 2
    from hbdk4.compiler import load, compile
    args.output.mkdir(parents=True, exist_ok=False)
    report = dict(scope='official_six_camera_environment_baseline', camera_count=6,
                  target='nash-m', trained_for_trash=False, hf_revision=REVISION,
                  input_bc_sha256=COMPILER_HASHES[ARTIFACT], environment=environment,
                  opt=args.opt, jobs=args.jobs, status='started', board_validated=False)
    path = args.output/'compile_report.json'
    path.write_text(json.dumps(report, indent=2)+'\n')
    begin = time.monotonic()
    try:
        module = load(str(args.bc))
        # Compile the supplied deployment graph; do not re-quantize the reference.
        target = args.output/'official_six_camera_recompiled.hbm'
        hbm = compile(module, str(target), march='nash-m', opt=args.opt,
                      jobs=args.jobs, debug=True, progress_bar=True)
        report.update(status='compiled', elapsed_s=time.monotonic()-begin,
                      hbm_sha256=sha256(target), hbm_bytes=target.stat().st_size,
                      hbm=describe_hbm(hbm))
    except Exception as exc:
        report.update(status='failed', elapsed_s=time.monotonic()-begin, error=str(exc))
        raise
    finally:
        path.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
