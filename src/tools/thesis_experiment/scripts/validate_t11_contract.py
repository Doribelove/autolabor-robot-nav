#!/usr/bin/env python3
"""Validate the frozen T11 study, scene manifest and preregistration."""

import json
from pathlib import Path
import sys

from thesis_experiment.t11_contract import T11ContractError, validate_t11_contract


ROOT = Path("/home/robot/robot_ws_base_rl")


def main():
    try:
        result = validate_t11_contract(
            ROOT / "config/thesis_experiments/t11_formal.yaml", ROOT
        )
    except (OSError, ValueError, T11ContractError) as exc:
        print(json.dumps({"status": "invalid", "error": str(exc)}, sort_keys=True))
        return 1
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
