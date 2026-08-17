#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export OPERATOR_NAV_MODE=fast_lio
exec "$SCRIPT_DIR/operator_all_in_one.sh" "$@"
