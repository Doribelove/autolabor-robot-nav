#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/load_config.sh"
MARKER="$DUAL_HOST_WS/runtime/motion_authorized.ok"

case "${1:-}" in
  --revoke)
    if [[ -f "$MARKER" ]]; then
      rm -f "$MARKER"
    fi
    echo "Motion authorization revoked. Keep MOTION_ENABLED=false."
    ;;
  --confirm-elevated-estop)
    [[ "$CAN_PORT_CONFIRMED" == true ]] || {
      echo "Confirm the CAN mapping in dual_host.env first." >&2
      exit 2
    }
    mkdir -p "$DUAL_HOST_WS/runtime"
    touch "$MARKER"
    echo "Temporary motion marker created. You must also set MOTION_ENABLED=true."
    echo "Revoke it immediately after the elevated low-speed test."
    ;;
  *)
    echo "Usage: $0 --confirm-elevated-estop | --revoke" >&2
    exit 2
    ;;
esac
