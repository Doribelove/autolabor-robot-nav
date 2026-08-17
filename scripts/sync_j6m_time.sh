#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/load_config.sh"

target="$(dual_host_select_ssh)" || {
  echo "J6M SSH is unavailable at both configured addresses." >&2
  exit 2
}

host_epoch="$(date +%s)"
host_iso="$(date -Is)"
ssh "$target" "date -s '@$host_epoch' >/dev/null; date -Is"
remote_epoch="$(ssh "$target" 'date +%s')"
skew=$((remote_epoch - host_epoch))
(( skew < 0 )) && absolute_skew=$((-skew)) || absolute_skew=$skew
if (( absolute_skew > 2 )); then
  echo "Clock synchronization failed: skew=${skew}s (host $host_iso)." >&2
  exit 3
fi
echo "J6M clock synchronized; absolute skew <= 2 s."
