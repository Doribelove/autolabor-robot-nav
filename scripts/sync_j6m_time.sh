#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/load_config.sh"

target="$(dual_host_select_ssh)" || {
  echo "J6M SSH is unavailable at both configured addresses." >&2
  exit 2
}

# Keep one authenticated remote shell open so SSH connection setup latency is
# not baked into the timestamp.  Whole-second synchronization can leave J6M
# almost one second behind NVIDIA, which is enough for cross-host sensor stamps
# to be rejected as future data even though both streams are live.
time_ssh_in=""
time_ssh_pid=""
cleanup_time_sync() {
  if [[ -n "$time_ssh_in" ]]; then
    printf 'exit\n' >&"$time_ssh_in" 2>/dev/null || true
  fi
  if [[ -n "$time_ssh_pid" ]]; then
    wait "$time_ssh_pid" 2>/dev/null || true
  fi
}
trap cleanup_time_sync EXIT

coproc J6M_TIME_SSH {
  ssh -o BatchMode=yes -o ConnectTimeout=3 -T \
    "$target" 'bash --noprofile --norc'
}
time_ssh_out="${J6M_TIME_SSH[0]}"
time_ssh_in="${J6M_TIME_SSH[1]}"
time_ssh_pid="$J6M_TIME_SSH_PID"
printf 'echo __J6M_TIME_READY__\n' >&"$time_ssh_in"
IFS= read -r ready <&"$time_ssh_out"
if [[ "$ready" != __J6M_TIME_READY__ ]]; then
  echo "Clock synchronization failed: persistent SSH shell did not become ready." >&2
  exit 3
fi

max_skew_ns=100000000
absolute_skew_ns=$((max_skew_ns + 1))
skew_ns=0
round_trip_ns=0
for attempt in 1 2 3; do
  host_epoch="$(date +%s.%N)"
  printf 'date -s "@%s" >/dev/null\n' "$host_epoch" >&"$time_ssh_in"

  local_before_ns="$(date +%s%N)"
  printf 'date +%%s%%N\n' >&"$time_ssh_in"
  IFS= read -r remote_epoch_ns <&"$time_ssh_out"
  local_after_ns="$(date +%s%N)"
  if [[ ! "$local_before_ns" =~ ^[0-9]+$ ||
        ! "$remote_epoch_ns" =~ ^[0-9]+$ ||
        ! "$local_after_ns" =~ ^[0-9]+$ ]]; then
    echo "Clock synchronization failed: invalid nanosecond clock sample." >&2
    exit 3
  fi

  round_trip_ns=$((local_after_ns - local_before_ns))
  local_midpoint_ns=$(((local_before_ns + local_after_ns) / 2))
  skew_ns=$((remote_epoch_ns - local_midpoint_ns))
  (( skew_ns < 0 )) && absolute_skew_ns=$((-skew_ns)) || absolute_skew_ns=$skew_ns
  (( absolute_skew_ns <= max_skew_ns )) && break
done

printf 'date -Is\n' >&"$time_ssh_in"
IFS= read -r remote_iso <&"$time_ssh_out"
echo "$remote_iso"
if (( absolute_skew_ns > max_skew_ns )); then
  echo "Clock synchronization failed: midpoint skew $((skew_ns / 1000000)) ms after 3 attempts." >&2
  exit 3
fi
echo "J6M clock synchronized; midpoint skew $((skew_ns / 1000000)) ms (SSH round trip $((round_trip_ns / 1000000)) ms)."

cleanup_time_sync
time_ssh_in=""
time_ssh_pid=""
trap - EXIT
