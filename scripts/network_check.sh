#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/load_config.sh"

MARKER="$DUAL_HOST_WS/runtime/network_cutover.ok"
failed=0

check_address() {
  local interface="$1" address="$2" label="$3"
  if [[ -z "$interface" ||
        ( ! -d "$DUAL_HOST_SYS_CLASS_NET_ROOT/$interface" &&
          ! -L "$DUAL_HOST_SYS_CLASS_NET_ROOT/$interface" ) ]]; then
    echo "ERR $label: hardware interface is unresolved" >&2
    failed=1
    return
  fi
  if ip -o -4 address show dev "$interface" | awk '{print $4}' | grep -Fxq "$address/24"; then
    echo "OK  $label: $interface has $address/24"
  else
    echo "ERR $label: $interface does not have $address/24" >&2
    failed=1
  fi
}

check_mac() {
  local interface="$1" expected="$2" label="$3" actual
  if [[ -z "$interface" || -z "$expected" ]]; then
    echo "ERR $label: interface or resolved permanent MAC is absent" >&2
    failed=1
    return
  fi
  actual="$(dual_host_interface_permanent_mac "$interface" 2>/dev/null || true)"
  if [[ -z "$actual" ]]; then
    echo "ERR $label: cannot read permanent MAC for $interface" >&2
    failed=1
    return
  fi
  if [[ "${actual,,}" == "${expected,,}" ]]; then
    echo "OK  $label: $interface has permanent MAC ${actual^^}"
  else
    echo "ERR $label: $interface has permanent MAC ${actual^^}, expected ${expected^^}" >&2
    failed=1
  fi
}

check_link_state() {
  local interface="$1" connection="$2" label="$3" carrier managed active
  if [[ -z "$interface" ||
        ( ! -d "$DUAL_HOST_SYS_CLASS_NET_ROOT/$interface" &&
          ! -L "$DUAL_HOST_SYS_CLASS_NET_ROOT/$interface" ) ]]; then
    return
  fi
  if [[ -r "$DUAL_HOST_SYS_CLASS_NET_ROOT/$interface/carrier" ]]; then
    carrier="$(<"$DUAL_HOST_SYS_CLASS_NET_ROOT/$interface/carrier")"
  else
    carrier=""
  fi
  if [[ "$carrier" == 1 ]]; then
    echo "OK  $label: $interface has Ethernet carrier"
  else
    echo "ERR $label: $interface has no Ethernet carrier" >&2
    failed=1
  fi
  managed="$(LC_ALL=C nmcli -e no -g GENERAL.NM-MANAGED device show "$interface" 2>/dev/null || true)"
  active="$(LC_ALL=C nmcli -e no -g GENERAL.CONNECTION device show "$interface" 2>/dev/null || true)"
  if [[ "$managed" == yes ]]; then
    echo "OK  $label: $interface is managed by NetworkManager"
  else
    echo "ERR $label: $interface is not managed by NetworkManager" >&2
    failed=1
  fi
  if [[ "$active" == "$connection" ]]; then
    echo "OK  $label: active profile is $connection"
  else
    echo "ERR $label: active profile is ${active:---}, expected $connection" >&2
    failed=1
  fi
}

if [[ -z "$NVIDIA_J6M_INTERFACE" || -z "$NVIDIA_LIVOX_INTERFACE" ]]; then
  echo "ERR one or both dedicated Ethernet roles are unresolved" >&2
  failed=1
elif ! dual_host_network_roles_are_distinct; then
  echo "ERR J6M and MID360 resolved to the same hardware interface" >&2
  failed=1
else
  echo "OK  J6M and MID360 use distinct hardware interfaces"
fi

if ! python3 - "$NVIDIA_J6M_IP" "$NVIDIA_LIVOX_IP" <<'PY'
import ipaddress
import sys

first = ipaddress.ip_interface(sys.argv[1] + "/24").network
second = ipaddress.ip_interface(sys.argv[2] + "/24").network
raise SystemExit(0 if first != second else 1)
PY
then
  echo "ERR J6M and MID360 interfaces are configured in the same /24." >&2
  failed=1
else
  echo "OK  robot and sensor networks are distinct /24 subnets"
fi

check_address "$NVIDIA_J6M_INTERFACE" "$NVIDIA_J6M_IP" "J6M link"
check_address "$NVIDIA_LIVOX_INTERFACE" "$NVIDIA_LIVOX_IP" "MID360 link"
check_mac "$NVIDIA_J6M_INTERFACE" "${NVIDIA_J6M_MAC:-}" "J6M link"
check_mac "$NVIDIA_LIVOX_INTERFACE" "${NVIDIA_LIVOX_MAC:-}" "MID360 link"
check_link_state "$NVIDIA_J6M_INTERFACE" "$NVIDIA_J6M_CONNECTION" "J6M link"
check_link_state "$NVIDIA_LIVOX_INTERFACE" "$NVIDIA_LIVOX_CONNECTION" "MID360 link"

default_device="$(ip route show default | awk 'NR == 1 {for (i=1; i<=NF; ++i) if ($i == "dev") print $(i+1)}')"
if [[ "$default_device" == "$NVIDIA_J6M_INTERFACE" || "$default_device" == "$NVIDIA_LIVOX_INTERFACE" ]]; then
  echo "ERR a robot Ethernet interface owns the default route: $default_device" >&2
  failed=1
else
  echo "OK  default route remains on ${default_device:-an external interface}"
fi

if [[ -n "$NVIDIA_J6M_INTERFACE" ]] &&
   ping -I "$NVIDIA_J6M_INTERFACE" -c 3 -W 1 "$J6M_IP" >/dev/null 2>&1; then
  echo "OK  NVIDIA -> J6M ping"
else
  echo "ERR cannot ping J6M $J6M_IP via $NVIDIA_J6M_INTERFACE" >&2
  failed=1
fi

if [[ -n "$NVIDIA_LIVOX_INTERFACE" ]] &&
   ping -I "$NVIDIA_LIVOX_INTERFACE" -c 3 -W 1 "$MID360_IP" >/dev/null 2>&1; then
  echo "OK  NVIDIA -> MID360 ping"
else
  echo "ERR cannot ping MID360 $MID360_IP via $NVIDIA_LIVOX_INTERFACE" >&2
  failed=1
fi

if timeout 5 ssh -o BatchMode=yes -o ConnectTimeout=3 "$J6M_SSH" \
    "ping -c 3 -W 1 '$NVIDIA_J6M_IP' >/dev/null"; then
  echo "OK  J6M -> NVIDIA ping"
else
  echo "ERR J6M cannot ping NVIDIA $NVIDIA_J6M_IP" >&2
  failed=1
fi

if [[ ! -f "$MARKER" ]]; then
  echo "ERR network cutover marker is absent: $MARKER" >&2
  failed=1
fi

exit "$failed"
