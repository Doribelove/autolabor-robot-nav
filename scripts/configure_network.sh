#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/load_config.sh"

usage() {
  echo "Usage: sudo $0 --apply | $0 --status" >&2
}

if [[ "${1:-}" == "--status" ]]; then
  ip -br -4 address show dev "$NVIDIA_J6M_INTERFACE" 2>/dev/null || true
  if [[ "$NVIDIA_LIVOX_INTERFACE" != "$NVIDIA_J6M_INTERFACE" ]]; then
    ip -br -4 address show dev "$NVIDIA_LIVOX_INTERFACE" 2>/dev/null || true
  fi
  selected="$(dual_host_select_ssh 2>/dev/null || true)"
  if [[ -n "$selected" ]]; then
    ssh "$selected" "ip -br -4 address; hrut_ipfull g '$J6M_INTERFACE'"
  else
    echo "J6M is not reachable at either configured address." >&2
  fi
  exit 0
fi

if [[ "${1:-}" != "--apply" ]]; then
  usage
  exit 2
fi
if [[ "$(id -u)" != 0 ]]; then
  echo "Network cutover must run as root on NVIDIA." >&2
  usage
  exit 2
fi

# Local address changes need root, while SSH should keep using the desktop
# user's existing key and known_hosts when this script was entered via sudo.
SSH_COMMAND=(ssh)
if [[ -n "${SUDO_USER:-}" && "$SUDO_USER" != root ]]; then
  SSH_COMMAND=(sudo -H -u "$SUDO_USER" ssh)
fi

python3 - "$NVIDIA_J6M_IP" "$NVIDIA_LIVOX_IP" "$NVIDIA_LIVOX_PROBE_IP" "$MID360_IP" <<'PY'
import ipaddress
import sys

robot = ipaddress.ip_interface(sys.argv[1] + "/24").network
sensor_host = ipaddress.ip_interface(sys.argv[2] + "/24")
sensor_probe = ipaddress.ip_interface(sys.argv[3] + "/24")
sensor_device = ipaddress.ip_address(sys.argv[4])
sensor = sensor_host.network
if robot == sensor:
    raise SystemExit("robot and MID360 networks must differ")
if sensor_probe.network != sensor:
    raise SystemExit("MID360 probe address must be on the sensor network")
if sensor_probe.ip in (sensor_host.ip, sensor_device):
    raise SystemExit("MID360 probe address must be unique")
PY

[[ -e "/sys/class/net/$NVIDIA_J6M_INTERFACE" ]] || {
  echo "Missing NVIDIA J6M interface: $NVIDIA_J6M_INTERFACE" >&2
  exit 3
}
[[ -e "/sys/class/net/$NVIDIA_LIVOX_INTERFACE" ]] || {
  echo "Missing NVIDIA MID360 interface: $NVIDIA_LIVOX_INTERFACE" >&2
  exit 3
}
[[ "$NVIDIA_J6M_INTERFACE" != "$NVIDIA_LIVOX_INTERFACE" ]] || {
  echo "J6M and MID360 require two different NVIDIA interfaces." >&2
  exit 3
}
for interface in "$NVIDIA_J6M_INTERFACE" "$NVIDIA_LIVOX_INTERFACE"; do
  if [[ ! -r "/sys/class/net/$interface/carrier" ]] ||
     [[ "$(<"/sys/class/net/$interface/carrier")" != 1 ]]; then
    echo "$interface has no Ethernet carrier; connect and power its peer first." >&2
    exit 3
  fi
done

# A USB MID360 adapter may already carry the final 192.168.1.50 address while
# J6M is still on the temporary 192.168.1.0/24 network. Pin the old J6M address
# to its physical interface so the duplicate connected routes cannot send SSH
# toward the MID360 adapter.
j6m_host_route_added=false
cleanup_j6m_host_route() {
  if [[ "$j6m_host_route_added" == true ]]; then
    ip route del "$J6M_CURRENT_IP/32" dev "$NVIDIA_J6M_INTERFACE" 2>/dev/null || true
    j6m_host_route_added=false
  fi
}

current_target=""
if timeout 4 "${SSH_COMMAND[@]}" -o BatchMode=yes -o ConnectTimeout=3 \
    "$J6M_SSH" true >/dev/null 2>&1; then
  current_target="$J6M_SSH"
else
  existing_j6m_route="$(ip route show exact "$J6M_CURRENT_IP/32")"
  if [[ -n "$existing_j6m_route" ]]; then
    existing_j6m_device="$(awk '{for (i=1; i<=NF; ++i) if ($i == "dev") {print $(i+1); exit}}' <<<"$existing_j6m_route")"
    [[ "$existing_j6m_device" == "$NVIDIA_J6M_INTERFACE" ]] || {
      echo "Existing J6M host route uses $existing_j6m_device, not $NVIDIA_J6M_INTERFACE." >&2
      exit 4
    }
  else
    ip route add "$J6M_CURRENT_IP/32" dev "$NVIDIA_J6M_INTERFACE" src "$NVIDIA_CURRENT_J6M_IP"
    j6m_host_route_added=true
  fi
  if timeout 4 "${SSH_COMMAND[@]}" -o BatchMode=yes -o ConnectTimeout=3 \
      "$J6M_SSH_CURRENT" true >/dev/null 2>&1; then
    current_target="$J6M_SSH_CURRENT"
  fi
fi
trap cleanup_j6m_host_route EXIT
[[ -n "$current_target" ]] || {
  echo "J6M is not reachable with SSH; refusing network cutover." >&2
  exit 4
}

# The working J6M link currently owns 192.168.1.50. Probe the MID360 with a
# temporary, unique sensor-side address so the same IPv4 address is never live
# on the MID360 and J6M interfaces at once.
probe_added=false
probe_route_added=false
cleanup_probe() {
  if [[ "$probe_route_added" == true ]]; then
    ip route del "$MID360_IP/32" dev "$NVIDIA_LIVOX_INTERFACE" 2>/dev/null || true
    probe_route_added=false
  fi
  if [[ "$probe_added" == true ]]; then
    ip address del "$NVIDIA_LIVOX_PROBE_IP/24" dev "$NVIDIA_LIVOX_INTERFACE" 2>/dev/null || true
    probe_added=false
  fi
}
cleanup_preflight() {
  cleanup_probe
  cleanup_j6m_host_route
}
trap cleanup_preflight EXIT
ip link set "$NVIDIA_LIVOX_INTERFACE" up
if ip -o -4 address show dev "$NVIDIA_LIVOX_INTERFACE" |
    awk '{print $4}' | grep -Fxq "$NVIDIA_LIVOX_IP/24"; then
  probe_source="$NVIDIA_LIVOX_IP"
else
  if ! ip -o -4 address show dev "$NVIDIA_LIVOX_INTERFACE" |
      awk '{print $4}' | grep -Fxq "$NVIDIA_LIVOX_PROBE_IP/24"; then
    ip address add "$NVIDIA_LIVOX_PROBE_IP/24" dev "$NVIDIA_LIVOX_INTERFACE"
    probe_added=true
  fi
  probe_source="$NVIDIA_LIVOX_PROBE_IP"
fi
if ip route show exact "$MID360_IP/32" | grep -q .; then
  probe_route_device="$(ip route get "$MID360_IP" from "$probe_source" | awk '{for (i=1; i<=NF; ++i) if ($i == "dev") {print $(i+1); exit}}')"
  [[ "$probe_route_device" == "$NVIDIA_LIVOX_INTERFACE" ]] || {
    echo "An existing MID360 host route uses $probe_route_device, not $NVIDIA_LIVOX_INTERFACE." >&2
    exit 4
  }
else
  ip route add "$MID360_IP/32" dev "$NVIDIA_LIVOX_INTERFACE" src "$probe_source"
  probe_route_added=true
fi
ping -I "$probe_source" -c 3 -W 1 "$MID360_IP" >/dev/null || {
  echo "MID360 $MID360_IP is not reachable; J6M address was not changed." >&2
  exit 4
}
echo "MID360 preflight passed from $probe_source via $NVIDIA_LIVOX_INTERFACE."
cleanup_probe
trap cleanup_j6m_host_route EXIT

if ! ip -o -4 address show dev "$NVIDIA_J6M_INTERFACE" |
    awk '{print $4}' | grep -Fxq "$NVIDIA_J6M_IP/24"; then
  ip address add "$NVIDIA_J6M_IP/24" dev "$NVIDIA_J6M_INTERFACE"
fi

"${SSH_COMMAND[@]}" "$current_target" "set -eu
  ip link set '$J6M_INTERFACE' up
  if ! ip -o -4 address show dev '$J6M_INTERFACE' | awk '{print \$4}' | grep -Fxq '$J6M_IP/24'; then
    ip address add '$J6M_IP/24' dev '$J6M_INTERFACE'
  fi"

ping -I "$NVIDIA_J6M_INTERFACE" -c 3 -W 1 "$J6M_IP" >/dev/null
"${SSH_COMMAND[@]}" -o BatchMode=yes -o StrictHostKeyChecking=accept-new \
  "$J6M_SSH" "ping -c 3 -W 1 '$NVIDIA_J6M_IP' >/dev/null"

# J6M's vendor boot service reads this hardware-backed setting on every boot.
"${SSH_COMMAND[@]}" "$J6M_SSH" \
  "hrut_ipfull s '$J6M_INTERFACE' '$J6M_IP' 255.255.255.0 '$NVIDIA_J6M_IP'; hrut_ipfull g '$J6M_INTERFACE'"

cleanup_j6m_host_route

bind_profile_to_hardware() {
  local connection="$1" interface="$2" mac="$3"
  if [[ -n "$mac" ]]; then
    # USB Ethernet interface names can change after a reboot.  Leave the
    # transient ethN name unset and let NetworkManager match the permanent MAC.
    nmcli connection modify "$connection" \
      connection.interface-name "" \
      connection.autoconnect yes \
      802-3-ethernet.mac-address "$mac"
  else
    nmcli connection modify "$connection" \
      connection.interface-name "$interface" \
      connection.autoconnect yes
  fi
}

connection="$NVIDIA_J6M_CONNECTION"
if ! nmcli -t -f NAME connection show | grep -Fxq "$connection"; then
  connection="$(nmcli -t -f NAME,DEVICE connection show --active |
    awk -F: -v device="$NVIDIA_J6M_INTERFACE" '$2 == device {print $1; exit}')"
fi
[[ -n "$connection" ]] || {
  echo "Cannot determine NetworkManager profile for $NVIDIA_J6M_INTERFACE." >&2
  exit 5
}

bind_profile_to_hardware "$connection" "$NVIDIA_J6M_INTERFACE" "$NVIDIA_J6M_MAC"
nmcli connection modify "$connection" \
  ipv4.method manual \
  ipv4.addresses "$NVIDIA_J6M_IP/24" \
  ipv4.never-default yes \
  ipv4.gateway "" \
  ipv6.method disabled
nmcli connection up "$connection" ifname "$NVIDIA_J6M_INTERFACE"

ping -I "$NVIDIA_J6M_INTERFACE" -c 5 -W 1 "$J6M_IP" >/dev/null

# 192.168.1.50 is now free from the old J6M link and can safely become the
# persistent MID360 host address.
if ! nmcli -t -f NAME connection show | grep -Fxq "$NVIDIA_LIVOX_CONNECTION"; then
  active_livox_connection="$(nmcli -t -f NAME,DEVICE connection show --active |
    awk -F: -v device="$NVIDIA_LIVOX_INTERFACE" '$2 == device {print $1; exit}')"
  if [[ -n "$active_livox_connection" ]]; then
    nmcli connection modify "$active_livox_connection" connection.id "$NVIDIA_LIVOX_CONNECTION"
  else
    nmcli connection add type ethernet \
      ifname "$NVIDIA_LIVOX_INTERFACE" \
      con-name "$NVIDIA_LIVOX_CONNECTION"
  fi
fi
bind_profile_to_hardware "$NVIDIA_LIVOX_CONNECTION" "$NVIDIA_LIVOX_INTERFACE" "$NVIDIA_LIVOX_MAC"
nmcli connection modify "$NVIDIA_LIVOX_CONNECTION" \
  ipv4.method manual \
  ipv4.addresses "$NVIDIA_LIVOX_IP/24" \
  ipv4.never-default yes \
  ipv4.gateway "" \
  ipv6.method disabled
nmcli connection up "$NVIDIA_LIVOX_CONNECTION" ifname "$NVIDIA_LIVOX_INTERFACE"

sensor_route="$(ip route get "$MID360_IP" | awk '{for (i=1; i<=NF; ++i) if ($i == "dev") {print $(i+1); exit}}')"
[[ "$sensor_route" == "$NVIDIA_LIVOX_INTERFACE" ]] || {
  echo "MID360 route uses $sensor_route, expected $NVIDIA_LIVOX_INTERFACE." >&2
  exit 5
}
ping -I "$NVIDIA_LIVOX_INTERFACE" -c 3 -W 1 "$MID360_IP" >/dev/null

"${SSH_COMMAND[@]}" "$J6M_SSH" "set -eu
  ip route replace default via '$NVIDIA_J6M_IP' dev '$J6M_INTERFACE'
  if ip -o -4 address show dev '$J6M_INTERFACE' | awk '{print \$4}' | grep -Fxq '$J6M_CURRENT_IP/24'; then
    ip address del '$J6M_CURRENT_IP/24' dev '$J6M_INTERFACE'
  fi"

mkdir -p "$DUAL_HOST_WS/runtime"
touch "$DUAL_HOST_WS/runtime/network_cutover.ok"
echo "Network cutover completed: NVIDIA $NVIDIA_J6M_IP <-> J6M $J6M_IP."
echo "MID360 remains on NVIDIA $NVIDIA_LIVOX_IP <-> $MID360_IP."
