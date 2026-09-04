#!/usr/bin/env bash

# Runtime network repair helpers.  This file is sourced by start_dual_host.sh
# after load_config.sh; sourcing it must not change network state.

DUAL_HOST_NETWORK_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if ! declare -F dual_host_refresh_local_interfaces >/dev/null 2>&1; then
  echo "network_prepare.sh must be sourced after load_config.sh" >&2
  return 2 2>/dev/null || exit 2
fi

dual_host_nmcli() {
  LC_ALL=C nmcli "$@"
}

dual_host_network_wait_seconds() {
  local value="${1:-90}"
  [[ "$value" =~ ^[1-9][0-9]*$ ]] || value=90
  printf '%s\n' "$value"
}

dual_host_network_poll_seconds() {
  local value="${DUAL_HOST_NETWORK_POLL_SEC:-1}"
  [[ "$value" =~ ^([0-9]+([.][0-9]+)?|[.][0-9]+)$ ]] || value=1
  printf '%s\n' "$value"
}

dual_host_describe_detected_interfaces() {
  local interface_dir interface mac carrier
  echo "Detected Ethernet identities:" >&2
  for interface_dir in "$DUAL_HOST_SYS_CLASS_NET_ROOT"/*; do
    [[ -d "$interface_dir" || -L "$interface_dir" ]] || continue
    interface="${interface_dir##*/}"
    [[ "$interface" != lo ]] || continue
    mac="$(dual_host_interface_permanent_mac "$interface" 2>/dev/null || true)"
    if [[ -r "$interface_dir/carrier" ]]; then
      carrier="$(<"$interface_dir/carrier")"
    else
      carrier="unknown"
    fi
    printf '  - %s MAC=%s carrier=%s\n' "$interface" "${mac:-unknown}" \
      "${carrier:-unknown}" >&2
  done
}

dual_host_wait_for_network_role() {
  local prefix="$1" label="$2"
  local interface_variable="${prefix}_INTERFACE"
  local mac_variable="${prefix}_MAC"
  local configured_mac_variable="${prefix}_CONFIGURED_MAC"
  local source_variable="${prefix}_IDENTITY_SOURCE"
  local usb_id_variable="${prefix}_USB_ID"
  local usb_serial_variable="${prefix}_USB_SERIAL"
  local wait_seconds deadline poll interface mac configured_mac source usb_id usb_serial

  wait_seconds="$(dual_host_network_wait_seconds "${DUAL_HOST_DEVICE_WAIT_SEC:-20}")"
  poll="$(dual_host_network_poll_seconds)"
  deadline=$((SECONDS + wait_seconds))
  while true; do
    dual_host_refresh_local_interfaces
    interface="${!interface_variable-}"
    mac="${!mac_variable-}"
    if [[ -n "$interface" &&
          ( -d "$DUAL_HOST_SYS_CLASS_NET_ROOT/$interface" ||
            -L "$DUAL_HOST_SYS_CLASS_NET_ROOT/$interface" ) ]] &&
       [[ "$(dual_host_interface_permanent_mac "$interface" 2>/dev/null || true)" == "${mac,,}" ]]; then
      configured_mac="${!configured_mac_variable-}"
      source="${!source_variable-unknown}"
      echo "$label adapter: $interface, permanent MAC ${mac^^}, identity $source."
      if [[ "$source" == usb-id+serial && "${configured_mac,,}" != "${mac,,}" ]]; then
        echo "$label configured MAC ${configured_mac^^} was not present; accepted the exact configured USB ID + serial identity."
      fi
      return 0
    fi
    if (( SECONDS >= deadline )); then
      usb_id="${!usb_id_variable-}"
      usb_serial="${!usb_serial_variable-}"
      configured_mac="${!configured_mac_variable-}"
      echo "$label adapter was not uniquely resolved within ${wait_seconds}s." >&2
      echo "Expected MAC=${configured_mac:-unset}, USB_ID=${usb_id:-unset}, USB_SERIAL=${usb_serial:-unset}." >&2
      dual_host_describe_detected_interfaces
      return 1
    fi
    sleep "$poll"
  done
}

dual_host_wait_for_nm_managed() {
  local interface="$1" label="$2" wait_seconds deadline poll managed last_error=""
  wait_seconds="$(dual_host_network_wait_seconds "${DUAL_HOST_DEVICE_WAIT_SEC:-20}")"
  poll="$(dual_host_network_poll_seconds)"
  deadline=$((SECONDS + wait_seconds))
  while true; do
    if dual_host_nmcli device show "$interface" >/dev/null 2>&1; then
      if last_error="$(dual_host_nmcli device set "$interface" managed yes 2>&1)"; then
        managed="$(dual_host_nmcli -e no -g GENERAL.NM-MANAGED device show "$interface" 2>/dev/null || true)"
        if [[ "$managed" == yes ]]; then
          return 0
        fi
      fi
    fi
    if (( SECONDS >= deadline )); then
      echo "$label adapter $interface could not be made NetworkManager-managed." >&2
      [[ -z "$last_error" ]] || echo "nmcli: $last_error" >&2
      return 1
    fi
    sleep "$poll"
  done
}

dual_host_wait_for_carrier() {
  local prefix="$1" label="$2"
  local interface_variable="${prefix}_INTERFACE"
  local wait_seconds deadline poll interface carrier_file
  wait_seconds="$(dual_host_network_wait_seconds "${DUAL_HOST_NETWORK_WAIT_SEC:-90}")"
  poll="$(dual_host_network_poll_seconds)"
  deadline=$((SECONDS + wait_seconds))
  while true; do
    dual_host_refresh_local_interfaces
    interface="${!interface_variable-}"
    carrier_file="$DUAL_HOST_SYS_CLASS_NET_ROOT/$interface/carrier"
    if [[ -n "$interface" && -r "$carrier_file" ]] &&
       [[ "$(<"$carrier_file")" == 1 ]]; then
      return 0
    fi
    if (( SECONDS >= deadline )); then
      echo "$label adapter ${interface:-unresolved} has no Ethernet carrier after ${wait_seconds}s; check cable and peer power." >&2
      return 1
    fi
    sleep "$poll"
  done
}

dual_host_profile_value() {
  local field="$1" connection="$2"
  dual_host_nmcli -e no -g "$field" connection show "$connection" 2>/dev/null || true
}

dual_host_interface_has_address() {
  local interface="$1" address="$2"
  ip -o -4 address show dev "$interface" 2>/dev/null |
    awk '{print $4}' | grep -Fxq "$address/24"
}

dual_host_prepare_profile() {
  local prefix="$1" label="$2" connection="$3" address="$4"
  local interface_variable="${prefix}_INTERFACE" mac_variable="${prefix}_MAC"
  local interface mac profile_interface profile_mac profile_autoconnect
  local profile_method profile_addresses profile_never_default profile_gateway
  local profile_ipv6 normalized_profile_mac active_connection profile_changed=false
  local wait_seconds deadline poll last_error=""

  dual_host_refresh_local_interfaces
  interface="${!interface_variable-}"
  mac="${!mac_variable-}"
  [[ -n "$interface" && -n "$mac" ]] || {
    echo "$label hardware identity became unresolved before profile activation." >&2
    return 1
  }
  dual_host_wait_for_nm_managed "$interface" "$label"
  dual_host_wait_for_carrier "$prefix" "$label"

  # A USB re-enumeration can rename ethN while carrier is coming up.
  dual_host_refresh_local_interfaces
  interface="${!interface_variable-}"
  mac="${!mac_variable-}"
  [[ -n "$interface" && -n "$mac" ]] || {
    echo "$label adapter disappeared while waiting for carrier." >&2
    return 1
  }
  dual_host_wait_for_nm_managed "$interface" "$label"

  if ! dual_host_nmcli connection show "$connection" >/dev/null 2>&1; then
    echo "$label NetworkManager profile is missing: $connection" >&2
    return 1
  fi
  profile_interface="$(dual_host_profile_value connection.interface-name "$connection")"
  profile_mac="$(dual_host_profile_value 802-3-ethernet.mac-address "$connection")"
  profile_autoconnect="$(dual_host_profile_value connection.autoconnect "$connection")"
  profile_method="$(dual_host_profile_value ipv4.method "$connection")"
  profile_addresses="$(dual_host_profile_value ipv4.addresses "$connection")"
  profile_never_default="$(dual_host_profile_value ipv4.never-default "$connection")"
  profile_gateway="$(dual_host_profile_value ipv4.gateway "$connection")"
  profile_ipv6="$(dual_host_profile_value ipv6.method "$connection")"
  normalized_profile_mac="${profile_mac//\\/}"
  if [[ -n "$profile_interface" || "${normalized_profile_mac,,}" != "${mac,,}" ||
        "$profile_autoconnect" != yes || "$profile_method" != manual ||
        "$profile_addresses" != "$address/24" || "$profile_never_default" != yes ||
        -n "$profile_gateway" || "$profile_ipv6" != disabled ]]; then
    echo "Repairing $label profile $connection for $interface (persistent MAC ${mac^^})..."
    dual_host_nmcli connection modify "$connection" \
      connection.interface-name "" \
      connection.autoconnect yes \
      802-3-ethernet.mac-address "$mac" \
      ipv4.method manual \
      ipv4.addresses "$address/24" \
      ipv4.never-default yes \
      ipv4.gateway "" \
      ipv6.method disabled
    profile_changed=true
  fi

  active_connection="$(dual_host_nmcli -e no -g GENERAL.CONNECTION device show "$interface" 2>/dev/null || true)"
  if [[ "$profile_changed" == true || "$active_connection" != "$connection" ]] ||
     ! dual_host_interface_has_address "$interface" "$address"; then
    echo "Activating $label profile $connection on $interface..."
    if ! last_error="$(dual_host_nmcli --wait 15 connection up "$connection" ifname "$interface" 2>&1)"; then
      echo "$label profile activation failed on $interface: $last_error" >&2
      return 1
    fi
  fi

  wait_seconds="$(dual_host_network_wait_seconds "${DUAL_HOST_NETWORK_WAIT_SEC:-90}")"
  poll="$(dual_host_network_poll_seconds)"
  deadline=$((SECONDS + wait_seconds))
  while true; do
    dual_host_refresh_local_interfaces
    if [[ "${!interface_variable-}" == "$interface" ]] &&
       dual_host_interface_has_address "$interface" "$address"; then
      active_connection="$(dual_host_nmcli -e no -g GENERAL.CONNECTION device show "$interface" 2>/dev/null || true)"
      if [[ "$active_connection" == "$connection" ]]; then
        echo "$label profile ready: $interface has $address/24 via $connection."
        return 0
      fi
    fi
    if (( SECONDS >= deadline )); then
      echo "$label profile $connection did not provide $address/24 on $interface within ${wait_seconds}s." >&2
      return 1
    fi
    sleep "$poll"
  done
}

dual_host_wait_for_peer() {
  local label="$1" interface="$2" peer="$3"
  local wait_seconds deadline poll
  wait_seconds="$(dual_host_network_wait_seconds "${DUAL_HOST_NETWORK_WAIT_SEC:-90}")"
  poll="$(dual_host_network_poll_seconds)"
  deadline=$((SECONDS + wait_seconds))
  while ! ping -I "$interface" -c 1 -W 1 "$peer" >/dev/null 2>&1; do
    if (( SECONDS >= deadline )); then
      echo "$label peer $peer is unreachable through $interface after ${wait_seconds}s." >&2
      return 1
    fi
    sleep "$poll"
  done
  echo "$label peer reachable: $peer via $interface."
}

dual_host_run_network_check() {
  if declare -F dual_host_network_check_override >/dev/null 2>&1; then
    dual_host_network_check_override
  else
    "$DUAL_HOST_NETWORK_SCRIPT_DIR/network_check.sh"
  fi
}

dual_host_wait_for_full_network_check() {
  local wait_seconds deadline poll output=""
  wait_seconds="$(dual_host_network_wait_seconds "${DUAL_HOST_NETWORK_WAIT_SEC:-90}")"
  poll="$(dual_host_network_poll_seconds)"
  deadline=$((SECONDS + wait_seconds))
  while true; do
    if output="$(dual_host_run_network_check 2>&1)"; then
      printf '%s\n' "$output"
      return 0
    fi
    if (( SECONDS >= deadline )); then
      echo "Dual-host network self-check did not pass within ${wait_seconds}s:" >&2
      printf '%s\n' "$output" >&2
      return 1
    fi
    sleep "$poll"
  done
}

dual_host_prepare_network() {
  dual_host_wait_for_network_role NVIDIA_J6M J6M
  dual_host_wait_for_network_role NVIDIA_LIVOX MID360
  dual_host_refresh_local_interfaces
  if ! dual_host_network_roles_are_distinct; then
    echo "J6M and MID360 did not resolve to two distinct USB Ethernet devices." >&2
    dual_host_describe_detected_interfaces
    return 1
  fi

  dual_host_prepare_profile NVIDIA_J6M J6M "$NVIDIA_J6M_CONNECTION" "$NVIDIA_J6M_IP"
  dual_host_prepare_profile NVIDIA_LIVOX MID360 "$NVIDIA_LIVOX_CONNECTION" "$NVIDIA_LIVOX_IP"
  dual_host_refresh_local_interfaces
  if ! dual_host_network_roles_are_distinct; then
    echo "Network identities changed during profile activation; refusing to continue." >&2
    return 1
  fi
  dual_host_wait_for_peer J6M "$NVIDIA_J6M_INTERFACE" "$J6M_IP"
  dual_host_wait_for_peer MID360 "$NVIDIA_LIVOX_INTERFACE" "$MID360_IP"
  dual_host_wait_for_full_network_check
}

# Visual maintenance needs only the control link to J6M, which owns roscore.
# The MID360 identity, carrier, profile and peer are intentionally untouched.
dual_host_prepare_j6m_network() {
  dual_host_wait_for_network_role NVIDIA_J6M J6M
  dual_host_prepare_profile NVIDIA_J6M J6M "$NVIDIA_J6M_CONNECTION" "$NVIDIA_J6M_IP"
  dual_host_wait_for_peer J6M "$NVIDIA_J6M_INTERFACE" "$J6M_IP"
}
