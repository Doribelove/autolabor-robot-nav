#!/usr/bin/env bash

# Bounded process lifecycle helpers for the dual-host launchers.
#
# Safety rules:
#   * a process is signalled only after a PID file, start-time token and command
#     line identify it as one of our launchers;
#   * escalation is limited to the verified launcher's /proc descendant tree;
#   * PID 1, this shell and every ancestor of this shell are never signalled;
#   * there is intentionally no pgrep/pkill/killall or process-group fallback;
#   * orphan recovery scans /proc read-only and requires an exact launch token,
#     or an exact workspace/ROS-master identity plus a caller-owned whitelist.
#
# This file is sourced by other scripts; do not enable shell options here.

DUAL_HOST_STOP_GRACE_SEC="${DUAL_HOST_STOP_GRACE_SEC:-15}"
DUAL_HOST_STOP_TERM_SEC="${DUAL_HOST_STOP_TERM_SEC:-5}"
DUAL_HOST_STOP_KILL_SEC="${DUAL_HOST_STOP_KILL_SEC:-2}"
DUAL_HOST_PROC_ROOT="${DUAL_HOST_PROC_ROOT:-/proc}"

dual_host_proc_stat_fields() {
  local pid="${1:-}" stat_text remainder
  local -a fields=()
  [[ "$pid" =~ ^[0-9]+$ ]] && (( pid > 1 )) || return 1
  [[ -r "$DUAL_HOST_PROC_ROOT/$pid/stat" ]] || return 1
  stat_text="$(<"$DUAL_HOST_PROC_ROOT/$pid/stat")"
  [[ "$stat_text" == *") "* ]] || return 1
  # /proc/PID/stat encloses comm in parentheses. Use the last ') ' so names
  # containing spaces or ')' cannot shift the remaining fields.
  remainder="${stat_text##*) }"
  read -r -a fields <<<"$remainder"
  (( ${#fields[@]} >= 20 )) || return 1
  printf '%s\n' "${fields[*]}"
}

dual_host_process_is_running() {
  local pid="${1:-}" fields state
  [[ "$pid" =~ ^[0-9]+$ ]] && (( pid > 1 )) || return 1
  if [[ "$DUAL_HOST_PROC_ROOT" == /proc ]]; then
    kill -0 "$pid" 2>/dev/null || return 1
  fi
  fields="$(dual_host_proc_stat_fields "$pid" 2>/dev/null)" || return 1
  state="${fields%% *}"
  [[ -n "$state" && "$state" != Z ]]
}

dual_host_process_start_ticks() {
  local pid="${1:-}" fields
  local -a values=()
  fields="$(dual_host_proc_stat_fields "$pid")" || return 1
  read -r -a values <<<"$fields"
  # Field 22 in /proc/PID/stat is index 19 after removing PID and comm.
  [[ "${values[19]:-}" =~ ^[0-9]+$ ]] || return 1
  printf '%s\n' "${values[19]}"
}

dual_host_pid_parent() {
  local pid="${1:-}" fields
  local -a values=()
  fields="$(dual_host_proc_stat_fields "$pid")" || return 1
  read -r -a values <<<"$fields"
  [[ "${values[1]:-}" =~ ^[0-9]+$ ]] || return 1
  printf '%s\n' "${values[1]}"
}

dual_host_pid_command() {
  local pid="${1:-}"
  [[ "$pid" =~ ^[0-9]+$ && -r "$DUAL_HOST_PROC_ROOT/$pid/cmdline" ]] || return 1
  tr '\0' ' ' <"$DUAL_HOST_PROC_ROOT/$pid/cmdline"
}

dual_host_process_record() {
  local pid="${1:-}" start_ticks
  dual_host_process_is_running "$pid" || return 1
  start_ticks="$(dual_host_process_start_ticks "$pid")" || return 1
  printf '%s:%s\n' "$pid" "$start_ticks"
}

dual_host_record_is_running() {
  local record="${1:-}" pid start_ticks actual_ticks
  pid="${record%%:*}"
  start_ticks="${record#*:}"
  [[ "$pid" =~ ^[0-9]+$ && "$start_ticks" =~ ^[0-9]+$ ]] || return 1
  dual_host_process_is_running "$pid" || return 1
  actual_ticks="$(dual_host_process_start_ticks "$pid")" || return 1
  [[ "$actual_ticks" == "$start_ticks" ]]
}

dual_host_pid_matches() {
  local pid="${1:-}" expected="${2:-}" command
  dual_host_process_is_running "$pid" || return 1
  command="$(dual_host_pid_command "$pid" 2>/dev/null || true)"
  [[ -n "$command" && "$command" =~ $expected ]]
}

dual_host_pid_is_self_or_ancestor() {
  # BASHPID, unlike $$, changes inside process substitutions. This matters
  # because orphan discovery itself runs in a process substitution and must
  # never rediscover that temporary scanner as a managed process.
  local candidate="${1:-}" cursor="${BASHPID:-$$}" parent
  [[ "$candidate" =~ ^[0-9]+$ ]] || return 1
  while [[ "$cursor" =~ ^[0-9]+$ ]] && (( cursor > 1 )); do
    [[ "$cursor" != "$candidate" ]] || return 0
    parent="$(dual_host_pid_parent "$cursor" 2>/dev/null || true)"
    [[ "$parent" =~ ^[0-9]+$ && "$parent" != "$cursor" ]] || break
    cursor="$parent"
  done
  return 1
}

dual_host_write_pid_file() {
  local pid_file="$1" pid="$2" start_ticks temporary
  start_ticks="$(dual_host_process_start_ticks "$pid")" || {
    echo "Cannot record PID $pid because its process identity is unavailable." >&2
    return 1
  }
  mkdir -p "$(dirname "$pid_file")"
  temporary="${pid_file}.tmp.$$"
  printf '%s %s\n' "$pid" "$start_ticks" >"$temporary"
  mv -f -- "$temporary" "$pid_file"
}

dual_host_pid_file_pid() {
  local pid_file="$1" pid start_ticks extra
  [[ -f "$pid_file" ]] || return 1
  read -r pid start_ticks extra <"$pid_file" || return 1
  [[ "$pid" =~ ^[0-9]+$ && -z "${extra:-}" ]] || return 1
  if [[ -n "${start_ticks:-}" ]]; then
    [[ "$start_ticks" =~ ^[0-9]+$ ]] || return 1
  fi
  printf '%s\n' "$pid"
}

dual_host_pid_file_is_owned() {
  local pid_file="$1" expected="$2" pid recorded_ticks extra actual_ticks command
  [[ -f "$pid_file" ]] || return 1
  read -r pid recorded_ticks extra <"$pid_file" || return 1
  if [[ ! "$pid" =~ ^[0-9]+$ || -n "${extra:-}" ||
        ( -n "${recorded_ticks:-}" && ! "$recorded_ticks" =~ ^[0-9]+$ ) ]]; then
    echo "Removing invalid dual-host PID file: $pid_file" >&2
    rm -f -- "$pid_file"
    return 1
  fi
  if ! dual_host_process_is_running "$pid"; then
    rm -f -- "$pid_file"
    return 1
  fi
  actual_ticks="$(dual_host_process_start_ticks "$pid" 2>/dev/null || true)"
  if [[ -n "${recorded_ticks:-}" && "$actual_ticks" != "$recorded_ticks" ]]; then
    echo "Ignoring stale PID file $pid_file: PID $pid has been reused." >&2
    rm -f -- "$pid_file"
    return 1
  fi
  command="$(dual_host_pid_command "$pid" 2>/dev/null || true)"
  if [[ -z "$command" || ! "$command" =~ $expected ]]; then
    echo "Ignoring stale PID file $pid_file: PID $pid is not the expected managed command." >&2
    rm -f -- "$pid_file"
    return 1
  fi
  return 0
}

dual_host_remove_pid_file_if_unchanged() {
  local pid_file="$1" original_line="$2" current_line
  [[ -f "$pid_file" ]] || return 0
  current_line="$(sed -n '1p' "$pid_file" 2>/dev/null || true)"
  [[ "$current_line" != "$original_line" ]] || rm -f -- "$pid_file"
}

dual_host_pid_uid() {
  local pid="${1:-}" key real_uid _
  [[ "$pid" =~ ^[0-9]+$ ]] && (( pid > 1 )) || return 1
  [[ -r "$DUAL_HOST_PROC_ROOT/$pid/status" ]] || return 1
  while read -r key real_uid _; do
    if [[ "$key" == Uid: ]]; then
      [[ "$real_uid" =~ ^[0-9]+$ ]] || return 1
      printf '%s\n' "$real_uid"
      return 0
    fi
  done <"$DUAL_HOST_PROC_ROOT/$pid/status"
  return 1
}

dual_host_pid_environment_value() {
  local pid="${1:-}" wanted_key="${2:-}" entry key
  [[ "$pid" =~ ^[0-9]+$ ]] && (( pid > 1 )) || return 1
  [[ -n "$wanted_key" && -r "$DUAL_HOST_PROC_ROOT/$pid/environ" ]] || return 1
  while IFS= read -r entry; do
    key="${entry%%=*}"
    if [[ "$key" == "$wanted_key" ]]; then
      printf '%s\n' "${entry#*=}"
      return 0
    fi
  done < <(tr '\0' '\n' <"$DUAL_HOST_PROC_ROOT/$pid/environ")
  return 1
}

dual_host_pid_environment_equals() {
  local pid="$1" key="$2" expected="$3" actual
  actual="$(dual_host_pid_environment_value "$pid" "$key" 2>/dev/null)" || return 1
  [[ "$actual" == "$expected" ]]
}

dual_host_protected_pids() {
  local cursor="${BASHPID:-$$}" parent
  while [[ "$cursor" =~ ^[0-9]+$ ]] && (( cursor > 1 )); do
    printf '%s\n' "$cursor"
    parent="$(dual_host_pid_parent "$cursor" 2>/dev/null || true)"
    [[ "$parent" =~ ^[0-9]+$ && "$parent" != "$cursor" ]] || break
    cursor="$parent"
  done
}

dual_host_collect_tagged_process_records() {
  local token_file="$1" workspace="$2" token proc_path pid uid record protected_pid
  local managed_uid="${DUAL_HOST_MANAGED_UID:-$(id -u)}"
  local -A protected=()
  [[ -r "$token_file" ]] || return 0
  IFS= read -r token <"$token_file" || return 0
  [[ "$token" =~ ^[A-Za-z0-9][A-Za-z0-9._:-]{15,127}$ ]] || {
    echo "Ignoring invalid dual-host run token: $token_file" >&2
    return 0
  }
  while IFS= read -r protected_pid; do
    [[ -n "$protected_pid" ]] && protected["$protected_pid"]=1
  done < <(dual_host_protected_pids)

  for proc_path in "$DUAL_HOST_PROC_ROOT"/[0-9]*; do
    [[ -e "$proc_path/stat" ]] || continue
    pid="${proc_path##*/}"
    [[ "$pid" =~ ^[0-9]+$ ]] && (( pid > 1 )) || continue
    [[ -z "${protected[$pid]:-}" ]] || continue
    uid="$(dual_host_pid_uid "$pid" 2>/dev/null || true)"
    [[ "$uid" == "$managed_uid" ]] || continue
    dual_host_pid_environment_equals "$pid" DUAL_HOST_RUN_TOKEN "$token" || continue
    dual_host_pid_environment_equals "$pid" DUAL_HOST_WS "$workspace" || continue
    record="$(dual_host_process_record "$pid" 2>/dev/null || true)"
    [[ -n "$record" ]] && printf '%s\n' "$record"
  done
}

dual_host_collect_workspace_process_records() {
  local workspace="$1" ros_master_uri="$2" matcher="$3"
  local proc_path pid uid command record protected_pid
  local managed_uid="${DUAL_HOST_MANAGED_UID:-$(id -u)}"
  local -A protected=()
  [[ "$matcher" =~ ^[A-Za-z_][A-Za-z0-9_]*$ ]] || return 2
  declare -F "$matcher" >/dev/null || return 2
  while IFS= read -r protected_pid; do
    [[ -n "$protected_pid" ]] && protected["$protected_pid"]=1
  done < <(dual_host_protected_pids)

  for proc_path in "$DUAL_HOST_PROC_ROOT"/[0-9]*; do
    [[ -e "$proc_path/stat" ]] || continue
    pid="${proc_path##*/}"
    [[ "$pid" =~ ^[0-9]+$ ]] && (( pid > 1 )) || continue
    [[ -z "${protected[$pid]:-}" ]] || continue
    uid="$(dual_host_pid_uid "$pid" 2>/dev/null || true)"
    [[ "$uid" == "$managed_uid" ]] || continue
    dual_host_pid_environment_equals "$pid" DUAL_HOST_WS "$workspace" || continue
    dual_host_pid_environment_equals "$pid" ROS_MASTER_URI "$ros_master_uri" || continue
    command="$(dual_host_pid_command "$pid" 2>/dev/null || true)"
    [[ -n "$command" ]] || continue
    "$matcher" "$pid" "$command" || continue
    record="$(dual_host_process_record "$pid" 2>/dev/null || true)"
    [[ -n "$record" ]] && printf '%s\n' "$record"
  done
}

dual_host_stop_records() {
  local label="$1"
  shift
  local record pid status=0
  local -a tracked=() interrupt_records=() terminate_records=() remaining=()
  local -A seen=()

  for record in "$@"; do
    [[ "$record" =~ ^[0-9]+:[0-9]+$ ]] || continue
    pid="${record%%:*}"
    [[ -z "${seen[$pid]:-}" ]] || continue
    seen[$pid]=1
    dual_host_record_is_running "$record" || continue
    if dual_host_pid_is_self_or_ancestor "$pid"; then
      echo "Refusing to stop protected $label PID $pid." >&2
      status=1
      continue
    fi
    tracked+=("$record")
    if dual_host_process_ignores_interrupt "$pid"; then
      terminate_records+=("$record")
    else
      interrupt_records+=("$record")
    fi
  done

  (( ${#tracked[@]} > 0 )) || return "$status"
  echo "Stopping ${#tracked[@]} provenance-verified $label process(es)..."
  (( ${#interrupt_records[@]} == 0 )) ||
    dual_host_signal_records INT "${interrupt_records[@]}" || status=1
  (( ${#terminate_records[@]} == 0 )) ||
    dual_host_signal_records TERM "${terminate_records[@]}" || status=1
  if dual_host_wait_for_records "$DUAL_HOST_STOP_GRACE_SEC" "${tracked[@]}"; then
    return "$status"
  fi

  remaining=()
  for record in "${tracked[@]}"; do
    dual_host_record_is_running "$record" && remaining+=("$record")
  done
  if (( ${#remaining[@]} > 0 )); then
    echo "$label still has live verified processes; sending TERM only to those PIDs." >&2
    dual_host_signal_records TERM "${remaining[@]}" || status=1
    if dual_host_wait_for_records "$DUAL_HOST_STOP_TERM_SEC" "${remaining[@]}"; then
      return "$status"
    fi
  fi

  remaining=()
  for record in "${tracked[@]}"; do
    dual_host_record_is_running "$record" && remaining+=("$record")
  done
  if (( ${#remaining[@]} > 0 )); then
    echo "$label still has live verified processes; sending KILL only to those PIDs." >&2
    dual_host_signal_records KILL "${remaining[@]}" || status=1
    dual_host_wait_for_records "$DUAL_HOST_STOP_KILL_SEC" "${remaining[@]}" || status=1
  fi

  remaining=()
  for record in "${tracked[@]}"; do
    dual_host_record_is_running "$record" && remaining+=("$record")
  done
  if (( ${#remaining[@]} > 0 )); then
    echo "Failed to stop every provenance-verified $label process:" >&2
    for record in "${remaining[@]}"; do
      pid="${record%%:*}"
      printf '  PID %s: %s\n' "$pid" \
        "$(dual_host_pid_command "$pid" 2>/dev/null || echo unavailable)" >&2
    done
    return 1
  fi
  return "$status"
}

dual_host_child_pids() {
  local parent="${1:-}" children_file proc_path child actual_parent
  [[ "$parent" =~ ^[0-9]+$ ]] && (( parent > 1 )) || return 1
  children_file="$DUAL_HOST_PROC_ROOT/$parent/task/$parent/children"
  if [[ -r "$children_file" ]]; then
    while read -r child; do
      [[ "$child" =~ ^[0-9]+$ ]] && printf '%s\n' "$child"
    done < <(tr ' ' '\n' <"$children_file")
    return 0
  fi

  # Some vendor kernels omit task/PID/children. The fallback is read-only and
  # derives direct children solely from each process's PPID in /proc/PID/stat;
  # it does not select or signal processes by their name or command line.
  for proc_path in "$DUAL_HOST_PROC_ROOT"/[0-9]*; do
    [[ -e "$proc_path/stat" ]] || continue
    child="${proc_path##*/}"
    [[ "$child" =~ ^[0-9]+$ ]] && (( child > 1 )) || continue
    actual_parent="$(dual_host_pid_parent "$child" 2>/dev/null || true)"
    [[ "$actual_parent" == "$parent" ]] && printf '%s\n' "$child"
  done
}

dual_host_process_tree_records() {
  local root="${1:-}" pid child record index=0
  local -a queue=()
  local -A seen=()
  [[ "$root" =~ ^[0-9]+$ ]] && (( root > 1 )) || return 1
  queue+=("$root")
  while (( index < ${#queue[@]} )); do
    pid="${queue[index]}"
    index=$((index + 1))
    [[ -z "${seen[$pid]:-}" ]] || continue
    seen[$pid]=1
    record="$(dual_host_process_record "$pid" 2>/dev/null || true)"
    [[ -n "$record" ]] || continue
    printf '%s\n' "$record"
    for child in $(dual_host_child_pids "$pid"); do
      [[ "$child" =~ ^[0-9]+$ ]] || continue
      queue+=("$child")
      (( ${#queue[@]} <= 4096 )) || {
        echo "Managed process tree exceeds the 4096-process safety limit." >&2
        return 1
      }
    done
  done
}

dual_host_process_ignores_interrupt() {
  local pid="${1:-}" key value ignored_mask="" low_nibble
  [[ "$pid" =~ ^[0-9]+$ ]] && (( pid > 1 )) || return 1
  [[ -r "$DUAL_HOST_PROC_ROOT/$pid/status" ]] || return 1
  while read -r key value _; do
    if [[ "$key" == SigIgn: ]]; then
      ignored_mask="${value,,}"
      break
    fi
  done <"$DUAL_HOST_PROC_ROOT/$pid/status"
  [[ "$ignored_mask" =~ ^[0-9a-f]+$ ]] || return 1
  low_nibble="${ignored_mask: -1}"
  # SIGINT is signal 2, represented by bit 1 of the least-significant nibble.
  [[ "$low_nibble" == 2 || "$low_nibble" == 3 ||
     "$low_nibble" == 6 || "$low_nibble" == 7 ||
     "$low_nibble" == a || "$low_nibble" == b ||
     "$low_nibble" == e || "$low_nibble" == f ]]
}

dual_host_wait_for_records() {
  local timeout_sec="$1"
  shift
  local deadline=$((SECONDS + timeout_sec)) record alive
  while true; do
    alive=false
    for record in "$@"; do
      if dual_host_record_is_running "$record"; then
        alive=true
        break
      fi
    done
    [[ "$alive" == true ]] || return 0
    (( SECONDS < deadline )) || return 1
    sleep 0.2
  done
}

dual_host_send_signal() {
  local signal="$1" pid="$2"
  builtin kill "-$signal" "$pid"
}

dual_host_signal_records() {
  local signal="$1"
  shift
  local record pid status=0
  for record in "$@"; do
    dual_host_record_is_running "$record" || continue
    pid="${record%%:*}"
    if (( pid <= 1 )) || dual_host_pid_is_self_or_ancestor "$pid"; then
      echo "Refusing to signal protected PID $pid while stopping the dual-host stack." >&2
      status=1
      continue
    fi
    if ! dual_host_send_signal "$signal" "$pid" 2>/dev/null; then
      # Exiting between the identity check and kill(2) is a successful stop,
      # not a reason to broaden the target set or fail the whole shutdown.
      dual_host_record_is_running "$record" && status=1
    fi
  done
  return "$status"
}

dual_host_stop_pid_file() {
  local pid_file="$1" label="$2" expected="$3"
  local original_line pid recorded_ticks actual_ticks root_record record status=0
  local -a tracked=() remaining=()
  [[ -f "$pid_file" ]] || return 0
  original_line="$(sed -n '1p' "$pid_file" 2>/dev/null || true)"
  if ! dual_host_pid_file_is_owned "$pid_file" "$expected"; then
    return 0
  fi
  read -r pid recorded_ticks <<<"$original_line"
  actual_ticks="$(dual_host_process_start_ticks "$pid")" || return 0
  root_record="$pid:$actual_ticks"
  if dual_host_pid_is_self_or_ancestor "$pid"; then
    echo "Refusing to stop protected $label PID $pid." >&2
    return 1
  fi

  mapfile -t tracked < <(dual_host_process_tree_records "$pid")
  (( ${#tracked[@]} > 0 )) || tracked=("$root_record")
  echo "Stopping $label (PID $pid)..."
  if dual_host_process_ignores_interrupt "$pid"; then
    echo "$label PID $pid ignores INT; sending TERM only to its recorded process tree." >&2
    dual_host_signal_records TERM "${tracked[@]}" || status=1
    if dual_host_wait_for_records "$DUAL_HOST_STOP_TERM_SEC" "${tracked[@]}"; then
      dual_host_remove_pid_file_if_unchanged "$pid_file" "$original_line"
      return "$status"
    fi
  else
    dual_host_signal_records INT "$root_record" || status=1
    if (( status == 0 )) && dual_host_wait_for_records "$DUAL_HOST_STOP_GRACE_SEC" "${tracked[@]}"; then
      dual_host_remove_pid_file_if_unchanged "$pid_file" "$original_line"
      return 0
    fi
  fi

  remaining=()
  for record in "${tracked[@]}"; do
    dual_host_record_is_running "$record" && remaining+=("$record")
  done
  if (( ${#remaining[@]} > 0 )); then
    echo "$label still has live recorded processes; sending TERM only to those PIDs." >&2
    dual_host_signal_records TERM "${remaining[@]}" || status=1
    if dual_host_wait_for_records "$DUAL_HOST_STOP_TERM_SEC" "${remaining[@]}"; then
      dual_host_remove_pid_file_if_unchanged "$pid_file" "$original_line"
      return "$status"
    fi
  fi

  remaining=()
  for record in "${tracked[@]}"; do
    dual_host_record_is_running "$record" && remaining+=("$record")
  done
  if (( ${#remaining[@]} > 0 )); then
    echo "$label still has live recorded processes; sending KILL only to those PIDs." >&2
    dual_host_signal_records KILL "${remaining[@]}" || status=1
    dual_host_wait_for_records "$DUAL_HOST_STOP_KILL_SEC" "${remaining[@]}" || status=1
  fi

  remaining=()
  for record in "${tracked[@]}"; do
    dual_host_record_is_running "$record" && remaining+=("$record")
  done
  if (( ${#remaining[@]} > 0 )); then
    echo "Failed to stop every recorded $label process:" >&2
    for record in "${remaining[@]}"; do
      pid="${record%%:*}"
      printf '  PID %s: %s\n' "$pid" "$(dual_host_pid_command "$pid" 2>/dev/null || echo unavailable)" >&2
    done
    return 1
  fi
  dual_host_remove_pid_file_if_unchanged "$pid_file" "$original_line"
  return "$status"
}
