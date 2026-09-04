#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="/home/slam/robot_j6m_ws"
ARCHIVE="${WORKSPACE}/.codex_snapshots/locateanything_20260903_2145.tar.gz"
EXPECTED_SHA256="b9104ab6c96e84391cb32f5c815b62f82c43cc287de22dd95fea3e063b196fe8"
SERVICE="autolabor-dual-host.service"

usage() {
    cat <<'EOF'
Usage:
  ./scripts/restore_locateanything_2145.sh --check
  ./scripts/restore_locateanything_2145.sh --apply

--check verifies the immutable 21:45 archive and reports whether archived
files differ from the workspace. --apply restores the archived NVIDIA vision,
Qt, and launch/configuration files. For safety, --apply refuses to run while
the managed dual-host service is active. It does not deploy to J6M or restart
the vehicle stack.
EOF
}

MODE="${1:---check}"
if [[ "${MODE}" != "--check" && "${MODE}" != "--apply" ]]; then
    usage >&2
    exit 2
fi

if [[ ! -f "${ARCHIVE}" ]]; then
    echo "ERROR: missing 21:45 snapshot: ${ARCHIVE}" >&2
    exit 1
fi

ACTUAL_SHA256="$(sha256sum "${ARCHIVE}" | awk '{print $1}')"
if [[ "${ACTUAL_SHA256}" != "${EXPECTED_SHA256}" ]]; then
    echo "ERROR: 21:45 snapshot checksum mismatch" >&2
    echo "expected=${EXPECTED_SHA256}" >&2
    echo "actual=${ACTUAL_SHA256}" >&2
    exit 1
fi

while IFS= read -r member; do
    case "${member}" in
        /*|../*|*/../*|..)
            echo "ERROR: unsafe archive member: ${member}" >&2
            exit 1
            ;;
    esac
done < <(tar -tzf "${ARCHIVE}")

STAGING="$(mktemp -d /tmp/locateanything_2145_check.XXXXXX)"
trap 'rm -rf -- "${STAGING}"' EXIT
tar -xzf "${ARCHIVE}" -C "${STAGING}"

DIFFERENT=0
while IFS= read -r member; do
    [[ -f "${STAGING}/${member}" ]] || continue
    if [[ ! -f "${WORKSPACE}/${member}" ]] || ! cmp -s "${STAGING}/${member}" "${WORKSPACE}/${member}"; then
        echo "DIFF ${member}"
        DIFFERENT=$((DIFFERENT + 1))
    fi
done < <(tar -tzf "${ARCHIVE}")

if [[ "${MODE}" == "--check" ]]; then
    echo "OK snapshot_sha256=${ACTUAL_SHA256} archived_files=110 differences=${DIFFERENT}"
    exit 0
fi

if systemctl --user is-active --quiet "${SERVICE}"; then
    echo "ERROR: ${SERVICE} is active; stop it with:" >&2
    echo "  ${WORKSPACE}/scripts/start_dual_host.sh --stop" >&2
    exit 3
fi

tar -xzf "${ARCHIVE}" -C "${WORKSPACE}"

POST_DIFFERENT=0
while IFS= read -r member; do
    [[ -f "${STAGING}/${member}" ]] || continue
    if [[ ! -f "${WORKSPACE}/${member}" ]] || ! cmp -s "${STAGING}/${member}" "${WORKSPACE}/${member}"; then
        echo "ERROR: restore verification failed: ${member}" >&2
        POST_DIFFERENT=$((POST_DIFFERENT + 1))
    fi
done < <(tar -tzf "${ARCHIVE}")
if (( POST_DIFFERENT != 0 )); then
    exit 1
fi

echo "OK restored LocateAnything/Qt configuration and source to 2026-09-03 21:45"
echo "The stack remains stopped. Rebuild locally before the next unified start:"
echo "  ${WORKSPACE}/scripts/build_workspace.sh"
