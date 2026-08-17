#!/usr/bin/env python3

"""Remove unreachable ROS registrations whose XML-RPC URI is on one host."""

import argparse
import socket
import sys
from urllib.parse import urlparse

import rosgraph
import rosnode


def resolved_addresses(host):
    addresses = {host}
    try:
        addresses.update(
            item[4][0]
            for item in socket.getaddrinfo(host, 0, 0, 0, socket.SOL_TCP)
        )
    except socket.gaierror:
        pass
    return addresses


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", required=True)
    parser.add_argument(
        "--node",
        action="append",
        required=True,
        dest="requested_nodes",
        help="exact managed ROS node name; repeat for each node",
    )
    parser.add_argument(
        "--fail-if-live",
        action="store_true",
        help="return non-zero when a requested registration is still reachable",
    )
    args = parser.parse_args()

    master = rosgraph.Master("/dual_host_stop_cleanup")
    try:
        registered_names = set(rosnode.get_node_names())
    except Exception as error:  # ROS master may already be gone.
        print("ROS master unavailable; skipping stale-registration cleanup: %s" % error)
        return 0

    node_names = sorted(set(args.requested_nodes) & registered_names)
    target_addresses = resolved_addresses(args.host)
    stale = []
    live = []
    foreign = []
    for node_name in node_names:
        try:
            uri = master.lookupNode(node_name)
        except Exception:
            stale.append(node_name)
            continue
        node_host = urlparse(uri).hostname
        if not node_host or not (resolved_addresses(node_host) & target_addresses):
            foreign.append("%s (%s)" % (node_name, node_host or "unknown host"))
            continue
        try:
            reachable = rosnode.rosnode_ping(
                node_name, max_count=1, verbose=False, skip_cache=True
            )
        except Exception:
            reachable = False
        (live if reachable else stale).append(node_name)

    if stale:
        rosnode.cleanup_master_blacklist(master, stale)
        print("Purged stale NVIDIA ROS registrations: %s" % ", ".join(sorted(stale)))
    else:
        print("No stale NVIDIA ROS registrations found.")
    if live:
        print("Live NVIDIA ROS nodes still registered: %s" % ", ".join(sorted(live)))
    if foreign:
        print(
            "Requested ROS names are owned by another host and were not changed: %s"
            % ", ".join(sorted(foreign))
        )
    if args.fail_if_live and (live or foreign):
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
