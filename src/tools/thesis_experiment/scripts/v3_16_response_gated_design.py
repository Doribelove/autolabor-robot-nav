#!/usr/bin/env python3
"""Single-factor V3-16 world materializer with postbranch actor response."""

import copy
import xml.etree.ElementTree as ET

import v3_13_multiactor_design as base


RESPONSE_ENABLE_SIM_S = 12.55


def render_world(scene, actor_policies, seed):
    world_text, actor_specs = base.render_world(
        scene, actor_policies, seed
    )
    root = ET.fromstring(world_text)
    plugins = [
        item for item in root.findall(".//plugin")
        if item.attrib.get("filename") == "libv3_reactive_actor_plugin.so"
    ]
    if len(plugins) != len(actor_specs):
        raise RuntimeError("reactive actor plugin roster differs")
    for plugin in plugins:
        if plugin.find("response_enable_time") is not None:
            raise RuntimeError("response gate already present")
        ET.SubElement(plugin, "response_enable_time").text = (
            "{:.9f}".format(RESPONSE_ENABLE_SIM_S)
        )
    base.indent(root)
    updated_specs = []
    for spec in actor_specs:
        value = copy.deepcopy(spec)
        value["response_enable_sim_s"] = RESPONSE_ENABLE_SIM_S
        value["prebranch_response_disabled"] = True
        updated_specs.append(value)
    return (
        '<?xml version="1.0"?>\n'
        + ET.tostring(root, encoding="unicode")
        + "\n",
        updated_specs,
    )

