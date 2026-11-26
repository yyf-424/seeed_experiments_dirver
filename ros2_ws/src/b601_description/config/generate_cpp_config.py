"""Generate typed C++ values from b601_project.yaml at configure time."""

import json
import math
from pathlib import Path
import sys

import yaml


def literal(value):
    return json.dumps(value, ensure_ascii=True)


def constants(group):
    types = {float: "double", int: "int", bool: "bool", str: "const char *"}
    for name, value in group.items():
        if type(value) in types:
            yield f"inline constexpr {types[type(value)]} {name} = {literal(value)};"
        elif isinstance(value, list):
            items = ", ".join(map(literal, value))
            yield f"inline const std::vector<double> {name} = {{{items}}};"


def rotation(origin):
    r, p, y = origin["rpy"]
    cr, cp, cy = map(math.cos, (r, p, y))
    sr, sp, sy = map(math.sin, (r, p, y))
    return [
        cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
        -sp, cp * sr, cp * cr,
    ]


def vector(values):
    return "{" + ", ".join(map(literal, values)) + "}"


def render(cfg):
    dm = cfg["hardware"]["dm"]
    mit = cfg["controllers"]["mit"]
    profiles = mit["profiles"]
    arm_joints = cfg["robot"]["arm_joints"]
    profile = profiles[mit["instances"]["b601_mit_arm"]["profile"]]
    gains = {key: [profile["per_joint"][name][key] for name in arm_joints]
             for key in ("kp_move", "kd_move")}
    groups = {
        "dm": {**dm, **dm["temperature"], **cfg["implementation"]["dm"]},
        "mujoco": {**dm["thermal"], **cfg["implementation"]["mujoco"]},
        "mit": {**profile["group"], **gains, **cfg["implementation"]["mit"],
                "gravity_feedback_timeout_sec": cfg["safety"]["feedback_timeout_sec"],
                "gravity_m_s2": cfg["model"]["gravity_m_s2"]},
    }
    # Skip nonnumeric ROS arrays. They remain in the controller YAML.
    groups["mit"] = {k: v for k, v in groups["mit"].items()
                     if not isinstance(v, list) or all(type(x) in (int, float) for x in v)}
    lines = ["// Generated from b601_project.yaml.", "#pragma once", "#include <array>",
             "#include <cstddef>", "#include <vector>", "", "namespace b601_config {"]
    for group, values in groups.items():
        lines.extend([f"namespace {group} {{", *constants(values), "}"])
    lines.append("}")
    lines.append("""
namespace b601_mit_controller::chain {
struct JointOrigin {
  const char * name;
  double xyz[3];
  double rot[9];
  double axis[3];
};

struct GravityLink {
  const char * name;
  int joint_depth;
  double xyz[3];
  double rot[9];
};
""")
    model = cfg["model"]["joints"]
    lines.append(f"inline constexpr std::size_t kNumJoints = {len(arm_joints)};")
    lines.append("inline constexpr std::array<JointOrigin, kNumJoints> kJoints = {{")
    for name in arm_joints:
        joint = model[name]
        origin = joint["origin"]
        fields = [literal(name), vector(origin["xyz"]), vector(rotation(origin)),
                  vector(joint["axis"]["xyz"])]
        lines.append("  {" + ", ".join(fields) + "},")
    lines.append("}};")
    valid_links = [(f"link{i + 1}", i) for i, _ in enumerate(arm_joints) if i]
    fixed = ["end_link", "finger_left_link", "finger_right_link"]
    lines.append(f"inline constexpr std::size_t kNumGravityLinks = {len(valid_links) + len(fixed)};")
    lines.append("inline constexpr std::array<GravityLink, kNumGravityLinks> kGravityLinks = {{")
    identity = {"xyz": [0, 0, 0], "rpy": [0, 0, 0]}
    for name, depth in valid_links + [(name, len(arm_joints) - 1) for name in fixed]:
        origin = model["end_joint"]["origin"] if name in fixed else identity
        fields = [literal(name), literal(depth), vector(origin["xyz"]), vector(rotation(origin))]
        lines.append("  {" + ", ".join(fields) + "},")
    lines.extend(["}};", "}", ""])
    return "\n".join(lines)


if __name__ == "__main__":
    source, destination = map(Path, sys.argv[1:])
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(render(yaml.safe_load(source.read_text())), encoding="utf-8")
