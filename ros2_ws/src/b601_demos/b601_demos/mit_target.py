"""Encode and validate sparse joint feedforward for a MIT target."""
from collections.abc import Mapping
from math import isfinite

from b601_target_interfaces.msg import JointFeedforward, MitTarget


def make_mit_target(target, gravity_ff=None, friction_ff=None):
    command = MitTarget(target=target)
    for field, values in (("gravity_ff", gravity_ff), ("friction_ff", friction_ff)):
        if values is None:
            continue
        if not isinstance(values, Mapping) or not values.keys() <= set(target.name):
            raise ValueError(f"{field} must be keyed by target joint name")
        entries = [JointFeedforward(joint=name, effort=float(value)) for name, value in values.items()]
        if any(not isfinite(entry.effort) for entry in entries):
            raise ValueError(f"{field} must contain finite effort values")
        setattr(command, field, entries)
    return command


def feedforward_by_joint(entries, joints, field):
    values = {}
    for entry in entries:
        if entry.joint not in joints or entry.joint in values or not isfinite(entry.effort):
            raise ValueError(f"{field} needs a finite effort for each unique known joint")
        values[entry.joint] = entry.effort
    return values
