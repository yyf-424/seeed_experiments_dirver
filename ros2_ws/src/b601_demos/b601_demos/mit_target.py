"""Encode and validate sparse joint feedforward for a MIT target."""
from collections.abc import Mapping
from math import isfinite
from time import monotonic

from b601_target_interfaces.msg import JointFeedforward, MitTarget


def measured_positions(joints, feedback_sample, feedback_timeout):
    feedback, received_at = feedback_sample
    if (received_at is None or monotonic() - received_at > feedback_timeout
            or any(not isfinite(feedback.get(name, float("nan"))) for name in joints)):
        return None
    return {name: feedback[name] for name in joints}


def positions_from_deltas(joints, deltas, feedback_sample, feedback_timeout):
    """Expand an upper-layer joint edit using a single measured feedback sample."""
    if not isinstance(deltas, Mapping) or not deltas or not deltas.keys() <= set(joints):
        raise ValueError("deltas must be keyed by controlled joint name")
    increments = {name: float(value) for name, value in deltas.items()}
    if not all(isfinite(value) for value in increments.values()):
        raise ValueError("deltas must contain finite position increments")
    feedback = measured_positions(joints, feedback_sample, feedback_timeout)
    if feedback is None:
        raise RuntimeError("delta target requires fresh measured positions for all controlled joints")
    return {name: feedback[name] + increments.get(name, 0.0) for name in joints}


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
