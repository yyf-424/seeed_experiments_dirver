"""Shared contract for contact-force providers and clients."""

from dataclasses import dataclass
from math import hypot, isfinite

from b601_target_interfaces.msg import LoadEstimate, LoadEstimates
from geometry_msgs.msg import Point, PointStamped, Vector3
from std_msgs.msg import Header

LOAD_FRAME = "base_link"
LOAD_TOPIC = "/b601_load/estimates"
DEFAULT_LOAD_API = "b601_demos.contact_load_provider:create_provider"


@dataclass(frozen=True)
class LoadSite:
    link: str
    xyz_m: tuple[float, float, float]


@dataclass(frozen=True)
class LoadReading:
    """A location's status and optional external force in base_link axes."""

    status: str
    force_n: tuple[float, float, float] | None = None


# Load-identification candidate points, expressed in the URDF link frames.
LOAD_SITES = {
    "upper_arm": LoadSite("link2", (-0.135, 0.035, -0.030)),
    "forearm": LoadSite("link3", (0.120, -0.045, -0.030)),
    "wrist": LoadSite("link4", (0.068, -0.060, -0.030)),
    "gripper": LoadSite("end_link", (-0.090, 0.033, 0.025)),
}


def parse_load_positions(value):
    names = tuple(name.strip() for name in value.split(","))
    if len(set(names)) != len(names) or not set(names) <= LOAD_SITES.keys():
        raise ValueError(f"load_positions must list distinct locations from {', '.join(LOAD_SITES)}")
    return names


def load_estimates(sites, readings, status, stamp=None):
    header = Header(frame_id=LOAD_FRAME)
    if stamp is not None:
        header.stamp = stamp
    estimates = []
    for name, site in sites.items():
        reading = readings.get(name, LoadReading(status))
        force = () if reading.force_n is None else tuple(float(axis) for axis in reading.force_n)
        if reading.force_n is not None and (len(force) != 3 or not isfinite(hypot(*force))):
            raise ValueError(f"invalid load force for {name}")
        application_point = PointStamped(
            header=Header(frame_id=site.link, stamp=header.stamp),
            point=Point(x=site.xyz_m[0], y=site.xyz_m[1], z=site.xyz_m[2]),
        )
        estimates.append(LoadEstimate(
            location=name, application_point=application_point, status=reading.status,
            force_valid=bool(force), force=Vector3(x=force[0], y=force[1], z=force[2]) if force else Vector3(),
        ))
    return LoadEstimates(header=header, estimates=estimates)


def load_snapshot(message):
    stamp = message.header.stamp
    estimates = {}
    for reading in message.estimates:
        point = reading.application_point
        force = [reading.force.x, reading.force.y, reading.force.z] if reading.force_valid else None
        magnitude = hypot(*force) if force is not None else None
        if magnitude is not None and not isfinite(magnitude):
            continue
        estimates[reading.location] = {
            "status": reading.status,
            "link": point.header.frame_id,
            "point_xyz_m": [point.point.x, point.point.y, point.point.z],
            "force_n": force,
            "magnitude_n": magnitude,
            "direction": [axis / magnitude for axis in force] if magnitude else None,
        }
    return {
        "frame_id": message.header.frame_id,
        "stamp_ns": str(stamp.sec * 1_000_000_000 + stamp.nanosec) if stamp.sec or stamp.nanosec else None,
        "estimates": estimates,
    }
