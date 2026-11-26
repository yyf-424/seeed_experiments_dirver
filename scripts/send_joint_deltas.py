#!/usr/bin/env python3
"""Send one seven-axis measured-relative target through the local B601 web console."""

import argparse
import json
import sys
from math import isfinite
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


JOINTS = (*[f"joint{axis}" for axis in range(1, 7)], "finger_left")


def post(port, endpoint, payload):
    url = f"http://127.0.0.1:{port}/api/{endpoint}"
    request = Request(url, data=json.dumps(payload).encode(),
                      headers={"Content-Type": "application/json"})
    try:
        with urlopen(request, timeout=3) as response:
            return json.load(response)
    except HTTPError as error:
        body = error.read().decode(errors="replace")
        try:
            message = json.loads(body).get("message", body)
        except (ValueError, AttributeError):
            message = body
        raise RuntimeError(f"HTTP {error.code} {message}") from error
    except URLError as error:
        raise RuntimeError(f"无法连接本地网页端口 {port}（{error.reason}）") from error


def main():
    parser = argparse.ArgumentParser(
        usage="%(prog)s [--port PORT] J1 J2 J3 J4 J5 J6 GRIPPER",
        description="发送一次七轴增量，J1–J6 单位 rad，主动夹爪单位 m。",
    )
    parser.add_argument("--port", type=int, default=8765, help="本地网页端口，默认 8765")
    # Parse options separately so negative exponents such as -1e-3 remain numeric arguments.
    args, values = parser.parse_known_args()
    if len(values) != len(JOINTS):
        parser.error("请按 J1–J6、主动夹爪的顺序提供七个增量")
    try:
        increments = [float(value) for value in values]
    except ValueError:
        parser.error("增量必须是数值")
    if not all(isfinite(value) for value in increments):
        parser.error("增量必须是有限数值")

    try:
        session = {"session_id": post(args.port, "select", {})["session_id"]}
        try:
            result = post(args.port, "delta_target", {
                **session, "sequence": 1, "deltas": dict(zip(JOINTS, increments)),
            })
        finally:
            try:
                post(args.port, "release", session)
            except (OSError, RuntimeError, ValueError) as error:
                print(f"释放会话失败（{error}）", file=sys.stderr)
    except (OSError, RuntimeError, ValueError, KeyError) as error:
        parser.exit(1, f"{parser.prog}: {error}\n")

    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
