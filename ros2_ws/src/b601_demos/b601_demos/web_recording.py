"""Write a timestamped web console recording without blocking ROS callbacks."""
import json
from math import isfinite
from pathlib import Path
from queue import SimpleQueue
from threading import Lock, Thread
from time import monotonic_ns, time_ns


def serializable(value):
    if isinstance(value, dict):
        return {key: serializable(item) for key, item in value.items() if key != "session_id"}
    if isinstance(value, (list, tuple)):
        return [serializable(item) for item in value]
    return None if isinstance(value, float) and not isfinite(value) else value


class WebRecording:
    def __init__(self, directory, report_error):
        self.directory = Path(directory).expanduser().resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.path = self.directory / "events.jsonl"
        self.stream = self.path.open("x", encoding="utf-8", buffering=1)
        self.pending = SimpleQueue()
        self.lock = Lock()
        self.closed = False
        self.error = None
        self.report_error = report_error
        self.writer = Thread(target=self.write, name="b601-web-recording", daemon=True)
        self.writer.start()

    def add(self, event, **fields):
        with self.lock:
            if not self.closed and self.error is None:
                self.pending.put({
                    "event": event, "unix_ns": time_ns(), "monotonic_ns": monotonic_ns(), **fields,
                })

    def write(self):
        try:
            with self.stream as stream:
                while (row := self.pending.get()) is not None:
                    stream.write(json.dumps(serializable(row), ensure_ascii=False, allow_nan=False) + "\n")
        except OSError as error:
            self.error = str(error)
            self.report_error(f"网页记录写入失败：{error}")

    def close(self):
        with self.lock:
            if not self.closed:
                self.closed = True
                self.pending.put(None)
        self.writer.join()
