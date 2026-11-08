"""Hardware-independent mission logic. Angles are radians, times seconds.

Vision boxes are center-x, center-y, width, height normalized by image dimensions;
center coordinates use [-1, 1] and width/height use fractions of the full image.
A Frame is immutable for capture: its timestamp and pixels refer to the same image.
"""
from dataclasses import dataclass, field
from pathlib import Path
from typing import Protocol
import json
import math


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


@dataclass
class Config:
    goal: list
    targets: list
    dt: float = 0.1
    mission_timeout: float = 100.0
    navigation_timeout: float = 35.0
    target_timeout: float = 18.0
    frame_max_age: float = 0.35
    hold_seconds: float = 0.7
    center_tolerance: float = 0.06
    image_speed_limit: float = 0.12
    sharpness_min: float = 0.65
    min_target_width: float = 0.06
    base_speed_limit: float = 0.015
    base_yaw_rate_limit: float = 0.03
    pan_limit: float = 0.85
    tilt_limit: float = 0.65
    gimbal_rate_limit: float = 0.6
    servo_gain: float = 1.3
    dropout_target: int = 1
    dropout_after: float = 0.4
    dropout_seconds: float = 2.0

    def __post_init__(self):
        for k, v in vars(self).items():
            if isinstance(v, (float, int)) and (not math.isfinite(v) or v < 0):
                raise ValueError(f"Invalid {k}: {v}")
        for k in ("dt", "mission_timeout", "navigation_timeout", "target_timeout",
                  "hold_seconds", "pan_limit", "tilt_limit", "gimbal_rate_limit"):
            if getattr(self, k) <= 0:
                raise ValueError(f"{k} must be positive")
        if len(self.goal) != 3 or not all(math.isfinite(v) for v in self.goal):
            raise ValueError("goal must contain finite x, y, yaw")
        ids = [t["id"] for t in self.targets]
        if not ids or len(set(ids)) != len(ids) or any(type(i) is not int or not 0 <= i < 50 for i in ids):
            raise ValueError("targets require unique DICT_4X4_50 IDs (0..49)")
        for t in self.targets:
            if len(t["world"]) != 3 or not all(math.isfinite(v) for v in t["world"]):
                raise ValueError("target world coordinates must be finite x,y,z")

    @classmethod
    def load(cls, path):
        return cls(**json.loads(Path(path).read_text()))


@dataclass
class Frame:
    stamp: float
    boxes: dict = field(default_factory=dict)
    sharpness: float = 0.0
    png: bytes = b""


@dataclass
class Observation:
    now: float
    nav: str  # running, succeeded, failed
    base_speed: float
    base_yaw_rate: float
    odom_age: float
    frame: Frame | None
    pose: tuple = (0.0, 0.0, 0.0)


class Backend(Protocol):
    def start_navigation(self, goal): ...
    def tick(self, dt, target_id) -> Observation: ...
    def point(self, pan, tilt): ...
    def stop(self): ...
    def close(self): ...


class Mission:
    def __init__(self, config, backend, output):
        self.c, self.b, self.output = config, backend, Path(output)
        self.output.mkdir(parents=True, exist_ok=True)
        (self.output / "captures").mkdir(exist_ok=True)
        self.state = "NAVIGATE"
        self.index = 0
        self.pan = self.tilt = 0.0
        self.started = self.target_started = None
        self.stable_since = self.lost_since = None
        self.previous = None
        self.events, self.captures, self.telemetry = [], [], []
        self.reason = None
        self.b.start_navigation(self.c.goal)

    @property
    def done(self):
        return self.state in ("COMPLETE", "FAILED")

    @property
    def target_id(self):
        return self.c.targets[min(self.index, len(self.c.targets)-1)]["id"]

    def transition(self, state, now, reason=None):
        if self.state != state:
            self.events.append({"time": now, "from": self.state, "to": state,
                                "target": self.target_id, "reason": reason})
            self.state = state

    def fail(self, now, reason):
        self.reason = reason
        self.transition("FAILED", now, reason)
        self.b.stop()

    def step(self, o):
        if self.done:
            return
        if self.started is None:
            self.started = o.now
        if o.now - self.started > self.c.mission_timeout:
            return self.fail(o.now, "mission_timeout")
        f = o.frame
        fresh = f is not None and 0 <= o.now - f.stamp <= self.c.frame_max_age
        box = f.boxes.get(self.target_id) if fresh else None
        self.telemetry.append({"time": round(o.now, 3), "state": self.state,
            "target": self.target_id, "pose": o.pose, "pan": self.pan, "tilt": self.tilt,
            "fresh": fresh, "boxes": f.boxes if fresh else {},
            "sharpness": f.sharpness if fresh else 0, "captures": len(self.captures)})
        if self.state == "NAVIGATE":
            if o.nav == "failed":
                return self.fail(o.now, "navigation_failed")
            if o.now - self.started > self.c.navigation_timeout:
                return self.fail(o.now, "navigation_timeout")
            if o.nav == "succeeded":
                self.b.stop()
                self.target_started = o.now
                self.transition("ACQUIRE", o.now)
            return
        if o.now - self.target_started > self.c.target_timeout:
            return self.fail(o.now, "target_timeout")
        stationary = (o.odom_age <= self.c.frame_max_age and
                      abs(o.base_speed) <= self.c.base_speed_limit and
                      abs(o.base_yaw_rate) <= self.c.base_yaw_rate_limit)
        if not stationary:
            self.stable_since = None
            self.b.stop()
            self.transition("SETTLE_BASE", o.now)
            return
        if box is None:
            self.previous = None
            self.stable_since = None
            if self.lost_since is None:
                self.lost_since = o.now
                self.search_origin = (self.pan, self.tilt)
            self.transition("REACQUIRE", o.now, "missing_or_stale_target")
            # Bounded horizontal sweep; hold briefly before moving. Last known tilt retained.
            elapsed = max(0, o.now - self.lost_since - 0.5)
            desired = self.search_origin[0] + 0.35 * math.sin(1.5 * elapsed)
            self.command(desired, self.search_origin[1])
            return
        if self.lost_since is not None:
            self.events.append({"time": o.now, "event": "reacquired", "target": self.target_id,
                                "latency": o.now - self.lost_since})
            self.lost_since = None
        x, y, w, h = box
        image_speed = float("inf")
        new_frame = self.previous is None or f.stamp > self.previous[0]
        if self.previous is not None and f.stamp > self.previous[0]:
            image_speed = math.hypot(x-self.previous[1], y-self.previous[2]) / (f.stamp-self.previous[0])
        if not new_frame:
            # Repeated frames never advance the stable dwell or trigger a capture.
            return
        self.previous = (f.stamp, x, y)
        self.command(self.pan + self.c.servo_gain*x*self.c.dt,
                     self.tilt - self.c.servo_gain*y*self.c.dt)
        fully_visible = abs(x) + w <= 0.95 and abs(y) + h <= 0.95
        good = (math.hypot(x, y) <= self.c.center_tolerance and fully_visible and
                w >= self.c.min_target_width and f.sharpness >= self.c.sharpness_min and
                image_speed <= self.c.image_speed_limit)
        if not good:
            self.stable_since = None
            self.transition("ALIGN", o.now)
            return
        self.transition("STABILIZE", o.now)
        if self.stable_since is None:
            self.stable_since = f.stamp
        if f.stamp - self.stable_since + 1e-9 < self.c.hold_seconds:
            return
        if not f.png:
            return self.fail(o.now, "capture_has_no_pixels")
        filename = f"captures/target-{self.target_id}.png"
        (self.output / filename).write_bytes(f.png)
        self.captures.append({"target": self.target_id, "name": self.c.targets[self.index]["name"],
            "time": o.now, "frame_stamp": f.stamp, "file": filename, "center_error": math.hypot(x,y),
            "image_speed": image_speed, "sharpness": f.sharpness, "hold_seconds": f.stamp-self.stable_since})
        self.events.append({"time": o.now, "event": "capture", "target": self.target_id})
        self.index += 1
        self.stable_since = self.previous = None
        self.target_started = o.now
        if self.index == len(self.c.targets):
            self.transition("COMPLETE", o.now)
            self.b.stop()
        else:
            self.transition("ACQUIRE", o.now)

    def command(self, pan, tilt):
        step = self.c.gimbal_rate_limit * self.c.dt
        self.pan = clamp(pan, max(-self.c.pan_limit, self.pan-step), min(self.c.pan_limit, self.pan+step))
        self.tilt = clamp(tilt, max(-self.c.tilt_limit, self.tilt-step), min(self.c.tilt_limit, self.tilt+step))
        self.b.point(self.pan, self.tilt)

    def report(self):
        return {"schema_version": 1, "success": self.state == "COMPLETE", "state": self.state,
            "failure_reason": self.reason, "required_targets": [t["id"] for t in self.c.targets],
            "captures": self.captures, "events": self.events,
            "elapsed_seconds": self.telemetry[-1]["time"] - self.started if self.telemetry else 0}
