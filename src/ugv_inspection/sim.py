"""Kinematic lab simulation. Not a validated physics model of the Waveshare rover.

Vision comes from geometric projection (no detector); sharpness is a motion proxy.
The fault injector interrupts only inspection images, never navigation sensing.
"""
import math
import random
from .core import Frame, Observation, clamp
from .render import png_bytes, rgb_frame

SCENARIOS = ('nominal', 'dropout', 'frozen-camera', 'permanent-dropout', 'navigation-failure', 'blurred-camera')


class Simulator:
    def __init__(self, config, scenario='dropout', seed=42):
        self.c, self.scenario = config, scenario
        self.rng = random.Random(seed)
        self.now = 0.0
        self.x, self.y, self.yaw = 0.0, self.rng.uniform(-0.12,0.12), 0.0
        self.pan = self.tilt = self.pan_cmd = self.tilt_cmd = 0.0
        self.nav = 'running'
        self.frame = None
        self.fault_started = None
        self.faults = []
        self.fault_active = False
        self.speed = self.yaw_rate = 0.0
        self.target_seen_at = None
        self.stopped = False

    def start_navigation(self, goal):
        self.goal = goal

    def point(self, pan, tilt):
        self.pan_cmd, self.tilt_cmd = pan, tilt

    def stop(self):
        self.speed = self.yaw_rate = 0.0
        self.stopped = True

    def close(self):
        self.stop()

    def tick(self, dt, target_id):
        self.now += dt
        if self.nav == 'running':
            if self.scenario == 'navigation-failure' and self.now >= 1:
                self.nav = 'failed'
                self.stop()
            else:
                gx, gy, gyaw = self.goal
                distance = math.hypot(gx-self.x, gy-self.y)
                heading = math.atan2(gy-self.y, gx-self.x) if distance > 0.03 else gyaw
                error = math.atan2(math.sin(heading-self.yaw), math.cos(heading-self.yaw))
                self.yaw_rate = clamp(2*error, -0.6, 0.6)
                self.yaw += self.yaw_rate*dt
                self.speed = min(0.25, distance) * max(0, math.cos(error)) if distance > 0.03 else 0
                self.x += self.speed*math.cos(self.yaw)*dt
                self.y += self.speed*math.sin(self.yaw)*dt
                if distance <= 0.03 and abs(error) < 0.02:
                    self.nav = 'succeeded'
                    self.stop()
        old_pan, old_tilt = self.pan, self.tilt
        rate = 0.7 * dt
        self.pan += clamp(self.pan_cmd-self.pan, -rate, rate)
        self.tilt += clamp(self.tilt_cmd-self.tilt, -rate, rate)
        motion = math.hypot(self.pan-old_pan, self.tilt-old_tilt) / dt
        if target_id == self.c.dropout_target and self.nav == 'succeeded':
            if self.target_seen_at is None:
                self.target_seen_at = self.now
            if self.fault_started is None and self.now-self.target_seen_at >= self.c.dropout_after:
                self.fault_started = self.now
        in_fault = (self.scenario in ('dropout','frozen-camera','permanent-dropout') and
                    self.fault_started is not None and
                    (self.scenario == 'permanent-dropout' or self.now-self.fault_started < self.c.dropout_seconds))
        if in_fault != self.fault_active:
            self.faults.append({'time': self.now, 'active': in_fault, 'type': self.scenario})
            self.fault_active = in_fault
        if in_fault:
            if self.scenario != 'frozen-camera':
                self.frame = None
        else:
            boxes = {}
            for t in self.c.targets:
                tx, ty, tz = t['world']
                dx, dy = tx-self.x, ty-self.y
                distance = math.hypot(dx,dy)
                # Abstract pan positive right; tilt positive up.
                az = math.atan2(dy,dx)-self.yaw+self.pan
                el = math.atan2(tz-0.25, distance)-self.tilt
                if abs(az) > math.radians(40) or abs(el) > math.radians(30):
                    continue
                x = -math.tan(az)/math.tan(math.radians(40))
                y = -math.tan(el)/math.tan(math.radians(30))
                x += self.rng.gauss(0,0.0005)
                y += self.rng.gauss(0,0.0005)
                w = 0.13 / max(0.1, 2*distance*math.tan(math.radians(40)))
                h = 0.13 / max(0.1, 2*distance*math.tan(math.radians(30)))
                boxes[t['id']] = [x,y,w,h]
            sharpness = 0.1 if self.scenario == 'blurred-camera' else max(0, 1-motion*0.8)
            self.frame = Frame(self.now, boxes, sharpness, png_bytes(rgb_frame(boxes,target_id)))
        return Observation(self.now,self.nav,self.speed,self.yaw_rate,0.0,self.frame,(self.x,self.y,self.yaw))
