import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from ugv_inspection.core import Config, Frame, Mission, Observation
from ugv_inspection.cli import run, ROOT
from ugv_inspection.sim import Simulator
from ugv_inspection.render import png_bytes, rgb_frame


class FakeBackend:
    def __init__(self):
        self.commands = []
        self.stops = 0
    def start_navigation(self,goal): pass
    def point(self,p,t): self.commands.append((p,t))
    def stop(self): self.stops += 1


class MissionTests(unittest.TestCase):
    def config(self,**kw):
        data = json.loads((ROOT/'config/panel.json').read_text())
        data.update(kw)
        return Config(**data)

    def scenario(self,name):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)/'run'
            result = run(self.config(),out,name,7)
            self.assertTrue((out/'replay.html').exists())
            self.assertTrue((out/'report.json').exists())
            for c in result['captures']:
                self.assertTrue((out/c['file']).read_bytes().startswith(b'\x89PNG'))
            return result

    def test_nominal_complete_unique_views(self):
        r = self.scenario('nominal')
        self.assertTrue(r['success'])
        self.assertEqual([c['target'] for c in r['captures']],[0,1,2])
        for c in r['captures']:
            self.assertLessEqual(c['center_error'],0.06)
            self.assertGreaterEqual(c['hold_seconds']+1e-9,0.7)

    def test_dropout_recovers_without_capturing_missing_frames(self):
        r = self.scenario('dropout')
        self.assertTrue(r['success'])
        start,end = [f['time'] for f in r['faults']]
        self.assertTrue(any(e.get('event')=='reacquired' for e in r['events']))
        self.assertFalse(any(start<=c['time']<end for c in r['captures']))
        self.assertGreater(next(c['time'] for c in r['captures'] if c['target']==1),end+0.6)

    def test_frozen_frame_recovers(self):
        r = self.scenario('frozen-camera')
        self.assertTrue(r['success'])
        start,end = [f['time'] for f in r['faults']]
        self.assertFalse(any(start<=c['time']<end for c in r['captures']))

    def test_permanent_dropout_fails_with_partial_evidence(self):
        r = self.scenario('permanent-dropout')
        self.assertFalse(r['success'])
        self.assertEqual(r['failure_reason'],'target_timeout')
        self.assertEqual([c['target'] for c in r['captures']],[0])

    def test_nav_failure_never_captures(self):
        r = self.scenario('navigation-failure')
        self.assertFalse(r['success'])
        self.assertEqual(r['failure_reason'],'navigation_failed')
        self.assertEqual(r['captures'],[])

    def test_blur_prevents_capture(self):
        r = self.scenario('blurred-camera')
        self.assertFalse(r['success'])
        self.assertEqual(r['captures'],[])

    def test_moving_base_blocks_capture_and_stops(self):
        b = FakeBackend()
        with tempfile.TemporaryDirectory() as tmp:
            m = Mission(self.config(),b,tmp)
            for i in range(30):
                t = i/10
                m.step(Observation(t,'succeeded',0.1,0,0,Frame(t,{0:[0,0,0.1,0.1]},1,b'pixels')))
            self.assertEqual(m.captures,[])
            self.assertGreater(b.stops,1)

    def test_repeated_frames_do_not_satisfy_stability(self):
        b = FakeBackend()
        with tempfile.TemporaryDirectory() as tmp:
            m = Mission(self.config(frame_max_age=10),b,tmp)
            f = Frame(0,{0:[0,0,0.1,0.1]},1,b'pixels')
            for i in range(30):
                m.step(Observation(i/10,'succeeded',0,0,0,f))
            self.assertEqual(m.captures,[])

    def test_stale_odometry_blocks_capture(self):
        b = FakeBackend()
        with tempfile.TemporaryDirectory() as tmp:
            m = Mission(self.config(),b,tmp)
            for i in range(30):
                t = i/10
                m.step(Observation(t,'succeeded',0,0,1,Frame(t,{0:[0,0,0.1,0.1]},1,b'pixels')))
            self.assertEqual(m.captures,[])

    def test_limits_bound_commands(self):
        b = FakeBackend()
        with tempfile.TemporaryDirectory() as tmp:
            m = Mission(self.config(),b,tmp)
            for _ in range(100): m.command(100,-100)
            self.assertAlmostEqual(m.pan,0.85)
            self.assertAlmostEqual(m.tilt,-0.65)
            previous = (0,0)
            for pair in b.commands:
                self.assertLessEqual(max(abs(x-y) for x,y in zip(pair,previous)),0.060000001)
                previous = pair

    def test_invalid_config_rejected(self):
        for kw in ({'dt':0},{'pan_limit':float('nan')},{'targets':[]},{'goal':[0,float('inf'),0]}):
            with self.assertRaises(ValueError): self.config(**kw)

    def test_output_not_overwritten(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(FileExistsError): run(self.config(),Path(tmp))

    def test_interrupt_stops_backend_and_writes_report(self):
        b = Simulator(self.config())
        with tempfile.TemporaryDirectory() as tmp, patch.object(b,'tick',side_effect=KeyboardInterrupt):
            r = run(self.config(),Path(tmp)/'run',backend=b)
            self.assertTrue(b.stopped)
            self.assertEqual(r['failure_reason'],'interrupted')

    def test_deterministic_seed(self):
        c = self.config()
        a,b = Simulator(c,seed=123),Simulator(c,seed=123)
        a.start_navigation(c.goal); b.start_navigation(c.goal)
        for _ in range(20):
            self.assertEqual(a.tick(c.dt,0),b.tick(c.dt,0))


if __name__ == '__main__':
    unittest.main()
