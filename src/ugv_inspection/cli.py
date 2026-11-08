import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time

from .core import Config, Mission
from .report import write_report
from .sim import SCENARIOS, Simulator
from .render import WIDTH, HEIGHT, rgb_frame

ROOT = Path(__file__).resolve().parents[2]


def run(config, output, scenario='dropout', seed=42, backend=None, realtime=False):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    (output/'config.json').write_text(json.dumps(asdict(config),indent=2)+'\n')
    b = backend or Simulator(config, scenario, seed)
    mission = None
    try:
        mission = Mission(config,b,output)
        while not mission.done:
            started = time.monotonic()
            observation = b.tick(config.dt,mission.target_id)
            mission.step(observation)
            if mission.telemetry:
                mission.telemetry[-1].update(state=mission.state, captures=len(mission.captures))
            if realtime:
                time.sleep(max(0,config.dt-(time.monotonic()-started)))
    except KeyboardInterrupt:
        if mission:
            mission.fail(mission.telemetry[-1]['time'] if mission.telemetry else 0,'interrupted')
    except Exception:
        if mission:
            mission.fail(mission.telemetry[-1]['time'] if mission.telemetry else 0,'backend_exception')
        raise
    finally:
        try:
            b.close()
        finally:
            if mission:
                report = mission.report()
                report.update(backend='ros2' if backend else 'simulation',scenario=scenario,seed=seed,
                              faults=getattr(b,'faults',[]),python=sys.version.split()[0])
                write_report(output, report, mission.telemetry)
    return report


def record(run_dir):
    """Render an explicitly synthetic observation replay to MP4 with system ffmpeg."""
    run_dir = Path(run_dir)
    if json.loads((run_dir/'report.json').read_text())['backend'] != 'simulation':
        raise ValueError('MP4 renderer is for synthetic demos; hardware captures are preserved as PNGs')
    ffmpeg = shutil.which('ffmpeg')
    if not ffmpeg:
        raise RuntimeError('Install system ffmpeg to use record (apt install ffmpeg / brew install ffmpeg)')
    output, partial = run_dir/'simulation.mp4', run_dir/'simulation.partial.mp4'
    if output.exists() or partial.exists():
        raise FileExistsError('Video already exists; choose a new demo run')
    config = json.loads((run_dir/'config.json').read_text())
    command = [ffmpeg,'-v','error','-n','-f','rawvideo','-pix_fmt','rgb24','-s',f'{WIDTH}x{HEIGHT}',
               '-r',str(1/config['dt']),'-i','-','-an','-c:v','libx264','-pix_fmt','yuv420p',
               '-movflags','+faststart',str(partial)]
    try:
        with subprocess.Popen(command,stdin=subprocess.PIPE) as process:
            try:
                for line in (run_dir/'telemetry.jsonl').read_text().splitlines():
                    row = json.loads(line)
                    process.stdin.write(rgb_frame(row['boxes'],row['target'],row['fresh']))
            finally:
                process.stdin.close()
            if process.wait() != 0:
                raise RuntimeError('ffmpeg failed')
        partial.rename(output)
    except BaseException:
        partial.unlink(missing_ok=True)
        raise
    return output


def main():
    p = argparse.ArgumentParser(description='Scripted rover inspection; simulation by default')
    subs = p.add_subparsers(dest='command',required=True)
    demo = subs.add_parser('demo')
    demo.add_argument('--scenario',choices=SCENARIOS,default='dropout')
    demo.add_argument('--seed',type=int,default=42)
    demo.add_argument('--config',type=Path,default=ROOT/'config/panel.json')
    demo.add_argument('--output',type=Path)
    demo.add_argument('--realtime',action='store_true')
    rec = subs.add_parser('record',help='Make a synthetic MP4 replay of a completed simulation run')
    rec.add_argument('run_dir',type=Path)
    bench = subs.add_parser('benchmark')
    bench.add_argument('--seeds',type=int,default=5)
    bench.add_argument('--output',type=Path,required=True)
    bench.add_argument('--config',type=Path,default=ROOT/'config/panel.json')
    args = p.parse_args()
    if args.command == 'record':
        print(record(args.run_dir))
        return 0
    if args.command == 'benchmark':
        if args.seeds <= 0:
            p.error('--seeds must be positive')
        args.output.mkdir(parents=True,exist_ok=False)
        results = []
        for scenario in SCENARIOS:
            for seed in range(args.seeds):
                out = args.output/f'{scenario}-{seed}'
                r = run(Config.load(args.config),out,scenario,seed)
                expected = scenario in ('nominal','dropout','frozen-camera')
                results.append({'scenario':scenario,'seed':seed,'success':r['success'],
                                'expected_success':expected,'as_expected':r['success']==expected,
                                'failure_reason':r['failure_reason']})
        summary = {'runs':results,'all_as_expected':all(r['as_expected'] for r in results)}
        (args.output/'benchmark.json').write_text(json.dumps(summary,indent=2)+'\n')
        print(json.dumps(summary,indent=2))
        return 0 if summary['all_as_expected'] else 1
    output = args.output or ROOT/'runs'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    r = run(Config.load(args.config),output,args.scenario,args.seed,realtime=args.realtime)
    print(f"{r['state']}: {len(r['captures'])}/{len(r['required_targets'])} views")
    print(f"Replay: {(output/'replay.html').resolve()}")
    print(f"Report: {(output/'report.json').resolve()}")
    return 0 if r['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
