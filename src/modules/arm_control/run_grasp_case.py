#!/usr/bin/env python3
"""Sim-time experiment runner. 'complete' means execution, never grasp success."""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shlex
import signal
import socket
import subprocess as sp
import time
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
ABORTS = {2: 'contact_precondition_failed', 3: 'support_removal_unconfirmed',
          4: 'invalid_probe_configuration', 5: 'startup_not_ready',
          6: 'sim_clock_stalled_or_missing', 7: 'sim_time_reversed',
          8: 'nonfinite_joint_state', 9: 'joint_feedback_stale',
          10: 'sim_command_deadline_missed'}

def read_rows(path):
    try:
        with path.open() as stream:
            rows = []
            for row in csv.DictReader(stream):
                try:
                    if None in row or any(v is None for v in row.values()):
                        continue  # Incomplete last line while the probe is writing.
                    rows.append({k: float(v) for k, v in row.items()})
                except (ValueError, TypeError):
                    continue
            return rows
    except FileNotFoundError:
        return []

def summarize(rows, code, log, seconds):
    events = [{'event': m[0], 'sim_s': float(m[1]), 'elapsed_sim_s': float(m[2])}
              for m in re.findall(r'EVENT (\w+) sim=([\d.eE+-]+) elapsed=([\d.eE+-]+)', log)]
    complete = (code == 0 and bool(rows)
                and rows[-1].get('elapsed_sim_s', -1) >= seconds
                and any(e['event'] == 'completed' for e in events)
                and any(e['event'] == 'release_confirmed' for e in events))
    monotonic = all(b['elapsed_sim_s'] > a['elapsed_sim_s'] for a, b in zip(rows, rows[1:]))
    complete = complete and monotonic
    result = dict(exit_code=code, complete=complete, events=events,
                  execution_status='completed' if complete else ABORTS.get(code, 'incomplete'),
                  grasp_status='not_evaluated', sim_rows_monotonic=monotonic)
    if not rows:
        return result
    result.update(elapsed_sim_s=rows[-1].get('elapsed_sim_s'), measured_wall_s=rows[-1]['wall_s'])
    last_row = rows[-1]
    if 'clock_reordered_messages' in last_row:
        result['clock_reordered_messages'] = int(last_row['clock_reordered_messages'])
    if 'contact_reordered_messages' in last_row:
        result['contact_reordered_messages'] = int(last_row['contact_reordered_messages'])
    reference = [r for r in rows if r.get('elapsed_sim_s', 0) < 20 and r.get('pose_seen') == 1]
    settled = [r for r in rows if r.get('elapsed_sim_s', 0) >= 25 and r.get('pose_seen') == 1]
    if reference and settled:
        initial, first, last = reference[-1], settled[0], rows[-1]
        def relative(r):
            return [r['cube_x'], r['cube_y'], r['cube_z']-r['base_z']]
        displacement = [1000*math.dist(relative(initial), relative(r)) for r in settled]
        result.update(relative_displacement_mm=max(displacement),
                      drop_mm=1000*(relative(initial)[2]-relative(last)[2]),
                      late_drift_mm=1000*math.dist(relative(first), relative(last)),
                      final_roll_deg=math.degrees(last['roll']),
                      final_pitch_deg=math.degrees(last['pitch']),
                      final_yaw_deg=math.degrees(last['yaw']),
                      last_bilateral=last['bilateral_frames']/max(1, last['contact_frames']),
                      joint_finite=all(r['finite']==1 for r in rows))
        # A gross-loss diagnostic only, not a holding-tolerance acceptance policy.
        if max(displacement) > 50:
            result['grasp_status'] = 'object_loss_observed'
    return result

def stop(process):
    if process is not None and process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except sp.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    p = argparse.ArgumentParser()
    p.add_argument('name')
    p.add_argument('--iterations', type=int, default=40)
    p.add_argument('--seed', type=int, default=123)
    p.add_argument('--seconds', type=int, default=60, help='Simulation seconds, not wall seconds')
    p.add_argument('--attitude', type=int, choices=range(7), default=0)
    p.add_argument('--pad-pitch', type=float, default=0.)
    p.add_argument('--port', type=int, default=11355)
    p.add_argument('--disturb', action='store_true')
    p.add_argument('--carry', action='store_true')
    p.add_argument('--sor', type=float, default=1.3)
    p.add_argument('--solver', choices=['quick', 'world'], default='quick')
    p.add_argument('--real-time-rate', type=float, default=1000., help='Physics updates per wall second; physical step stays 1 ms')
    p.add_argument('--wall-timeout', type=float, default=None)
    p.add_argument('--stall-seconds', type=float, default=15.)
    p.add_argument('--pause-at', type=float, default=None, help='Infrastructure test: pause after this elapsed sim time')
    p.add_argument('--pause-wall-seconds', type=float, default=2.)
    p.add_argument('--keep-build-artifacts', action='store_true',
                   help='Keep per-case compiled probe and .so files; default removes them after the run')
    args = p.parse_args()
    if not args.name.replace('_','').isalnum():
        p.error('name must contain only letters, digits or underscores')
    if not 45 <= args.seconds <= 600 or not 1024 <= args.port <= 65535:
        p.error('seconds must be 45..600 and port must be 1024..65535')
    if args.disturb and args.seconds < 115 or args.carry and args.seconds < 85:
        p.error('disturbance needs >=115 sim seconds; carry needs >=85')
    if not math.isfinite(args.real_time_rate) or args.real_time_rate <= 0:
        p.error('real-time-rate must be finite and positive')
    if not math.isfinite(args.stall_seconds) or args.stall_seconds < 2:
        p.error('stall-seconds must be finite and >=2')
    if args.pause_at is not None and not 0 < args.pause_at < args.seconds:
        p.error('pause-at must lie inside the simulation interval')
    if not math.isfinite(args.pause_wall_seconds) or args.pause_wall_seconds <= 0:
        p.error('pause-wall-seconds must be finite and positive')
    if args.wall_timeout is not None and (not math.isfinite(args.wall_timeout) or args.wall_timeout <= 0):
        p.error('wall-timeout must be finite and positive')
    folder = ROOT/'build/grasp_validation'/args.name
    if folder.exists():
        p.error('case directory exists; choose a new name to preserve all artifacts')
    # Do not attach to or terminate someone else's Gazebo master.
    with socket.socket() as sock:
        sock.settimeout(.2)
        if sock.connect_ex(('127.0.0.1', args.port)) == 0:
            p.error('Gazebo port is already occupied')
    folder.mkdir(parents=True)
    env = os.environ.copy()
    for key in list(env):
        if key.startswith('SO101_'):
            del env[key]
    env.update(GAZEBO_MASTER_URI=f'http://127.0.0.1:{args.port}',
               GAZEBO_MODEL_PATH=str(ROOT/'Tools/simulation/gazebo-classic/sitl_gazebo-classic/models'),
               GAZEBO_PLUGIN_PATH='/home/pcz/super_ws/devel/lib',
               LD_LIBRARY_PATH='/opt/ros/noetic/lib:/home/pcz/super_ws/devel/lib',
               SO101_TEST_SECONDS=str(args.seconds), SO101_ATTITUDE_CASE=str(args.attitude),
               SO101_STALL_SECONDS=str(args.stall_seconds))
    if args.disturb:
        env['SO101_DISTURB']='1'
    if args.carry:
        env['SO101_CARRY']='1'
    manifest = {'arguments': vars(args), 'clock': 'WorldUpdateBegin/grasp/clock', 'physics_step_s': .001}
    report = dict(complete=False, execution_status='setup_failed', grasp_status='not_evaluated')
    server = probe = None
    exit_code = None
    wall_events = []
    compiled_artifacts = []
    try:
        fixture_args = ['python3', str(HERE/'grasp_fixture.py'), str(folder/'fixture.world'),
                        '--iterations', str(args.iterations), '--pad-pitch', str(args.pad_pitch),
                        '--sor', str(args.sor), '--solver', args.solver]
        if args.disturb:
            fixture_args.append('--load-plugin')
        if args.carry:
            fixture_args.append('--carry')
        sp.run(fixture_args, check=True, timeout=15)
        world = folder/'fixture.world'
        tree = ET.parse(world)
        tree.find('./world/physics/real_time_update_rate').text=str(args.real_time_rate)
        flags = shlex.split(sp.check_output(['pkg-config','--cflags','--libs','gazebo'],text=True,timeout=10))
        probe_binary=folder/'grasp_probe'
        clock_library=folder/'libgrasp_clock.so'
        compiled_artifacts += [probe_binary, clock_library]
        with (folder/'build.log').open('w') as build:
            sp.run(['g++','-std=c++17',str(HERE/'grasp_probe.cpp'),'-o',str(probe_binary)]+flags+['-pthread'],
                   check=True, stdout=build, stderr=sp.STDOUT, timeout=120)
            sp.run(['g++','-std=c++17','-shared','-fPIC','-DSO101_CLOCK_PLUGIN',str(HERE/'grasp_probe.cpp'),'-o',str(clock_library)]+flags+['-pthread'],
                   check=True, stdout=build, stderr=sp.STDOUT, timeout=120)
            ET.SubElement(tree.find('world'),'plugin',name='grasp_clock',filename=str(clock_library))
            if args.disturb or args.carry:
                library=folder/'libgrasp_load.so'
                compiled_artifacts.append(library)
                sp.run(['g++','-std=c++17','-shared','-fPIC',str(HERE/'grasp_load_plugin.cpp'),'-o',str(library)]+flags+['-pthread'],
                       check=True, stdout=build, stderr=sp.STDOUT, timeout=120)
                tree.find("./world/plugin[@name='fixture_load']").set('filename',str(library))
        tree.write(world,encoding='unicode',xml_declaration=True)
        sources = [HERE/'grasp_probe.cpp',HERE/'run_grasp_case.py',HERE/'grasp_fixture.py',
                   world,folder/'grasp_probe',clock_library,
                   ROOT/'Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/so101/so101.sdf',
                   ROOT/'Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/test_cube/model.sdf',
                   Path('/home/pcz/super_ws/devel/lib/libso101_joint_position_controller.so')]
        if args.disturb or args.carry:
            sources += [HERE/'grasp_load_plugin.cpp',folder/'libgrasp_load.so']
        manifest['sha256']={str(path): digest(path) for path in sources}
        version=sp.run(['gzserver','--version'],stdout=sp.PIPE,stderr=sp.STDOUT,text=True,timeout=10)
        manifest['gazebo_version']=version.stdout.strip()
        manifest['gazebo_version_exit_code']=version.returncode
        (folder/'manifest.json').write_text(json.dumps(manifest,indent=2))
        with (folder/'server.log').open('w') as log, (folder/'probe.csv').open('w') as data, (folder/'probe.log').open('w') as errors:
            server=sp.Popen(['gzserver','-u','--seed',str(args.seed),str(world)],env=env,stdout=log,stderr=sp.STDOUT)
            probe=sp.Popen([str(probe_binary)],env=env,stdout=data,stderr=errors)
            started=time.monotonic()
            limit=args.wall_timeout or (30+args.seconds*max(4.,1500/args.real_time_rate))
            pause_started=None
            paused_once=False
            while probe.poll() is None:
                wall=time.monotonic()-started
                if server.poll() is not None:
                    report['execution_status']='server_exited'
                    break
                if wall > limit:
                    report['execution_status']='wall_watchdog_timeout'
                    break
                if args.pause_at is not None:
                    rows=read_rows(folder/'probe.csv')
                    if not paused_once and rows and rows[-1].get('elapsed_sim_s',0)>=args.pause_at:
                        sp.run(['gz','world','-p','1'],env=env,check=True,timeout=5)
                        pause_started=time.monotonic()
                        paused_once=True
                        wall_events.append(dict(event='pause_requested',wall_s=wall,elapsed_sim_s=rows[-1]['elapsed_sim_s']))
                    if pause_started is not None and time.monotonic()-pause_started>=args.pause_wall_seconds:
                        sp.run(['gz','world','-p','0'],env=env,check=True,timeout=5)
                        pause_started=None
                        wall_events.append(dict(event='resume_requested',wall_s=time.monotonic()-started))
                time.sleep(.05)
            exit_code=probe.poll()
        stop(probe)
        if exit_code is None:
            report['exit_code']=probe.returncode
        else:
            report=summarize(read_rows(folder/'probe.csv'),exit_code,(folder/'probe.log').read_text(),args.seconds)
    except Exception as error:
        report.update(execution_status='infrastructure_error',error=str(error))
    finally:
        stop(probe)
        stop(server)
        removed = []
        if not args.keep_build_artifacts:
            for artifact in compiled_artifacts:
                try:
                    if artifact.exists():
                        artifact.unlink()
                        removed.append(artifact.name)
                except OSError as error:
                    report.setdefault('cleanup_warnings', []).append(f'{artifact}: {error}')
        report.update(arguments=vars(args),wall_control_events=wall_events,
                      removed_build_artifacts=removed)
        (folder/'summary.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(report),flush=True)
    if not report['complete']:
        raise SystemExit(1)

if __name__=='__main__':
    main()
