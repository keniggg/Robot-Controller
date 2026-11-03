#!/usr/bin/env python3
"""Explicit camera-only FPS trial. No motion, selection, SDK option or temperature calls.

Only /camera/fps may be changed. SIGINT is limited to identity-bound camera
owners. A failed 15 FPS attempt restores 30 FPS once, after the trial owner has
actually exited. Existing source/quality gates and persistent YAML are untouched.
"""
import argparse
from copy import deepcopy
import hashlib
import http.client
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import xmlrpc.client

W = Path(__file__).resolve().parents[1]
MOUNT = Path('/media/zhuyupei/SanStand')
EXT = MOUNT/'实验记录/Robot-Controller'
CAMERA = '/supervisor_camera'
HELPER = W/'.runtime/gen4_root_cause/reload_camera_visual_gate.py'
LOADER = W/'.runtime/load_regional_production_20261006.py'


def require(ok, message):
    if not ok:
        raise RuntimeError(message)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
    return m


def geometry(camera):
    r = camera['runtime_profile']
    keys = ('device', 'color_intrinsics', 'native_depth_intrinsics',
            'depth_to_color_extrinsics', 'alignment', 'geometry_intrinsics',
            'geometry_intrinsics_source', 'depth_scale_m_per_unit',
            'color_projection_correction', 'rectification')
    require(all(k in r for k in keys), 'complete prior runtime geometry required')
    return {k: deepcopy(r[k]) for k in keys}


def stable_config(camera):
    return {k: deepcopy(v) for k,v in camera.items() if k != 'runtime_profile'}


def validate_advertised(evidence, camera, fps):
    require(evidence.get('serial') == camera['runtime_profile']['device']['serial_number'],
            'advertised profiles belong to another camera')
    require(evidence.get('sdk') == '2.56.5', 'profile evidence SDK mismatch')
    for stream, fmt in (('color', 'bgr8'), ('depth', 'z16')):
        require(any(p.get('stream') == 'stream.'+stream and p.get('format') == 'format.'+fmt
                    and p.get('width') == camera['stream_width']
                    and p.get('height') == camera['stream_height'] and p.get('fps') == fps
                    for p in evidence.get('profiles', [])), 'advertised profile missing: '+stream)


def confirmed_profile(before, after, envelope, fps, *, old_activation, start_ns, now_ns):
    """Return exact newly active SDK evidence, or None while startup is pending."""
    r = after.get('runtime_profile', {})
    activation = r.get('geometry_activation_stamp_ns')
    if not activation or str(activation) == str(old_activation) or int(activation) < start_ns:
        return None
    expected = stable_config(before); expected['fps'] = fps
    require(stable_config(after) == expected, 'camera policy or output geometry changed beyond fps')
    require(geometry(after) == geometry(before), 'active SDK/rectified geometry changed')
    old_preset = before['runtime_profile'].get('mode_depth_preset', {})
    new_preset = r.get('mode_depth_preset', {})
    for key in ('mode', 'preset', 'configured_unknown_preset', 'baseline_sha256'):
        require(key in old_preset and new_preset.get(key) == old_preset[key], 'preset changed: '+key)
    if not isinstance(envelope, dict) or envelope.get('schema') != 1 or envelope.get('encoding') != 'json':
        return None
    p = json.loads(envelope['payload_json'])
    if p.get('schema') != 1 or str(p.get('geometry_activation_stamp_ns')) != str(activation):
        return None
    good = []
    for e in p.get('entries', []):
        ns = e.get('source_stamp_ns')
        if (type(ns) is not int or ns < max(start_ns, int(activation))
                or not 0 <= now_ns-ns <= 2_000_000_000
                or str(e.get('geometry_activation_stamp_ns')) != str(activation)):
            continue
        ev = e.get('evidence', {})
        if ev.get('result') != 'complete' or ev.get('pair_timing', {}).get('accepted') is not True:
            continue
        matched = True
        for stream, fmt in (('color', 'bgr8'), ('depth', 'z16')):
            raw = ev.get('raw_frames', {}).get(stream, {})
            matched &= raw.get('present') is True and raw.get('stream_fps') == fps
        if matched:
            good.append(e)
    return {'geometry_activation_stamp_ns': str(activation), 'exact_read_entries': good[-3:]} if good else None


class CameraPending(RuntimeError):
    pass


class Transport(xmlrpc.client.Transport):
    def make_connection(self, host):
        return http.client.HTTPConnection(host, timeout=1.)


class Rpc:
    def __init__(self):
        self.caller = '/camera_fps_trial_'+str(os.getpid())
        self.master = xmlrpc.client.ServerProxy(os.environ.get('ROS_MASTER_URI', 'http://localhost:11311'), transport=Transport())
    def call(self, method, *args):
        code, text, value = getattr(self.master, method)(self.caller, *args)
        require(code == 1, method+': '+str(text))
        return value
    def get(self, name):
        return self.call('getParam', name)
    def fps(self, value):
        require(value in (15, 30), 'only 15/30 fps supported')
        self.call('setParam', '/camera/fps', value)
        require(self.get('/camera/fps') == value, 'fps write not confirmed')
    def pid(self, name):
        code, detail, uri = self.master.lookupNode(self.caller, name)
        if code != 1 and detail == 'unknown node ['+name+']':
            raise CameraPending(detail)
        require(code == 1, 'lookupNode: '+str(detail))
        code, detail, value = xmlrpc.client.ServerProxy(uri, transport=Transport()).getPid(self.caller)
        require(code == 1, 'getPid: '+str(detail))
        return int(value)


def wait(predicate, seconds, label):
    deadline = time.monotonic()+seconds
    last_error = None
    while time.monotonic() < deadline:
        try:
            result = predicate()
            if result:
                return result
        except (CameraPending, ConnectionError, OSError, xmlrpc.client.Error) as exc:
            last_error = repr(exc)
        time.sleep(.2)
    raise RuntimeError('timeout '+label+'; last_transport_error='+str(last_error))


def transaction(io, fps):
    try:
        io.stop_current()
        io.launch(fps)
        io.confirm(fps)
        io.verify_core()
        io.report['requested_profile_confirmed'] = True
    except BaseException as exc:
        io.report['error'] = repr(exc)
        if fps == 15 and (io.stop_requested or io.leg):
            io.cleanup = True
            io.report['restore_30_attempted'] = True
            try:
                io.stop_current()
                io.launch(30)
                io.confirm(30)
                io.verify_core()
                io.report['restored_30_confirmed'] = True
            except BaseException as recovery:
                io.report['restore_30_error'] = repr(recovery)
        raise


class Runtime:
    def __init__(self, args, h, m):
        self.args, self.h, self.m = args, h, m
        self.rpc = Rpc(); self.current = None; self.cleanup = False; self.stop_requested = False
        self.leg = 0
        self.pending_unit = None; self.start_command_completed = False
        self.signaled = set()
        self.report = dict(schema=1, requested_fps=args.fps, requested_profile_confirmed=False,
            robot_commands_sent=False, torque_commands_sent=False, temperature_queries_sent=False,
            selection_changed=False, persistent_configuration_changed=False,
            new_alignment_confirmation_created=False, motion_authorized=False,
            supplied_authorization_text=args.authorization_text, authorization_message_id=None,
            authorization_scope='camera-only FPS diagnostic; text supplied by invoking parent, no invented user event',
            events=[], limitations=['Profile readiness does not prove image publication, support fit, cross-view fusion, or grasp success.',
                'Wait bounds include one pending 1 s RPC timeout; SDK shutdown may not cooperate, in which case no second owner is started.'])
    def save(self, event):
        self.report['events'].append({'event':event, 'wall_ns':time.time_ns(), 'monotonic':time.monotonic()})
        p=self.args.output/'result.json'; tmp=p.with_suffix('.tmp')
        tmp.write_text(json.dumps(self.report, ensure_ascii=False, sort_keys=True, indent=2)+'\n');tmp.replace(p)
    def prepare(self):
        self.before = self.rpc.get('/camera')
        require(self.before.get('fps') in (15,30), 'current fps must be 15 or 30')
        require(self.before['fps'] != self.args.fps, 'already at requested fps; no replacement needed')
        require(self.before.get('simulate') is False and self.before.get('fallback_to_simulation') is False
            and self.before.get('align_depth_to_color') is True
            and self.before.get('pair_rectification',{}).get('enabled') is True, 'real aligned rectified camera required')
        geometry(self.before)
        require(all(k in self.before['runtime_profile'].get('mode_depth_preset', {})
                    for k in ('mode', 'preset', 'configured_unknown_preset', 'baseline_sha256')),
                'complete current preset evidence required')
        evidence=json.loads(self.args.supported_profiles.read_text())
        for fps in (15,30): validate_advertised(evidence,self.before,fps)
        self.current=self.h.identity(self.rpc.pid(CAMERA)); self.original=deepcopy(self.current)
        self.normal=self.m.camera_command(self.current['argv'],self.h)
        self.environment=self.h.camera_environment(self.current['pid'])
        self.core={name:self.h.identity(self.rpc.pid(name)) for name in self.m.CORE}
        paths=[Path(__file__).resolve(), HELPER, LOADER,
               W/'src/alicia_flexible_grasp_supervisor/scripts/camera_node.py']
        paths+=sorted((W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision').glob('*.py'))
        self.sources={str(p):sha(p) for p in paths}
        self.report.update(camera_parameters_before=self.before,old_camera=self.original,core_identities=self.core,
            source_sha256=self.sources, advertised_profiles_path=str(self.args.supported_profiles),
            advertised_profiles_sha256=sha(self.args.supported_profiles), advertised_profiles=evidence)
        self.save('prepared')
    def verify_core(self):
        for name, old in self.core.items():
            require(self.h.alive(old) and self.rpc.pid(name)==old['pid'], 'core owner changed: '+name)
        for path,digest in self.sources.items(): require(sha(path)==digest, 'source changed: '+path)
        self.report['core_identities_preserved']=True
    def bind_pending_owner(self):
        if self.pending_unit is None or self.current is not None:
            return
        result = subprocess.run(['systemctl', '--user', 'show', self.pending_unit,
            '--property=LoadState,ActiveState,SubState,MainPID'], capture_output=True, text=True, timeout=3)
        values = dict(line.split('=',1) for line in result.stdout.splitlines() if '=' in line)
        pid = int(values.get('MainPID', '0'))
        if pid:
            owner = self.h.identity(pid)
            require(owner['argv'] == self.expected_argv and owner['exe_sha256'] == self.original['exe_sha256'],
                    'new camera executable/argv mismatch')
            self.current = owner
            self.report['legs'][-1]['owner'] = owner
            return
        # Only a completed start client plus definite absent/exited unit proves
        # no future child needs ownership. Timeout with unknown unit is blocked.
        require(self.start_command_completed and (values.get('LoadState') == 'not-found'
                or values.get('ActiveState') in ('inactive', 'failed')),
                'camera service ownership unresolved; refusing a second camera')
        self.pending_unit = None

    def stop_current(self):
        self.bind_pending_owner()
        if self.current is None: return
        owner_key = (self.current['pid'], self.current['start_ticks'])
        if self.h.alive(self.current):
            if owner_key not in self.signaled:
                # An already-bound new systemd child can be stopped even when
                # startup has not registered its ROS API. The old owner must
                # still match the API before its first SIGINT.
                if owner_key == (self.original['pid'], self.original['start_ticks']):
                    require(self.rpc.pid(CAMERA) == self.current['pid'], 'camera ownership changed before SIGINT')
                self.stop_requested = True
                self.save('camera_sigint_intent')
                os.kill(self.current['pid'], signal.SIGINT)
                self.signaled.add(owner_key)
            wait(lambda:not self.h.alive(self.current),12.,'identity-bound camera exit')
        self.save('camera_owner_exited')
        self.current = None; self.pending_unit = None
    def launch(self, fps):
        require(self.current is None, 'camera must exit before profile change')
        # A stale master URI is harmless; an unrelated live owner is not.
        try: other=self.rpc.pid(CAMERA)
        except (CameraPending,ConnectionRefusedError,ConnectionResetError): other=None
        require(other is None, 'another camera is live: '+str(other))
        self.verify_core()
        self.rpc.fps(fps); self.report['current_requested_fps']=fps
        self.leg+=1; folder=self.args.output/('leg%d_%dfps'%(self.leg,fps));folder.mkdir();(folder/'ros').mkdir()
        self.start_ns=time.time_ns(); self.old_activation=self.rpc.get('/camera')['runtime_profile']['geometry_activation_stamp_ns']
        self.unit='alicia-camera-fps-%d-%d-%d'%(os.getpid(),self.start_ns,self.leg)
        tail=[x for x in self.normal[2:] if not x.startswith('__log:=')]+['__log:='+str(folder/'camera.ros.log')]
        self.expected_argv=self.normal[:2]+tail
        command=['systemd-run','--user','--collect','--unit='+self.unit,'--property=Type=exec','--property=Restart=no',
            '--property=WorkingDirectory='+self.original['cwd'],
            '--property=StandardOutput=append:'+str(folder/'camera.stdout.log'),
            '--property=StandardError=append:'+str(folder/'camera.stderr.log')]
        for key,value in self.environment.items():command.append('--setenv='+key+'='+value)
        command+=['--setenv=ROS_LOG_DIR='+str(folder/'ros'),'--setenv=PYTHONUNBUFFERED=1','--setenv=PYTHONDONTWRITEBYTECODE=1']+self.expected_argv
        self.report.setdefault('legs',[]).append({'fps':fps,'start_ns':self.start_ns,'unit':self.unit,'command':command})
        self.pending_unit=self.unit; self.start_command_completed=False
        self.save('fps_configured_starting_camera')
        try:
            result=subprocess.run(command,capture_output=True,text=True,timeout=10)
            self.start_command_completed=True
            (folder/'systemd_start.txt').write_text(result.stdout+result.stderr)
            require(result.returncode==0,'camera systemd start failed')
        finally:
            self.bind_pending_owner()
        require(self.current is not None,'camera service did not start')
        self.save('new_camera_started')
    def confirm(self, fps):
        def ready():
            require(self.current is not None and self.h.alive(self.current),'new camera exited before profile confirmation')
            if self.rpc.pid(CAMERA)!=self.current['pid']:return None
            after=self.rpc.get('/camera')
            proof=confirmed_profile(self.before,after,self.rpc.get('/camera_read_diagnostics'),fps,
                old_activation=self.old_activation,start_ns=self.start_ns,now_ns=time.time_ns())
            if proof:
                self.report['legs'][-1].update(profile_confirmation=proof,camera_parameters_after=after)
            return proof
        wait(ready,25.,'new actual SDK profile and unchanged geometry')
        self.save('actual_profile_confirmed')


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output',type=Path,required=True);ap.add_argument('--fps',type=int,choices=(15,30),required=True)
    ap.add_argument('--authorization-text',required=True)
    ap.add_argument('--supported-profiles',type=Path)
    a=ap.parse_args()
    require(a.authorization_text.strip() and len(a.authorization_text)<=8192,'literal authorization text required')
    require(MOUNT.is_mount(),'SanStand unavailable; no fallback')
    a.output=a.output.resolve();require(EXT.resolve() in a.output.parents,'output must be a new external directory')
    a.supported_profiles=(a.supported_profiles or a.output.parent/'supported_profiles.json').resolve()
    require(EXT.resolve() in a.supported_profiles.parents and a.supported_profiles.is_file(),'external profile evidence required')
    a.output.mkdir(parents=True,exist_ok=False)
    h=load('fps_camera_owner',HELPER);m=load('fps_camera_policy',LOADER)
    io=Runtime(a,h,m);rc=2
    def interrupted(sig,_frame):
        io.report.setdefault('signals',[]).append(sig)
        if not io.cleanup: raise RuntimeError('interrupted by signal '+str(sig))
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,interrupted)
    try:
        io.prepare();transaction(io,a.fps);rc=0
    except BaseException as exc:
        io.report['error']=repr(exc)
    finally:
        io.save('finished')
        (a.output/'SHA256SUMS').write_text(sha(a.output/'result.json')+'  result.json\n'+sha(__file__)+'  '+str(Path(__file__).resolve())+'\n')
        print(json.dumps({'output':str(a.output),'requested_profile_confirmed':io.report['requested_profile_confirmed'],
              'restored_30_confirmed':io.report.get('restored_30_confirmed',False),'error':io.report.get('error')}),flush=True)
    return rc


if __name__=='__main__':raise SystemExit(main())
