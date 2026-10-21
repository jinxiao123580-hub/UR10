#!/usr/bin/env python3
"""Local-only, read-only force/UR recording; optionally wrap an experiment.

--run COMMAND ... starts COMMAND only after both streams have arrived.
Recording does not tare sensors, send robot commands, or upload data.
Source clocks are preserved, not assumed synchronized.
"""
import argparse
import csv
from contextlib import ExitStack
import datetime as dt
import json
import math
from pathlib import Path
import signal
import socket
import struct
import subprocess
import threading
import time
from dataset_motion_storage import MotionStorage


def save(path,data):
    temporary=path.with_suffix('.tmp')
    temporary.write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    temporary.replace(path)


def frames(sock,stop):
    """Yield complete frames; preserve stream order and packet boundaries."""
    buffer=bytearray()
    while not stop.is_set():
        try:
            chunk=sock.recv(65536)
        except socket.timeout:
            continue
        if not chunk:
            raise ConnectionError('UR stream closed')
        buffer.extend(chunk)
        while len(buffer)>=4:
            size=struct.unpack_from('>I',buffer)[0]
            if not 540<=size<=4096:
                raise ValueError('invalid UR realtime frame length %d'%size)
            if len(buffer)<size:
                break
            frame=bytes(buffer[:size]);del buffer[:size]
            yield frame


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--name',default='experiment')
    p.add_argument('--host',default='192.168.1.3')
    p.add_argument('--port',type=int,default=30013)
    p.add_argument('--topic',default='/ft_sensor/wrench_raw')
    p.add_argument('--output-root',type=Path,default=Path(__file__).resolve().parents[1]/'outputs/experiment_datasets')
    p.add_argument('--duration',type=float,default=60,help='standalone recording seconds; wrapped runs continue until experiment exits')
    p.add_argument('--collection',type=Path,help='optional session JSON to watch; finish after session completes')
    p.add_argument('--idle-seconds',type=float,default=30,help='pause disk recording after this long without motion')
    p.add_argument('--pre-seconds',type=float,default=2,help='bounded pre-motion buffer')
    p.add_argument('--continuous',action='store_true',help='disable idle pause for static drift experiments')
    p.add_argument('--raw-frames',action='store_true',help='also retain gzip binary UR frames; duplicates CSV information')
    p.add_argument('--watch',action='store_true',help='standalone: keep listening until Ctrl+C, pause/resume on motion')
    p.add_argument('--run',nargs=argparse.REMAINDER,help='experiment argv, not a shell command; must be LAST option')
    args=p.parse_args()
    if not math.isfinite(args.duration) or args.duration<=0:
        p.error('duration must be positive and finite')
    if not math.isfinite(args.idle_seconds) or args.idle_seconds<1 or not math.isfinite(args.pre_seconds) or not 0<=args.pre_seconds<=10:
        p.error('idle-seconds must be >=1; pre-seconds 0..10')
    if args.run==[]:
        p.error('--run needs an experiment command')
    if args.run and args.collection:
        p.error('--run and --collection are separate recording modes')
    if Path(args.name).name!=args.name or args.name in ['.','..']:
        p.error('name must be a simple filename')
    folder=args.output_root/(dt.datetime.now().strftime('%Y%m%d-%H%M%S-%f')+'-'+args.name)
    folder.mkdir(parents=True,exist_ok=False)
    print('LOCAL DATASET:',folder,flush=True)
    meta={'schema_version':1,'status':'starting','name':args.name,'host':args.host,'port':args.port,'topic':args.topic,
          'started_at':dt.datetime.now().astimezone().isoformat(),'experiment_argv':args.run,
          'cwd':str(Path.cwd()),'local_only':True,'source_clocks_synchronized':False,
          'timestamps':'receipt wall/monotonic; ROS header source; UR controller uptime preserved separately',
          'frames':'force frame_id from ROS; UR TCP in controller base, active TCP not removed',
          'units':{'force':'N','torque':'Nm','joint':'rad','position':'m','rotvec':'rad','time':'s'},
          'force_rows':0,'ur_rows':0,'ur_connections':0,'errors':[],
          'coverage':'begins at recorder start; no reconstruction of earlier data'}
    save(folder/'metadata.json',meta)
    import rclpy
    from geometry_msgs.msg import WrenchStamped
    rclpy.init();node=rclpy.create_node('experiment_dataset_recorder')
    stop=threading.Event();child=None
    receipt={'force':None,'ur':None};robot_thread=None
    def log_error(stream,error,wall):
        meta['error_count']=meta.get('error_count',0)+1
        meta['errors'].append({'stream':stream,'error':str(error),'wall_s':wall})
        del meta['errors'][:-100]
    try:
        with ExitStack() as stack:
            mf=stack.enter_context((folder/'markers.csv').open('w',newline=''))
            log=stack.enter_context((folder/'experiment.log').open('w'))
            mw=csv.writer(mf)
            mw.writerow(['wall_s','monotonic_s','event','value'])
            def mark(event,value):mw.writerow([time.time(),time.monotonic(),event,value]);mf.flush()
            storage=MotionStorage(stack,folder,meta,args.idle_seconds,args.pre_seconds,args.continuous,args.raw_frames,mark)
            def callback(msg):
                w=msg.wrench;values=[w.force.x,w.force.y,w.force.z,w.torque.x,w.torque.y,w.torque.z]
                source=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                wall,mono=time.time(),time.monotonic()
                if not all(math.isfinite(v) for v in values):
                    log_error('force','nonfinite value',wall);return
                storage.accept('force',[wall,mono,source,msg.header.frame_id,*values]);receipt['force']=mono
            sub=node.create_subscription(WrenchStamped,args.topic,callback,200)
            def robot():
                while not stop.is_set():
                    try:
                        with socket.create_connection((args.host,args.port),timeout=2) as sock:
                            sock.settimeout(.2);meta['ur_connections']+=1
                            for frame in frames(sock,stop):
                                wall,mono=time.time(),time.monotonic()
                                values=[*struct.unpack_from('>6d',frame,252),*struct.unpack_from('>6d',frame,444),*struct.unpack_from('>6d',frame,492)]
                                stamp=struct.unpack_from('>d',frame,4)[0]
                                if not all(math.isfinite(v) for v in [stamp,*values]):
                                    raise ValueError('nonfinite UR frame')
                                storage.accept('ur',[wall,mono,stamp,*values],frame);receipt['ur']=mono
                    except Exception as exc:
                        log_error('ur',exc,time.time())
                        stop.wait(.5)
            robot_thread=threading.Thread(target=robot,daemon=True);robot_thread.start()
            def shutdown_writer():
                stop.set();robot_thread.join(timeout=4);storage.finish()
            stack.callback(shutdown_writer)
            started=time.monotonic();last_flush=started;marker=None;launched=False;missing_since=None
            while True:
                rclpy.spin_once(node,timeout_sec=.05)
                now=time.monotonic()
                ready=all(receipt[k] is not None and now-receipt[k]<.25 for k in receipt) and meta['force_received']>=20 and meta['ur_received']>=20
                if args.run and not launched:
                    if ready:
                        child=subprocess.Popen(args.run,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                        launched=True;meta['experiment_pid']=child.pid;mark('experiment_start',json.dumps(args.run));meta['status']='recording'
                        print('两路数据已就绪，已启动实验；输出见 experiment.log',flush=True)
                    elif now-started>15:
                        raise RuntimeError('streams not ready within 15 s; experiment NOT started')
                elif not args.run:
                    meta['status']='recording'
                if child is not None and child.poll() is not None:
                    meta['experiment_exit_code']=child.returncode;mark('experiment_exit',child.returncode);break
                if child is not None:
                    missing_since=None if ready else (missing_since if missing_since is not None else now)
                    if missing_since is not None and now-missing_since>.5:
                        mark('stream_loss','one or both streams stale; interrupting experiment')
                        raise RuntimeError('paired data stream lost; interrupting wrapped experiment, dataset marked incomplete')
                if not args.run and not args.watch and now-started>=args.duration:break
                if args.collection and args.collection.exists():
                    try:
                        d=json.loads(args.collection.read_text());value=(d.get('status'),d.get('pending'))
                        if marker!=value:mark('collection',json.dumps(value));marker=value
                        if value[0] not in ['collecting','preflight'] and now-started>10:break
                    except (OSError,json.JSONDecodeError):pass
                if now-last_flush>=2:
                    storage.flush();save(folder/'metadata.json',meta);last_flush=now
            stop.set();robot_thread.join(timeout=4)
            meta['status']='recorded' if ready and not meta['errors'] else 'partial'
    except BaseException as exc:
        meta['status']='interrupted' if isinstance(exc,KeyboardInterrupt) else 'failed';meta['error']=str(exc)
        if child is not None and child.poll() is None:
            import os
            os.killpg(child.pid,signal.SIGINT)
            try:child.wait(timeout=10)
            except subprocess.TimeoutExpired:meta['experiment_may_still_be_running']=True
        if not isinstance(exc,KeyboardInterrupt):raise
    finally:
        stop.set()
        if robot_thread:robot_thread.join(timeout=4)
        meta['finished_at']=dt.datetime.now().astimezone().isoformat()
        meta['last_received_monotonic_s']=receipt
        save(folder/'metadata.json',meta)
        node.destroy_node();rclpy.shutdown()
        print('SAVED:',folder,'FORCE:',meta['force_rows'],'UR:',meta['ur_rows'],'STATUS:',meta['status'],flush=True)


if __name__=='__main__':main()
