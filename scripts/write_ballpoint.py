#!/usr/bin/env python3
"""Manual A4 registration and conservative fixed-height ballpoint writing.

Default: offline plan only. --teach records three corners without motion.
--execute enables bounded moves; --air adds 5 mm to all pen-down points.
26.968 mm is a user-supplied horizontal-plane approximation, not a fit.
"""
import argparse
import datetime as dt
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation
import probe_paper_height as probe
from calibrate_pen_tcp import capture, save
from calibrate_paper_plane_auto import tcp_target, run_contact

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'outputs/paper_writing'
TCP = ROOT / 'outputs/tcp_calibration/ballpoint-pen-20261016.json'
STROKES = [
    [(107.348,144.103),(116.255,142.144),(123.916,142.322),
     (145.828,145.172),(181.101,152.120),(191.612,151.764)],
    [(141.018,139.293),(149.747,133.236),(146.006,73.735),(142.265,70.350)],
    [(73.500,61.977),(80.091,59.127),(87.217,57.880),
     (130.151,63.403),(206.932,68.747),(223.500,62.690)],
]


def paper_axes(corners):
    p = np.asarray(corners, dtype=float)
    if p.shape != (3,3) or not np.isfinite(p).all():
        raise ValueError('three finite pen-tip corner coordinates required')
    x, y = p[1,:2]-p[0,:2], p[2,:2]-p[0,:2]
    widths = [float(np.linalg.norm(x)), float(np.linalg.norm(y))]
    if abs(widths[0]-.297) > .010 or abs(widths[1]-.210) > .010:
        raise ValueError('角点距离不符合横向 A4 的 297×210 mm；请核对示教顺序')
    x, y = x/widths[0], y/widths[1]
    if abs(float(x@y)) > math.sin(math.radians(3)):
        raise ValueError('纸面 X/Y 不垂直（超过 3°）')
    if x[0]*y[1]-x[1]*y[0] <= 0:
        raise ValueError('角点顺序反向：必须左下、右下、左上，从纸面上方观察')
    # Retain rigid metric coordinates: never stretch the supplied glyph.
    y = np.array([-x[1], x[0]])
    return p[0,:2], np.column_stack((x,y)), widths


def unordered_corners(corners):
    """Infer a landscape A4 rectangle from any three distinct vertices.

    Unmarked paper has a 180-degree ambiguity. Choose its long axis toward
    base +X (toward +Y for an effectively vertical long edge).
    """
    p = np.asarray(corners,dtype=float)
    if p.shape != (3,3) or not np.isfinite(p).all():
        raise ValueError('需要三个有限角点')
    candidates = []
    for i in range(3):
        other = [j for j in range(3) if j != i]
        v = p[other,:2]-p[i,:2]
        lengths = np.linalg.norm(v,axis=1)
        if min(lengths) < .001:
            continue
        cosine = abs(float(v[0]@v[1]/np.prod(lengths)))
        if cosine <= math.sin(math.radians(3)) and np.allclose(sorted(lengths),[.210,.297],atol=.010,rtol=0):
            candidates.append((i,other,v,lengths))
    if len(candidates) != 1:
        raise ValueError('三个点不能唯一组成横向 A4 的三个角；请勿重复或选边中点')
    i,other,v,lengths = candidates[0]
    x = v[int(np.argmax(lengths))]/max(lengths)
    if x[0] < -1e-6 or (abs(x[0]) <= 1e-6 and x[1] < 0):
        x = -x
    y = np.array([-x[1],x[0]])
    xy = np.vstack((p[:,:2],p[other[0],:2]+p[other[1],:2]-p[i,:2]))
    axes = np.column_stack((x,y))
    local = xy @ axes
    origin = axes @ local.min(axis=0)
    # Use observed side lengths, not an affine stretch of the glyph.
    ordered_xy = [origin,origin+x*max(lengths),origin+y*min(lengths)]
    ordered = [[*a,float(np.mean(p[:,2]))] for a in ordered_xy]
    paper_axes(ordered)
    return ordered


def make_plan(reg, z_mm, air):
    origin, axes, widths = paper_axes(reg['corners_pen_tip_base_m'])
    points = np.asarray([p for s in STROKES for p in s])
    if np.any(points < 0) or np.any(points > [297,210]):
        raise ValueError('trajectory outside A4')
    area = float(np.prod(np.ptp(points,axis=0))/(297*210))
    if area > .25:
        raise ValueError('glyph bounding rectangle exceeds one quarter of paper')
    z = z_mm/1000 + (.005 if air else 0)
    strokes = [[[* (origin + axes @ (np.asarray(p)/1000)), z] for p in s] for s in STROKES]
    return {'kind':'fixed_height_ballpoint', 'user_plane_z_mm':z_mm,
            'plane_is_unvalidated_horizontal_approximation':True,
            'air_test':air, 'paper_size_mm':[297,210], 'corner_spans_m':widths,
            'bounding_area_ratio':area, 'strokes_paper_xy_mm':STROKES,
            'strokes_pen_tip_base_m':strokes,
            'lift_mm':5., 'overload_delta_n':5., 'tcp_sha256':reg['tcp_sha256']}


def writing_state(reader, force, baseline, host, endpoint, script, recovery, air):
    if air:
        return probe.motion_state(reader,force,baseline,host,endpoint,script,recovery)
    # No automatic retry while writing: an interruption may alter a stroke.
    return probe.fresh(reader,max_wait_seconds=.25)


def read_stationary(reader,host,allow_reconnect=True):
    """Retry a silent idle socket, never accept cached state or send a move."""
    for attempt in range(3):
        try:
            return probe.stationary_state(reader)
        except (TimeoutError,ConnectionError,OSError):
            probe.stop(host)
            if not allow_reconnect or attempt==2:
                raise
            reader.connect(timeout_seconds=2)
            print('静止预检未收到新状态：已发送停止并重连，重新采集停稳窗口（%d/2）。' % (attempt+1),flush=True)


def motion_parameters(on_paper, horizontal=False):
    # 2 mm air steps remain within the 2.5 mm reconnect envelope.
    # Contact motion retains 1 mm finite steps and gentler acceleration.
    if horizontal:
        return (.030,.100,.002)
    return (.003,.005,.001) if on_paper else (.010,.020,.002)


def unload_pen(reader,force,baseline,host,offset,entry,remaining):
    """Conservative upward-only unloading, not calibrated normal-force control.

    Start below the hard 5 N gate; stop lateral movement before any lift.
    Never tare under contact, never lower to chase missing ink.
    """
    probe.stop(host)
    lifted=0.
    while probe.force_level(force,baseline)>=1.5:
        if lifted+.0001>remaining+1e-9:
            raise RuntimeError('抬笔卸力达到 1 mm 总上限，需重新标定纸面；禁止继续写字')
        q,tcp,_=probe.stationary_state(reader)
        target=tcp.copy(); target[2]+=.0001
        probe.model_gate(q,tcp,target,offset)
        probe.move_step(reader,force,baseline,host,target,.0005,False)
        lifted+=.0001
        entry.setdefault('force_unloading',[]).append({'up_mm':.1,'force_delta_n':probe.force_level(force,baseline)})
    return lifted


def apply_contact_height(plan, contact):
    if contact.get('status') != 'contact_recorded_retracted_2mm':
        raise RuntimeError('触纸未成功并抬笔，禁止写字')
    actual = np.asarray(contact['contact_pen_tip_base_m'],dtype=float)
    first = np.asarray(plan['strokes_pen_tip_base_m'][0][0])
    if actual.shape != (3,) or not np.isfinite(actual).all() or np.linalg.norm(actual[:2]-first[:2])>.0005:
        raise RuntimeError('触纸位置偏离第一笔起点，禁止写字')
    if abs(actual[2]-first[2])>.006:
        raise RuntimeError('测得高度偏移超过 6 mm，禁止写字')
    plan['requested_plane_z_mm'] = plan['user_plane_z_mm']
    plan['measured_contact_z_mm'] = float(actual[2]*1000)
    plan['height_source'] = 'single-point 1 N loaded pen contact; tilt not calibrated'
    for stroke in plan['strokes_pen_tip_base_m']:
        for point in stroke:
            point[2] = float(actual[2])


def stroke_route(plan,safe_z,air):
    route=[]
    for stroke in plan['strokes_pen_tip_base_m']:
        route.append(([*stroke[0][:2],safe_z],False))
        route.append((stroke[0],not air))
        route.extend((p,not air) for p in stroke[1:])
        route.append(([*stroke[-1][:2],safe_z],not air))
    return route


def teach(args, tip, digest):
    if args.registration.exists():
        raise ValueError('示教文件已存在；换 --registration 文件名，不覆盖旧示教')
    reg = {'tcp_sha256':digest,'corners_pen_tip_base_m':[],
           'corner_height_usage':'recorded only; XY registration ignores corner Z; writing Z supplied separately',
           'corner_order':['lower_left','lower_right','upper_left'],
           'axis_convention':'long edge toward base +X; if vertical toward +Y; paper +Y is CCW 90 degrees'}
    for name in ['任意第一个角','另一个角（不要重复）','第三个不同的角']:
        input('手动将笔尖对准%s的 XY，停稳后回车（高度不用于纸面标定）：' % name)
        sample = capture(args.host,30011,1)
        if sample['max_motion_mm'] > .2 or sample['max_motion_deg'] > .1:
            raise RuntimeError('示教时未停稳；未接受该角点')
        pose = np.asarray(sample['flange_pose_m_rad'])
        point = pose[:3] + Rotation.from_rotvec(pose[3:]).apply(tip)
        reg['corners_pen_tip_base_m'].append(point.tolist())
        reg.setdefault('measurements',[]).append(sample)
        print('笔尖 XYZ(mm):',np.round(point*1000,3).tolist())
    reg['taught_corners_unordered_base_m'] = reg['corners_pen_tip_base_m']
    reg['corners_pen_tip_base_m'] = unordered_corners(reg['taught_corners_unordered_base_m'])
    origin,axes,_ = paper_axes(reg['corners_pen_tip_base_m'])
    print('自动原点 XY(mm):',np.round(origin*1000,3).tolist())
    print('纸面 +X 基座方向:',np.round(axes[:,0],6).tolist())
    print('纸面 +Y 基座方向:',np.round(axes[:,1],6).tolist())
    # Derive the original controller orientation from flange and active offset.
    first = reg['measurements'][0]
    reg['tcp_orientation_rotvec'] = (Rotation.from_rotvec(first['flange_pose_m_rad'][3:]) *
        Rotation.from_rotvec(first['active_tcp_offset_m_rad'][3:])).as_rotvec().tolist()
    reg['active_tcp_offset_m_rad'] = first['active_tcp_offset_m_rad']
    save(args.registration,reg)
    print('只读示教已保存：',args.registration)


def execute(args, plan, reg, tip, report):
    reader = force = None
    sent = False
    try:
        info = capture(args.host,30011,1)
        offset = np.asarray(info['active_tcp_offset_m_rad'])
        if not np.allclose(offset,reg['active_tcp_offset_m_rad'],atol=1e-8,rtol=0):
            raise RuntimeError('控制器 TCP 已改变，禁止执行旧示教')
        reader = probe.RealtimeReader(args.host,30013)
        q,tcp,_ = read_stationary(reader,args.host)
        orientation = np.asarray(reg['tcp_orientation_rotvec'])
        if (Rotation.from_rotvec(tcp[3:]).inv()*Rotation.from_rotvec(orientation)).magnitude() > np.deg2rad(.2):
            raise RuntimeError('请恢复示教第一个角点时的笔姿态')
        flange = probe.flange_pose(tcp,offset)
        down = Rotation.from_rotvec(flange[3:]).apply([0,0,1])
        if down[2] > -math.cos(math.radians(20)):
            raise RuntimeError('笔需接近竖直向下（倾斜不超过 20°）')
        current = probe.tip_position(tcp,offset,tip)
        gap = current[2]-plan['user_plane_z_mm']/1000
        if not .003 <= gap <= .100:
            raise RuntimeError('开始前请手动抬笔到纸面上方 3–100 mm')
        force = probe.RawForce()
        baseline,report['baseline'] = force.baseline()
        probe.controller_ready(args.host)
        safe_z = max(current[2],plan['user_plane_z_mm']/1000+.010)
        route = [([*current[:2],safe_z],False)]
        if args.air:
            route.extend(stroke_route(plan,safe_z,True))
        else:
            first=plan['strokes_pen_tip_base_m'][0][0]
            route.extend([([*first[:2],safe_z],False),
                          ([*first[:2],plan['user_plane_z_mm']/1000+.003],False)])
        report['status'] = 'executing'
        upward_correction=0.
        save(args.output,report)
        for leg,(point,on_paper) in enumerate(route):
            q,tcp,_ = read_stationary(reader,args.host,allow_reconnect=not on_paper)
            point=list(point)
            contact_z=plan.get('measured_contact_z_mm',plan['user_plane_z_mm'])/1000
            at_writing_height=on_paper and abs(point[2]-contact_z)<1e-7
            if at_writing_height:
                point[2]+=upward_correction
            target = tcp_target(point,orientation,offset,tip)
            gate = probe.model_gate(q,tcp,target,offset)
            entry = {'leg':leg,'on_paper':on_paper,'target_pen_tip_base_m':point,'gate':gate,'completed':False}
            report.setdefault('legs',[]).append(entry)
            save(args.output,report)
            start = tcp.copy()
            horizontal = np.linalg.norm(target[:2]-start[:2])>.0001 and abs(target[2]-start[2])<.0002
            speed,acceleration,step = motion_parameters(on_paper,horizontal)
            entry['speed_mm_s'] = speed*1000
            entry['acceleration_mm_s2'] = acceleration*1000
            count = max(1,int(math.ceil(np.linalg.norm(target[:3]-start[:3])/step)))
            print('轨迹段 %d/%d：%.1f mm，速度 %.1f mm/s，%d 个有限小段' %
                  (leg+1,len(route),np.linalg.norm(target[:3]-start[:3])*1000,speed*1000,count),flush=True)
            for i in range(count):
                endpoint = start+(target-start)*(i+1)/count
                level=probe.force_level(force,baseline)
                if horizontal and at_writing_height and level>=2.0:
                    extra=unload_pen(reader,force,baseline,args.host,offset,entry,.001-upward_correction)
                    upward_correction+=extra
                    start[2]+=extra; target[2]+=extra; endpoint[2]+=extra
                    report['upward_force_correction_mm']=upward_correction*1000
                    save(args.output,report)
                # During air travel, sustained writing force is not expected.
                script = probe.finite_movel_program('pen_write_step',endpoint,acceleration,speed)
                sent = True
                probe.command(args.host,script)
                timeout = probe.finite_move_timeout(speed,acceleration,step)
                deadline = time.monotonic()+timeout
                recovery = []
                quiet = 0
                while time.monotonic()<deadline:
                    level = probe.force_level(force,baseline)
                    if horizontal and at_writing_height and level>=2.0:
                        extra=unload_pen(reader,force,baseline,args.host,offset,entry,.001-upward_correction)
                        upward_correction+=extra
                        start[2]+=extra; target[2]+=extra; endpoint[2]+=extra
                        report['upward_force_correction_mm']=upward_correction*1000
                        save(args.output,report)
                        script=probe.finite_movel_program('pen_write_step',endpoint,acceleration,speed)
                        probe.command(args.host,script)
                        deadline=time.monotonic()+timeout
                        quiet=0
                    # force_level enforces the requested raw 5 N overload
                    # gate. A 1 N contact trigger belongs to paper probing,
                    # not instantaneous rejection during free-space writing.
                    previous_recoveries = len(recovery)
                    # The motion phase, not the CLI mode, decides recovery.
                    # Real writing still starts with free-space positioning.
                    q,actual,velocity = writing_state(reader,force,baseline,args.host,endpoint,script,recovery,not on_paper)
                    if len(recovery) > previous_recoveries:
                        entry.setdefault('realtime_recoveries',[]).append(dict(recovery[-1],step=i+1))
                        save(args.output,report)
                        deadline = time.monotonic()+timeout
                        quiet = 0
                    if min(np.min(q-probe.LOWER),np.min(probe.UPPER-q)) < np.deg2rad(3):
                        raise RuntimeError('实时关节限位余量低于 3°')
                    if (Rotation.from_rotvec(actual[3:]).inv()*Rotation.from_rotvec(orientation)).magnitude()>np.deg2rad(.2):
                        raise RuntimeError('写字时笔姿态改变')
                    done = np.linalg.norm(actual[:3]-endpoint[:3])<.0001 and np.linalg.norm(velocity[:3])<.0001
                    quiet = quiet+1 if done else 0
                    if quiet>=9:
                        break
                else:
                    raise RuntimeError('写字有限小段未到位')
                entry['completed_steps'] = i+1
                if (i+1)%10 == 0:
                    save(args.output,report)
                    print('  小段 %d/%d' % (i+1,count),flush=True)
            entry['completed'] = True
            save(args.output,report)
            print('完成轨迹段',leg+1,'/',len(route),flush=True)
            if not args.air and leg==2:
                print('第一笔起点重新触纸：1 N 接触、5 N 过载、最多下探 6 mm；失败不写字。',flush=True)
                contact_path=args.output.with_name(args.output.stem+'-contact.json')
                report['contact_measurement']=str(contact_path)
                report['status']='probing_before_writing'
                save(args.output,report)
                # The child owns ROS and realtime acquisition during probing.
                force.close(); force=None
                reader.close(); reader=None
                probe.close_commands()
                contact=run_contact(args.host,args.tcp_calibration,contact_path)
                apply_contact_height(plan,contact)
                safe_z=max(safe_z,plan['measured_contact_z_mm']/1000+.010)
                route.extend(stroke_route(plan,safe_z,False))
                reader=probe.RealtimeReader(args.host,30013)
                force=probe.RawForce()
                baseline,report['writing_baseline']=force.baseline()
                report['status']='executing'
                save(args.output,report)
                print('写字高度已更新为 %.3f mm（单点测高，不校正倾斜）' % plan['measured_contact_z_mm'],flush=True)
        report['status'] = 'completed_air' if args.air else 'completed_fixed_height_motion'
    except BaseException as exc:
        if sent:
            try:
                probe.stop(args.host)
            except Exception as stop_error:
                report['stop_error'] = str(stop_error)
        report['status'],report['error'] = 'stopped_on_error',str(exc)
        raise
    finally:
        if force:
            report['force_trace'] = probe.force_snapshot(force)
            force.close()
        if reader:
            reader.close()
        probe.close_commands()
        save(args.output,report)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host',default='192.168.1.3')
    p.add_argument('--registration',type=Path,default=OUT/'a4-registration.json')
    p.add_argument('--tcp-calibration',type=Path,default=TCP)
    p.add_argument('--paper-z-mm',type=float,default=26.968)
    p.add_argument('--teach',action='store_true')
    p.add_argument('--execute',action='store_true')
    p.add_argument('--air',action='store_true')
    p.add_argument('--output',type=Path,default=OUT/('writing-'+dt.datetime.now().strftime('%Y%m%d-%H%M%S-%f')+'.json'))
    args = p.parse_args()
    if not math.isfinite(args.paper_z_mm):
        p.error('paper height must be finite')
    if args.teach and args.execute:
        p.error('示教和执行必须分开')
    raw = args.tcp_calibration.read_bytes()
    calibration = json.loads(raw)
    if not calibration['result']['passed']:
        raise ValueError('TCP 未通过验证')
    digest = hashlib.sha256(raw).hexdigest()
    tip = np.asarray(calibration['result']['flange_to_pen_tip_m'])
    if args.teach:
        teach(args,tip,digest)
        return
    reg = json.loads(args.registration.read_text())
    if reg['tcp_sha256'] != digest:
        raise ValueError('TCP 标定已改变，重新示教纸角')
    if args.output.exists():
        raise ValueError('output exists; choose another output')
    plan = make_plan(reg,args.paper_z_mm,args.air)
    report = {'plan':plan,'registration':str(args.registration),'status':'planned_no_motion'}
    save(args.output,report)
    print('外接面积占比 %.2f%%；纸面高度 %.3f mm；%s' % (100*plan['bounding_area_ratio'],args.paper_z_mm,'空走' if args.air else '固定高度落笔'))
    print('OUTPUT:',args.output)
    if args.execute:
        execute(args,plan,reg,tip,report)
    else:
        print('只生成轨迹，未连接机器人、未发运动。')


if __name__=='__main__':
    main()
