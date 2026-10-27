#!/usr/bin/env python3
"""Precompute all IK and timed samples, then replay locally without robot IO.

Input JSON: seed_q_rad[6], targets_base_tcp_m_rad[N][6], mode pose|axis,
tip_offset_m[3], requested_duration_s. First target MUST match seed FK.
Output is a simulation candidate, never authorization to execute on hardware.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation, Slerp
from ur10_nullspace_ik import UR10NullspaceIK


def poses_from_rows(rows):
    rows=np.asarray(rows,dtype=float)
    if rows.ndim!=2 or rows.shape[1]!=6 or len(rows)<2 or not np.isfinite(rows).all():
        raise ValueError('need at least two finite xyz/rotvec targets')
    out=np.repeat(np.eye(4)[None],len(rows),axis=0)
    out[:,:3,3]=rows[:,:3];out[:,:3,:3]=Rotation.from_rotvec(rows[:,3:]).as_matrix()
    return out


def spline_derivative_bounds(spline):
    """Exact component extrema over each cubic segment, not sampled estimates."""
    vmax=np.zeros(6);amax=np.zeros(6)
    for i,h in enumerate(np.diff(spline.x)):
        c=spline.c[:,i,:]
        for j in range(6):
            locations=[0.,h]
            if abs(c[0,j])>1e-15:
                root=-c[1,j]/(3*c[0,j])
                if 0<root<h:locations.append(root)
            vmax[j]=max(vmax[j],max(abs(3*c[0,j]*x*x+2*c[1,j]*x+c[2,j]) for x in locations))
            amax[j]=max(amax[j],abs(2*c[1,j]),abs(6*c[0,j]*h+2*c[1,j]))
    return vmax,amax


def prepare(document,collision_check,period=.008,velocity_limit=.15,acceleration_limit=.3):
    started=time.perf_counter()
    if not all(np.isfinite(v) and v>0 for v in [period,velocity_limit,acceleration_limit]):raise ValueError('invalid timing/limits')
    duration=float(document.get('requested_duration_s',10))
    if not np.isfinite(duration) or duration<=0:raise ValueError('invalid duration')
    ik=UR10NullspaceIK(tip_offset=document.get('tip_offset_m',[0,0,0]))
    mode=document.get('mode','pose');seed=np.asarray(document['seed_q_rad'],dtype=float)
    targets=poses_from_rows(document['targets_base_tcp_m_rad'])
    _,_,pe,re=ik.task(seed,targets[0],mode)
    if pe>1e-5 or re>1e-4:raise ValueError('first target is not seed pose; separately plan an approach')
    if np.any(seed<ik.lower) or np.any(seed>ik.upper):raise ValueError('seed violates margin')
    s_original=np.linspace(0,1,len(targets));s_knots=[0.];q_knots=[seed.copy()]
    orientation=Slerp(s_original,Rotation.from_matrix(targets[:,:3,:3]))

    def target_at(s):
        T=np.eye(4);T[:3,:3]=orientation(float(s)).as_matrix()
        T[:3,3]=[np.interp(s,s_original,targets[:,j,3]) for j in range(3)]
        return T

    def advance(s0,q0,s1,depth=0):
        target=target_at(s1);r=ik.solve(target,q0,mode,max_seed_step_deg=3)
        if not r['success']:
            if depth>=8:raise RuntimeError('IK failed at progress %.8f; no partial trajectory published'%s1)
            mid=(s0+s1)/2;qmid=advance(s0,q0,mid,depth+1)
            return advance(mid,qmid,s1,depth+1)
        q=np.array(r['q']);s_knots.append(s1);q_knots.append(q)
        return q

    q=seed.copy()
    for i in range(len(targets)-1):
        distance=np.linalg.norm(targets[i+1,:3,3]-targets[i,:3,3])
        angle=np.linalg.norm(Rotation.from_matrix(targets[i+1,:3,:3]@targets[i,:3,:3].T).as_rotvec())
        count=max(1,int(np.ceil(distance/.001)),int(np.ceil(angle/np.deg2rad(.5))))
        for s in np.linspace(s_original[i],s_original[i+1],count+1)[1:]:
            q=advance(s_knots[-1],q,float(s))
    spline=CubicSpline(s_knots,np.asarray(q_knots),bc_type=((1,np.zeros(6)),(1,np.zeros(6))))
    # Check joint position extrema analytically too: output sampling alone can
    # miss a spline overshoot between ticks.
    for i,h in enumerate(np.diff(spline.x)):
        for j in range(6):
            c=spline.c[:,i,j]
            roots=np.roots(np.trim_zeros(np.array([3*c[0],2*c[1],c[2]]),'f'))
            locations=[0.,h]+[float(r.real) for r in roots if abs(r.imag)<1e-10 and 0<r.real<h]
            values=[np.polyval(c,x) for x in locations]
            if min(values)<ik.lower[j] or max(values)>ik.upper[j]:raise RuntimeError('analytic spline joint margin overshoot')
    vbound,abound=spline_derivative_bounds(spline)
    duration=max(duration,float(vbound.max()/velocity_limit),float(np.sqrt(abound.max()/acceleration_limit)))*1.02
    samples=int(np.ceil(duration/period))+1
    if samples>1000000:raise ValueError('trajectory exceeds 1M sample budget')
    duration=(samples-1)*period;t=np.arange(samples)*period;s=t/duration
    q=spline(s);qd=spline(s,1)/duration;qdd=spline(s,2)/duration**2
    if np.any(q<ik.lower) or np.any(q>ik.upper):raise RuntimeError('spline overshoots joint margin')
    errors=[];gaps=[];jumps=[]
    for i,(progress,joints) in enumerate(zip(s,q)):
        # Check FK on output samples, not merely at original IK knots.
        _,_,pe,re=ik.task(joints,target_at(progress),mode)
        if pe>.0002 or re>np.deg2rad(.1):raise RuntimeError('interpolated task error at sample %d: %.3fmm %.3fdeg'%(i,pe*1000,np.rad2deg(re)))
        errors.append([pe,re])
        previous=q[i-1] if i else joints
        jump=float(np.max(np.abs(joints-previous)));jumps.append(jump)
        steps=max(1,int(np.ceil(jump/np.deg2rad(.5))))
        for alpha in np.linspace(0,1,steps+1):
            gap=float(collision_check(previous+alpha*(joints-previous)))
            if not np.isfinite(gap) or gap<.02:raise RuntimeError('collision gate failed at output sample %d'%i)
            gaps.append(gap)
    errors=np.array(errors)
    return {'schema_version':1,'status':'offline_simulation_candidate','execution_allowed':False,
            'units':{'q':'rad','qd':'rad/s','qdd':'rad/s^2','t':'s'},'mode':mode,
            'joint_names':ik.names,'start_q_rad':seed.tolist(),'period_s':period,'duration_s':duration,
            'velocity_limit_rad_s':velocity_limit,'acceleration_limit_rad_s2':acceleration_limit,
            'analytic_max_velocity_rad_s':(vbound/duration).tolist(),'analytic_max_acceleration_rad_s2':(abound/duration**2).tolist(),
            'max_position_error_mm':float(errors[:,0].max()*1000),'max_orientation_error_deg':float(np.rad2deg(errors[:,1].max())),
            'minimum_checked_clearance_mm':min(gaps)*1000,'max_output_joint_step_deg':float(np.rad2deg(max(jumps))),
            'interpolated_task_gates':{'position_m':.0002,'orientation_rad':float(np.deg2rad(.1))},
            'ik_knots':len(q_knots),'precompute_seconds':time.perf_counter()-started,
            'collision_scope':'supplied checker; CLI checks attachment proxies against selected arm links only, not full scene',
            'model_urdf':str(ik.urdf_path),
            'model_urdf_sha256':hashlib.sha256(ik.urdf_path.read_bytes()).hexdigest(),'tip_offset_m':ik.tip.tolist(),
            't_s':t.tolist(),'q_rad':q.tolist(),'qd_rad_s':qd.tolist(),'qdd_rad_s2':qdd.tolist()}


def replay(document,realtime=False,max_lateness=.05):
    """Yield cached targets; never computes IK. Late cycle aborts, no burst catch-up."""
    if not np.isfinite(max_lateness) or max_lateness<=0:raise ValueError('invalid lateness')
    t=np.asarray(document['t_s'],dtype=float);q=np.asarray(document['q_rad'],dtype=float)
    if len(t)<2 or q.shape!=(len(t),6) or not np.isfinite(q).all() or not np.isfinite(t).all() or t[0]!=0 or np.any(np.diff(t)<=0):raise ValueError('invalid cache')
    start=time.monotonic()
    for stamp,joints in zip(t,q):
        if realtime:
            delay=start+stamp-time.monotonic()
            if delay>0:time.sleep(delay)
            if time.monotonic()-(start+stamp)>max_lateness:raise RuntimeError('output deadline missed; stopped, no catch-up burst')
        yield float(stamp),joints.tolist()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    sub=p.add_subparsers(dest='command',required=True)
    plan=sub.add_parser('prepare');plan.add_argument('--input',type=Path,required=True);plan.add_argument('--output',type=Path,required=True)
    play=sub.add_parser('replay');play.add_argument('--input',type=Path,required=True);play.add_argument('--realtime',action='store_true')
    args=p.parse_args();d=json.loads(args.input.read_text())
    if args.command=='prepare':
        from self_collision import SelfCollisionModel
        collision=SelfCollisionModel();out=prepare(d,lambda q:collision.min_clearance(q)[0])
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(out,indent=2)+'\n')
        print(json.dumps({k:v for k,v in out.items() if k not in ['t_s','q_rad','qd_rad_s','qdd_rad_s2']},indent=2))
    else:
        for t,q in replay(d,args.realtime):print(json.dumps({'t_s':t,'q_rad':q}),flush=True)


if __name__=='__main__':main()
