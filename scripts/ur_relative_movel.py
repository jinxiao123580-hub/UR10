#!/usr/bin/env python3
"""Guarded small relative Cartesian move using UR controller-native movel."""
import argparse, json, math, os, socket, time
import numpy as np
import pinocchio as pin
from check_move_plan import BASE_FROM_URDF_ROOT, check_segment, pose_from_tcp_target, robot_pose
from read_ur_state import dashboard
from record_ur_trajectory import RealtimeReader
from self_collision import SelfCollisionModel
from ur_pose_ik import UR10IK

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--dx",type=float,default=0); p.add_argument("--dy",type=float,default=0); p.add_argument("--dz",type=float,default=0)
    p.add_argument("--speed",type=float,default=0.015); p.add_argument("--acceleration",type=float,default=0.03)
    p.add_argument("--max-distance",type=float,default=0.03); p.add_argument("--execute",action="store_true")
    p.add_argument("--dry-run",action="store_true",help="read state and check the path without motion")
    p.add_argument("--output",default="outputs/vision/relative-movel-result.json")
    a=p.parse_args()
    if a.execute == a.dry_run: p.error("select exactly one of --dry-run and --execute")
    distance=math.sqrt(a.dx*a.dx+a.dy*a.dy+a.dz*a.dz)
    if distance<=0 or distance>a.max_distance or a.max_distance>0.15: raise RuntimeError("move distance outside gate")
    if not 0.005<=a.speed<=0.02 or not 0.005<=a.acceleration<=0.05:
        raise RuntimeError("speed/acceleration outside low-speed gate")
    reader=RealtimeReader("192.168.1.3",port=30013); q,before,velocity=reader.read(max_wait_seconds=5)
    target=[before[0]+a.dx,before[1]+a.dy,before[2]+a.dz,*before[3:]]
    if min(target[2],before[2])<0.25: raise RuntimeError("tool0 Z below 0.25 m gate")
    if np.linalg.norm(velocity[:3])>0.003: raise RuntimeError("robot is not stationary")
    robotmode=dashboard("robotmode"); safety=dashboard("safetystatus")
    program_state=dashboard("programState")
    if (robotmode!="Robotmode: RUNNING" or safety!="Safetystatus: NORMAL" or
            not (program_state or "").startswith("STOPPED")):
        raise RuntimeError("UR controller not RUNNING/NORMAL/STOPPED: %s / %s / %s" %
                           (robotmode,safety,program_state))
    ik=UR10IK()
    error=pin.log6((BASE_FROM_URDF_ROOT*ik.pose(q)).inverse()*robot_pose(before)).vector
    if np.linalg.norm(error[:3])>0.010 or np.linalg.norm(error[3:])>0.020:
        raise RuntimeError("controller TCP disagrees with model FK")
    gate=check_segment(ik,robot_pose(before),pose_from_tcp_target(target),q,80,
                       SelfCollisionModel(),0.020)
    worst=gate["worst"]
    if (gate["reasons"] or worst["joint_margin_rad"]<math.radians(5) or
            worst["min_singular"]<0.02):
        raise RuntimeError("path gate rejected: %s" % (gate["reasons"][:3] or [worst]))
    print("minimum model self-clearance %.1f mm" %
          (worst["min_camera_clearance_m"]*1000.0),flush=True)
    program="def guarded_relative_movel():\n  movel(p[%s], a=%.6f, v=%.6f)\n  sleep(0.4)\nend\nguarded_relative_movel()\n" % (", ".join("%.9f"%x for x in target),a.acceleration,a.speed)
    print("before",[round(x,6) for x in before]); print("target",[round(x,6) for x in target],flush=True)
    if a.dry_run:
        reader.close()
        print("DRY-RUN: no motion sent")
        return
    q_now,before_now,velocity_now=reader.read(max_wait_seconds=5)
    if (math.dist(before_now[:3],before[:3])>0.001 or
            np.max(np.abs(np.asarray(q_now)-np.asarray(q)))>0.01 or
            np.linalg.norm(velocity_now[:3])>0.003):
        raise RuntimeError("robot moved after preflight; no motion sent")
    with socket.create_connection(("192.168.1.3",30002),timeout=3) as s: s.sendall(program.encode("ascii"))
    # Allow the controller to finish long, deliberately slow moves.  The old
    # fixed 8 s watchdog could report a false failure while URScript was still
    # executing the command on port 30002.
    deadline=time.monotonic()+max(15.0, distance/a.speed*2.5+8.0); after=before
    settled=False
    while time.monotonic()<deadline:
        try:
            _,after,velocity=reader.read(max_wait_seconds=3)
        except TimeoutError:
            with socket.create_connection(("192.168.1.3",30002),timeout=3) as s: s.sendall(b"stopj(2.0)\n")
            raise RuntimeError("UR state stream lost during motion; stopj sent")
        if math.dist(after[:3],target[:3])<0.001 and math.sqrt(sum(x*x for x in velocity[:3]))<0.002:
            settled=True
            break
    if not settled:
        with socket.create_connection(("192.168.1.3",30002),timeout=3) as s: s.sendall(b"stopj(2.0)\n")
        raise RuntimeError("relative move did not settle; stopj sent")
    reader.close(); error=math.dist(after[:3],target[:3])*1000
    result={"executed_at":time.strftime("%Y-%m-%dT%H:%M:%S%z"),"before_tcp":list(before),"target_tcp":target,"after_tcp":list(after),"delta_m":[a.dx,a.dy,a.dz],"final_error_mm":error,"gripper_command_sent":False}
    root=os.path.dirname(os.path.dirname(os.path.abspath(__file__))); out=a.output if os.path.isabs(a.output) else os.path.join(root,a.output); os.makedirs(os.path.dirname(out),exist_ok=True)
    with open(out,"w") as f: json.dump(result,f,indent=2); f.write("\n")
    print("after",[round(x,6) for x in after]); print("error_mm %.3f"%error); print("OUTPUT",out)
    if error>2: raise SystemExit(2)
if __name__=="__main__": main()
