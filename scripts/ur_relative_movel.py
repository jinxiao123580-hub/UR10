#!/usr/bin/env python3
"""Guarded small relative Cartesian move using UR controller-native movel."""
import argparse, json, math, os, socket, time
from record_ur_trajectory import RealtimeReader

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--dx",type=float,default=0); p.add_argument("--dy",type=float,default=0); p.add_argument("--dz",type=float,default=0)
    p.add_argument("--speed",type=float,default=0.015); p.add_argument("--acceleration",type=float,default=0.03)
    p.add_argument("--max-distance",type=float,default=0.03); p.add_argument("--execute",action="store_true")
    p.add_argument("--output",default="outputs/vision/relative-movel-result.json")
    a=p.parse_args()
    if not a.execute: p.error("real motion requires --execute")
    distance=math.sqrt(a.dx*a.dx+a.dy*a.dy+a.dz*a.dz)
    if distance<=0 or distance>a.max_distance: raise RuntimeError("move distance outside gate")
    reader=RealtimeReader("192.168.1.3"); _,before,_=reader.read()
    target=[before[0]+a.dx,before[1]+a.dy,before[2]+a.dz,*before[3:]]
    if target[2]<0.25: raise RuntimeError("tool0 Z below 0.25 m gate")
    program="def guarded_relative_movel():\n  movel(p[%s], a=%.6f, v=%.6f)\n  sleep(1.0)\nend\n" % (", ".join("%.9f"%x for x in target),a.acceleration,a.speed)
    print("before",[round(x,6) for x in before]); print("target",[round(x,6) for x in target],flush=True)
    with socket.create_connection(("192.168.1.3",30002),timeout=3) as s: s.sendall(program.encode("ascii"))
    # Allow the controller to finish long, deliberately slow moves.  The old
    # fixed 8 s watchdog could report a false failure while URScript was still
    # executing the command on port 30002.
    deadline=time.monotonic()+max(8.0, distance/max(a.speed, 1e-6)*3.0+3.0); after=before
    while time.monotonic()<deadline:
        _,after,velocity=reader.read()
        if math.dist(after[:3],target[:3])<0.001 and math.sqrt(sum(x*x for x in velocity[:3]))<0.002: break
    reader.close(); error=math.dist(after[:3],target[:3])*1000
    result={"executed_at":time.strftime("%Y-%m-%dT%H:%M:%S%z"),"before_tcp":list(before),"target_tcp":target,"after_tcp":list(after),"delta_m":[a.dx,a.dy,a.dz],"final_error_mm":error,"gripper_command_sent":False}
    root=os.path.dirname(os.path.dirname(os.path.abspath(__file__))); out=a.output if os.path.isabs(a.output) else os.path.join(root,a.output); os.makedirs(os.path.dirname(out),exist_ok=True)
    with open(out,"w") as f: json.dump(result,f,indent=2); f.write("\n")
    print("after",[round(x,6) for x in after]); print("error_mm %.3f"%error); print("OUTPUT",out)
    if error>2: raise SystemExit(2)
if __name__=="__main__": main()
