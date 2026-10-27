#!/usr/bin/env python3
"""Offline-only UR10 IK: own URDF FK/Jacobian, DLS task and SVD null space.

No controller IK, Pinocchio or robot communications are used by this module.
Targets are in URDF `base` coordinates, not base_link/world. Tool offset is
expressed in tool0 axes. Collision checking is a separate, mandatory gate
before any future physical execution.
"""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation


def skew(v):
    x,y,z=v
    return np.array([[0,-z,y],[z,0,-x],[-y,x,0]])


class UR10NullspaceIK:
    def __init__(self, urdf=None, base='base', tool='tool0', tip_offset=(0,0,0), margin_deg=3):
        urdf = urdf or Path(__file__).resolve().parents[1]/'outputs/ik/ur10_factory_20261027.urdf'
        self.urdf_path=Path(urdf).resolve()
        root=ET.parse(self.urdf_path).getroot()
        self.by_child={j.find('child').get('link'):j for j in root.findall('joint')}
        def chain(link):
            result=[]
            while link in self.by_child:
                j=self.by_child[link]; result.append(j);link=j.find('parent').get('link')
            return list(reversed(result))
        self.chain=chain(tool)
        self.names=[j.get('name') for j in self.chain if j.get('type')=='revolute']
        if len(self.names)!=6: raise ValueError('expected six revolute joints')
        if any(j.get('type')!='fixed' for j in chain(base)):
            raise ValueError('base must be fixed to model root')
        if base not in {l.get('name') for l in root.findall('link')}:
            raise ValueError('unknown base link')
        self.base_inv=np.linalg.inv(self._walk(chain(base),np.zeros(6))[0])
        limits=[j.find('limit') for j in self.chain if j.get('type')=='revolute']
        if not np.isfinite(margin_deg) or margin_deg<3:raise ValueError('at least 3 degree margin required')
        self.margin_deg=margin_deg
        self.lower=np.array([float(l.get('lower')) for l in limits])+np.deg2rad(margin_deg)
        self.upper=np.array([float(l.get('upper')) for l in limits])-np.deg2rad(margin_deg)
        self.tip=np.asarray(tip_offset,dtype=float)
        if self.tip.shape!=(3,) or not np.isfinite(self.tip).all():raise ValueError('invalid tool offset')

    @staticmethod
    def _walk(chain,q):
        T=np.eye(4);axes=[];origins=[];i=0
        for j in chain:
            o=j.find('origin'); A=np.eye(4)
            if o is not None:
                A[:3,:3]=Rotation.from_euler('xyz',np.fromstring(o.get('rpy','0 0 0'),sep=' ')).as_matrix()
                A[:3,3]=np.fromstring(o.get('xyz','0 0 0'),sep=' ')
            T=T@A
            if j.get('type')=='revolute':
                axis=np.fromstring(j.find('axis').get('xyz'),sep=' ');axis/=np.linalg.norm(axis)
                axes.append(T[:3,:3]@axis);origins.append(T[:3,3].copy())
                B=np.eye(4);B[:3,:3]=Rotation.from_rotvec(axis*q[i]).as_matrix();T=T@B;i+=1
            elif j.get('type')!='fixed':raise ValueError('unsupported joint type')
        return T,axes,origins

    def fk_jacobian(self,q):
        q=np.asarray(q,dtype=float)
        if q.shape!=(6,) or not np.isfinite(q).all():raise ValueError('invalid q')
        T,axes,origins=self._walk(self.chain,q);T=self.base_inv@T
        T[:3,3]+=T[:3,:3]@self.tip
        R=self.base_inv[:3,:3];p=self.base_inv[:3,3]
        axes=[R@a for a in axes];origins=[R@o+p for o in origins]
        J=np.array([np.r_[np.cross(a,T[:3,3]-o),a] for a,o in zip(axes,origins)]).T
        return T,J

    def task(self,q,target,mode):
        T,J=self.fk_jacobian(q)
        pos=target[:3,3]-T[:3,3]
        if mode=='pose':
            rot=Rotation.from_matrix(target[:3,:3]@T[:3,:3].T).as_rotvec()
            return np.r_[pos,.3*rot],np.vstack([J[:3],.3*J[3:]]),np.linalg.norm(pos),np.linalg.norm(rot)
        if mode!='axis':raise ValueError('mode must be pose or axis')
        a,b=T[:3,2],target[:3,2]
        angle=np.arccos(np.clip(a@b,-1,1))
        return np.r_[pos,.3*(b-a)],np.vstack([J[:3],-.3*skew(a)@J[3:]]),np.linalg.norm(pos),angle

    def cost(self,q):
        half=(self.upper-self.lower)/2;u=(q-(self.lower+self.upper)/2)/half
        return float(np.sum(u*u))

    def solve_initial(self,target,seed,mode='pose',max_iterations=400):
        """Initial configuration search, NOT a trajectory command.

        Wider 45-degree neighborhood is explicit. A separate collision-checked,
        time-parameterized approach path is required from the robot's state.
        """
        result=self.solve(target,seed,mode,max_iterations,max_seed_step_deg=45.0)
        result['purpose']='initial_configuration_search_only'
        return result

    def solve(self,target,seed,mode='pose',max_iterations=400,max_seed_step_deg=3.0):
        if not isinstance(max_iterations,int) or max_iterations<1:raise ValueError('invalid iteration limit')
        if not np.isfinite(max_seed_step_deg) or max_seed_step_deg<=0:raise ValueError('invalid seed trust region')
        target=np.asarray(target,dtype=float);q=np.asarray(seed,dtype=float).copy()
        if target.shape!=(4,4) or not np.isfinite(target).all() or not np.allclose(target[3],[0,0,0,1]):raise ValueError('invalid target')
        if not np.allclose(target[:3,:3].T@target[:3,:3],np.eye(3),atol=1e-7) or np.linalg.det(target[:3,:3])<.999:raise ValueError('invalid rotation')
        if q.shape!=(6,) or not np.isfinite(q).all() or np.any(q<self.lower) or np.any(q>self.upper):raise ValueError('seed outside joint margin')
        seed_q=q.copy();radius=np.deg2rad(max_seed_step_deg)
        initial_cost=self.cost(q);secondary_steps=0
        for iteration in range(max_iterations):
            e,J,pe,re=self.task(q,target,mode)
            U,s,Vt=np.linalg.svd(J,full_matrices=True)
            rank=int(np.sum(s>1e-7));Z=Vt[rank:].T
            # Exact SVD null projector; damped inverse alone is not a null projector.
            N=Z@Z.T
            damping=.001 if s[rank-1]>.02 else .01
            primary=Vt.T@((s/(s*s+damping*damping))*(U.T@e))
            half=(self.upper-self.lower)/2
            gradient=2*(q-(self.lower+self.upper)/2)/(half*half)
            converged=pe<1e-5 and re<1e-4
            # Solve the primary task first. Secondary motion must not consume
            # the finite tracking neighborhood before primary convergence.
            dq=primary-(.3*N@gradient if converged else 0)
            scale=max(1,np.max(np.abs(dq))/.08);dq/=scale
            if converged and (rank==6 or secondary_steps>=30):break
            accepted=False
            for alpha in [1,.5,.25,.125,.0625,.03125]:
                candidate=q+alpha*dq
                if np.any(candidate<self.lower) or np.any(candidate>self.upper):continue
                if np.max(np.abs(candidate-seed_q))>radius:continue
                ce,_,cp,cr=self.task(candidate,target,mode)
                if (not converged and np.linalg.norm(ce)<np.linalg.norm(e)-1e-12) or (converged and cp<1e-5 and cr<1e-4 and self.cost(candidate)<self.cost(q)-1e-10):
                    q=candidate;accepted=True
                    if converged:secondary_steps+=1
                    break
            if not accepted:break
        e,J,pe,re=self.task(q,target,mode);s=np.linalg.svd(J,compute_uv=False);rank=int(np.sum(s>1e-7))
        return {'q':q.tolist(),'success':bool(pe<1e-5 and re<1e-4), 'position_error_m':float(pe),
                'orientation_error_rad':float(re),'task_rank':rank,'nullity':6-rank,'iterations':iteration+1,
                'initial_center_cost':initial_cost,'final_center_cost':self.cost(q),
                'max_seed_step_deg':float(np.rad2deg(np.max(np.abs(q-seed_q)))),'seed_trust_region_deg':float(max_seed_step_deg),
                'purpose':'local_tracking_candidate','failure_reason':None if pe<1e-5 and re<1e-4 else 'local_neighborhood_or_convergence_failure',
                'minimum_joint_margin_deg':float(np.rad2deg(np.minimum(q-self.lower,self.upper-q).min())+self.margin_deg),
                'mode':mode,'execution_allowed':False,'collision_checked':False}
