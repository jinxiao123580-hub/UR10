import pathlib,json,sys,hashlib
from audit_download import ROOT, GROUPS, EXP
sys.path.insert(0,str(ROOT/'python-deps'))
import numpy as np,cv2
from scipy.spatial.transform import Rotation
r=json.loads((ROOT/'results/audit.json').read_text())
def avg(ts):
    t=np.eye(4);t[:3,3]=np.mean([x[:3,3] for x in ts],0);u,_,v=np.linalg.svd(np.sum([x[:3,:3] for x in ts],0));t[:3,:3]=u@np.diag([1,1,np.linalg.det(u@v)])@v;return t
def dist(a,b):return [float(np.linalg.norm(a[:3,3]-b[:3,3])*1000),float(np.degrees(Rotation.from_matrix(a[:3,:3].T@b[:3,:3]).magnitude()))]
def rt(rv,t):
    a=np.eye(4);a[:3,:3]=cv2.Rodrigues(np.array(rv))[0];a[:3,3]=t;return a
def fit(p,q):
    pc=p.mean(0);qc=q.mean(0);u,_,v=np.linalg.svd((p-pc).T@(q-qc));rot=v.T@np.diag([1,1,np.linalg.det(v.T@u.T)])@u.T;a=np.eye(4);a[:3,:3]=rot;a[:3,3]=qc-rot@pc;return a
obj=np.zeros((54,3));obj[:,:2]=np.mgrid[0:9,0:6].T.reshape(-1,2)*.006
stored={};details=[]
X=None
for g in r['groups']:
    folder=ROOT/EXP if g=='initial' else ROOT/EXP/g
    ds=json.loads((folder/'dataset.json').read_text());cc=[];dd=[];qs=[]
    for s in ds['samples']:
        if not s.get('accepted'):continue
        row=next(x for x in r['samples'] if x['id']==s['sample_id']);A=np.array(row['A']);B=np.array(row['B'])
        if X is None:X=np.linalg.inv(A)@np.array(row['C'])@np.linalg.inv(B)
        k=np.array(s['camera_info']['k']).reshape(3,3);d=np.array(s['camera_info']['d']);c=np.array(s['corners_px'])
        ok,rv,tv=cv2.solvePnP(obj,c,k,d,flags=cv2.SOLVEPNP_ITERATIVE);assert ok;S=rt(rv,tv.reshape(3));proj,_=cv2.projectPoints(obj,rv,tv,k,d)
        original=rt(s['target_to_camera']['rvec_rad'],s['target_to_camera']['translation_m']);cc.append(A@X@S)
        z=np.load(folder/'samples'/s['sample_id']/'cloud.npz');xyz=z['camera_xyz_m'];pts=[];ids=[]
        for i,(u,v) in enumerate(c):
            u,v=int(round(u)),int(round(v));p=xyz[v-3:v+4,u-3:u+4].reshape(-1,3);p=p[np.all(np.isfinite(p),1)]
            if len(p)>=3:pts.append(np.median(p,0));ids.append(i)
        D=fit(obj[ids],np.array(pts));dd.append(A@X@D)
        frames=json.loads((folder/'samples'/s['sample_id']/'robot_frames.json').read_text())['frames'];q=np.array([f['q_rad'] for f in frames]);qs.append(q.mean(0))
        details.append({'id':s['sample_id'],'stored_corner_pnp_vs_archive':dist(original,S),'stored_corner_rms_px':float(np.sqrt(np.mean(np.sum((proj.reshape(-1,2)-c)**2,1)))),'tcp_mean_maxdiff':row['tcp_mean_maxdiff'],'npz_tcp_delta':row['npz_tcp_delta'],'q_mean':q.mean(0).tolist(),'q_within_frame_std_rad':q.std(0).tolist()})
    stored[g]={'C':avg(cc).tolist(),'CD':avg(dd).tolist(),'q_mean':np.mean(qs,0).tolist(),'C_repeat_rms':np.sqrt(np.mean(np.array([dist(avg(cc),c) for c in cc])**2,0)).tolist()}
pairs={};robust={}
for ga,gb in [('pose-change','translation-only'),('pose-change','return-to-pose'),('return-to-pose','xminus-160'),('return-to-pose','x-return')]:
    name=ga+' -> '+gb
    pairs[name]={key:dist(np.array(stored[ga][key]),np.array(stored[gb][key])) for key in ['C','CD']}
    pairs[name]['q_delta_deg']=(np.degrees(np.array(stored[gb]['q_mean'])-stored[ga]['q_mean'])).tolist()
    robust[name]={}
    for radius in ['1','2','3','5']:
        aa=avg([np.array(s['depth_radii'][radius]['C']) for s in r['samples'] if s['group']==ga]);bb=avg([np.array(s['depth_radii'][radius]['C']) for s in r['samples'] if s['group']==gb]);robust[name]['radius_'+radius]=dist(aa,bb)
    ca=[np.array(s['classic']['C']) for s in r['samples'] if s['group']==ga and s['classic']];cb=[np.array(s['classic']['C']) for s in r['samples'] if s['group']==gb and s['classic']]
    robust[name]['classic']=dist(avg(ca),avg(cb))
supp={'stored_corner_groups':stored,'stored_corner_pairs':pairs,'robustness':robust,'sample_checks':details}
(ROOT/'results/supplement.json').write_text(json.dumps(supp,indent=2))
print(json.dumps({'pairs':pairs,'robustness':robust,'max_archive_pnp_delta':[max(d['stored_corner_pnp_vs_archive'][i] for d in details) for i in [0,1]],'max_tcp_mean_difference':max(d['tcp_mean_maxdiff'] for d in details)},indent=2))
