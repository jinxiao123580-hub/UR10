"""Offline, retrospective frozen-parameter validation using only archived files.

Select with image residuals on first three groups, never with base-frame closure.
Re-detect all images using SB + the chosen cornerSubPix window. No ROS/network.
"""
import pathlib,sys,json,ast,datetime,csv,hashlib
ROOT=pathlib.Path(__file__).parent/'offline-audit-0489e5d';sys.path.insert(0,str(ROOT/'python-deps'))
import cv2,numpy as np,yaml
from scipy.spatial.transform import Rotation
BASE=ROOT/'outputs/handeye/error-source-20261009';OUT=ROOT/'results/frozen';OUT.mkdir(exist_ok=True)
diag=json.loads((ROOT/'results/deeper/diagnostics.json').read_text());old=json.loads((ROOT/'results/audit.json').read_text())
TRAIN=['initial','pose-change','translation-only'];TEST=['return-to-pose','xminus-160','x-return']
OBJ=np.zeros((54,3));OBJ[:,:2]=np.mgrid[0:9,0:6].T.reshape(-1,2)*.006
calpath=ROOT/'config/handeye_eye_in_hand_20260921.yaml';hashbefore=hashlib.sha256(calpath.read_bytes()).hexdigest();cal=yaml.safe_load(calpath.read_text());X=np.eye(4);X[:3,:3]=cal['rotation_matrix'];X[:3,3]=cal['translation_m']
def rt(rv,t):
    a=np.eye(4);a[:3,:3]=cv2.Rodrigues(np.asarray(rv,dtype=float))[0];a[:3,3]=np.asarray(t).reshape(3);return a
def avg(ts):
    a=np.eye(4);a[:3,3]=np.mean([t[:3,3] for t in ts],0);u,_,v=np.linalg.svd(np.sum([t[:3,:3] for t in ts],0));a[:3,:3]=u@np.diag([1,1,np.linalg.det(u@v)])@v;return a
def distance(a,b):return [float(np.linalg.norm(b[:3,3]-a[:3,3])*1000),float(np.degrees(Rotation.from_matrix(a[:3,:3].T@b[:3,:3]).magnitude()))]
def canonical(c,ref):
    return min([c,c[::-1],c.reshape(6,9,2)[:,::-1].reshape(-1,2),c.reshape(6,9,2)[::-1].reshape(-1,2)],key=lambda x:np.linalg.norm(x-ref))
def rms(x):return float(np.sqrt(np.mean(np.sum(x*x,1))))
def fit(p,q):
    pc=p.mean(0);qc=q.mean(0);u,_,v=np.linalg.svd((p-pc).T@(q-qc));r=v.T@np.diag([1,1,np.linalg.det(v.T@u.T)])@u.T;a=np.eye(4);a[:3,:3]=r;a[:3,3]=qc-r@pc;return a,q-(p@r.T+a[:3,3])
scores={}
for m in ['SB','classic_w3','classic_w5','classic_w7','classic_w9']:
    ss=[s for s in diag['samples'] if s['group'] in TRAIN]
    scores[m]={'all_training_frames_below_0.5px':all(s['methods'][m]['pnp_rms_px']<.5 for s in ss),'equal_group_mean_homography_rms_px':float(np.mean([diag['groups'][g][m]['homography_rms_px_mean'] for g in TRAIN])),'equal_group_mean_pnp_rms_px':float(np.mean([diag['groups'][g][m]['pnp_rms_px_mean'] for g in TRAIN]))}
chosen=min([m for m in scores if scores[m]['all_training_frames_below_0.5px']],key=lambda m:scores[m]['equal_group_mean_homography_rms_px'])
window=int(chosen.split('_w')[1]) if '_w' in chosen else None
source=ast.parse((ROOT/'scripts/check_handeye_checkerboard.py').read_text());node=next(n for n in source.body if isinstance(n,ast.FunctionDef) and n.name=='detect');env={'cv2':cv2,'np':np};exec(compile(ast.Module(body=[node],type_ignores=[]),'<offline>','exec'),env)
records=[]
for s0 in diag['samples']:
    group=s0['group'];folder=BASE if group=='initial' else BASE/group;sd=folder/'samples'/s0['id'];s=json.loads((sd/'sample.json').read_text())
    image=cv2.imdecode(np.fromfile(sd/'image.png',np.uint8),cv2.IMREAD_COLOR);gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    found,c=env['detect'](gray,(9,6));assert found
    if window:c=cv2.cornerSubPix(gray,c,(window,window),(-1,-1),(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_MAX_ITER,50,1e-4))
    c=canonical(c.reshape(-1,2),np.array(s['corners_px'])).astype(float)
    diff=rms(c-np.array(s0['methods'][chosen]['corners_px']));assert diff<.001,'SB/selected-classic convergence failed'
    k=np.array(s['camera_info']['k']).reshape(3,3);d=np.array(s['camera_info']['d'])
    f=json.loads((sd/'robot_frames.json').read_text())['frames'];tcp=np.mean([x['tcp_pose'] for x in f],0);A=rt(tcp[3:],tcp[:3])
    ok,rv,t=cv2.solvePnP(OBJ,c,k,d,flags=cv2.SOLVEPNP_ITERATIVE);assert ok;B=rt(rv,t);C=A@X@B
    proj,_=cv2.projectPoints(OBJ,rv,t,k,d);res=proj.reshape(-1,2)-c
    H,_=cv2.findHomography(OBJ[:,:2],c,0);hr=cv2.perspectiveTransform(OBJ[:,:2].reshape(-1,1,2),H).reshape(-1,2)-c
    z=np.load(sd/'cloud.npz');xyz=z['camera_xyz_m'];depths={}
    for rad in [1,2,3,5]:
        pts=[];ids=[]
        for i,(u,v) in enumerate(c):
            u,v=int(round(u)),int(round(v));p=xyz[max(0,v-rad):v+rad+1,max(0,u-rad):u+rad+1].reshape(-1,3);p=p[np.all(np.isfinite(p),1)&(p[:,2]>0)]
            if len(p)>=3:pts.append(np.median(p,0));ids.append(i)
        D,dr=fit(OBJ[ids],np.array(pts));expected=OBJ[ids]@B[:3,:3].T+B[:3,3]
        depths[str(rad)]={'count':len(ids),'D':D.tolist(),'CD':(A@X@D).tolist(),'fit_rms_mm':rms(dr)*1000,'pnp_xyz_disagreement_rms_mm':rms(np.array(pts)-expected)*1000}
    row={'id':s['sample_id'],'group':group,'captured_at':s['captured_at'],'timestamp_s':datetime.datetime.fromisoformat(s['captured_at']).timestamp(),'selected_method_reference_corner_rms_px':diff,'A':A.tolist(),'B':B.tolist(),'C':C.tolist(),'k':k.tolist(),'d':d.tolist(),'corners_px':c.tolist(),'pnp_rms_px':rms(res),'homography_rms_px':rms(hr),'residual_px':res.tolist(),'depths':depths}
    records.append(row)
    print(group,s['sample_id'],'px',round(row['pnp_rms_px'],4),'depth',depths['3']['count'],flush=True)
groups={}
for g in diag['groups']:
    rr=[r for r in records if r['group']==g];C=avg([np.array(r['C']) for r in rr]);CD=avg([np.array(r['depths']['3']['CD']) for r in rr]);err=np.array([distance(C,np.array(r['C'])) for r in rr]);de=np.array([distance(CD,np.array(r['depths']['3']['CD'])) for r in rr])
    groups[g]={'n':len(rr),'training':g in TRAIN,'time_s_mean':float(np.mean([r['timestamp_s'] for r in rr])),'C':C.tolist(),'CD':CD.tolist(),'A':avg([np.array(r['A']) for r in rr]).tolist(),'B':avg([np.array(r['B']) for r in rr]).tolist(),'pnp_repeat_rms':np.sqrt(np.mean(err**2,0)).tolist(),'depth_repeat_rms':np.sqrt(np.mean(de**2,0)).tolist(),'pnp_rms_px_mean':float(np.mean([r['pnp_rms_px'] for r in rr])),'homography_rms_px_mean':float(np.mean([r['homography_rms_px'] for r in rr])),'depth_count':[r['depths']['3']['count'] for r in rr],'depth_pnp_xyz_rms_mm_mean':float(np.mean([r['depths']['3']['pnp_xyz_disagreement_rms_mm'] for r in rr])),'depth_fit_rms_mm_mean':float(np.mean([r['depths']['3']['fit_rms_mm'] for r in rr]))}
pairs={}
for ga,gb in [('pose-change','translation-only'),('translation-only','return-to-pose'),('pose-change','return-to-pose'),('return-to-pose','xminus-160'),('xminus-160','x-return'),('return-to-pose','x-return'),('pose-change','x-return')]:
    p={}
    for key in ['A','C','CD']:
        aa=np.array(groups[ga][key]);bb=np.array(groups[gb][key]);p[key+'_distance']=distance(aa,bb);p[key+'_delta_xyz_mm']=((bb[:3,3]-aa[:3,3])*1000).tolist()
    for key in ['C','CD']:
        delta=np.array(p[key+'_delta_xyz_mm']);axis=np.array(p['A_delta_xyz_mm']);axis/=np.linalg.norm(axis);along=float(delta@axis);p[key+'_parallel_to_reported_motion_mm']=along;p[key+'_perpendicular_to_reported_motion_mm']=float(np.sqrt(max(0,delta@delta-along**2)))
    p['time_gap_minutes']=(groups[gb]['time_s_mean']-groups[ga]['time_s_mean'])/60
    p['camera_length_minus_reported_tcp_mm']=float(np.linalg.norm(np.array(groups[gb]['B'])[:3,3]-np.array(groups[ga]['B'])[:3,3])*1000-p['A_distance'][0])
    pairs[ga+' -> '+gb]=p
# Compare image evidence for the near-return poses to the anchor board pose.
anchor=np.array(groups['pose-change']['C']);same=[]
for g in ['pose-change','return-to-pose','x-return']:
    fields=[]
    for row in records:
        if row['group']!=g:continue
        exp=np.linalg.inv(np.array(row['A'])@X)@anchor;rv=cv2.Rodrigues(exp[:3,:3])[0];pred,_=cv2.projectPoints(OBJ,rv,exp[:3,3],np.array(row['k']),np.array(row['d']));err=np.array(row['corners_px'])-pred.reshape(-1,2);fields.append(err)
    fields=np.array(fields);mean=fields.mean(0)
    same.append({'group':g,'time_minutes_from_anchor':(groups[g]['time_s_mean']-groups['pose-change']['time_s_mean'])/60,'observed_minus_anchor_prediction_mean_px':mean.mean(0).tolist(),'rms_px':rms(mean),'mean_center_shift_px_norm':float(np.linalg.norm(mean.mean(0))),'mean_field_px':mean.tolist()})
assert hashlib.sha256(calpath.read_bytes()).hexdigest()==hashbefore
report={'validation_scope':'Retrospective split of previously inspected data, not a new unseen dataset or absolute accuracy validation.','selection':{'training_groups':TRAIN,'test_groups':TEST,'scores':scores,'selected':chosen,'implemented':'SB+cornerSubPix_'+str(window),'rule':'All training frame PnP RMS<0.5px; smallest equal-group mean homography RMS; no base closure in selection.'},'calibration_sha256_unchanged':hashbefore,'groups':groups,'pairs':pairs,'near_return_image_check':same,'samples':records}
(OUT/'validation.json').write_text(json.dumps(report,indent=2))
with (OUT/'sample-results.csv').open('w',encoding='utf-8-sig',newline='') as f:
    w=csv.writer(f);w.writerow(['group','id','time','pnp_rms_px','homography_rms_px','depth_count','depth_fit_rms_mm','depth_pnp_xyz_rms_mm'])
    for r in records:w.writerow([r['group'],r['id'],r['captured_at'],r['pnp_rms_px'],r['homography_rms_px'],r['depths']['3']['count'],r['depths']['3']['fit_rms_mm'],r['depths']['3']['pnp_xyz_disagreement_rms_mm']])
print(json.dumps({'selection':report['selection'],'group_quality':{g:{k:v for k,v in s.items() if k not in ['A','B','C','CD']} for g,s in groups.items()},'pairs':pairs,'near_returns':[{k:v for k,v in s.items() if k!='mean_field_px'} for s in same]},indent=2))
