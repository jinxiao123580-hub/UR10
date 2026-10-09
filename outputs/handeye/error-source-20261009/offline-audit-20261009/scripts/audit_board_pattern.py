"""Held-out diagnostic of a shared board-index residual pattern; no calibration export."""
import pathlib,sys,json
ROOT=pathlib.Path(__file__).parent/'offline-audit-0489e5d';sys.path.insert(0,str(ROOT/'python-deps'))
import numpy as np,cv2
from scipy.spatial.transform import Rotation
d=json.loads((ROOT/'results/deeper/diagnostics.json').read_text());r=json.loads((ROOT/'results/deeper/ray-geometry.json').read_text());original=json.loads((ROOT/'results/audit.json').read_text())
obj=np.zeros((54,3));obj[:,:2]=np.mgrid[0:9,0:6].T.reshape(-1,2)*.006
training=['initial','pose-change','translation-only'];correction=np.mean([r['groups'][g]['classic_w7']['mean_pattern_mm'] for g in training],0)/1000
adjusted=obj.copy();adjusted[:,:2]+=correction
def avg(ts):
    a=np.eye(4);a[:3,3]=np.mean([x[:3,3] for x in ts],0);u,_,v=np.linalg.svd(np.sum([x[:3,:3] for x in ts],0));a[:3,:3]=u@np.diag([1,1,np.linalg.det(u@v)])@v;return a
def distance(a,b):return [float(np.linalg.norm(a[:3,3]-b[:3,3])*1000),float(np.degrees(Rotation.from_matrix(a[:3,:3].T@b[:3,:3]).magnitude()))]
rows=[]
for s in d['samples']:
    old=next(x for x in original['samples'] if x['id']==s['id']);folder=ROOT/'outputs/handeye/error-source-20261009'
    if s['group']!='initial':folder/=s['group']
    raw=json.loads((folder/'samples'/s['id']/'sample.json').read_text());K=np.array(raw['camera_info']['k']).reshape(3,3);D=np.array(raw['camera_info']['d']);c=np.array(s['methods']['classic_w7']['corners_px'])
    ok,rv,t=cv2.solvePnP(adjusted,c,K,D,flags=cv2.SOLVEPNP_ITERATIVE);assert ok
    B=np.eye(4);B[:3,:3]=cv2.Rodrigues(rv)[0];B[:3,3]=t.reshape(3);proj,_=cv2.projectPoints(adjusted,rv,t,K,D);res=proj.reshape(-1,2)-c
    H,_=cv2.findHomography(adjusted[:,:2],c,0);pr=cv2.perspectiveTransform(adjusted[:,:2].reshape(-1,1,2),H).reshape(-1,2)-c
    A=np.array(old['A']);X=np.linalg.inv(A)@np.array(old['C'])@np.linalg.inv(np.array(old['B']))
    rows.append({'group':s['group'],'id':s['id'],'heldout':s['group'] not in training,'original_pnp_rms_px':s['methods']['classic_w7']['pnp_rms_px'],'adjusted_pnp_rms_px':float(np.sqrt(np.mean(np.sum(res**2,1)))),'original_homography_rms_px':s['methods']['classic_w7']['homography_rms_px'],'adjusted_homography_rms_px':float(np.sqrt(np.mean(np.sum(pr**2,1)))),'C':(A@X@B).tolist()})
groups={}
for g in d['groups']:
    rows_g=[s for s in rows if s['group']==g];groups[g]={key:float(np.mean([s[key] for s in rows_g])) for key in ['original_pnp_rms_px','adjusted_pnp_rms_px','original_homography_rms_px','adjusted_homography_rms_px']};groups[g]['C']=avg([np.array(s['C']) for s in rows_g]).tolist()
pairs={}
for ga,gb in [('pose-change','translation-only'),('pose-change','return-to-pose'),('return-to-pose','xminus-160'),('return-to-pose','x-return')]:pairs[ga+' -> '+gb]=distance(np.array(groups[ga]['C']),np.array(groups[gb]['C']))
report={'warning':'This shared image-derived board-index pattern is a diagnostic, not measured board geometry. It can include detector/optical bias. No production parameters are modified.','training_groups':training,'heldout_groups':['return-to-pose','xminus-160','x-return'],'correction_mm':(correction*1000).tolist(),'groups':groups,'pairs':pairs,'samples':rows}
(ROOT/'results/deeper/board-pattern.json').write_text(json.dumps(report,indent=2))
print(json.dumps({'groups':{g:{k:v for k,v in val.items() if k!='C'} for g,val in groups.items()},'pairs':pairs,'pattern_rms_mm':float(np.sqrt(np.mean(np.sum((correction*1000)**2,1))))},indent=2))
