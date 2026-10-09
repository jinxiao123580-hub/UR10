"""Offline detector/geometry/depth diagnostics; no network or hardware imports."""
import pathlib,sys,json,csv
ROOT=pathlib.Path(__file__).parent/'offline-audit-0489e5d'
sys.path.insert(0,str(ROOT/'python-deps'))
import numpy as np,cv2
from scipy.spatial.transform import Rotation
R=json.loads((ROOT/'results/audit.json').read_text()); OUT=ROOT/'results/deeper';OUT.mkdir(exist_ok=True)
OBJ=np.zeros((54,3));OBJ[:,:2]=np.mgrid[0:9,0:6].T.reshape(-1,2)*.006
GRID=np.mgrid[0:9,0:6].T.reshape(-1,2); PARITY=(-1.)**(GRID[:,0]+GRID[:,1])
def rt(rv,t):
    a=np.eye(4);a[:3,:3]=cv2.Rodrigues(np.asarray(rv))[0];a[:3,3]=np.asarray(t).reshape(3);return a
def avg(ts):
    t=np.eye(4);t[:3,3]=np.mean([x[:3,3] for x in ts],0);u,_,v=np.linalg.svd(np.sum([x[:3,:3] for x in ts],0));t[:3,:3]=u@np.diag([1,1,np.linalg.det(u@v)])@v;return t
def delta(a,b):
    return [float(np.linalg.norm(a[:3,3]-b[:3,3])*1000),float(np.degrees(Rotation.from_matrix(a[:3,:3].T@b[:3,:3]).magnitude()))]
def rms2(x):return float(np.sqrt(np.mean(np.sum(x*x,axis=1))))
def canonical(c,reference):
    opts=[c,c[::-1],c.reshape(6,9,2)[:,::-1].reshape(-1,2),c.reshape(6,9,2)[::-1].reshape(-1,2)]
    return min(opts,key=lambda x:np.linalg.norm(x-reference))
def pnp(c,k,d,mask=None):
    if mask is None:mask=np.ones(54,dtype=bool)
    ok,rv,t=cv2.solvePnP(OBJ[mask],c[mask].astype(float),k,d,flags=cv2.SOLVEPNP_ITERATIVE);assert ok
    proj,_=cv2.projectPoints(OBJ,rv,t,k,d);return rt(rv,t),proj.reshape(-1,2)-c
def tangent(c):
    cc=c.reshape(6,9,2);u=np.gradient(cc,axis=1).reshape(-1,2);v=np.gradient(cc,axis=0).reshape(-1,2)
    return u/np.linalg.norm(u,axis=1)[:,None],v/np.linalg.norm(v,axis=1)[:,None]
def geometry(c,k,d):
    B,res=pnp(c,k,d);H,_=cv2.findHomography(OBJ[:,:2],c,0)
    hp=cv2.perspectiveTransform(OBJ[:,:2].astype(float).reshape(-1,1,2),H).reshape(-1,2);hr=hp-c
    u,v=tangent(c);along=np.column_stack((np.sum(hr*u,1),np.sum(hr*v,1)))
    parity=np.mean(along*PARITY[:,None],axis=0)
    after=along-PARITY[:,None]*parity
    # parity projection is diagnostic only; it is not used to correct production data.
    explained=1-float(np.sum(after**2)/np.sum(along**2))
    held=[]
    for axis in [0,1]:
        for value in range([9,6][axis]):
            mask=GRID[:,axis]!=value;bb,rr=pnp(c,k,d,mask)
            held.append({'axis':'column' if axis==0 else 'row','index':value,'pose_delta':delta(B,bb),'heldout_rms_px':rms2(rr[~mask]),'training_rms_px':rms2(rr[mask])})
    polar=[]
    for sign in [-1,1]:
        bb,rr=pnp(c,k,d,PARITY==sign);polar.append({'parity':sign,'B':bb.tolist(),'pose_delta':delta(B,bb),'heldout_rms_px':rms2(rr[PARITY!=sign])})
    return {'B':B.tolist(),'pnp_rms_px':rms2(res),'homography_rms_px':rms2(hr),'residual_px':res.tolist(),'homography_residual_px':hr.tolist(),'parity_tangent_amplitude_px':parity.tolist(),'parity_energy_fraction':explained,'leave_row_column_out':held,'parity_fits':polar}
results=[];Xs=[]
for old in R['samples']:
    g=old['group'];folder=ROOT/'outputs/handeye/error-source-20261009'
    if g!='initial':folder=folder/g
    sdir=folder/'samples'/old['id'];s=json.loads((sdir/'sample.json').read_text());k=np.array(s['camera_info']['k']).reshape(3,3);d=np.array(s['camera_info']['d'])
    img=cv2.imdecode(np.fromfile(sdir/'image.png',np.uint8),cv2.IMREAD_COLOR);gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    ref=np.array(old['corners_px'],dtype=np.float32);found,classic=cv2.findChessboardCorners(gray,(9,6),cv2.CALIB_CB_ADAPTIVE_THRESH|cv2.CALIB_CB_NORMALIZE_IMAGE);assert found
    methods={'SB':ref.astype(float)}
    for win in [3,5,7,9]:
        cc=cv2.cornerSubPix(gray,classic.copy(),(win,win),(-1,-1),(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_MAX_ITER,50,1e-4));methods['classic_w'+str(win)]=canonical(cc.reshape(-1,2),ref).astype(float)
    cc=cv2.cornerSubPix(gray,ref.reshape(-1,1,2).copy(),(7,7),(-1,-1),(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_MAX_ITER,50,1e-4));methods['SB_subpix7']=cc.reshape(-1,2).astype(float)
    A=np.array(old['A']);X=np.linalg.inv(A)@np.array(old['C'])@np.linalg.inv(np.array(old['B']));Xs.append(X)
    row={'group':g,'id':old['id'],'methods':{}}
    for name,c in methods.items():
        val=geometry(c,k,d);val['corners_px']=c.tolist();val['C']=(A@X@np.array(val['B'])).tolist();val['pixel_shift_from_SB_rms']=rms2(c-ref);val['shift_from_SB_mean_px']=np.mean(c-ref,0).tolist()
        row['methods'][name]=val
    # Dense board interior: plane-only diagnostic avoids the missing X corners.
    z=np.load(sdir/'cloud.npz');xyz=z['camera_xyz_m'];cx=ref.astype(float)
    H,_=cv2.findHomography(OBJ[:,:2],cx,0);inv=np.linalg.inv(H)
    x0,y0=np.floor(cx.min(0)-3).astype(int);x1,y1=np.ceil(cx.max(0)+3).astype(int)
    uu,vv=np.meshgrid(np.arange(max(0,x0),min(1280,x1+1)),np.arange(max(0,y0),min(1024,y1+1)))
    uv=np.column_stack((uu.ravel(),vv.ravel()));bp=cv2.perspectiveTransform(uv.astype(float).reshape(-1,1,2),inv).reshape(-1,2)
    # Inset footprint; this tests surface normal only, not the known checkerboard frame origin.
    inside=(bp[:,0]>.002)&(bp[:,0]<.046)&(bp[:,1]>.002)&(bp[:,1]<.028)
    pp=xyz[vv.ravel(),uu.ravel()].astype(float);finite=np.all(np.isfinite(pp),1)&(pp[:,2]>0);valid=inside&finite
    p=pp[valid];bc=bp[valid];assert len(p)>100
    center=np.median(p,0);u,sv,v=np.linalg.svd(p-center,full_matrices=False);normal=v[-1]
    if normal@np.array(old['B'])[:3,2]<0:normal=-normal
    offset=np.median(p@normal);res=p@normal-offset
    # Trim once at 0.8mm to expose sensitivity, not to silently discard inconvenient depth.
    trim=np.abs(res)<.0008;p2=p[trim];_,_,v2=np.linalg.svd(p2-p2.mean(0),full_matrices=False);normal2=v2[-1]
    if normal2@normal<0:normal2=-normal2
    nbase=A[:3,:3]@X[:3,:3]@normal2
    dplane={'valid_points':len(p),'valid_fraction_inside':float(valid.sum()/inside.sum()),'rms_mm':float(np.sqrt(np.mean(res**2))*1000),'p95_abs_mm':float(np.percentile(np.abs(res),95)*1000),'trimmed_count':len(p2),'trimmed_rms_mm':float(np.sqrt(np.mean(((p2-p2.mean(0))@normal2)**2))*1000),'camera_normal':normal2.tolist(),'base_normal':nbase.tolist(),'trim_normal_change_deg':float(np.degrees(np.arccos(np.clip(normal@normal2,-1,1))))}
    tiles=[]
    for a in range(4):
        for b in range(3):
            mask=(bc[:,0]>=.002+a*.011)&(bc[:,0]<.002+(a+1)*.011)&(bc[:,1]>=.002+b*(.026/3))&(bc[:,1]<.002+(b+1)*(.026/3))
            tiles.append({'column':a,'row':b,'count':int(mask.sum()),'median_plane_offset_mm':float(np.median(res[mask])*1000) if mask.any() else None})
    dplane['tile_offsets']=tiles;row['dense_plane']=dplane
    results.append(row);print(g,old['id'],'SB/classic',round(row['methods']['SB']['pnp_rms_px'],3),round(row['methods']['classic_w7']['pnp_rms_px'],3),'plane',round(dplane['rms_mm'],3),flush=True)
methods=list(results[0]['methods']);groups={}
for g in R['groups']:
    rr=[x for x in results if x['group']==g];groups[g]={}
    for name in methods:
        v=[x['methods'][name] for x in rr];groups[g][name]={'C':avg([np.array(x['C']) for x in v]).tolist(),'pnp_rms_px_mean':float(np.mean([x['pnp_rms_px'] for x in v])),'homography_rms_px_mean':float(np.mean([x['homography_rms_px'] for x in v])),'parity_energy_fraction_mean':float(np.mean([x['parity_energy_fraction'] for x in v])),'repeat_rms':np.sqrt(np.mean(np.array([delta(avg([np.array(y['C']) for y in v]),np.array(x['C'])) for x in v])**2,axis=0)).tolist()}
    normals=np.array([x['dense_plane']['base_normal'] for x in rr]);mean=normals.mean(0);mean/=np.linalg.norm(mean)
    groups[g]['dense_plane']={'base_normal':mean.tolist(),'repeat_angle_rms_deg':float(np.sqrt(np.mean(np.degrees(np.arccos(np.clip(normals@mean,-1,1)))**2))),'rms_mm_mean':float(np.mean([x['dense_plane']['rms_mm'] for x in rr])),'valid_fraction_mean':float(np.mean([x['dense_plane']['valid_fraction_inside'] for x in rr]))}
pairs={}
for ga,gb in [('pose-change','translation-only'),('pose-change','return-to-pose'),('return-to-pose','xminus-160'),('return-to-pose','x-return')]:
    pair={name:delta(np.array(groups[ga][name]['C']),np.array(groups[gb][name]['C'])) for name in methods}
    na=np.array(groups[ga]['dense_plane']['base_normal']);nb=np.array(groups[gb]['dense_plane']['base_normal']);pair['dense_plane_normal_deg']=float(np.degrees(np.arccos(np.clip(na@nb,-1,1))));pairs[ga+' -> '+gb]=pair
report={'method_notes':'Dense plane is PCA, trimmed once at abs plane residual <0.8mm; shared ROI with image homography. Parity fit is diagnostic and not a correction. No calibration changes.','groups':groups,'pairs':pairs,'samples':results}
(OUT/'diagnostics.json').write_text(json.dumps(report,indent=2))
with (OUT/'detector-comparison.csv').open('w',encoding='utf-8-sig',newline='') as f:
    w=csv.writer(f);w.writerow(['pair','method','translation_mm','rotation_deg'])
    for pair,vals in pairs.items():
        for m in methods:w.writerow([pair,m,*vals[m]])
print(json.dumps({'pairs':pairs,'group_quality':{g:{m:{k:v for k,v in val.items() if k!='C'} for m,val in methods.items()} for g,methods in groups.items()}},indent=2))
