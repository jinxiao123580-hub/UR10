"""Hardware-free audit. Reads commit-pinned local files; writes only new results."""
import sys, pathlib, json, ast, hashlib, platform, time
ROOT=pathlib.Path(__file__).parent/'offline-audit-0489e5d'
sys.path.insert(0,str(ROOT/'python-deps'))
import numpy as np, cv2, scipy, yaml
from scipy.spatial.transform import Rotation
GROUPS=['initial','pose-change','translation-only','return-to-pose','xminus-160','x-return']
EXP=ROOT/'outputs/handeye/error-source-20261009'
OUT=ROOT/'results'; OUT.mkdir(exist_ok=True)
def rt(r,t):
    a=np.eye(4); a[:3,:3]=cv2.Rodrigues(np.asarray(r,dtype=float))[0]; a[:3,3]=t; return a
def avg(ts):
    a=np.eye(4); a[:3,3]=np.mean([t[:3,3] for t in ts],axis=0)
    u,_,v=np.linalg.svd(np.sum([t[:3,:3] for t in ts],axis=0)); a[:3,:3]=u@np.diag([1,1,np.linalg.det(u@v)])@v; return a
def dist(a,b):
    return [float(np.linalg.norm(b[:3,3]-a[:3,3])*1000),float(np.degrees(np.linalg.norm(Rotation.from_matrix(a[:3,:3].T@b[:3,:3]).as_rotvec())))]
def rms(x): return float(np.sqrt(np.mean(np.asarray(x)**2)))
def fit(p,q):
    pc=p.mean(0); qc=q.mean(0); u,sv,v=np.linalg.svd((p-pc).T@(q-qc)); r=v.T@np.diag([1,1,np.linalg.det(v.T@u.T)])@u.T
    t=qc-r@pc; a=np.eye(4); a[:3,:3]=r; a[:3,3]=t
    residual=q-(p@r.T+t); return a,residual,sv
def solve(obj,c,k,d,flag=cv2.SOLVEPNP_ITERATIVE):
    ok,r,t=cv2.solvePnP(obj,np.asarray(c,dtype=float),k,d,flags=flag)
    assert ok; proj,j=cv2.projectPoints(obj,r,t,k,d); residual=proj.reshape(-1,2)-c
    return rt(r,t.reshape(3)),residual,j[:,:6]
def canon(c,stored):
    variants=[c,c[::-1],c.reshape(6,9,2)[:,::-1].reshape(-1,2),c.reshape(6,9,2)[::-1].reshape(-1,2)]
    return min(variants,key=lambda x:np.linalg.norm(x-stored))
def depth_points(xyz,c,radius):
    points=[]; ids=[]; sizes=[]
    for i,(u,v) in enumerate(c):
        u,v=int(round(u)),int(round(v)); p=xyz[max(0,v-radius):v+radius+1,max(0,u-radius):u+radius+1].reshape(-1,3)
        p=p[np.all(np.isfinite(p),axis=1)&(p[:,2]>0)]
        if len(p)>=3: points.append(np.median(p,axis=0)); ids.append(i); sizes.append(len(p))
    return np.array(points),np.array(ids,dtype=int),sizes
source=ast.parse((ROOT/'scripts/check_handeye_checkerboard.py').read_text(encoding='utf-8'))
node=next(n for n in source.body if isinstance(n,ast.FunctionDef) and n.name=='detect')
env={'cv2':cv2,'np':np}; exec(compile(ast.Module(body=[node],type_ignores=[]),'<offline detector>','exec'),env)
detector=env['detect']
calpath=ROOT/'config/handeye_eye_in_hand_20260921.yaml'; before=hashlib.sha256(calpath.read_bytes()).hexdigest()
cal=yaml.safe_load(calpath.read_text()); X=np.eye(4); X[:3,:3]=cal['rotation_matrix']; X[:3,3]=cal['translation_m']
rng=np.random.default_rng(20261009)
obj=np.zeros((54,3)); obj[:,:2]=np.mgrid[0:9,0:6].T.reshape(-1,2)*.006
records=[]; summaries={}
for group in GROUPS:
    folder=EXP if group=='initial' else EXP/group
    dataset=json.loads((folder/'dataset.json').read_text())
    local=[]
    for s in dataset['samples']:
        if not s.get('accepted'):continue
        sid=s['sample_id']; folder_s=folder/'samples'/sid
        deadline=time.time()+1800
        while not (ROOT/'download-manifest.json').exists():
            if time.time()>deadline: raise TimeoutError('raw download incomplete')
            time.sleep(2)
        image=cv2.imdecode(np.fromfile(folder_s/'image.png',dtype=np.uint8),cv2.IMREAD_COLOR); assert image is not None and image.shape[:2]==(1024,1280)
        sample=json.loads((folder_s/'sample.json').read_text()); assert sample['sample_id']==sid
        frames=json.loads((folder_s/'robot_frames.json').read_text()); f=frames['frames']; assert len(f)==frames['frame_count']==s['robot']['frame_count']
        tcp=np.array([x['tcp_pose'] for x in f]); q=np.array([x['q_rad'] for x in f]); assert np.all(np.isfinite(tcp)) and np.all(np.isfinite(q))
        A=rt(tcp[:,3:].mean(0),tcp[:,:3].mean(0)); Ar=avg([rt(x[3:],x[:3]) for x in tcp])
        stored=np.asarray(s['corners_px']); gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        found,c=detector(gray,(9,6)); assert found; c=canon(c.reshape(-1,2),stored)
        k=np.array(s['camera_info']['k']).reshape(3,3); d=np.array(s['camera_info']['d'])
        B,res,j=solve(obj,c,k,d); storedB=rt(s['target_to_camera']['rvec_rad'],s['target_to_camera']['translation_m'])
        C=A@X@B
        with np.load(folder_s/'cloud.npz',allow_pickle=False) as z:
            for key in z.files: z[key] # decompress every member
            xyz=z['camera_xyz_m'].astype(float); assert xyz.shape==(1024,1280,3)
            assert str(z['sample_id'].item())==sid
            npzA=z['base_from_tool0'].copy(); npzB=z['camera_from_target'].copy()
        pts,ids,sizes=depth_points(xyz,c,3); assert len(ids)>=3
        D,dres,sv=fit(obj[ids],pts); CD=A@X@D
        expected=obj[ids]@B[:3,:3].T+B[:3,3]
        # Homoscedastic linear covariance is a sensitivity measure, not calibrated uncertainty.
        cov=np.linalg.pinv(j.T@j)*(.05**2)
        mc=[]
        for _ in range(80):
            bn,_,_=solve(obj,c+rng.normal(0,.05,c.shape),k,d); mc.append(dist(B,bn))
        boot=[]
        for _ in range(120):
            ii=rng.integers(0,len(ids),len(ids)); dn,_,_=fit(obj[ids[ii]],pts[ii]); boot.append(dist(D,dn))
        radii={}
        for radius in [1,2,3,5]:
            pp,ii,_=depth_points(xyz,c,radius)
            if len(ii)>=3:
                dd,rr,_=fit(obj[ii],pp); radii[str(radius)]={'count':len(ii),'fit_rms_mm':rms(np.linalg.norm(rr,axis=1)*1000),'delta_from_r3':dist(D,dd),'C':(A@X@dd).tolist()}
        classic_ok,classic=cv2.findChessboardCorners(gray,(9,6),cv2.CALIB_CB_ADAPTIVE_THRESH|cv2.CALIB_CB_NORMALIZE_IMAGE)
        classic_record=None
        if classic_ok:
            cc=cv2.cornerSubPix(gray,classic,(7,7),(-1,-1),(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_MAX_ITER,50,1e-4))
            cc=canon(cc.reshape(-1,2),stored); cb,cr,_=solve(obj,cc,k,d)
            classic_record={'rms_px':rms(np.linalg.norm(cr,axis=1)),'delta_pnp':dist(B,cb),'C':(A@X@cb).tolist()}
        ippe=[]
        ok,rs,ts,_=cv2.solvePnPGeneric(obj,c,k,d,flags=cv2.SOLVEPNP_IPPE)
        for ri,ti in zip(rs,ts):
            pp,_=cv2.projectPoints(obj,ri,ti,k,d); ippe.append({'rms_px':rms(np.linalg.norm(pp.reshape(-1,2)-c,axis=1)),'delta_iterative':dist(B,rt(ri,ti.reshape(3)))})
        camera_tests={}
        for name,kk,dd in [('focal_plus_1pct',k.copy(),d.copy()),('cx_plus_1px',k.copy(),d.copy()),('k1_plus_.01',k.copy(),d.copy())]:
            if name.startswith('focal'):kk[0,0]*=1.01;kk[1,1]*=1.01
            elif name.startswith('cx'):kk[0,2]+=1
            else:dd[0]+=.01
            bb,rr,_=solve(obj,c,kk,dd); camera_tests[name]={'C':(A@X@bb).tolist(),'rms_px':rms(np.linalg.norm(rr,axis=1))}
        row={'group':group,'id':sid,'frames':len(f),'image_shape':list(image.shape),'cloud_shape':list(xyz.shape),'cloud_finite_fraction':float(np.mean(np.all(np.isfinite(xyz),axis=2))),
             'tcp_mean_maxdiff':float(np.max(np.abs(tcp.mean(0)-s['robot']['base_to_tool0_tcp_mean']))),'tcp_rotation_average_difference':dist(A,Ar),
             'tcp_max_position_motion_mm':float(np.max(np.linalg.norm(tcp[:,:3]-tcp[:,:3].mean(0),axis=1))*1000),'tcp_max_angle_from_first_deg':max(dist(rt(tcp[0,3:],tcp[0,:3]),rt(x[3:],x[:3]))[1] for x in tcp),
             'corners_delta_stored_rms_px':rms(np.linalg.norm(c-stored,axis=1)),'pnp_delta_stored':dist(storedB,B),'npz_tcp_delta':dist(A,npzA),'npz_pnp_delta':dist(B,npzB),
             'rms_px':rms(np.linalg.norm(res,axis=1)),'max_px':float(np.max(np.linalg.norm(res,axis=1))), 'residual_px':res.tolist(),'corners_px':c.tolist(),
             'A':A.tolist(),'B':B.tolist(),'C':C.tolist(),'D':D.tolist(),'CD':CD.tolist(),'depth_count':len(ids),'depth_indices':ids.tolist(),
             'depth_xyz_vs_pnp_rms_mm':rms(np.linalg.norm(pts-expected,axis=1)*1000),'depth_fit_rms_mm':rms(np.linalg.norm(dres,axis=1)*1000),'depth_pnp_delta':dist(B,D),
             'depth_span_mm':(np.ptp(obj[ids,:2],axis=0)*1000).tolist(),'depth_design_singular_values':sv.tolist(),
             'pixel_noise_.05px_mc_rms':np.sqrt(np.mean(np.array(mc)**2,axis=0)).tolist(),'linear_cov_rotation_std_deg':(np.sqrt(np.diag(cov)[:3])*180/np.pi).tolist(),'linear_cov_translation_std_mm':(np.sqrt(np.diag(cov)[3:])*1000).tolist(),
             'depth_bootstrap_rms':np.sqrt(np.mean(np.array(boot)**2,axis=0)).tolist(),'depth_radii':radii,'classic':classic_record,'ippe':ippe,'camera_sensitivity':camera_tests}
        records.append(row);local.append(row)
        print(group,sid,'px',round(row['rms_px'],4),'depth',len(ids),flush=True)
    summary={'n':len(local),'rms_px_mean':float(np.mean([r['rms_px'] for r in local])),'depth_count':[r['depth_count'] for r in local],
             'depth_xyz_vs_pnp_rms_mm_mean':float(np.mean([r['depth_xyz_vs_pnp_rms_mm'] for r in local])),'depth_fit_rms_mm_mean':float(np.mean([r['depth_fit_rms_mm'] for r in local]))}
    for key in ['A','B','C','CD']:
        mean=avg([np.array(r[key]) for r in local]); errors=np.array([dist(mean,np.array(r[key])) for r in local])
        summary[key]=mean.tolist(); summary[key+'_repeat_rms']=np.sqrt(np.mean(errors**2,axis=0)).tolist()
    summaries[group]=summary
    (OUT/'partial.json').write_text(json.dumps({'samples':records,'groups':summaries},indent=2))
pairs={}
for ga,gb in [('initial','pose-change'),('pose-change','translation-only'),('translation-only','return-to-pose'),('pose-change','return-to-pose'),('return-to-pose','xminus-160'),('xminus-160','x-return'),('return-to-pose','x-return'),('pose-change','xminus-160'),('pose-change','x-return')]:
    v={}
    for key in ['A','C','CD']:
        aa=np.array(summaries[ga][key]);bb=np.array(summaries[gb][key]);v[key+'_distance']=dist(aa,bb);v[key+'_delta_xyz_mm']=((bb[:3,3]-aa[:3,3])*1000).tolist()
        if key!='A':
            center=np.array([.024,.015,0]);v[key+'_center_delta_mm']=float(np.linalg.norm(bb[:3,:3]@center+bb[:3,3]-aa[:3,:3]@center-aa[:3,3])*1000)
    aa=np.array(summaries[ga]['A']);bb=np.array(summaries[gb]['A']);ba=np.array(summaries[ga]['B']);bc=np.array(summaries[gb]['B'])
    v['camera_translation_length_mm']=float(np.linalg.norm(bc[:3,3]-ba[:3,3])*1000)
    v['translation_length_mismatch_mm']=v['camera_translation_length_mm']-v['A_distance'][0]
    v['camera_rotation_deg']=dist(ba,bc)[1]
    v['fixed_extrinsic_rotation_orientation_lower_bound_deg']=max(0,v['camera_rotation_deg']-v['A_distance'][1])
    v['50mm_handeye_translation_error_max_effect_mm']=2*50*np.sin(np.radians(v['A_distance'][1])/2)
    pairs[ga+' -> '+gb]=v
camera_sensitivity={}
for name in records[0]['camera_sensitivity']:
    gs={g:avg([np.array(r['camera_sensitivity'][name]['C']) for r in records if r['group']==g]) for g in GROUPS}
    camera_sensitivity[name]={ga+' -> '+gb:dist(gs[ga],gs[gb]) for ga,gb in [('pose-change','translation-only'),('return-to-pose','xminus-160')]}
assert before==hashlib.sha256(calpath.read_bytes()).hexdigest()
report={'commit':'0489e5d505f3924ae5eb9cd39b791f70de69646e','versions':{'python':platform.python_version(),'numpy':np.__version__,'opencv':cv2.__version__,'scipy':scipy.__version__},'calibration_sha256':before,'samples':records,'groups':summaries,'pairs':pairs,'camera_sensitivity':camera_sensitivity}
(OUT/'audit.json').write_text(json.dumps(report,indent=2))
print(json.dumps({'groups':summaries,'pairs':pairs,'camera_sensitivity':camera_sensitivity},indent=2))
