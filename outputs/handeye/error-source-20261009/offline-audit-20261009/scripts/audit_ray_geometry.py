"""Check cloud pixel-ray consistency and board-fixed residual patterns offline."""
import pathlib,sys,json
ROOT=pathlib.Path(__file__).parent/'offline-audit-0489e5d';sys.path.insert(0,str(ROOT/'python-deps'))
import numpy as np,cv2
r=json.loads((ROOT/'results/deeper/diagnostics.json').read_text());records=[]
obj=np.zeros((54,3));obj[:,:2]=np.mgrid[0:9,0:6].T.reshape(-1,2)*.006
for row in r['samples']:
    g=row['group'];folder=ROOT/'outputs/handeye/error-source-20261009'
    if g!='initial':folder=folder/g
    sdir=folder/'samples'/row['id'];s=json.loads((sdir/'sample.json').read_text());k=np.array(s['camera_info']['k']).reshape(3,3)
    z=np.load(sdir/'cloud.npz');xyz=z['camera_xyz_m'].astype(float)
    yy,xx=np.mgrid[0:1024:12,0:1280:12];p=xyz[yy,xx].reshape(-1,3);pixels=np.column_stack((xx.ravel(),yy.ravel())).astype(float)
    valid=np.all(np.isfinite(p),1)&(p[:,2]>.05);p=p[valid];pixels=pixels[valid];rays=p[:,:2]/p[:,2:3]
    pred=rays@np.diag([k[0,0],k[1,1]])+k[:2,2]
    err=pred-pixels;design=np.column_stack((rays,np.ones(len(p))));coeff=np.linalg.lstsq(design,pixels,rcond=None)[0];ferr=design@coeff-pixels
    result={'group':g,'id':row['id'],'global_ray_samples':len(p),'camera_info_projection_rms_px':float(np.sqrt(np.mean(np.sum(err**2,1)))),'camera_info_projection_mean_px':err.mean(0).tolist(),'affine_ray_coefficients':coeff.tolist(),'affine_ray_fit_rms_px':float(np.sqrt(np.mean(np.sum(ferr**2,1)))),'patterns':{}}
    for name in ['SB','classic_w7']:
        c=np.array(row['methods'][name]['corners_px']);H,_=cv2.findHomography(obj[:,:2],c,0)
        mapped=cv2.perspectiveTransform(c.reshape(-1,1,2),np.linalg.inv(H)).reshape(-1,2)
        deviations=(mapped-obj[:,:2])*1000
        result['patterns'][name]={'board_inverse_homography_residual_mm':deviations.tolist(),'rms_mm':float(np.sqrt(np.mean(np.sum(deviations**2,1))))}
    c=np.array(row['methods']['SB']['corners_px']);H,_=cv2.findHomography(obj[:,:2],c,0)
    x0,y0=np.floor(c.min(0)).astype(int);x1,y1=np.ceil(c.max(0)).astype(int);uu,vv=np.meshgrid(np.arange(x0,x1+1),np.arange(y0,y1+1));uv=np.column_stack((uu.ravel(),vv.ravel()))
    board=cv2.perspectiveTransform(uv.astype(float).reshape(-1,1,2),np.linalg.inv(H)).reshape(-1,2)
    inside=(board[:,0]>.002)&(board[:,0]<.046)&(board[:,1]>.002)&(board[:,1]<.028)
    pp=xyz[vv.ravel(),uu.ravel()];finite=np.all(np.isfinite(pp),1)&(pp[:,2]>.05);mask=inside&finite
    pp=pp[mask];bp=board[mask];parity=(-1.)**(np.floor(bp[:,0]/.006).astype(int)+np.floor(bp[:,1]/.006).astype(int))
    normal=np.array(row['dense_plane']['camera_normal']);offset=np.median(pp@normal);e=(pp@normal-offset)*1000
    result['dense_plane_parity']={'median_offset_even_mm':float(np.median(e[parity==1])),'median_offset_odd_mm':float(np.median(e[parity==-1])),'even_minus_odd_mm':float(np.median(e[parity==1])-np.median(e[parity==-1]))}
    records.append(result)
groups={}
for g in r['groups']:
    rr=[x for x in records if x['group']==g];groups[g]={'camera_info_projection_rms_px_mean':float(np.mean([x['camera_info_projection_rms_px'] for x in rr])),'ray_fit_rms_px_mean':float(np.mean([x['affine_ray_fit_rms_px'] for x in rr])),'affine_ray_coefficients_mean':np.mean([x['affine_ray_coefficients'] for x in rr],0).tolist(),'parity_depth_even_minus_odd_mm_mean':float(np.mean([x['dense_plane_parity']['even_minus_odd_mm'] for x in rr]))}
    for name in ['SB','classic_w7']:
        patterns=np.array([x['patterns'][name]['board_inverse_homography_residual_mm'] for x in rr]);mean=patterns.mean(0)
        flat=patterns.reshape(len(patterns),-1);corr=np.corrcoef(flat);groups[g][name]={'mean_pattern_mm':mean.tolist(),'repeat_pattern_correlation_mean':float(np.mean(corr[np.triu_indices(len(rr),1)])),'pattern_rms_mm':float(np.sqrt(np.mean(np.sum(mean**2,1))))}
cross={}
for name in ['SB','classic_w7']:
    matrix=np.array([np.array(groups[g][name]['mean_pattern_mm']).ravel() for g in groups]);cross[name]=np.corrcoef(matrix).tolist()
report={'groups':groups,'cross_group_pattern_correlation_order':list(groups),'cross_group_pattern_correlation':cross,'samples':records}
(ROOT/'results/deeper/ray-geometry.json').write_text(json.dumps(report,indent=2))
print(json.dumps({'groups':{g:{key:(value if key not in ['SB','classic_w7'] else {a:b for a,b in value.items() if a!='mean_pattern_mm'}) for key,value in gg.items()} for g,gg in groups.items()},'cross':cross},indent=2))
