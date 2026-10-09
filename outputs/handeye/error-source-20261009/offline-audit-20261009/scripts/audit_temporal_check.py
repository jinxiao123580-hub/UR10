"""Time-associated diagnostics; estimates are not compensation/calibration."""
import pathlib,sys,json,csv
ROOT=pathlib.Path(__file__).parent/'offline-audit-0489e5d';sys.path.insert(0,str(ROOT/'python-deps'))
import numpy as np
from scipy.spatial.transform import Rotation
r=json.loads((ROOT/'results/frozen/validation.json').read_text());OUT=ROOT/'results/frozen'
near=[s for s in r['samples'] if s['group'] in ['pose-change','return-to-pose','x-return']]
origin=min(s['timestamp_s'] for s in near)
t=np.array([(s['timestamp_s']-origin)/60 for s in near]);design=np.column_stack((np.ones(len(t)),t));fits={}
for key in ['C','CD']:
    y=np.array([s['C'] if key=='C' else s['depths']['3']['CD'] for s in near])[:,:3,3]*1000
    coef=np.linalg.lstsq(design,y,rcond=None)[0];err=y-design@coef
    fits[key]={'slope_xyz_mm_per_min':coef[1].tolist(),'slope_norm_mm_per_min':float(np.linalg.norm(coef[1])),'residual_rms_mm':float(np.sqrt(np.mean(np.sum(err**2,1)))),'R_squared':float(1-np.sum(err**2)/np.sum((y-y.mean(0))**2)),'warning':'9 samples in 3 time clusters; robot moved between clusters. Time and movement history are confounded.'}
burst=[]
for g in r['groups']:
    ss=sorted([s for s in r['samples'] if s['group']==g],key=lambda s:s['timestamp_s']);a,b=ss[0],ss[-1]
    row={'group':g,'count':len(ss),'duration_s':b['timestamp_s']-a['timestamp_s']}
    for key in ['A','C','CD']:
        ta=np.array(a[key] if key!='CD' else a['depths']['3']['CD']);tb=np.array(b[key] if key!='CD' else b['depths']['3']['CD']);delta=(tb[:3,3]-ta[:3,3])*1000
        row[key+'_delta_xyz_mm']=delta.tolist();row[key+'_translation_mm']=float(np.linalg.norm(delta));row[key+'_rotation_deg']=float(np.degrees(Rotation.from_matrix(ta[:3,:3].T@tb[:3,:3]).magnitude()))
    delta_pixel=np.array(b['corners_px'])-np.array(a['corners_px']);row['image_corner_mean_shift_px']=delta_pixel.mean(0).tolist()
    burst.append(row)
alignment={}
for pair,vals in r['pairs'].items():
    p=np.array(vals['C_delta_xyz_mm']);d=np.array(vals['CD_delta_xyz_mm']);alignment[pair]={'cosine_similarity':float(p@d/(np.linalg.norm(p)*np.linalg.norm(d))),'PnP_minus_depth_shift_mm':float(np.linalg.norm(p-d))}
checks={'count_20':len(r['samples'])==20,'all_heldout_pnp_RMS_below_.5px':all(s['pnp_rms_px']<.5 for s in r['samples'] if s['group'] in r['selection']['test_groups']),'corner_identity_preserved':all(s['selected_method_reference_corner_rms_px']<.001 for s in r['samples']),'no_base_closure_used_for_selection':'no base closure' in r['selection']['rule'].lower()}
assert all(checks.values())
report={'checks':checks,'near_return_time_fit':fits,'short_static_bursts':burst,'PnP_depth_shift_alignment':alignment}
(OUT/'temporal.json').write_text(json.dumps(report,indent=2))
with (OUT/'pair-results.csv').open('w',encoding='utf-8-sig',newline='') as f:
    w=csv.writer(f);w.writerow(['pair','time_gap_minutes','TCP_translation_mm','TCP_rotation_deg','PnP_translation_mm','PnP_rotation_deg','depth_translation_mm','depth_rotation_deg','PnP_dx_mm','PnP_dy_mm','PnP_dz_mm'])
    for name,p in r['pairs'].items():w.writerow([name,p['time_gap_minutes'],*p['A_distance'],*p['C_distance'],*p['CD_distance'],*p['C_delta_xyz_mm']])
print(json.dumps(report,indent=2))
