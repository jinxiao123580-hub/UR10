"""Explanatory fit only: never emits or modifies a calibration file."""
import pathlib,sys,json
ROOT=pathlib.Path(__file__).parent/'offline-audit-0489e5d';sys.path.insert(0,str(ROOT/'python-deps'))
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
a=json.loads((ROOT/'results/audit.json').read_text());d=json.loads((ROOT/'results/deeper/diagnostics.json').read_text())
s=a['samples'][0];X=np.linalg.inv(np.array(s['A']))@np.array(s['C'])@np.linalg.inv(np.array(s['B']))
pairs=[('pose-change','translation-only'),('return-to-pose','xminus-160')];results={}
for method in ['SB','classic_w7']:
    As={g:np.array(a['groups'][g]['A']) for g in d['groups']}
    Bs={g:np.linalg.inv(As[g]@X)@np.array(d['groups'][g][method]['C']) for g in d['groups']}
    def residual(v):
        xx=X.copy();xx[:3,:3]=Rotation.from_rotvec(v).as_matrix()@X[:3,:3]
        return np.concatenate([(As[j]@xx@Bs[j])[:3,3]-(As[i]@xx@Bs[i])[:3,3] for i,j in pairs])*1000
    fit=least_squares(residual,np.zeros(3));errors=residual(fit.x).reshape(-1,3)
    results[method]={'explanatory_rotation_vector_deg':np.degrees(fit.x).tolist(),'angle_deg':float(np.degrees(np.linalg.norm(fit.x))),'remaining_translation_residual_mm':np.linalg.norm(errors,axis=1).tolist(),'remaining_xyz_mm':errors.tolist(),'camera_vs_reported_tcp_length_mismatch_mm':[float(np.linalg.norm(Bs[j][:3,3]-Bs[i][:3,3])*1000-a['pairs'][i+' -> '+j]['A_distance'][0]) for i,j in pairs]}
report={'warning':'Fit uses only the two outgoing translation pairs. It demonstrates explanatory ability, not identification of a true extrinsic error. Do not apply fitted values.','pairs':pairs,'results':results}
(ROOT/'results/deeper/position-explanation.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
