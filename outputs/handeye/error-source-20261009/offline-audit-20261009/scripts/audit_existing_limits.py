"""Quantify what the current motion excitation can and cannot identify."""
import sys,pathlib,json
ROOT=pathlib.Path(__file__).parent/'offline-audit-0489e5d';sys.path.insert(0,str(ROOT/'python-deps'))
import numpy as np
from scipy.spatial.transform import Rotation
r=json.loads((ROOT/'results/frozen/validation.json').read_text());ts=[np.array(s['A']) for s in r['groups'].values()];rows=[];rotations=[]
for i in range(len(ts)):
    for j in range(i+1,len(ts)):
        ra=ts[i][:3,:3].T@ts[j][:3,:3];rows.append(np.eye(3)-ra);rotations.append(float(np.degrees(Rotation.from_matrix(ra).magnitude())))
sv=np.linalg.svd(np.vstack(rows),compute_uv=False)
report={'group_means_used':list(r['groups']),'handeye_translation_design_singular_values':sv.tolist(),'condition_number':float(sv[0]/sv[-1]),'relative_tool_rotation_deg_all_pairs':rotations,'interpretation':'Only one substantial orientation change; other moves nearly preserve tool orientation. Full hand-eye translation calibration is poorly excited; repeated captures add repeatability, not new pose excitation.'}
(ROOT/'results/frozen/identifiability.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
