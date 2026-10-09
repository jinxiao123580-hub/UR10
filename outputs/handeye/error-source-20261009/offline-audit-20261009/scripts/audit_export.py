import csv,json,pathlib
ROOT=pathlib.Path(__file__).parent/'offline-audit-0489e5d'
r=json.loads((ROOT/'results/audit.json').read_text())
sample_fields=['group','id','frames','rms_px','max_px','corners_delta_stored_rms_px','depth_count','depth_xyz_vs_pnp_rms_mm','depth_fit_rms_mm']
with (ROOT/'results/samples.csv').open('w',encoding='utf-8-sig',newline='') as f:
    w=csv.DictWriter(f,fieldnames=sample_fields);w.writeheader()
    for row in r['samples']:w.writerow({key:row[key] for key in sample_fields})
rows=[]
for name,p in r['pairs'].items():
    row={'pair':name}
    for key in ['A','C','CD']:
        row[key+'_translation_mm'],row[key+'_rotation_deg']=p[key+'_distance']
        for axis,val in zip(['x','y','z'],p[key+'_delta_xyz_mm']):row[key+'_delta_'+axis+'_mm']=val
    rows.append(row)
with (ROOT/'results/pairs.csv').open('w',encoding='utf-8-sig',newline='') as f:
    w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
print('Exported samples.csv and pairs.csv')
