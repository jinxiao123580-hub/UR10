"""Copy hash-verified archived inputs and run offline analysis in a NEW directory."""
import argparse,pathlib,json,hashlib,shutil,subprocess,sys
p=argparse.ArgumentParser();p.add_argument('--repo',type=pathlib.Path,required=True);p.add_argument('--work',type=pathlib.Path,required=True);p.add_argument('--verify-only',action='store_true');a=p.parse_args()
HERE=pathlib.Path(__file__).resolve().parent
a.repo=a.repo.resolve();a.work=a.work.resolve()
manifest=json.loads((HERE/'input-manifest.json').read_text())
for f in manifest['files']:
    src=a.repo/f['path']
    if not src.is_file() or hashlib.sha256(src.read_bytes()).hexdigest()!=f['sha256']:raise SystemExit('Missing/modified pinned input: '+str(src))
print('Verified',len(manifest['files']),'pinned input files',flush=True)
if a.verify_only:sys.exit(0)
if a.work.exists():raise SystemExit('--work must not exist; original files are never overwritten')
a.work.mkdir(parents=True);root=a.work/'offline-audit-0489e5d';root.mkdir()
for f in manifest['files']:
    dest=root/f['path'];dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(a.repo/f['path'],dest)
shutil.copy2(HERE/'input-manifest.json',root/'download-manifest.json')
names=['audit_recompute','audit_supplement','audit_export','audit_deeper','audit_ray_geometry','audit_board_pattern','audit_position_explanation','audit_frozen_pipeline','audit_temporal_check','audit_existing_limits','audit_edge_check']
for name in names:shutil.copy2(HERE/'scripts'/(name+'.py'),a.work/(name+'.py'))
shutil.copy2(HERE/'scripts/audit_download.py',a.work/'audit_download.py')
for name in names:
    print('RUN',name,flush=True);subprocess.run([sys.executable,str(a.work/(name+'.py'))],cwd=a.work,check=True)
print('Results:',root/'results')
