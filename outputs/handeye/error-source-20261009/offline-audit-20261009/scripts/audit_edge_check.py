"""Exploratory grayscale edge-line intersection check; no hardware/network."""
import pathlib,sys,json
ROOT=pathlib.Path(__file__).parent/'offline-audit-0489e5d'
sys.path.insert(0,str(ROOT/'python-deps'))
import cv2,numpy as np
from scipy.spatial.transform import Rotation
data=json.loads((ROOT/'results/frozen/validation.json').read_text())
OBJ=np.zeros((54,3));OBJ[:,:2]=np.mgrid[0:9,0:6].T.reshape(-1,2)*.006
def average(ts):
    a=np.eye(4);a[:3,3]=np.mean([t[:3,3] for t in ts],0);u,_,v=np.linalg.svd(np.sum([t[:3,:3] for t in ts],0));a[:3,:3]=u@np.diag([1,1,np.linalg.det(u@v)])@v;return a
def distance(a,b):return [float(np.linalg.norm(a[:3,3]-b[:3,3])*1000),float(np.degrees(Rotation.from_matrix(a[:3,:3].T@b[:3,:3]).magnitude()))]
rows=[]
for s in data['samples']:
    folder=ROOT/'outputs/handeye/error-source-20261009'
    if s['group']!='initial':folder=folder/s['group']
    gray=cv2.imdecode(np.fromfile(folder/'samples'/s['id']/'image.png',np.uint8),cv2.IMREAD_GRAYSCALE).astype(np.float32)
    c=np.array(s['corners_px']);grid=c.reshape(6,9,2);out=[];quality=[]
    for idx,p in enumerate(c):
        y,x=divmod(idx,9);dirs=[grid[y,min(x+1,8)]-grid[y,max(x-1,0)],grid[min(y+1,5),x]-grid[max(y-1,0),x]];lines=[]
        for direction in dirs:
            direction=direction/np.linalg.norm(direction);normal=np.array([-direction[1],direction[0]]);points=[]
            for along in np.r_[np.arange(-8,-3,.5),np.arange(3.5,8.5,.5)]:
                offsets=np.arange(-3,3.01,.2);xy=p+along*direction+offsets[:,None]*normal
                values=cv2.remap(gray,xy[:,0].astype(np.float32)[None,:],xy[:,1].astype(np.float32)[None,:],cv2.INTER_LINEAR).ravel()
                grad=np.abs(np.gradient(values,.2));j=int(np.argmax(grad[2:-2]))+2
                # Local derivative centroid, polarity-independent, without cornerSubPix.
                ix=np.arange(max(0,j-2),min(len(grad),j+3));w=grad[ix];o=float(np.sum(offsets[ix]*w)/np.sum(w));points.append(p+along*direction+o*normal)
            pts=np.array(points);center=pts.mean(0);_,_,v=np.linalg.svd(pts-center);n=v[-1];lines.append((n,float(n@center)));quality.append(float(np.sqrt(np.mean(((pts-center)@n)**2))))
        out.append(np.linalg.solve(np.array([z[0] for z in lines]),np.array([z[1] for z in lines])))
    out=np.array(out);k=np.array(s['k']);d=np.array(s['d']);ok,rv,t=cv2.solvePnP(OBJ,out,k,d);assert ok
    B=np.eye(4);B[:3,:3]=cv2.Rodrigues(rv)[0];B[:3,3]=t.ravel();A=np.array(s['A']);oldB=np.array(s['B']);X=np.linalg.inv(A)@np.array(s['C'])@np.linalg.inv(oldB);C=A@X@B
    proj=cv2.projectPoints(OBJ,rv,t,k,d)[0].reshape(-1,2);rms=lambda z:float(np.sqrt(np.mean(np.sum(z*z,1))))
    rows.append({'id':s['id'],'group':s['group'],'corner_difference_rms_px':rms(out-c),'edge_line_fit_rms_px':float(np.sqrt(np.mean(np.array(quality)**2))),'pnp_rms_px':rms(proj-out),'C':C.tolist()})
groups={g:average([np.array(s['C']) for s in rows if s['group']==g]) for g in data['groups']}
pairs={a+' -> '+b:distance(groups[a],groups[b]) for a,b in [('pose-change','translation-only'),('pose-change','return-to-pose'),('return-to-pose','xminus-160'),('return-to-pose','x-return')]}
result={'status':'exploratory; ROI directions initialized from frozen corners; not an independent accuracy reference','samples':rows,'pairs_mm_deg':pairs,'group_means':{g:{k:float(np.mean([s[k] for s in rows if s['group']==g])) for k in ['corner_difference_rms_px','edge_line_fit_rms_px','pnp_rms_px']} for g in groups}}
(ROOT/'results/frozen/edge-check.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='samples'},indent=2))
