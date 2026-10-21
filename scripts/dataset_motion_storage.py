"""Bounded-memory motion-triggered gzip storage, no hardware dependencies."""
from collections import deque
import csv
import gzip
import math
from pathlib import Path
import threading

FORCE_HEADER=['received_wall_s','received_monotonic_s','source_ros_s','frame_id','Fx_N','Fy_N','Fz_N','Tx_Nm','Ty_Nm','Tz_Nm']
UR_HEADER=['received_wall_s','received_monotonic_s','controller_time_s']+['q%d_rad'%i for i in range(6)]+['tcp_'+n for n in ['x_m','y_m','z_m','rx_rad','ry_rad','rz_rad']]+['velocity_%d'%i for i in range(6)]


class MotionStorage:
    def __init__(self,stack,folder,meta,idle=30.,pre=2.,continuous=False,raw_frames=False,marker=None):
        self.meta,self.idle,self.pre,self.continuous=meta,idle,pre,continuous
        self.lock=threading.RLock();self.active=True;self.last_motion=None;self.anchor=None
        self.marker=marker or (lambda *a:None)
        self.buffers={'force':deque(maxlen=4000),'ur':deque(maxlen=4000)}
        self.pending={};self.raw={};self.preview={};self.files=[]
        self.stats=None
        sf=stack.enter_context(gzip.open(Path(folder)/'idle-force-summary.csv.gz','wt',newline=''))
        self.files.append(sf);self.stats_writer=csv.writer(sf)
        self.stats_writer.writerow(['start_monotonic_s','end_monotonic_s','count']+['mean_'+x for x in ['Fx','Fy','Fz','Tx','Ty','Tz']]+['std_'+x for x in ['Fx','Fy','Fz','Tx','Ty','Tz']])
        self.binary=stack.enter_context(gzip.open(Path(folder)/'ur-frames.bin.gz','wb')) if raw_frames else None
        if self.binary:self.files.append(self.binary)
        for name,header in [('force',FORCE_HEADER),('ur',UR_HEADER)]:
            f=stack.enter_context(gzip.open(Path(folder)/(name+'.csv.gz'),'wt',newline=''))
            s=stack.enter_context(gzip.open(Path(folder)/(name+'-50hz.csv.gz'),'wt',newline=''))
            self.files.extend([f,s]);self.raw[name]=csv.writer(f);self.preview[name]=csv.writer(s)
            self.raw[name].writerow(header);self.preview[name].writerow(header)
        self.meta.update({'storage':'gzip CSV streams','motion_triggered':not continuous,
                          'recording_active':True,
                          'idle_pause_seconds':idle,'pretrigger_seconds':pre,
                          'raw_binary_enabled':raw_frames,'pauses':0,
                          'preview':'50 Hz receipt-clock bins; force components averaged; UR is last sample per bin, NOT anti-aliased simulation input',
                          'force_received':0,'ur_received':0})

    def _write(self,name,row,frame=None):
        self.raw[name].writerow(row);self.meta[name+'_rows']+=1
        if name=='ur' and self.binary:self.binary.write(frame)
        bucket=math.floor(row[1]*50)
        old=self.pending.get(name)
        if old is not None and (old[0]!=bucket or (name=='force' and old[1][-1][3]!=row[3])):
            self._summary(name);old=None
        if old is None:self.pending[name]=[bucket,[]]
        self.pending[name][1].append(row)

    def _summary(self,name):
        old=self.pending.pop(name,None)
        if not old:return
        rows=old[1];row=list(rows[-1])
        if name=='force':
            row[4:]=[sum(r[i] for r in rows)/len(rows) for i in range(4,10)]
        self.preview[name].writerow(row)

    def _idle_stats(self,row):
        if self.stats is None:self.stats=[row[1],row[1],0,[0.]*6,[0.]*6]
        s=self.stats;s[1]=row[1];s[2]+=1
        for i,value in enumerate(row[4:]):
            delta=value-s[3][i];s[3][i]+=delta/s[2];s[4][i]+=delta*(value-s[3][i])
        if s[1]-s[0]>=30:self._flush_stats()

    def _flush_stats(self):
        if self.stats is not None:
            start,end,n,mean,m2=self.stats
            self.stats_writer.writerow([start,end,n,*mean,*[math.sqrt(max(0.,v/n)) for v in m2]])
            self.stats=None

    def accept(self,name,row,frame=None):
        with self.lock:
            now=row[1];self.meta[name+'_received']+=1
            if self.last_motion is None:self.last_motion=now
            if name=='ur':
                q=row[3:9];pos=row[9:12];v=row[15:21]
                moving=(math.sqrt(sum(x*x for x in v[:3]))>=.0001 or math.sqrt(sum(x*x for x in v[3:]))>=.001)
                if self.anchor is not None:
                    aq,ap=self.anchor
                    moving=moving or max(abs(a-b) for a,b in zip(q,aq))>=.0005 or math.sqrt(sum((a-b)**2 for a,b in zip(pos,ap)))>=.0001
                if self.anchor is None or moving:self.anchor=(list(q),list(pos))
                if moving:
                    self.last_motion=now
                    if not self.active:
                        self.active=True;self.marker('recording_resume','motion detected; prebuffer restored')
                        self.meta['recording_active']=True
                        self._flush_stats()
                        for stream,buffer in self.buffers.items():
                            for buffered,binary in buffer:
                                if now-buffered[1]<=self.pre:self._write(stream,buffered,binary)
                            buffer.clear()
            if not self.continuous and self.active and now-self.last_motion>=self.idle:
                self.active=False;self.meta['pauses']+=1
                self.meta['recording_active']=False
                for stream in list(self.pending):self._summary(stream)
                self.marker('recording_pause','stationary timeout; listeners remain active')
            if self.active:self._write(name,row,frame)
            else:
                if name=='force':self._idle_stats(row)
                buffer=self.buffers[name];buffer.append((row,frame if self.binary else None))
                while buffer and now-buffer[0][0][1]>self.pre:buffer.popleft()

    def flush(self):
        with self.lock:
            for f in self.files:f.flush()

    def finish(self):
        with self.lock:
            for name in list(self.pending):self._summary(name)
            self._flush_stats()
            self.flush()
