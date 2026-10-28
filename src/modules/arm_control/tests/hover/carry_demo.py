"""Short, low-speed physical carry after independently verified grasp."""
import math
import numpy as np
from known_target import quintic_segment

class CarryRoute:
    def __init__(self,start,goal,now):
        self.start=np.asarray(start,dtype=float);self.goal=np.asarray(goal,dtype=float);self.began=float(now)
        if any(p.shape!=(3,) for p in (self.start,self.goal)) or not np.isfinite([*self.start,*self.goal,now]).all():
            raise ValueError('Invalid carry endpoints/time')
        distance=float(np.linalg.norm(self.goal-self.start))
        if not .10<=distance<=.90 or self.goal[2]<self.start[2]+.08 or not .3<=self.goal[2]<=2.5:
            raise ValueError('Carry must rise >=80mm and travel 0.10..0.90m within the demo height envelope')
        # Extract vertically before lateral travel; smooth velocity/acceleration
        # at both ends. Peak 30mm/s leaves margin below unchanged 50mm/s gate.
        self.corner=self.start.copy();self.corner[2]=self.goal[2]
        self.durations=[max(4.,1.875*float(np.linalg.norm(b-a))/.03)
            for a,b in ((self.start,self.corner),(self.corner,self.goal))]
        self.duration=sum(self.durations)
        if self.duration>90:raise ValueError('Carry route exceeds bounded duration')

    def sample(self,now):
        if not math.isfinite(now) or now<self.began:raise ValueError('Invalid carry clock')
        dt=now-self.began
        if dt<=self.durations[0]:return quintic_segment(self.start,self.corner,dt,self.durations[0])
        return quintic_segment(self.corner,self.goal,dt-self.durations[0],self.durations[1])

    def finished(self,now):return now-self.began>=self.duration


def carry_evidence(rows,reference):
    if not rows:return dict(validated=False,samples=0)
    frames=sum(r['contact_frames'] for r in rows)
    bilateral=sum(r['contact_frames']*r['bilateral'] for r in rows)/max(1,frames)
    support=sum(r['contact_frames']*r['support'] for r in rows)/max(1,frames)
    drift=max(float(np.linalg.norm(np.asarray(r['object_local'])-reference)) for r in rows)
    duration=rows[-1]['sim_s']-rows[0]['sim_s']
    gaps=[b['sim_s']-a['sim_s'] for a,b in zip(rows,rows[1:])]
    displacement=float(np.linalg.norm(np.asarray(rows[-1]['object_world'][:3])-rows[0]['object_world'][:3]))
    valid=bool(duration>=5 and displacement>=.10 and gaps and all(0<g<=.3 for g in gaps) and frames and bilateral>=.9 and support<=.001 and drift<=.045 and
        all(r['phase']==7 and r['geometry_valid'] and not r['forbidden_contact'] for r in rows))
    return dict(validated=valid,samples=len(rows),contact_frames=frames,bilateral_fraction=bilateral,
        support_fraction=support,maximum_local_payload_drift_m=drift,
        observed_sim_seconds=duration,payload_displacement_m=displacement,maximum_observation_gap_s=max(gaps,default=None))
