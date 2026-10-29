"""Bounded climb then translation in the shared LIO/EKF map; no transport."""
import math
import numpy as np
from known_target import quintic_segment,vector


def wrap_yaw(angle):
    if not math.isfinite(angle):raise ValueError('Invalid yaw')
    return math.atan2(math.sin(angle),math.cos(angle))


class CandidateRoute:
    """A short folded-arm navigation leg, not an obstacle avoidance planner."""
    def __init__(self,start,goal,start_yaw,goal_yaw,began):
        self.start=vector(start,3,'route start').copy()
        self.goal=vector(goal,3,'route goal').copy()
        self.start_yaw=wrap_yaw(start_yaw)
        self.yaw_delta=wrap_yaw(goal_yaw-self.start_yaw)
        if not math.isfinite(began):raise ValueError('Invalid route start time')
        self.began=float(began)
        if self.goal[2]<self.start[2] or np.linalg.norm(self.goal-self.start)>2.:
            raise ValueError('Candidate route requires a climb within 2 m')
        self.top=self.start.copy();self.top[2]=self.goal[2]
        # The quintic blend derivative peaks at 1.875. Account for this rather
        # than treating distance/duration as the peak speed.
        self.climb_duration=max(4.,1.875*(self.goal[2]-self.start[2])/.10)
        self.travel_duration=max(4.,1.875*float(np.linalg.norm(self.goal-self.top))/.15,
            1.875*abs(self.yaw_delta)/.15)
        self.duration=self.climb_duration+self.travel_duration

    def _elapsed(self,now):
        if not math.isfinite(now) or now<self.began:raise ValueError('Invalid route clock')
        return float(now-self.began)

    def sample(self,now):
        elapsed=self._elapsed(now)
        if elapsed<=self.climb_duration:
            position,velocity=quintic_segment(self.start,self.top,elapsed,self.climb_duration)
        else:
            position,velocity=quintic_segment(self.top,self.goal,elapsed-self.climb_duration,self.travel_duration)
        u=float(np.clip(elapsed/self.duration,0.,1.))
        blend=10*u**3-15*u**4+6*u**5
        yaw=wrap_yaw(self.start_yaw+blend*self.yaw_delta)
        return position,velocity,yaw

    def finished(self,now):
        return self._elapsed(now)>=self.duration


def check_frame_agreement(lio_map,ekf_local):
    """Compare time-paired body poses; never infer or apply an origin correction.

    Freshness, matching source times and frame names are the caller's job.
    A finite discrepancy returns valid=False. Malformed transforms raise.
    """
    matrices=[]
    for value in (lio_map,ekf_local):
        matrix=np.asarray(value,dtype=float)
        if (matrix.shape!=(4,4) or not np.isfinite(matrix).all() or
                not np.allclose(matrix[3],[0.,0.,0.,1.],atol=1e-9) or
                not np.allclose(matrix[:3,:3].T@matrix[:3,:3],np.eye(3),atol=1e-6) or
                not np.isclose(np.linalg.det(matrix[:3,:3]),1.,atol=1e-6)):
            raise ValueError('Invalid navigation body pose')
        matrices.append(matrix)
    lio,ekf=matrices
    position=float(np.linalg.norm(lio[:3,3]-ekf[:3,3]))
    angle=math.degrees(math.acos(float(np.clip((np.trace(lio[:3,:3].T@ekf[:3,:3])-1)/2,-1.,1.))))
    return dict(valid=bool(position<=.08 and angle<=5.),position_error_m=position,
        rotation_error_deg=angle,position_limit_m=.08,rotation_limit_deg=5.,
        origin_correction_applied=False)
