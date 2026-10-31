"""Bounded LIO prediction to encoder time, separate from bracket-only audit.

This is a short constant-world-twist control estimate, not a new measurement.
The encoder timestamp, original LIO timestamps and prediction duration remain
explicit. The existing LioToolObserver.sample() never calls this helper.
"""
import math
import numpy as np
from lio_tool_observer import finite,inverse,pose_transform,rpy_transform,interpolate_pose,relative_tool_target


def _cross_matrix(vector):
    x,y,z=vector
    return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])


def _rotation_step(vector):
    angle=float(np.linalg.norm(vector));K=_cross_matrix(vector)
    if angle<1e-8:return np.eye(3)+K+.5*K@K
    return np.eye(3)+math.sin(angle)/angle*K+(1-math.cos(angle))/(angle*angle)*K@K


def predict_body(samples,joint_stamp,max_prediction=.12,max_linear_speed=.3,max_angular_speed=.5):
    """Interpolate a source bracket, or predict <=120ms from two LIO poses.

    Linear and angular velocities are in map coordinates. No timestamp is
    restamped: the returned pose is an estimate at the supplied encoder time.
    Extreme derivatives reject the estimate before any control eligibility.
    Ordinary hover eligibility remains the caller's responsibility.
    """
    if (not math.isfinite(joint_stamp) or joint_stamp<=0 or
            not math.isfinite(max_prediction) or not 0<=max_prediction<=.12 or
            not math.isfinite(max_linear_speed) or not 0<max_linear_speed<=.8 or
            not math.isfinite(max_angular_speed) or not 0<max_angular_speed<=1.):
        raise ValueError('CONTROL_SAMPLE_TIME')
    samples=list(samples)
    if len(samples)<2:raise ValueError('LIO_TWO_POSES_REQUIRED')
    stamps=[float(row[0]) for row in samples]
    if any(not math.isfinite(t) or t<=0 for t in stamps):raise ValueError('LIO_SOURCE_TIME')
    if any(b<=a for a,b in zip(stamps,stamps[1:])):raise ValueError('LIO_SOURCE_NONMONOTONIC')
    poses=[pose_transform(row[1]) for row in samples]
    if joint_stamp<stamps[0]:raise ValueError('LIO_NOT_BRACKETED')
    prediction=max(0.,joint_stamp-stamps[-1])
    if prediction>max_prediction+1e-9:raise ValueError('LIO_PREDICTION_LIMIT')
    if joint_stamp>=stamps[-1]:left=len(samples)-2;right=len(samples)-1
    else:
        right=next(i for i,t in enumerate(stamps) if t>joint_stamp)
        left=right-1
    gap=stamps[right]-stamps[left]
    if gap>.125+1e-9:raise ValueError('LIO_PAIR_GAP')
    a,b=poses[left],poses[right]
    velocity=(b[:3,3]-a[:3,3])/gap
    rotation=b[:3,:3]@a[:3,:3].T
    angle=math.acos(float(np.clip((np.trace(rotation)-1)*.5,-1.,1.)))
    # Source bounds keep pair angles <=.125rad, away from the ambiguous pi log.
    if np.linalg.norm(velocity)>max_linear_speed+1e-9 or angle/gap>max_angular_speed+1e-9:
        raise ValueError('LIO_DERIVATIVE_LIMIT')
    skew=np.array([rotation[2,1]-rotation[1,2],rotation[0,2]-rotation[2,0],rotation[1,0]-rotation[0,1]])
    omega=skew*(.5 if angle<1e-7 else angle/(2*math.sin(angle)))/gap
    if prediction>0:
        body=poses[-1].copy()
        body[:3,3]+=velocity*prediction
        body[:3,:3]=_rotation_step(omega*prediction)@body[:3,:3]
        mode='BOUNDED_CONSTANT_TWIST_PREDICTION'
    else:
        body,_=interpolate_pose(samples,joint_stamp,.125)
        mode='SOURCE_MEASURED' if joint_stamp in stamps else 'SOURCE_BRACKET_INTERPOLATION'
    inverse(body)
    return dict(body=body,linear_v_map=velocity,angular_v_map=omega,
        last_lio_stamp_s=stamps[-1],lio_before_stamp_s=stamps[left],lio_after_stamp_s=stamps[right],
        prediction_s=prediction,mode=mode)



def flight_control_sample(observer,filter_stamp,now):
    """Compare EKF and LIO at the filter's actual sample time.

    Reuse the arm control predictor; the pose remains an estimate, with its
    original LIO stamps and prediction interval exposed to the audit log.
    No target or joint observation is required for flight compensation.
    """
    if not math.isfinite(now) or now<=0:raise ValueError('CURRENT_TIME')
    if not math.isfinite(filter_stamp) or filter_stamp<=0:raise ValueError('FILTER_SOURCE_TIME')
    with observer.lock:
        if observer.error:raise ValueError(observer.error)
        samples=[(float(t),list(p)) for t,p in observer.body if t<=now][-3:]
    if len(samples)<2 or filter_stamp>now:return None
    source_stamp=samples[-1][0]
    if now-filter_stamp>.2+1e-9:raise ValueError('FILTER_STALE_OR_FUTURE')
    if now-source_stamp>.2+1e-9:raise ValueError('FLIGHT_LIO_STALE_OR_FUTURE')
    # A short source gap is an unavailable estimate, not permission to
    # extend prediction. No setpoint/entry decision uses an overlong sample.
    if filter_stamp<samples[0][0] or filter_stamp-samples[-1][0]>.12+1e-9:return None
    result=predict_body(samples,filter_stamp,max_linear_speed=.8,max_angular_speed=1.)
    result.update(stamp_s=filter_stamp,source_age_s=now-source_stamp)
    return result


def midpoint_body_odometry(samples):
    """Pair interval velocity with its midpoint pose/time for external fusion.

    Both source poses already refer to base_link. No extra sensor lever arm,
    extrapolation or receipt-time restamping is applied. The velocity is an
    interval estimate, not FAST-LIO's internal instantaneous state velocity.
    """
    samples=list(samples)
    if len(samples)!=2:raise ValueError('LIO_TWO_POSES_REQUIRED')
    stamp=.5*(float(samples[0][0])+float(samples[1][0]))
    # Transport must cover takeoff too. These are source sanity bounds, not
    # hover permission: sample_control retains its .3m/s / .5rad/s defaults.
    result=predict_body(samples,stamp,max_prediction=0.,max_linear_speed=.8,max_angular_speed=1.)
    rotation=result['body'][:3,:3]
    result.update(stamp_s=stamp,linear_v_body=rotation.T@result['linear_v_map'],
        angular_v_body=rotation.T@result['angular_v_map'])
    return result


def tool_velocity(config,q,dq,body,v_map,w_map,reference):
    """Velocity of a fixed gripper-link reference, including body and joints.

    The sixth joint moves the jaw and does not move gripper_link. Its velocity
    is deliberately excluded from this link reference's Jacobian.
    """
    q=finite(q,(6,));dq=finite(dq,(6,));body=finite(body,(4,4));inverse(body)
    velocity=finite(v_map,(3,));omega=finite(w_map,(3,));point=finite(reference,(3,))
    if len(config['joint_origins'])!=5:raise ValueError('INVALID_FK_CONFIG')
    chain=rpy_transform(config['mount_pose']);pivots=[];axes=[]
    for origin,angle in zip(config['joint_origins'],q[:5]):
        chain=chain@rpy_transform(origin)
        pivots.append(chain[:3,3].copy());axes.append(chain[:3,2].copy())
        chain=chain@rpy_transform([0,0,0,0,0,angle])
    endpoint=(chain@np.r_[point,1])[:3]
    joint_velocity=sum((np.cross(axis,endpoint-pivot)*speed
        for axis,pivot,speed in zip(axes,pivots,dq[:5])),np.zeros(3))
    offset_map=body[:3,:3]@endpoint
    result=velocity+np.cross(omega,offset_map)+body[:3,:3]@joint_velocity
    return finite(result,(3,)).copy()


def sample_control(observer,now):
    """Read the existing observer buffers without changing its audit cursor."""
    if not math.isfinite(now) or now<=0:raise ValueError('CURRENT_TIME')
    with observer.lock:
        if observer.error:raise ValueError(observer.error)
        if observer.target is None or len(observer.body)<2 or not observer.joints:return None
        # ROS /clock can arrive after a newer encoder/LIO callback. Select
        # actual buffered measurements no later than this clock read, rather
        # than treating the newest callback as a future-time sensor failure.
        body_samples=[(float(t),list(p)) for t,p in observer.body if t<=now][-3:]
        if len(body_samples)<2:return None
        # Pick the newest ACTUAL encoder sample within the available LIO
        # horizon, not an unpairable newer sample that discards valid history.
        joints=next((r for r in reversed(observer.joints) if 0<=now-r[0]<=.05+1e-9
            and body_samples[0][0]<=r[0]<=body_samples[-1][0]+.12+1e-9),None)
        if joints is None:return None
        stamp,q,dq=joints
        stamp=float(stamp);q=finite(q,(6,)).copy();dq=finite(dq,(6,)).copy()
        target=finite(observer.target,(4,4)).copy();reference=finite(observer.reference,(3,)).copy()
        target_stamp=float(observer.target_stamp);config=observer.config
    if not 0<=now-body_samples[-1][0]<=.15+1e-9:return None
    if not math.isfinite(target_stamp) or target_stamp<=0:raise ValueError('TARGET_SOURCE_TIME')
    if stamp<target_stamp:return None
    # Keep the 120ms bound. Abstain until a new real LIO pose arrives; the
    # native context source/receipt watchdogs remain active during this gap.
    if stamp<body_samples[0][0] or stamp-body_samples[-1][0]>.12+1e-9:return None
    predicted=predict_body(body_samples,stamp)
    body=predicted['body'];v=predicted['linear_v_map'];w=predicted['angular_v_map']
    result=relative_tool_target(config,body,q,target,reference)
    result={k:value.tolist() if isinstance(value,np.ndarray) else value for k,value in result.items()}
    tcp_reference=finite(config['tool_pose'],(6,))[:3]
    result.update(frame='map',source='LIO_BOUNDED_TWIST_ENCODER_FK',stamp_s=stamp,
        source_age_s=now-stamp,lio_source_age_s=now-predicted['last_lio_stamp_s'],
        last_lio_stamp_s=predicted['last_lio_stamp_s'],prediction_s=predicted['prediction_s'],
        prediction_mode=predicted['mode'],lio_before_stamp_s=predicted['lio_before_stamp_s'],
        lio_after_stamp_s=predicted['lio_after_stamp_s'],
        prediction_translation_m=float(np.linalg.norm(v)*predicted['prediction_s']),
        prediction_rotation_rad=float(np.linalg.norm(w)*predicted['prediction_s']),
        linear_velocity_map_m_s=v.tolist(),angular_velocity_map_rad_s=w.tolist(),
        reference_velocity_map_m_s=tool_velocity(config,q,dq,body,v,w,reference).tolist(),
        tcp_velocity_map_m_s=tool_velocity(config,q,dq,body,v,w,tcp_reference).tolist(),
        joint_names=list(config['joint_names']),joint_position_rad=q.tolist(),joint_velocity_rad_s=dq.tolist(),
        joint_time_basis='ROS_SIM_CLOCK_APPROXIMATE_PHYSICS_SAMPLE',map_from_body=body.tolist(),
        map_from_target=target.tolist(),target_observation_stamp_s=target_stamp,
        target_policy='STATIC_TARGET_LOCKED_AT_OBSERVATION',motion_permission=False,close_permission=False)
    return result
