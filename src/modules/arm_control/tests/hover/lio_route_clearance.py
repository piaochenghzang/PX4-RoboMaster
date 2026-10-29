"""Folded vehicle versus static LIO returns; unknown space is not certified."""
from pathlib import Path
from types import SimpleNamespace
import math
import xml.etree.ElementTree as ET
import numpy as np
from lio_tool_observer import finite,inverse,rpy_transform


def _pose(element,allowed_frames):
    if element is None:return np.eye(4)
    if element.get('relative_to','') not in allowed_frames:
        raise ValueError('Unsupported collision reference frame')
    return rpy_transform(finite(np.fromstring(element.text or '',sep=' '),(6,)))


def _box(collision,link_name):
    geometry=collision.find('geometry')
    if geometry is None or len(geometry)!=1 or geometry[0].tag!='box':
        raise ValueError('Only explicit collision boxes are supported')
    half=finite(np.fromstring(geometry.findtext('box/size',''),sep=' '),(3,))*.5
    if np.any(half<=0):raise ValueError('Invalid collision size')
    return _pose(collision.find('pose'),{'',link_name}),half


def body_collision_boxes(scene_path,config,q6):
    """Read geometry constants only; deliberately ignore world/model poses."""
    from fixed_demo_plan import FixedDemoPlan
    from check_approach_route import links,joints
    q=finite(q6,(6,));mount=rpy_transform(config['mount_pose'])
    model=ET.parse(Path(scene_path)).find("world/model[@name='yhang550']")
    if model is None:raise ValueError('Expanded yhang550 model required')
    frame=model.find("frame[@name='yhang550_base::__model__']/pose")
    base_frame=_pose(frame,{'','__model__'})
    if not np.allclose(base_frame,np.eye(4),atol=1e-8):
        raise ValueError('Body model frame must coincide with the body origin')
    arm_frame=_pose(model.find("frame[@name='so101::__model__']/pose"),{'','__model__'})
    if not np.allclose(arm_frame,mount,atol=1e-5):raise ValueError('Arm mounting geometry differs')
    mounted=[j for j in model.findall('joint') if j.findtext('child')=='so101::base_link']
    if (len(mounted)!=1 or mounted[0].get('type')!='fixed' or
            mounted[0].findtext('parent')!='yhang550_base::base_link'):
        raise ValueError('Arm must remain fixed to the body')
    base_pose=model.find("link[@name='so101::base_link']/pose")
    if base_pose is None or not np.allclose(_pose(base_pose,
            {'so101::__model__'}),np.eye(4),atol=1e-8):raise ValueError('Arm base frame differs')
    # The FK proxies come from production SO101. Refuse a scene whose arm
    # kinematic tree or zero-link frames no longer describe that same model.
    for expected in joints:
        name=expected.get('name');parent=expected.findtext('parent');child=expected.findtext('child')
        actual=model.find("joint[@name='so101::%s']"%name)
        if (actual is None or actual.get('type')!=expected.get('type') or
                actual.findtext('parent')!='so101::'+parent or actual.findtext('child')!='so101::'+child):
            raise ValueError('Expanded arm joint tree differs from production')
        if not np.allclose(_pose(actual.find('pose'),{'so101::'+parent}),
                _pose(expected.find('pose'),{parent}),atol=1e-5):
            raise ValueError('Expanded arm joint origin differs from production')
        actual_axis=actual.find('axis/xyz')
        if (actual_axis is None or actual_axis.get('expressed_in','') or
                actual.findtext('axis/use_parent_model_frame','false').strip() not in ('false','0') or
                not np.allclose(finite(np.fromstring(actual_axis.text or '',sep=' '),(3,)),
                    finite(np.fromstring(expected.findtext('axis/xyz',''),sep=' '),(3,)),atol=1e-8)):
            raise ValueError('Expanded arm joint axis differs from production')
        child_link=model.find("link[@name='so101::%s']"%child)
        if (child_link is None or child_link.find('pose') is None or
                not np.allclose(_pose(child_link.find('pose'),{'so101::'+name}),np.eye(4),atol=1e-8)):
            raise ValueError('Expanded arm child frame differs from production')
    expected_arm={}
    for name,link in links.items():
        for collision in link.findall('collision'):
            expected_arm[(name,collision.get('name'))]=_box(collision,name)
    seen_arm=set();boxes=[];seen_static=set()
    for link in model.findall('link'):
        name=link.get('name','');collisions=link.findall('collision')
        if not collisions:continue
        if name.startswith('so101::'):
            for collision in collisions:
                key=(name[7:],collision.get('name'))
                if key not in expected_arm or key in seen_arm:raise ValueError('Unexpected arm collision geometry')
                expected_T,expected_h=expected_arm[key];actual_T,actual_h=_box(collision,name)
                if not np.allclose(actual_T,expected_T,atol=1e-5) or not np.allclose(actual_h,expected_h,atol=1e-7):
                    raise ValueError('Expanded arm collision differs from production FK geometry')
                seen_arm.add(key)
            continue
        rotor=name in {'yhang550_base::rotor_'+str(i) for i in range(4)}
        if name not in ('yhang550_base::base_link','mid360s_mount') and not rotor:
            raise ValueError('Unsupported vehicle collision link: '+name)
        allowed={'','__model__','yhang550_base::__model__'}
        link_T=_pose(link.find('pose'),allowed)
        if name=='yhang550_base::base_link' and not np.allclose(link_T,np.eye(4),atol=1e-8):
            raise ValueError('Canonical body origin differs')
        if name!='yhang550_base::base_link':
            attached=[j for j in model.findall('joint') if j.findtext('child')==name]
            if len(attached)!=1 or attached[0].findtext('parent')!='yhang550_base::base_link':
                raise ValueError('Unexpected vehicle attachment')
            joint=attached[0]
            if rotor:
                axis=finite(np.fromstring(joint.findtext('axis/xyz',''),sep=' '),(3,))
                if (joint.get('type')!='revolute' or not np.allclose(axis,[0,0,1],atol=1e-8) or
                        joint.find('axis/xyz').get('expressed_in','') or
                        joint.findtext('axis/use_parent_model_frame','false').strip() not in ('false','0') or
                        not np.allclose(_pose(joint.find('pose'),{'',name}),np.eye(4),atol=1e-8)):
                    raise ValueError('Unsupported rotor axis')
            elif joint.get('type')!='fixed':raise ValueError('LiDAR mount must remain fixed')
        for collision in collisions:
            key=(name,collision.get('name'))
            if key in seen_static:raise ValueError('Duplicate collision name')
            seen_static.add(key);local,half=_box(collision,name)
            if rotor:
                if not np.allclose(local[:3,2],[0,0,1],atol=1e-8):raise ValueError('Unsupported tilted rotor collision')
                radius=float(np.linalg.norm(local[:2,3])+np.linalg.norm(half[:2]))
                disk=np.eye(4);disk[2,3]=local[2,3]
                local=disk;half=np.array([radius,radius,half[2]])
            boxes.append((name+'::'+str(collision.get('name')),link_T@local,half))
    if seen_arm!=set(expected_arm) or not seen_static:raise ValueError('Incomplete vehicle collision geometry')
    boxes.extend(FixedDemoPlan.boxes(SimpleNamespace(arm=mount),q[:5],float(q[5])))
    return boxes


def check_route(route,boxes,points_map,minimum=.016,tracking_margin=.03,point_margin=.02):
    """Conservative separation from observed points over the nominal route.

    CandidateRoute has two straight, monotonic quintic position legs and one
    monotonic shortest yaw blend. Splitting exactly at the climb corner makes
    endpoint translation plus radius*absolute yaw change bound every rigid
    point's travel on each interval. Five degrees of attitude deviation is
    additionally reserved, as a displacement (not an actual attitude test).
    Unknown / occluded space, moving objects and estimation error beyond the
    supplied margins remain outside this preflight check.
    """
    from lio_candidate_route import CandidateRoute
    if not isinstance(route,CandidateRoute):raise ValueError('Bounded CandidateRoute required')
    points=np.asarray(points_map,dtype=float)
    if points.ndim!=2 or points.shape[1:]!=(3,) or not len(points) or not np.isfinite(points).all():
        raise ValueError('Finite nonempty map points required')
    if any(not math.isfinite(v) or v<0 for v in (minimum,tracking_margin,point_margin)) or minimum<=0:
        raise ValueError('Invalid clearance margins')
    checked=[];radius=0.
    for name,T,half in boxes:
        T=finite(T,(4,4));inverse(T);half=finite(half,(3,))
        if np.any(half<=0):raise ValueError('Invalid collision half-size')
        checked.append((str(name),T,half))
        radius=max(radius,float(np.linalg.norm(T[:3,3])+np.linalg.norm(half)))
    if not checked:raise ValueError('Vehicle collision geometry required')
    attitude_margin=2.*radius*math.sin(math.radians(5.)*.5)
    reserve=tracking_margin+point_margin+attitude_margin
    # Distance is capped downward for efficiency, retaining a valid lower
    # bound. Returns farther than radius+cap cannot lower the result below cap.
    cap=.5
    reach=radius+cap
    lo=np.minimum.reduce([route.start,route.top,route.goal])-reach
    hi=np.maximum.reduce([route.start,route.top,route.goal])+reach
    near=points[np.all((points>=lo)&(points<=hi),axis=1)]
    def clearance(position,yaw):
        c,s=math.cos(yaw),math.sin(yaw);R=np.array([[c,-s,0],[s,c,0],[0,0,1]])
        body=(near-position)@R
        body=body[np.sum(body*body,axis=1)<=reach*reach]
        lower=cap
        for _,T,half in checked:
            if not len(body):break
            local=(body-T[:3,3])@T[:3,:3]
            lower=min(lower,float(np.min(np.linalg.norm(np.maximum(np.abs(local)-half,0.),axis=1))))
        return lower
    # dt<=1/15 guarantees <=10mm translation and <=.01rad yaw at the
    # CandidateRoute's .15m/s and .15rad/s bounds. Include the leg junction.
    times=[];start=float(route.began)
    for duration in (route.climb_duration,route.travel_duration):
        count=max(1,int(math.ceil(duration*15.)))
        times.extend(np.linspace(start,start+duration,count+1)[1:] if times else np.linspace(start,start+duration,count+1))
        start+=duration
    samples=[]
    for t in times:
        position,_,yaw=route.sample(float(t));samples.append((position,yaw))
    distances=[clearance(position,yaw) for position,yaw in samples]
    bound=math.inf;max_travel=0.
    for i,((a,ya),(b,yb)) in enumerate(zip(samples,samples[1:])):
        yaw_delta=abs(math.atan2(math.sin(yb-ya),math.cos(yb-ya)))
        travel=float(np.linalg.norm(b-a))+radius*yaw_delta
        max_travel=max(max_travel,travel)
        bound=min(bound,min(distances[i:i+2])-travel-reserve)
    return dict(valid=bool(bound>=minimum),min_clearance_m=float(bound),required_m=minimum,
        min_sampled_observed_clearance_m=min(distances),sampled_distance_cap_m=cap,
        maximum_interval_travel_bound_m=max_travel,samples=len(samples),points=len(points),
        collision_boxes=len(checked),body_corner_radius_m=radius,tracking_margin_m=tracking_margin,
        point_margin_m=point_margin,attitude_limit_deg=5.,attitude_margin_m=attitude_margin,
        scope='STATIC_OBSERVED_LIO_RETURNS_ONLY',unknown_space_certified=False,
        truth_inputs=False,actual_tracking_or_attitude_verified=False)
