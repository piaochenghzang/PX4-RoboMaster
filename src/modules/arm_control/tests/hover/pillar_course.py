"""Six wide posts, alternating openings; no preloaded planner map."""
import numpy as np

POST_SIZE=(1.2,1.2,3.4)
GATE_X=(2.,4.8,7.6)
LANE_Y=(.6,-.6,.6)
CORRIDOR_HALF_WIDTH=2.4
GRASP_STANDBY_OFFSET=np.array([-.9,-.6,0.])


def obstacles():
    rows=[]
    for i,(x,lane) in enumerate(zip(GATE_X,LANE_Y)):
        centers=(-.9,2.1) if lane>0 else (-2.1,.9)
        for j,y in enumerate(centers):
            rows.append(dict(name='demo_obstacle_pillar_%d_%d'%(i+1,j+1),
                center=(x,y,POST_SIZE[2]/2),size=POST_SIZE,
                color=(.85,.25,.1,1.) if i%2==0 else (.15,.4,.85,1.)))
    return tuple(rows)


def room():
    return tuple(dict(name='demo_obstacle_room_'+str(i),center=c,size=s,color=(.65,.65,.65,1.))
        for i,(c,s) in enumerate([((-3.,0.,2.25),(.1,4.8,4.5)),((14.,0.,2.25),(.1,4.8,4.5)),
            ((5.5,-2.4,2.25),(17.,.1,4.5)),((5.5,2.4,2.25),(17.,.1,4.5)),
            ((5.5,0.,4.5),(17.,4.8,.1))]))


def observation_goals(target,altitude):
    # Coarse search-area inputs only. The arm target must come from a later
    # source-timed raw-cloud snapshot, after these navigation legs finish.
    return [np.array([x+1.1,y,altitude]) for x,y in zip(GATE_X,LANE_Y)]+[
        np.array([target.object[0,3]-1.5,target.object[1,3],altitude])]


def gate_crossings(rows):
    crossings=[]
    for i,(x,lane) in enumerate(zip(GATE_X,LANE_Y)):
        found=None
        for a,b in zip(rows,rows[1:]):
            pa=np.asarray(a['body_world'][:3]);pb=np.asarray(b['body_world'][:3])
            if pa[0]<x<=pb[0]:
                u=(x-pa[0])/(pb[0]-pa[0]);q=pa+u*(pb-pa)
                found=dict(gate=i+1,sim_s=a['sim_s']+u*(b['sim_s']-a['sim_s']),
                    body_world=q.tolist(),lane_center_y_m=lane,
                    # Actual envelope is 0.45m; allow +/-0.40m about the lane
                    # center (the physical center gap half-width is 0.9m).
                    passed=bool(abs(q[1]-lane)<=.40 and .8<=q[2]<=2.5))
                break
        crossings.append(found or dict(gate=i+1,passed=False,reason='not_crossed'))
    return dict(passed=all(g['passed'] for g in crossings),gates=crossings)
