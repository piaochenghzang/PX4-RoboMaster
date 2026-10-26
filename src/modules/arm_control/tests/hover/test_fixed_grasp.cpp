#include "../../grasp/FixedGrasp.hpp"
#include <cassert>
#include <cstdio>
#include <limits>
using namespace so101_hover;

FixedInput healthy()
{
    FixedInput v{};auto &i=v.motion;auto &f=v.feedback;
    i.now=1000000;i.binding={1,2,3,4};i.context_fresh=i.lease_fresh=i.arm_ready=i.geometry_valid=true;
    i.hard_hover_ok=i.hover_ready=i.prepared=true;i.clearance=.05f;i.joint_speed=0;i.tool_relative_speed=0;
    i.goal=i.accepted=i.measured={.22f,.015f,.28f};
    v.profile=true;v.close_ready=true;v.jaw_sweep_valid=true;v.lift_goal={.22f,.015f,.24f};
    f.initialized=f.finite=true;f.joint_age=f.grasp_age=.02f;f.capabilities=15;f.profile=1;f.target_id=1;
    f.frames=100;f.sequence=1;f.epoch=1;f.source_identity=42;f.object_height=1.5f;f.jaw_speed=0;
    return v;
}
FixedAction tick(FixedGrasp &g,FixedInput &v)
{
    v.motion.now+=100000;++v.feedback.sequence;return g.step(v);
}
void closeAndLift(FixedGrasp &g,FixedInput &v)
{
    g.request(1,4,v.motion.binding,v);assert(g.phase==FixedPhase::Approach);
    g.request(2,5,v.motion.binding,v);assert(g.phase==FixedPhase::Align);
    for(int n=0;n<12;++n)tick(g,v);
    assert(g.phase==FixedPhase::Close);
    const auto closing=tick(g,v);assert(closing.jaw && closing.jaw_target==.04f);
    v.feedback.bilateral=1;v.feedback.fixed_force=v.feedback.moving_force=.1f;
    tick(g,v);assert(g.phase==FixedPhase::Verify);
    for(int n=0;n<12;++n)tick(g,v);
    assert(g.phase==FixedPhase::Lift);
}
int main()
{
    unsigned cases=0;
    { assert(precisionPositionReady(0.f) && precisionPositionReady(.010f));
      assert(!precisionPositionReady(-.001f) && !precisionPositionReady(.01001f));
      assert(!precisionPositionReady(std::numeric_limits<float>::quiet_NaN()));
      assert(!precisionPositionReady(std::numeric_limits<float>::infinity()));++cases;
    } // Fine flight close cannot rely only on the coarse 30mm hover permit.
    { assert(fixedCloseGeometryReady(.003f,.010f,.0872665f));
      assert(!fixedCloseGeometryReady(.00301f,0.f,0.f));
      assert(!fixedCloseGeometryReady(0.f,.01001f,0.f));
      assert(!fixedCloseGeometryReady(0.f,0.f,.08727f));
      for(float x:{-.001f,std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity()}) {
          assert(!fixedCloseGeometryReady(x,0.f,0.f));
          assert(!fixedCloseGeometryReady(0.f,x,0.f));
          assert(!fixedCloseGeometryReady(0.f,0.f,x));
      } ++cases;
    } // Demo speed allowance never accepts nonfinite/out-of-position evidence.
    { assert(!fixedCloseGeometryReady(.008f,.015f,.25f));
      assert(fixedCloseGeometryReady(.008f,.015f,.25f,true));
      for(float x:{-.001f,std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity()}) {
          assert(!fixedCloseGeometryReady(x,0.f,0.f,true));
          assert(!fixedCloseGeometryReady(0.f,x,0.f,true));
          assert(!fixedCloseGeometryReady(0.f,0.f,x,true));
      } ++cases;
    } // Only accepted Close may tolerate expected cube centering/rotation.
    { auto v=healthy();FixedGrasp g;g.request(1,4,v.motion.binding,v);
      assert(g.owns() && g.phase==FixedPhase::Approach);++cases; }
    { auto v=healthy();v.feedback.capabilities=31;FixedGrasp g;g.request(1,4,v.motion.binding,v);
      assert(!g.owns());++cases; } // Anchored bench capabilities are NOT flight capabilities.
    { auto v=healthy();v.profile=false;FixedGrasp g;g.request(1,4,v.motion.binding,v);assert(!g.owns());++cases; }
    { auto v=healthy();v.motion.prepared=false;FixedGrasp g;g.request(1,4,v.motion.binding,v);assert(!g.owns());++cases; }
    { auto v=healthy();auto wrong=v.motion.binding;wrong.plan++;FixedGrasp g;g.request(1,4,wrong,v);
      assert(!g.owns());++cases; }
    { auto v=healthy();v.lift_goal.z=.20f;FixedGrasp g;g.request(1,4,v.motion.binding,v);assert(!g.owns());++cases; }
    { auto v=healthy();v.lift_goal.z=.25f;FixedGrasp g;g.request(1,4,v.motion.binding,v);assert(!g.owns());++cases; }
    { auto v=healthy();FixedGrasp g;g.request(1,4,v.motion.binding,v);v.close_ready=false;
      g.request(2,5,v.motion.binding,v);assert(g.phase==FixedPhase::Approach && g.request_result==3);++cases; }
    { auto v=healthy();FixedGrasp g;g.request(1,4,v.motion.binding,v);v.feedback.grasp_age=.31f;
      auto a=tick(g,v);assert(g.phase==FixedPhase::Fault && a.stop);++cases; }
    { auto v=healthy();FixedGrasp g;g.request(1,4,v.motion.binding,v);v.motion.geometry_valid=false;
      auto a=tick(g,v);assert(g.phase==FixedPhase::Fault && a.stop);++cases; }
    { auto v=healthy();FixedGrasp g;g.request(1,4,v.motion.binding,v);v.motion.hover_ready=false;
      auto a=tick(g,v);assert(g.paused && a.stop);v.motion.hover_ready=true;tick(g,v);
      assert(g.paused);g.request(2,7,v.motion.binding,v);assert(!g.paused);++cases; }
    { auto v=healthy();FixedGrasp g;g.request(1,4,v.motion.binding,v);
      g.request(2,6,v.motion.binding,v);auto a=tick(g,v);
      assert(g.phase==FixedPhase::Canceled && g.owns() && a.stop && !a.jaw);++cases; }
    { auto v=healthy();FixedGrasp g;closeAndLift(g,v);
      v.feedback.object_height=1.53f;v.motion.measured=v.lift_goal;
      for(int n=0;n<12;++n)tick(g,v);assert(g.phase==FixedPhase::Hold);
      for(int n=0;n<52;++n)tick(g,v);assert(g.phase==FixedPhase::Done && g.actual_lift>.025f);++cases; }
    { auto v=healthy();FixedGrasp g;closeAndLift(g,v);v.feedback.bilateral=0;v.motion.hover_ready=false;
      for(int n=0;n<8;++n)tick(g,v);assert(g.phase==FixedPhase::Fault && g.fault==FixedFault::Lost);++cases; }
    { auto v=healthy();FixedGrasp g;g.request(1,4,v.motion.binding,v);v.feedback.epoch++;
      auto a=tick(g,v);assert(g.phase==FixedPhase::Fault && a.stop);++cases; }
    { auto v=healthy();v.feedback.fixed_force=std::numeric_limits<float>::quiet_NaN();
      FixedGrasp g;g.request(1,4,v.motion.binding,v);assert(!g.owns());++cases; }
    { auto v=healthy();FixedGrasp g;g.request(1,4,v.motion.binding,v);g.paused=true;g.fail(FixedFault::Geometry);
      g.request(2,7,v.motion.binding,v);assert(g.phase==FixedPhase::Fault && g.paused && g.request_result==3);++cases; }
    { auto v=healthy();FixedGrasp g;g.request(1,4,v.motion.binding,v);g.request(2,6,v.motion.binding,v);g.paused=true;
      g.request(3,7,v.motion.binding,v);assert(g.phase==FixedPhase::Canceled && g.paused && g.request_result==3);++cases; }
    { auto v=healthy();FixedGrasp g;g.request(1,4,v.motion.binding,v);v.motion.accepted.x-=.04f;v.motion.measured=v.motion.accepted;
      auto a=tick(g,v);assert(a.send && (a.goal-v.motion.accepted).norm()<=.002501f);
      assert(a.max_joint_step<=.020001f && a.max_jaw_step<=.010001f);++cases; }
    { auto v=healthy();FixedGrasp g;v.unfold_complete=true;
      g.request(1,4,v.motion.binding,v);v.motion.accepted.x-=.04f;v.motion.measured=v.motion.accepted;
      auto a=tick(g,v);assert(a.send && (a.goal-v.motion.accepted).norm()<=.001001f);
      v.motion.now+=10000;assert(!g.step(v).send);
      v.motion.now+=10000;a=g.step(v);assert(a.send && (a.goal-v.motion.accepted).norm()<=.001001f);
      ++cases; } // 1mm/20ms is the bounded 50mm/s Cartesian capacity.
    { auto v=healthy();FixedGrasp g;v.jaw_sweep_valid=false;
      g.request(1,4,v.motion.binding,v);assert(g.phase==FixedPhase::Approach);
      g.request(2,5,v.motion.binding,v);assert(g.phase==FixedPhase::Approach && g.request_result==3);
      ++cases; } // Current open geometry is not a full-stroke close certificate.
    { auto v=healthy();FixedGrasp g;v.unfold_complete=true;
      g.request(1,4,v.motion.binding,v);v.motion.accepted.x-=.04f;v.motion.measured=v.motion.accepted;
      assert(g.step(v).send);
      float total=0.f;uint64_t elapsed=0;
      for (int n=0;n<40;++n) {
          const uint64_t us=n%2 ? 21000 : 19000;elapsed+=us;v.motion.now+=us;
          const auto a=g.step(v);assert(a.send);
          const float seconds=float(std::min(uint64_t(20000),us))*1e-6f;
          const float distance=(a.goal-v.motion.accepted).norm();
          assert(distance<=fixedCartesianSpeed*seconds+1e-7f);
          assert(a.max_joint_step<=fixedArmJointSlew*seconds+1e-7f &&
              a.max_jaw_step<=fixedJawSlew*seconds+1e-7f);
          total+=distance;
      }
      assert(total<=fixedCartesianSpeed*float(elapsed)*1e-6f+1e-6f);++cases;
    } // Jitter must not halve cadence or increase any unit-time motion budget.
    { auto v=healthy();FixedGrasp g;v.unfold_complete=true;
      g.request(1,4,v.motion.binding,v);v.motion.accepted.x-=.04f;v.motion.measured=v.motion.accepted;
      g.step(v);v.motion.now+=200000;const auto a=g.step(v);
      assert(a.send && (a.goal-v.motion.accepted).norm()<=.001001f);
      assert(a.max_joint_step<=.008001f && a.max_jaw_step<=.004001f);++cases;
    } // A delayed callback still cannot accumulate a catch-up motion.
    { auto v=healthy();FixedGrasp g;v.unfold_complete=true;
      g.request(1,4,v.motion.binding,v);v.motion.accepted.x-=.04f;v.motion.measured=v.motion.accepted;
      g.step(v);float total=0.f,jaw_total=0.f,joint_total=0.f;unsigned sends=0;
      for(int n=0;n<250;++n) {
          v.motion.now+=4000;const auto a=g.step(v);jaw_total+=a.max_jaw_step;
          if(a.send) {
              ++sends;total+=(a.goal-v.motion.accepted).norm();joint_total+=a.max_joint_step;
              assert((a.goal-v.motion.accepted).norm()<=.000801f);
          }
      }
      assert(sends==62 && total<=.050001f && joint_total<=.400001f && jaw_total<=.200001f);
      ++cases;
    } // 250Hz calls preserve 50mm/s, .4rad/s arm and .2rad/s jaw budgets.
    { auto v=healthy();FixedGrasp g;v.unfold_complete=true;
      g.request(1,4,v.motion.binding,v);g.request(2,5,v.motion.binding,v);g.step(v);
      for(int n=0;n<49;++n) {v.motion.now+=4000;g.step(v);assert(g.phase==FixedPhase::Align);}
      v.motion.now+=4000;g.step(v);assert(g.phase==FixedPhase::Close);++cases;
    } // Demo alignment needs a real 200ms dwell, independent of call count.
    { auto v=healthy();FixedGrasp g;v.unfold_complete=true;
      g.request(1,4,v.motion.binding,v);g.request(2,5,v.motion.binding,v);g.step(v);
      v.motion.now+=180000;g.step(v);assert(g.phase==FixedPhase::Align);
      v.close_ready=false;v.motion.now+=4000;g.step(v);
      v.close_ready=true;v.motion.now+=4000;g.step(v);
      v.motion.now+=196000;g.step(v);assert(g.phase==FixedPhase::Align);
      v.motion.now+=4000;g.step(v);assert(g.phase==FixedPhase::Close);++cases;
    } // A failed live close check resets even the shorter demo dwell.
    { auto v=healthy();FixedGrasp g;
      g.request(1,4,v.motion.binding,v);g.request(2,5,v.motion.binding,v);
      for(int n=0;n<4;++n)tick(g,v);assert(g.phase==FixedPhase::Close);
      v.close_ready=false;auto a=tick(g,v);assert(!a.jaw && g.phase==FixedPhase::Close);
      v.close_ready=true;a=tick(g,v);assert(a.jaw && a.jaw_target==.04f);++cases;
    } // Relaxed admission never bypasses the continuing live closure check.
    { auto v=healthy();FixedGrasp g;closeAndLift(g,v);v.jaw_sweep_valid=false;
      auto a=tick(g,v);assert(g.phase==FixedPhase::Fault && g.fault==FixedFault::Geometry && a.stop);
      ++cases; } // Full-stroke loss while carrying is NOT ignored.
    std::printf("PASS: %u fixed-grasp logic cases (synthetic; not physical grasp proof)\n",cases);
}
