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
    v.profile=true;v.close_ready=true;v.jaw_sweep_valid=true;v.lift_goal={.22f,.015f,.25f};
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
    { assert(precisionPositionReady(0.f) && precisionPositionReady(.005f));
      assert(!precisionPositionReady(-.001f) && !precisionPositionReady(.00501f));
      assert(!precisionPositionReady(std::numeric_limits<float>::quiet_NaN()));
      assert(!precisionPositionReady(std::numeric_limits<float>::infinity()));++cases;
    } // Fine flight close cannot rely only on the coarse 30mm hover permit.
    { auto v=healthy();FixedGrasp g;g.request(1,4,v.motion.binding,v);
      assert(g.owns() && g.phase==FixedPhase::Approach);++cases; }
    { auto v=healthy();v.feedback.capabilities=31;FixedGrasp g;g.request(1,4,v.motion.binding,v);
      assert(!g.owns());++cases; } // Anchored bench capabilities are NOT flight capabilities.
    { auto v=healthy();v.profile=false;FixedGrasp g;g.request(1,4,v.motion.binding,v);assert(!g.owns());++cases; }
    { auto v=healthy();v.motion.prepared=false;FixedGrasp g;g.request(1,4,v.motion.binding,v);assert(!g.owns());++cases; }
    { auto v=healthy();auto wrong=v.motion.binding;wrong.plan++;FixedGrasp g;g.request(1,4,wrong,v);
      assert(!g.owns());++cases; }
    { auto v=healthy();v.lift_goal.z=.20f;FixedGrasp g;g.request(1,4,v.motion.binding,v);assert(!g.owns());++cases; }
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
      auto a=tick(g,v);assert(a.send && (a.goal-v.motion.accepted).norm()<=.003001f);++cases; }
    { auto v=healthy();FixedGrasp g;v.unfold_complete=true;
      g.request(1,4,v.motion.binding,v);v.motion.accepted.x-=.04f;v.motion.measured=v.motion.accepted;
      auto a=tick(g,v);assert(a.send && (a.goal-v.motion.accepted).norm()<=.000601f);
      v.motion.now+=10000;assert(!g.step(v).send);
      v.motion.now+=10000;a=g.step(v);assert(a.send && (a.goal-v.motion.accepted).norm()<=.000601f);
      ++cases; } // 0.6mm/20ms equals the original 3mm/100ms capacity.
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
          assert(distance<=.03f*seconds+1e-7f);
          assert(a.max_joint_step<=.2f*seconds+1e-7f && a.max_jaw_step<=.2f*seconds+1e-7f);
          total+=distance;
      }
      assert(total<=.03f*float(elapsed)*1e-6f+1e-6f);++cases;
    } // Jitter must not halve cadence or increase any unit-time motion budget.
    { auto v=healthy();FixedGrasp g;v.unfold_complete=true;
      g.request(1,4,v.motion.binding,v);v.motion.accepted.x-=.04f;v.motion.measured=v.motion.accepted;
      g.step(v);v.motion.now+=200000;const auto a=g.step(v);
      assert(a.send && (a.goal-v.motion.accepted).norm()<=.000601f);
      assert(a.max_joint_step<=.004001f && a.max_jaw_step<=.004001f);++cases;
    } // A delayed callback still cannot accumulate a catch-up motion.
    { auto v=healthy();FixedGrasp g;closeAndLift(g,v);v.jaw_sweep_valid=false;
      auto a=tick(g,v);assert(g.phase==FixedPhase::Fault && g.fault==FixedFault::Geometry && a.stop);
      ++cases; } // Full-stroke loss while carrying is NOT ignored.
    std::printf("PASS: %u fixed-grasp logic cases (synthetic; not physical grasp proof)\n",cases);
}
