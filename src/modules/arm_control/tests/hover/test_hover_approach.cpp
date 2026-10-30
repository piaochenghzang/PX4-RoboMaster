#include "../../grasp/HoverApproach.hpp"
#include "../../grasp/WristUnfold.hpp"
#include <algorithm>
#include <cassert>
#include <limits>
using namespace so101_hover;
ApproachInput good() {
    ApproachInput i; i.now=1000000; i.binding={1,2,3,4};
    i.context_fresh=i.lease_fresh=i.geometry_valid=i.arm_ready=i.hard_hover_ok=i.hover_ready=i.prepared=true;
    i.clearance_ready=true;
    i.clearance=.10f; i.joint_speed=0; i.tool_relative_speed=0;
    i.accepted=i.measured={.106f,.012f,-.005f}; i.goal={.17f,.035f,.05f};
    return i;
}
void begin(Approach &a,ApproachInput &i) { a.request(1,1,i.binding,i); assert(a.moving()); }
int main() {
    int count=0;
    { Approach a; auto i=good(); i.prepared=false; a.request(1,1,i.binding,i); assert(!a.owns()); ++count; }
    { Approach a; auto i=good(); auto b=i.binding; ++b.plan; a.request(1,1,b,i); assert(!a.owns()); ++count; }
    { Approach a; auto i=good(); begin(a,i); bool send,stop; auto p=a.step(i,send,stop);
      assert(send && !stop && std::abs((p-i.accepted).norm()-.0006f)<2e-8f && p.z>i.accepted.z);
      assert(std::abs(a.max_joint_step-.004f)<1e-8f); ++count; }
    { Approach a; auto i=good(); begin(a,i); bool send,stop; i.hover_ready=false; a.step(i,send,stop);
      assert(stop && !send && a.phase==ApproachPhase::Paused);
      i.hover_ready=true; i.now+=3000000; a.step(i,send,stop); assert(!send && a.phase==ApproachPhase::Paused);
      a.request(2,2,i.binding,i); assert(a.moving()); ++count; }
    { Approach a; auto i=good(); begin(a,i); bool send,stop; ++i.binding.target; a.step(i,send,stop);
      assert(stop && !send && a.fault==ApproachFault::BindingChanged); ++count; }
    { Approach a; auto i=good(); begin(a,i); bool send,stop; i.lease_fresh=false; a.step(i,send,stop);
      assert(stop && a.phase==ApproachPhase::Fault); i.lease_fresh=true; a.request(2,2,i.binding,i);
      assert(a.phase==ApproachPhase::Fault); ++count; }
    { Approach a; auto i=good(); begin(a,i); bool send,stop; i.context_fresh=false; a.step(i,send,stop);
      assert(stop && !send); ++count; }
    { Approach a; auto i=good(); begin(a,i); bool send,stop; i.hard_hover_ok=false; a.step(i,send,stop);
      assert(stop && a.fault==ApproachFault::HoverHard); ++count; }
    { Approach a; auto i=good(); begin(a,i); bool send,stop; i.clearance=.015f; a.step(i,send,stop);
      assert(stop && a.fault==ApproachFault::Geometry); ++count; }
    { Approach a; auto i=good(); begin(a,i); bool send,stop; i.geometry_valid=false; a.step(i,send,stop);
      assert(stop && !send); ++count; }
    { Approach a; auto i=good(); begin(a,i); bool send,stop; i.arm_ready=false; a.step(i,send,stop);
      assert(stop && a.phase==ApproachPhase::Fault); ++count; }
    { Approach a; auto i=good(); begin(a,i); auto wrong=i.binding; ++wrong.scene;
      bool send,stop; a.step(i,send,stop); assert(send && a.max_joint_step>0);
      a.request(2,3,wrong,i); assert(a.moving()); a.request(3,3,i.binding,i);
      a.step(i,send,stop);
      assert(a.phase==ApproachPhase::Canceled && stop && !send && a.max_joint_step==0); ++count; }
    { Approach a; auto i=good(); begin(a,i); bool send,stop;
      i.measured=i.accepted=i.goal; for(int k=0;k<12;++k) { a.step(i,send,stop); i.now+=100000; }
      assert(a.phase==ApproachPhase::Holding); ++count; }
    { Approach a; auto i=good(); begin(a,i); bool send,stop; i.now+=180000001; a.step(i,send,stop);
      assert(stop && a.fault==ApproachFault::Timeout); ++count; }
    { Approach a; auto i=good(); begin(a,i); a.request(1,3,i.binding,i); assert(a.moving()); ++count; }
    struct Limit { float min_rad,max_rad; };
    { Limit limits[2]{{-1.57f,1.57f},{-1.2f,1.2f}};
      float target[2]{-1.57f,.1f},anchor[2]{-1.57f,0},measured[2]{-1.57375f,0},out[2]{};
      const float radii[2]{1.1f,1.1f};
      // The old raw-measurement interpolation is outside the wrist limit.
      assert(measured[0]+.1f*(target[0]-measured[0])<limits[0].min_rad);
      assert(boundJointBudget(target,anchor,measured,limits,radii,.02f,out));
      assert(out[0]>=limits[0].min_rad && out[1]>=0 && out[1]<=.1f);
      assert(1.1f*(std::abs(out[0]-measured[0])+std::abs(out[1]-measured[1]))<=.020001f); ++count; }
    { Limit l[1]{{-1,1}}; float t[1]{.5f},a[1]{1},m[1]{1.1f},o[1]{42}; const float r[1]{1.1f};
      assert(!boundJointBudget(t,a,m,l,r,.05f,o) && o[0]==42); ++count; }
    { Limit l[1]{{-1,1}}; float t[1]{1.01f},a[1]{0},m[1]{0},o[1]{}; const float r[1]{1.1f};
      assert(!boundJointBudget(t,a,m,l,r,.5f,o)); ++count; }
    { Limit l[1]{{-1,1}}; float t[1]{.2f},a[1]{.1f},m[1]{.1f},o[1]{}; const float r[1]{1.1f};
      assert(boundJointBudget(t,a,m,l,r,1.f,o) && std::abs(o[0]-.2f)<1e-6f);
      m[0]=std::numeric_limits<float>::quiet_NaN(); assert(!boundJointBudget(t,a,m,l,r,1.f,o)); ++count; }
    { Limit l[2]{{-1,1},{-.5f,.5f}}; unsigned rng=101;
      for (int k=0;k<10000;++k) {
        auto random=[&rng]() { rng=1664525u*rng+1013904223u; return float(rng>>8)/16777216.f; };
        float t[2]{2*random()-1,random()-.5f},m[2]{2.2f*random()-1.1f,1.2f*random()-.6f},o[2]{};
        float a[2]{m[0]<-1?-1.f:(m[0]>1?1.f:m[0]),m[1]<-.5f?-.5f:(m[1]>.5f?.5f:m[1])};
        const float radii[2]{1.1f,1.1f};
        const float budget=.001f+.3f*random();
        if (boundJointBudget(t,a,m,l,radii,budget,o)) {
          for (int j=0;j<2;++j) assert(o[j]>=l[j].min_rad && o[j]<=l[j].max_rad);
          assert(1.1f*(std::abs(o[0]-m[0])+std::abs(o[1]-m[1]))<=budget+1e-6f);
        }
      } ++count; }
    { Limit l[1]{{-1,1}}; float t[1]{.3f},a[1]{.2f},m[1]{.1f},o[1]{}; const float r[1]{.5f};
      assert(boundJointBudget(t,a,m,l,r,.06f,o)); assert(o[0]>=a[0] && o[0]<=t[0]); ++count; }
    { Approach a; auto i=good(); begin(a,i); bool send,stop;
      a.pause(PauseReason::Clearance); i.clearance_ready=false; a.step(i,send,stop);
      assert(stop && !send && a.phase==ApproachPhase::Paused && a.fault==ApproachFault::None);
      a.request(2,2,i.binding,i); assert(a.phase==ApproachPhase::Paused && a.request_result==3);
      i.clearance_ready=true; i.now+=1000000; a.step(i,send,stop);
      assert(!send && a.resumeReady(i) && a.phase==ApproachPhase::Paused);
      a.request(3,2,i.binding,i); assert(a.moving() && a.pause_reason==PauseReason::None); ++count; }
    { Approach a; auto i=good(); begin(a,i); bool send,stop;
      i.measured=i.accepted=i.goal; i.joint_speed=.2f; i.tool_relative_speed=.011f;
      for(int k=0;k<12;++k) { a.step(i,send,stop); i.now+=100000; }
      assert(a.phase==ApproachPhase::Approaching);
      i.tool_relative_speed=.004f;
      for(int k=0;k<12;++k) { a.step(i,send,stop); i.now+=100000; }
      assert(a.phase==ApproachPhase::Holding && !send); ++count; }
    { Approach a; auto i=good(); i.tool_relative_speed=-.01f; a.request(1,1,i.binding,i); assert(!a.owns());
      i.tool_relative_speed=INFINITY; a.request(2,1,i.binding,i); assert(!a.owns()); ++count; }
    { Limit l[2]{{-1,1},{-1,1}}; const float sent[3]{.4f,.6f,.3f}; float frozen[2]{};
      for(int n=0;n<100;++n) { assert(freezeCommand(sent,l,frozen)); assert(frozen[0]==.4f && frozen[1]==.6f); }
      // Only arm slots are frozen; the jaw is not copied or retargeted.
      assert(sent[2]==.3f); float bad[2]{.4f,INFINITY};
      assert(!freezeCommand(bad,l,frozen) && frozen[0]==.4f && frozen[1]==.6f); ++count; }
    { Approach a; auto i=good(); i.unfold_complete=false; begin(a,i); bool send,stop;
      i.measured=i.accepted=i.goal;
      for(int k=0;k<20;++k) { a.step(i,send,stop); assert(!send && !stop); i.now+=100000; }
      assert(a.phase==ApproachPhase::Approaching);
      i.unfold_complete=true;
      for(int k=0;k<12;++k) { a.step(i,send,stop); i.now+=100000; }
      assert(a.phase==ApproachPhase::Holding); ++count; }
    { Approach a; auto i=good(); i.unfold_complete=false; begin(a,i); bool send,stop;
      a.step(i,send,stop); assert(!send && !stop);
      i.context_fresh=false; a.step(i,send,stop);
      assert(stop && !send && a.phase==ApproachPhase::Fault); ++count; }
    { Approach a; auto i=good(); i.unfold_complete=false; begin(a,i); bool send,stop;
      i.hover_ready=false; a.step(i,send,stop);
      assert(stop && !send && a.phase==ApproachPhase::Paused);
      a.request(2,3,i.binding,i); assert(a.phase==ApproachPhase::Canceled); ++count; }
    { const float folded[5]{-.07f,-1.175f,1.45f,1.3f,-1.57f};
      float measured[5]{-.07f,-1.175f,1.45f,1.3f,-1.57f};
      assert(knownFoldedArm(folded,measured));
      measured[3]=-1.3f; assert(!knownFoldedArm(folded,measured));
      measured[3]=1.34f; assert(!knownFoldedArm(folded,measured));
      measured[3]=NAN; assert(!knownFoldedArm(folded,measured)); ++count; }
    { const float folded[5]{-.07f,-1.175f,1.45f,1.3f,-1.57f};float desired[5]{};
      assert(wristUnfoldStep(folded,folded,0,.02f,desired)==UnfoldResult::Move);
      for(int j=0;j<5;++j) assert(std::abs(desired[j]-folded[j])<=.020001f);
      assert(desired[3]<folded[3]);
      for(int j=0;j<5;++j) { if(j!=3) assert(desired[j]==folded[j]); }
      ++count; }
    { const float folded[5]{-.07f,-1.175f,1.45f,1.3f,-1.57f};float desired[5]{};
      assert(wristUnfoldStep(folded,folded,0,.02f,desired,true)==UnfoldResult::Move);
      assert(std::abs(desired[2]-1.449f)<1e-6f && desired[3]<folded[3]);
      float inward[5]{-.07f,-1.175f,1.449f,1.0f,-1.57f};
      assert(wristUnfoldStep(inward,inward,0,.02f,desired,false)==UnfoldResult::Move);
      assert(std::abs(desired[2]-1.45f)<1e-6f); ++count; }
    { const float unfolded[5]{-.07f,-1.175f,1.45f,-1.3f,-1.57f};
      float measured[5]{-.07f,-1.175f,1.45f,-1.34f,-1.57f},desired[5]{};
      assert(wristUnfoldStep(unfolded,measured,0,.02f,desired)==UnfoldResult::Wait);
      measured[3]=-1.3f;
      assert(wristUnfoldStep(unfolded,measured,.051f,.02f,desired)==UnfoldResult::Wait);
      assert(wristUnfoldStep(unfolded,measured,.049f,.02f,desired)==UnfoldResult::Settled); ++count; }
    { const float folded[5]{-.07f,-1.175f,1.45f,1.3f,-1.57f};float desired[5]{42,42,42,42,42};
      const float invalid_steps[]{0.f,.03f,INFINITY};
      for(float step:invalid_steps) {
          assert(wristUnfoldStep(folded,folded,0,step,desired)==UnfoldResult::Invalid);
          for(float value:desired) assert(value==42);
      }
      assert(wristUnfoldStep(folded,folded,NAN,.02f,desired)==UnfoldResult::Invalid);
      ++count; }
    // Callback timing changes command granularity, never the existing speed.
    { Approach a; auto i=good(); begin(a,i); bool send,stop;
      auto p=a.step(i,send,stop); assert(send); i.accepted=i.measured=p;
      i.now+=15999; a.step(i,send,stop);
      assert(!send && !stop && a.max_joint_step==0);
      ++i.now; p=a.step(i,send,stop);
      assert(send && std::abs((p-i.accepted).norm()-.00048f)<2e-8f);
      assert(std::abs(a.max_joint_step-.0032f)<1e-8f); ++count; }
    { Approach a; auto i=good(); begin(a,i); bool send,stop;
      auto p=a.step(i,send,stop); assert(send); i.accepted=i.measured=p;
      i.now+=19000; p=a.step(i,send,stop);
      assert(send && std::abs((p-i.accepted).norm()-.00057f)<2e-8f);
      assert(std::abs(a.max_joint_step-.0038f)<1e-8f); ++count; }
    // A delayed callback or temporarily ineligible joint speed grants only
    // one 20 ms allowance. It must not deliver the missed trajectory in a burst.
    { Approach a; auto i=good(); begin(a,i); bool send,stop;
      auto p=a.step(i,send,stop); assert(send); i.accepted=i.measured=p;
      i.now+=20000; i.joint_speed=.101f; a.step(i,send,stop);
      assert(!send && a.max_joint_step==0);
      i.now+=1000000; i.joint_speed=0; p=a.step(i,send,stop);
      assert(send && std::abs((p-i.accepted).norm()-.0006f)<2e-8f);
      assert(std::abs(a.max_joint_step-.004f)<1e-8f);
      i.accepted=i.measured=p; ++i.now; a.step(i,send,stop);
      assert(!send && a.max_joint_step==0); ++count; }
    // Rapid explicit pause/resume must retain the last send time: otherwise
    // each resume could reissue a full first-step budget in less than 16 ms.
    { Approach a; auto i=good(); begin(a,i); bool send,stop;
      auto p=a.step(i,send,stop); assert(send); i.accepted=i.measured=p;
      i.now+=1000; i.hover_ready=false; a.step(i,send,stop);
      assert(stop && !send && a.max_joint_step==0);
      i.now+=1000; i.hover_ready=true; a.request(2,2,i.binding,i); a.step(i,send,stop);
      assert(a.moving() && !send && !stop && a.max_joint_step==0);
      i.now+=14000; p=a.step(i,send,stop);
      assert(send && std::abs((p-i.accepted).norm()-.00048f)<2e-8f);
      assert(std::abs(a.max_joint_step-.0032f)<1e-8f); ++count; }
    { Approach a; auto i=good(); begin(a,i); bool send,stop;
      auto p=a.step(i,send,stop); assert(send); i.accepted=i.measured=p;
      i.now+=20000; i.hover_ready=false; a.step(i,send,stop); assert(stop && !send);
      i.now+=3000000; i.hover_ready=true; a.step(i,send,stop);
      assert(!send && a.phase==ApproachPhase::Paused && a.max_joint_step==0);
      a.request(2,2,i.binding,i); p=a.step(i,send,stop);
      assert(send && std::abs((p-i.accepted).norm()-.0006f)<2e-8f);
      assert(std::abs(a.max_joint_step-.004f)<1e-8f); ++count; }
    // Finishing the wrist unfold starts a fresh bounded Cartesian allowance,
    // regardless of how long the internal unfold took.
    { Approach a; auto i=good(); i.unfold_complete=false; begin(a,i); bool send,stop;
      a.step(i,send,stop); assert(!send && a.max_joint_step==0);
      i.now+=20000000; i.unfold_complete=true; auto p=a.step(i,send,stop);
      assert(send && std::abs((p-i.accepted).norm()-.0006f)<2e-8f);
      assert(std::abs(a.max_joint_step-.004f)<1e-8f); ++count; }
    // Reusing an old instance through a clean value reset must erase
    // cadence history; a clock reversal without reset remains a latched fault.
    { Approach a; auto i=good(); begin(a,i); bool send,stop;
      a.step(i,send,stop); assert(send);
      --i.now; a.step(i,send,stop);
      assert(stop && !send && a.fault==ApproachFault::Timeout && a.max_joint_step==0);
      a=Approach{}; i=good(); begin(a,i); auto p=a.step(i,send,stop);
      assert(send && !stop && std::abs((p-i.accepted).norm()-.0006f)<2e-8f);
      assert(std::abs(a.max_joint_step-.004f)<1e-8f); ++count; }
    // A deterministic jitter stream verifies cumulative Cartesian/joint slew
    // from the first allowance, including missed callbacks and 16 ms sends.
    { Approach a; auto i=good(); i.goal={1.f,0,0}; i.accepted=i.measured={0,0,0}; begin(a,i);
      const uint64_t began=i.now; bool send,stop; float travel=0,joint_travel=0;
      const uint64_t intervals[]{1000,3000,12000,3000,16000,17000,19000,20000,21000,100000};
      uint64_t previous=0;
      for (int k=0;k<1000;++k) {
        auto p=a.step(i,send,stop); assert(!stop);
        if (send) {
          const float moved=(p-i.accepted).norm();
          if (previous) assert(i.now-previous>=16000);
          const float seconds=float(previous ? std::min(uint64_t(20000),i.now-previous) : 20000)*1e-6f;
          assert(moved<=.03f*seconds+3e-8f && a.max_joint_step<=.2f*seconds+1e-8f);
          travel+=moved; joint_travel+=a.max_joint_step; previous=i.now;
          i.accepted=i.measured=p;
        } else assert(a.max_joint_step==0);
        const float available=float(i.now-began+20000)*1e-6f;
        assert(travel<=.03f*available+1e-6f && joint_travel<=.2f*available+1e-5f);
        i.now+=intervals[k%10];
      } ++count; }
    return count==41 ? 0 : 1;
}
