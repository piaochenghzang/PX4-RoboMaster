#include "../../grasp/HoverApproach.hpp"
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
      assert(send && !stop && (p-i.accepted).norm()<=.003001f && p.z>i.accepted.z); ++count; }
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
      a.request(2,3,wrong,i); assert(a.moving()); a.request(3,3,i.binding,i);
      assert(a.phase==ApproachPhase::Canceled); ++count; }
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
    return count==25 ? 0 : 1;
}
