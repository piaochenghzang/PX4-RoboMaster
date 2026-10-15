#pragma once
#include "GraspStateMachine.hpp"

namespace so101_hover {
using Point=so101_grasp::Point;
enum class TargetResult:uint8_t { Applied, WaitClearance, Rejected };
template<unsigned N,class Limit>
bool freezeCommand(const float *command,const Limit *limits,float (&out)[N])
{
    // Never retarget to raw feedback: that unloads a gravity-bearing servo
    // and ratchets the arm downward on repeated pauses. Freeze the setpoint
    // already sent to the actuator, not a still-pending future IK target.
    for (unsigned j=0;j<N;++j) {
        if (!std::isfinite(command[j]) || command[j]<limits[j].min_rad || command[j]>limits[j].max_rad) return false;
    }
    for (unsigned j=0;j<N;++j) out[j]=command[j];
    return true;
}
template<unsigned N>
float pendingDisplacement(const float (&anchor)[N],const float (&measured)[N],const float (&radii)[N])
{
    float reserve=0;
    for (unsigned j=0;j<N;++j) {
        if (!std::isfinite(anchor[j]) || !std::isfinite(measured[j]) ||
            !std::isfinite(radii[j]) || radii[j]<=0) return INFINITY;
        reserve+=radii[j]*std::abs(anchor[j]-measured[j]);
    }
    return reserve;
}
// Interpolate between the last accepted target and a legal IK candidate,
// never from raw feedback. Feedback can be outside a command limit or lag
// behind it. Reserve that pending tracking displacement BEFORE adding motion.
template<unsigned N,class Limit>
bool boundJointBudget(const float (&desired)[N],const float (&anchor)[N],const float (&measured)[N],
    const Limit *limits,const float (&radii)[N],float budget,float (&out)[N])
{
    if (!std::isfinite(budget) || budget<=0) return false;
    float reserve=0,travel=0;
    for (unsigned j=0;j<N;++j) {
        const auto &l=limits[j];
        if (!std::isfinite(desired[j]) || !std::isfinite(anchor[j]) || !std::isfinite(measured[j]) ||
            !std::isfinite(radii[j]) || radii[j]<=0 ||
            !std::isfinite(l.min_rad) || !std::isfinite(l.max_rad) || l.min_rad>l.max_rad ||
            desired[j]<l.min_rad || desired[j]>l.max_rad || anchor[j]<l.min_rad || anchor[j]>l.max_rad) return false;
        reserve+=radii[j]*std::abs(anchor[j]-measured[j]);
        travel+=radii[j]*std::abs(desired[j]-anchor[j]);
    }
    if (reserve>=budget) return false;
    const float scale=travel>budget-reserve ? (budget-reserve)/travel : 1.f;
    for (unsigned j=0;j<N;++j) {
        const float value=anchor[j]+scale*(desired[j]-anchor[j]);
        out[j]=value<limits[j].min_rad ? limits[j].min_rad : (value>limits[j].max_rad ? limits[j].max_rad : value);
    }
    return true;
}
struct Binding {
    uint64_t task{0},plan{0},target{0},scene{0};
    bool valid() const { return task && plan && target && scene; }
    bool operator==(const Binding &b) const { return task==b.task && plan==b.plan && target==b.target && scene==b.scene; }
};
enum class ApproachPhase:uint8_t { Idle, Approaching, Holding, Paused, Fault, Canceled };
enum class ApproachFault:uint8_t { None, ContextOrLease, BindingChanged, HoverHard, Geometry, IK, Timeout };
enum class PauseReason:uint8_t { None, Hover, Clearance };
struct ApproachInput {
    uint64_t now{0}; Binding binding;
    bool context_fresh{false},lease_fresh{false},geometry_valid{false},arm_ready{false};
    bool hard_hover_ok{false},hover_ready{false},prepared{false};
    bool clearance_ready{false};
    float clearance{0},joint_speed{INFINITY},tool_relative_speed{INFINITY};
    Point goal,accepted,measured;
};
class Approach {
public:
    bool owns() const { return phase!=ApproachPhase::Idle; }
    bool moving() const { return phase==ApproachPhase::Approaching || phase==ApproachPhase::Holding; }
    bool safe(const ApproachInput &i) const {
        return i.binding.valid() && i.context_fresh && i.lease_fresh && i.geometry_valid && i.arm_ready &&
            i.hard_hover_ok && i.goal.finite() && i.accepted.finite() && i.measured.finite() &&
            std::isfinite(i.clearance) && i.clearance>.016f && std::isfinite(i.joint_speed) &&
            std::isfinite(i.tool_relative_speed) && i.tool_relative_speed>=0;
    }
    bool resumeReady(const ApproachInput &i) const {
        return phase==ApproachPhase::Paused && binding==i.binding && safe(i) && i.hover_ready &&
            (pause_reason!=PauseReason::Clearance || i.clearance_ready);
    }
    void pause(PauseReason why) {
        if (moving()) { phase=ApproachPhase::Paused; pause_reason=why; stable_since=0; stop_pending=true; }
    }
    void request(uint64_t id,uint8_t action,const Binding &b,const ApproachInput &i) {
        if (!id || id<=last_id) return;
        last_id=id; request_result=3;
        if (action==3 && owns() && b==binding) {
            phase=ApproachPhase::Canceled; stop_pending=true; request_result=1; return;
        }
        if (action==2 && b==binding && resumeReady(i)) {
            phase=ApproachPhase::Approaching; pause_reason=PauseReason::None; stable_since=0; request_result=1; return;
        }
        if (action!=1 || owns() || !b.valid() || !(b==i.binding) || !safe(i) || !i.hover_ready || !i.prepared) return;
        binding=b; phase=ApproachPhase::Approaching; started=i.now; request_result=1;
    }
    void fail(ApproachFault f) { if (owns()) { fault=f; phase=ApproachPhase::Fault; stop_pending=true; } }
    Point step(const ApproachInput &i,bool &send,bool &stop) {
        send=stop=false;
        if (!owns()) return {};
        if (moving() || phase==ApproachPhase::Paused) {
            if (!(binding==i.binding)) fail(ApproachFault::BindingChanged);
            else if (!i.context_fresh || !i.lease_fresh || !i.arm_ready) fail(ApproachFault::ContextOrLease);
            else if (!i.hard_hover_ok) fail(ApproachFault::HoverHard);
            else if (!i.geometry_valid || !safe(i)) fail(ApproachFault::Geometry);
            else if (i.now<last_now || i.now-started>180000000) fail(ApproachFault::Timeout);
        }
        last_now=i.now;
        if (moving() && !i.hover_ready) {
            pause(PauseReason::Hover);
        }
        if (stop_pending) { stop_pending=false; stop=true; return {}; }
        if (!moving()) return {};
        error=(i.goal-i.measured).norm();
        // A world-fixed tool must compensate body motion with MOVING joints.
        // This 10 mm/s arrival test is for no-close offset approach only; the
        // separate close-geometry 5 mm/s gate and all joint limits are unchanged.
        if (error<=.015f && i.tool_relative_speed<=.010f) {
            if (!stable_since) stable_since=i.now;
            if (i.now-stable_since>=1000000) phase=ApproachPhase::Holding;
        } else { stable_since=0; phase=ApproachPhase::Approaching; }
        const auto delta=i.goal-i.accepted; const float distance=delta.norm();
        if (distance>.001f && (!last_send || i.now-last_send>=100000) &&
            (i.accepted-i.measured).norm()<=.015f && i.joint_speed<=.10f) {
            last_send=i.now; send=true;
            return i.accepted+delta*(distance>.003f ? .003f/distance : 1.f);
        }
        return {};
    }
    Binding binding; ApproachPhase phase{ApproachPhase::Idle}; ApproachFault fault{ApproachFault::None};
    PauseReason pause_reason{PauseReason::None};
    uint64_t last_id{0}; uint8_t request_result{0}; float error{INFINITY};
private:
    uint64_t started{0},last_now{0},last_send{0},stable_since{0}; bool stop_pending{false};
};
}
