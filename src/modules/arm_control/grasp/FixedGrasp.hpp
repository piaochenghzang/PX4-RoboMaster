#pragma once
#include "HoverApproach.hpp"
#include <algorithm>

namespace so101_hover {
constexpr float fixedLiftDistance=.04f; // Tool extraction; actual payload acceptance stays 25mm.
constexpr float fixedCartesianSpeed=.05f;
constexpr float fixedArmJointSlew=.4f;
constexpr float fixedJawSlew=.2f;
inline bool precisionPositionReady(float error) {
    // Body motion is compensated in the live TCP target.  Keep the actual
    // finger alignment/orientation gate authoritative during close handover.
    return std::isfinite(error) && error>=0.f && error<=.010f;
}
inline bool fixedCloseGeometryReady(float alignment,float speed,float orientation,bool closing=false) {
    // Fixed SITL demo: at most 2mm relative travel during the 200ms dwell.
    // This does not change the generic prepare-only hover gate.
    // Contact intentionally moves/centers the cube relative to its OPEN-jaw
    // reference. Entry pose is not a continued-closing safety certificate.
    // The caller still requires live hover, full jaw sweep and motion reserve.
    return std::isfinite(alignment) && alignment>=0.f &&
        std::isfinite(speed) && speed>=0.f &&
        std::isfinite(orientation) && orientation>=0.f &&
        (closing || (alignment<=.003f && speed<=.010f && orientation<=.0872665f));
}
// SITL fixed-point grasp only. Never reuses the anchored bench authorization.
enum class FixedPhase:uint8_t { Idle,Approach,Align,Close,Verify,Lift,Hold,Done,Fault,Canceled };
enum class FixedFault:uint8_t { None,Contract,Feedback,Geometry,IK,Timeout,Lost,Support,Clock };
struct FixedInput {
    ApproachInput motion;
    so101_grasp::Input feedback;
    bool profile{false},close_ready{false},unfold_complete{false},jaw_sweep_valid{false};
    bool contact_only{false}; // Source category, independent of the current force capability.
    Point lift_goal;
};
struct FixedAction {
    bool send{false},jaw{false},stop{false};
    Point goal; float jaw_target{.3f},max_joint_step{.02f},max_jaw_step{.02f};
};
class FixedGrasp {
public:
    bool owns() const { return phase!=FixedPhase::Idle; }
    bool reportsStatus(bool fixed_context) const { return owns() || (fixed_context && last_id); }
    bool carrying() const { return phase>=FixedPhase::Lift && phase<=FixedPhase::Done; }
    bool safe(const FixedInput &v) const {
        const auto &i=v.motion; const auto &f=v.feedback;
        const float values[]{f.bilateral,f.other,f.support_arm,f.jaw_speed};
        for(float x:values)if(!std::isfinite(x))return false;
        if (v.contact_only ? !std::isnan(f.support) : !std::isfinite(f.support)) return false;
        if (!feedbackCapabilities(v) || (owns() && v.contact_only!=contact_mode)) return false;
        if ((f.capabilities&2) && (!std::isfinite(f.fixed_force) || !std::isfinite(f.moving_force) ||
                f.fixed_force<0 || f.moving_force<0)) return false;
        if (!v.contact_only && (!std::isfinite(f.object_height) || !f.object_local.finite())) return false;
        return (!(phase>=FixedPhase::Close && phase<=FixedPhase::Done) || v.jaw_sweep_valid) &&
            v.profile && i.binding.valid() && i.context_fresh && i.lease_fresh && i.arm_ready &&
            i.geometry_valid && i.hard_hover_ok && std::isfinite(i.clearance) && i.clearance>.016f &&
            i.goal.finite() && i.accepted.finite() && i.measured.finite() && v.lift_goal.finite() &&
            std::abs((v.lift_goal-i.goal).norm()-fixedLiftDistance)<.001f &&
            (!v.contact_only || std::isfinite(toolProjection(v))) &&
            f.finite && f.profile==1 && f.target_id==1 &&
            f.joint_age<.2f && f.grasp_age<.3f && f.other>=0 && f.other<1e-6f &&
            f.support_arm>=0 && f.support_arm<1e-6f &&
            std::isfinite(i.joint_speed) && i.joint_speed>=0 &&
            std::isfinite(i.tool_relative_speed) && i.tool_relative_speed>=0;
    }
    void request(uint64_t id,uint8_t action,const Binding &b,const FixedInput &v) {
        if (!id || id<=last_id) return;
        last_id=id; request_result=3;
        if (owns() && b==binding && action==6) {
            phase=FixedPhase::Canceled; stop_pending=true; request_result=1; return;
        }
        if (owns() && phase!=FixedPhase::Fault && phase!=FixedPhase::Canceled &&
            b==binding && action==7 && paused && safe(v) && v.motion.hover_ready) {
            paused=false; last_send=0; request_result=1; return;
        }
        if (action==5 && phase==FixedPhase::Approach && !paused && b==binding && safe(v) &&
            v.motion.hover_ready && v.close_ready && v.jaw_sweep_valid) {
            enter(FixedPhase::Align,v.motion.now); request_result=1; return;
        }
        if (action!=4 || owns() || !b.valid() || !(b==v.motion.binding) || !safe(v) ||
            !v.motion.hover_ready || !v.motion.prepared) return;
        binding=b; epoch=v.feedback.epoch; identity=v.feedback.source_identity;
        contact_mode=v.contact_only; actual_lift=contact_mode ? NAN : 0.f; tool_progress=NAN;
        enter(FixedPhase::Approach,v.motion.now); started=v.motion.now; request_result=1;
    }
    void fail(FixedFault why) {
        if (owns()) { phase=FixedPhase::Fault; fault=why; stop_pending=true; }
    }
    FixedAction step(const FixedInput &v) {
        FixedAction out{}; const auto &i=v.motion; const auto &f=v.feedback;
        if (!owns()) return out;
        const uint64_t period=v.unfold_complete ? 20000 : 50000;
        const uint64_t elapsed=last_now && i.now>=last_now ? std::min(period,i.now-last_now) : period;
        out.max_jaw_step=fixedJawSlew*float(elapsed)*1e-6f;
        const bool live=phase!=FixedPhase::Fault && phase!=FixedPhase::Canceled;
        if (live) {
            if (last_now && i.now<last_now) fail(FixedFault::Clock);
            else if (!(binding==i.binding) || !v.profile || !i.context_fresh || !i.lease_fresh || !i.hard_hover_ok)
                fail(FixedFault::Contract);
            else if (!i.arm_ready || f.joint_age>=.2f || f.grasp_age>=.3f || !f.finite ||
                !feedbackCapabilities(v) || v.contact_only!=contact_mode ||
                f.profile!=1 || f.target_id!=1 || f.epoch!=epoch || f.source_identity!=identity)
                fail(FixedFault::Feedback);
            else if (!safe(v)) fail(FixedFault::Geometry);
            else if (i.now-started>240000000) fail(FixedFault::Timeout);
        }
        last_now=i.now;
        if (stop_pending) { stop_pending=false; out.stop=true; return out; }
        if (phase==FixedPhase::Fault || phase==FixedPhase::Canceled) return out;
        if (!i.hover_ready && !paused) { paused=true; stable_since=0; out.stop=true; return out; }
        const bool fresh=f.sequence!=last_sequence;
        if (fresh) last_sequence=f.sequence;
        const bool both=(f.capabilities&2) && f.frames && f.bilateral>=.9f && f.fixed_force>.01f && f.moving_force>.01f;
        // Only source 1 observes payload-support contact. Source 2 cannot claim release.
        const bool source1_unsupported=!contact_mode && f.support<=.001f;
        if (carrying()) {
            if (contact_mode) {
                // This is tool extraction relative to the CURRENT world-fixed
                // goal, expressed in the current arm frame. It is not a payload
                // height measurement or a fabricated object pose.
                tool_progress=toolProjection(v)-initial_tool_projection;
                actual_lift=NAN;
            } else {
                actual_lift=f.object_height-initial_height;
                if ((f.object_local-grasp_reference).norm()>.045f) fail(FixedFault::Lost);
            }
            if (!both) {
                if (!lost_since) lost_since=i.now;
                if (i.now-lost_since>500000) fail(FixedFault::Lost);
            } else lost_since=0;
            if (f.other>0 || f.support_arm>0) fail(FixedFault::Geometry);
            if (!contact_mode && (phase==FixedPhase::Hold || phase==FixedPhase::Done) &&
                !source1_unsupported) fail(FixedFault::Support);
            if (stop_pending) { stop_pending=false; out.stop=true; return out; }
        }
        if (paused) return out; // Payload monitoring continues while motion is paused.
        // Source 2 confirms retained contact and measured tool extraction only.
        // Actual payload lift/support release is checked by the offline observer.
        const bool extraction_ready=contact_mode ? tool_progress>=.025f : actual_lift>=.025f && source1_unsupported;
        if (phase==FixedPhase::Align) {
            // This SITL fixed-point demo already checks live close geometry
            // every cycle, including during closure. A short alignment dwell
            // avoids requiring a hovering vehicle to be perfectly still for 1s.
            if (dwell(v.close_ready && v.jaw_sweep_valid && i.hover_ready,i.now,200000)) enter(FixedPhase::Close,i.now);
        } else if (phase==FixedPhase::Close) {
            // Stop advancing if live hover/geometry/motion budget degrades.
            if (v.close_ready) { out.jaw=true; out.jaw_target=.04f; }
            if (both && std::abs(f.jaw_speed)<.02f && fresh) enter(FixedPhase::Verify,i.now);
            if (i.now-entered>20000000) fail(FixedFault::Timeout);
        } else if (phase==FixedPhase::Verify) {
            if (!both) enter(FixedPhase::Close,i.now);
            else if (fresh && dwell(both && std::abs(f.jaw_speed)<.02f,i.now,1000000)) {
                if (contact_mode) {
                    initial_tool_projection=toolProjection(v); tool_progress=0.f; actual_lift=NAN;
                } else { initial_height=f.object_height; grasp_reference=f.object_local; }
                enter(FixedPhase::Lift,i.now);
            }
        } else if (phase==FixedPhase::Lift) {
            if (dwell(extraction_ready && both &&
                (v.lift_goal-i.measured).norm()<=.005f,i.now,1000000)) enter(FixedPhase::Hold,i.now);
            if (i.now-entered>45000000) fail(FixedFault::Timeout);
        } else if (phase==FixedPhase::Hold) {
            if (!extraction_ready || !both) { stable_since=0; }
            else if (dwell(true,i.now,5000000)) enter(FixedPhase::Done,i.now);
        }
        if (stop_pending) { stop_pending=false; out.stop=true; return out; }
        if (phase==FixedPhase::Approach) { out.jaw=true; out.jaw_target=.3f; }
        if (phase==FixedPhase::Verify || carrying()) { out.jaw=true; out.jaw_target=.04f; }
        const Point goal=carrying() ? v.lift_goal : i.goal;
        const auto delta=goal-i.accepted; const float distance=delta.norm();
        // IK admission is independent of the finer joint-trajectory cadence.
        // Do not lose an entire update on small scheduler/clock jitter.
        // Budget ACTUAL elapsed time, capped at one period: no catch-up jump
        // and no increase above the bounded Cartesian / arm-joint rates.
        const uint64_t minimum_interval=v.unfold_complete ? 16000 : period;
        if (distance>.0002f && (!last_send || i.now-last_send>=minimum_interval) &&
            (i.accepted-i.measured).norm()<=.015f && i.joint_speed<=.20f) {
            const float seconds=float(last_send ? std::min(period,i.now-last_send) : period)*1e-6f;
            const float max_step=fixedCartesianSpeed*seconds;
            out.max_joint_step=fixedArmJointSlew*seconds;
            last_send=i.now; out.send=true; out.goal=i.accepted+delta*(distance>max_step ? max_step/distance : 1.f);
        }
        return out;
    }
    FixedPhase phase{FixedPhase::Idle}; FixedFault fault{FixedFault::None}; Binding binding;
    bool paused{false}; uint64_t last_id{0}; uint8_t request_result{0}; float actual_lift{0};
    float tool_progress{NAN}; // Internal extraction estimate; never published as object lift.
private:
    uint64_t entered{0},started{0},last_now{0},last_send{0},stable_since{0},lost_since{0};
    uint32_t epoch{0},last_sequence{0}; uint16_t identity{0};
    bool stop_pending{false},contact_mode{false}; float initial_height{0},initial_tool_projection{0}; Point grasp_reference;
    static bool feedbackCapabilities(const FixedInput &v) {
        return v.contact_only ? (v.feedback.capabilities==1 || v.feedback.capabilities==3) : v.feedback.capabilities==15;
    }
    static float toolProjection(const FixedInput &v) {
        const auto axis=v.lift_goal-v.motion.goal,offset=v.motion.measured-v.motion.goal;
        return (axis.x*offset.x+axis.y*offset.y+axis.z*offset.z)/axis.norm();
    }
    void enter(FixedPhase p,uint64_t now) { phase=p; entered=now; stable_since=0; }
    bool dwell(bool good,uint64_t now,uint64_t duration) {
        if (!good) { stable_since=0; return false; }
        if (!stable_since) stable_since=now;
        return now-stable_since>=duration;
    }
};
}
