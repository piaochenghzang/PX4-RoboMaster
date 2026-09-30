#include "GraspStateMachine.hpp"
#include <algorithm>

namespace so101_grasp {
bool GraspStateMachine::dwell(bool good, uint64_t now, uint64_t duration)
{
    if (!good) { qualified_since=0; return false; }
    if (!qualified_since) qualified_since=now;
    return now-qualified_since >= duration;
}
bool GraspStateMachine::clamped(uint64_t now) const
{
    double frames=0, both=0, fixed=0, moving=0;
    uint64_t oldest=now;
    for (const auto &w : windows) {
        if (!w.frames || now<w.time || now-w.time>1000000) continue;
        frames+=w.frames; both+=double(w.frames)*double(w.both);
        fixed+=double(w.frames)*double(w.fixed); moving+=double(w.frames)*double(w.moving);
        oldest=std::min(oldest,w.time);
    }
    return frames>0 && now-oldest>=800000 && both/frames>=.9 && fixed/frames>.01 && moving/frames>.01;
}
RequestResult GraspStateMachine::request(uint64_t id, uint8_t action, uint16_t profile, uint32_t target,
    float duration, const Input &in)
{
    if (!id) return RequestResult::Invalid;
    if (id==last_id) return last_request_result;
    if (id<last_id) return RequestResult::Invalid;
    last_id=id;
    if (action==2 && owns()) {
        enter(Phase::Aborted,in.now); result=Result::Canceled; fault=Fault::None; stop_pending=true;
        return last_request_result=RequestResult::Accepted;
    }
    if (action==3 && (phase==Phase::Aborted || phase==Phase::Fault || phase==Phase::Done)) {
        if (payload!=Payload::Empty && payload!=Payload::Lost) return last_request_result=RequestResult::PayloadLocked;
        *this=GraspStateMachine{}; last_id=id;
        return last_request_result=RequestResult::Accepted;
    }
    if (action!=1 || profile!=1 || target!=1 || !std::isfinite(duration) || duration<5.f || duration>120.f) {
        return last_request_result=RequestResult::Invalid;
    }
    if (owns()) return last_request_result=RequestResult::Busy;
    if (!in.initialized || !in.finite || in.capabilities!=31 || in.profile!=1 || in.target_id!=1
        || in.joint_age>=.2f || in.grasp_age>=.3f) return last_request_result=RequestResult::Invalid;
    active_id=id; hold_seconds=duration; result=Result::Running; enter(Phase::Precheck,in.now);
    last_now=in.now; source_identity=in.source_identity; epoch=in.epoch; identity_set=true;
    last_sequence=in.sequence;
    return last_request_result=RequestResult::Accepted;
}
Action GraspStateMachine::drive(const Point &goal, float jaw, float max_step, const Input &in, bool &arrived)
{
    arrived=false;
    const auto delta=goal-in.accepted_target;
    const float distance=delta.norm();
    if (!stage_sent || distance>.0002f) {
        qualified_since=0;
        if ((!last_send || in.now-last_send>=500000) && (!stage_sent || benchNear(in))) {
            const float fraction=distance>max_step ? max_step/distance : 1.f;
            last_send=in.now; stage_sent=true;
            return {ActionType::Target,in.accepted_target+delta*fraction,jaw};
        }
    } else arrived=dwell(benchNear(in),in.now,500000);
    return {};
}
Action GraspStateMachine::step(const Input &in)
{
    if (phase==Phase::Idle) return {};
    const bool latched=phase==Phase::Fault || phase==Phase::Aborted;
    if (!latched) {
        if (in.now<last_now) fail(Fault::ClockReset);
        else if (!in.finite) fail(Fault::Nonfinite);
        else if (in.joint_age>.3f) fail(Fault::JointStale);
        else if (in.grasp_age>.3f) fail(Fault::GraspStale);
        else if (identity_set && (in.epoch!=epoch || in.source_identity!=source_identity
            || in.profile!=1 || in.target_id!=1)) fail(Fault::SourceChanged);
        else if (in.capabilities!=31) fail(Fault::InvalidSensor);
    }
    last_now=in.now;
    if (stop_pending) { stop_pending=false; return {ActionType::Stop,{},-.15f}; }
    if (phase==Phase::Fault || phase==Phase::Aborted) return {};
    if (in.joint_age>=.2f) return {};
    const bool fresh=in.sequence!=last_sequence;
    if (fresh) {
        last_sequence=in.sequence;
        windows[window_cursor++%16]={in.now,in.frames,in.bilateral,in.support,in.fixed_force,in.moving_force};
    }
    alignment_error=in.alignment_delta_base.norm();
    const bool carrying=phase>=Phase::ExitSupport && phase<=Phase::Done;
    if (carrying) {
        actual_lift=in.object_height-initial_height;
        if (phase==Phase::Done) hold_drift=(in.object_local-hold_reference).norm();
        if ((in.object_local-grasp_reference).norm()>.045f) { payload=Payload::Lost; fail(Fault::Separated); }
        else if (in.other>.2f || in.support_arm>.2f) fail(Fault::Obstructed);
        else if (in.bilateral<.5f) {
            if (!lost_since) lost_since=in.now;
            if (in.now-lost_since>500000) fail(Fault::ContactLost);
        } else lost_since=0;
        if (phase>=Phase::Lift && phase<=Phase::Done && in.support>.01f) fail(Fault::SupportRecontact);
        if (stop_pending) { stop_pending=false; return {ActionType::Stop,{},-.15f}; }
    }
    const float elapsed=(in.now-entered)*1e-6f;
    float timeout=60.f;
    switch (phase) {
    case Phase::Precheck: timeout=5.f; break;
    case Phase::Approach: timeout=35.f; break;
    case Phase::Align: timeout=120.f; break;
    case Phase::Close: timeout=12.f; break;
    case Phase::Verify: timeout=3.f; break;
    case Phase::Settle: timeout=5.f; break;
    case Phase::Hold: timeout=hold_seconds+1.f; break;
    case Phase::Done: return {};
    default: break;
    }
    if (elapsed>timeout) {
        fail(phase==Phase::Close || phase==Phase::Verify ? Fault::Unconfirmed : Fault::Timeout);
        stop_pending=false; return {ActionType::Stop,{},-.15f};
    }
    bool arrived=false; Action action;
    switch (phase) {
    case Phase::Precheck:
        if (dwell(in.initialized && benchNear(in),in.now,500000)) enter(Phase::Approach,in.now);
        break;
    case Phase::Approach:
        action=drive({.17f,.035f,.05f},.6f,.02f,in,arrived);
        if (arrived) { enter(Phase::Align,in.now); aligned_count=0; aligned_sample=0; }
        break;
    case Phase::Align:
        if (in.other>.2f || in.support_arm>.2f || alignment_error>.10f) {
            fail(Fault::Obstructed); stop_pending=false; return {ActionType::Stop,{},.6f};
        }
        if (alignment_error<=.003f && in.joint_speed<=.05f && fresh) {
            if (!aligned_sample || in.now-aligned_sample>=600000) {
                aligned_sample=in.now;
                if (++aligned_count>=3) {
                    pickup=in.accepted_target; enter(Phase::Close,in.now); payload=Payload::Candidate;
                }
            }
        } else if (alignment_error>.003f || in.joint_speed>.05f) {
            aligned_count=0; aligned_sample=0;
            if (alignment_error<=.003f) break;
            if (!last_send || in.now-last_send>=1000000) {
                auto delta=in.alignment_delta_base*.6f; const float length=delta.norm();
                if (length>.003f) delta=delta*(.003f/length);
                last_send=in.now; action={ActionType::Target,in.accepted_target+delta,.6f};
            }
        }
        break;
    case Phase::Close:
        if (!stage_sent) { stage_sent=true; action={ActionType::JawOnly,{},-.15f}; }
        if (elapsed>=2.f && clamped(in.now) && std::abs(in.jaw_speed)<.02f) enter(Phase::Verify,in.now);
        break;
    case Phase::Verify:
        if (dwell(clamped(in.now) && std::abs(in.jaw_speed)<.02f,in.now,1000000)) {
            initial_height=in.object_height; grasp_reference=in.object_local; payload=Payload::Clamped;
            cleared=pickup+Point{0,.07f,0}; clear_since=0; enter(Phase::ExitSupport,in.now);
        }
        break;
    case Phase::ExitSupport:
        action=drive(cleared,-.15f,.003f,in,arrived);
        if (!arrived || in.support>.01f) clear_since=0;
        else if (!clear_since) clear_since=in.now;
        if (clear_since && in.now-clear_since>=1000000) {
            if (in.clearance<.003f) { fail(Fault::Clearance); }
            else { lifted=cleared+Point{0,0,-.035f}; enter(Phase::Lift,in.now); }
        }
        break;
    case Phase::Lift:
        action=drive(lifted,-.15f,.003f,in,arrived);
        if (arrived) {
            if (actual_lift<.025f) fail(Fault::LiftInsufficient);
            else { payload=Payload::Held; enter(Phase::RetractClear,in.now); }
        }
        break;
    case Phase::RetractClear:
        action=drive({.12f,cleared.y,lifted.z},-.15f,.003f,in,arrived);
        if (arrived) enter(Phase::Retract,in.now);
        break;
    case Phase::Retract:
        action=drive({.12f,.015f,.03f},-.15f,.003f,in,arrived);
        if (arrived) enter(Phase::Settle,in.now);
        break;
    case Phase::Settle:
        if (dwell(benchNear(in),in.now,2000000)) {
            if (actual_lift<.025f) fail(Fault::LiftInsufficient);
            else {
                motion_shift=(in.object_local-grasp_reference).norm(); hold_reference=in.object_local;
                hold_min_height=in.object_height; hold_frames=hold_both=hold_support=hold_fixed=hold_moving=0;
                enter(Phase::Hold,in.now);
            }
        }
        break;
    case Phase::Hold:
        hold_drift=(in.object_local-hold_reference).norm(); hold_min_height=std::min(hold_min_height,in.object_height);
        if (fresh) {
            hold_frames+=in.frames; hold_both+=double(in.frames)*double(in.bilateral);
            hold_support+=double(in.frames)*double(in.support);
            hold_fixed+=double(in.frames)*double(in.fixed_force); hold_moving+=double(in.frames)*double(in.moving_force);
        }
        if (elapsed>=hold_seconds) {
            if (hold_frames<=0 || hold_both/hold_frames<.9 || hold_support/hold_frames>.001
                || hold_fixed/hold_frames<=.01 || hold_moving/hold_frames<=.01 || hold_min_height<=initial_height) fail(Fault::Retention);
            else { completed_success=true; result=Result::Succeeded; enter(Phase::Done,in.now); }
        }
        break;
    default: break;
    }
    if (stop_pending) { stop_pending=false; return {ActionType::Stop,{},-.15f}; }
    return action;
}
}
