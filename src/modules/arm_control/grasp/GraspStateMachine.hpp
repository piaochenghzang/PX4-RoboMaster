#pragma once
#include <cmath>
#include <cstdint>

namespace so101_grasp {
struct Point {
    float x{0}, y{0}, z{0};
    Point() = default;
    Point(float a, float b, float c) : x(a), y(b), z(c) {}
    Point operator+(const Point &b) const { return {x+b.x,y+b.y,z+b.z}; }
    Point operator-(const Point &b) const { return {x-b.x,y-b.y,z-b.z}; }
    Point operator*(float k) const { return {x*k,y*k,z*k}; }
    float norm() const { return std::sqrt(x*x+y*y+z*z); }
    bool finite() const { return std::isfinite(x)&&std::isfinite(y)&&std::isfinite(z); }
};
// Only a known initial target outside the workspace can recover inward.
// No outward motion, larger step, or general workspace bypass is permitted.
inline bool monotonicRecovery(const Point &from, const Point &to, const Point &low, const Point &high, float step)
{
    if (!from.finite() || !to.finite() || (to-from).norm()>step+1e-6f) return false;
    const float a[]{from.x,from.y,from.z}, b[]{to.x,to.y,to.z};
    const float lo[]{low.x,low.y,low.z}, hi[]{high.x,high.y,high.z};
    bool outside=false, progress=false;
    for (int k=0; k<3; ++k) {
        if (a[k]<lo[k]) {
            outside=true; if (b[k]<a[k] || b[k]>hi[k]) return false;
            progress|=b[k]>a[k]+1e-6f;
        } else if (a[k]>hi[k]) {
            outside=true; if (b[k]>a[k] || b[k]<lo[k]) return false;
            progress|=b[k]<a[k]-1e-6f;
        } else if (b[k]<lo[k] || b[k]>hi[k]) return false;
    }
    return outside && progress;
}
enum class Phase : uint8_t { Idle, Precheck, Approach, Align, Close, Verify, ExitSupport,
    Lift, RetractClear, Retract, Settle, Hold, Done, Aborted, Fault };
enum class Result : uint8_t { None, Running, Succeeded, Canceled, Failed };
enum class Payload : uint8_t { Empty, Candidate, Clamped, Held, Lost, Unknown };
enum class Fault : uint8_t { None, InvalidRequest, JointStale, GraspStale, SourceChanged,
    ClockReset, InvalidSensor, Nonfinite, IKRejected, Timeout, Unconfirmed, Separated,
    ContactLost, Obstructed, SupportRecontact, Clearance, LiftInsufficient, Retention };
enum class RequestResult : uint8_t { None, Accepted, Busy, Invalid, PayloadLocked };
enum class ActionType : uint8_t { None, Target, JawOnly, Stop };
struct Action { ActionType type{ActionType::None}; Point target; float jaw{.6f}; };
struct Input {
    uint64_t now{0}, sample_stamp{0};
    bool initialized{false}, finite{false}, strict_arrived{false};
    float joint_age{INFINITY}, grasp_age{INFINITY}, position_error{INFINITY};
    float joint_error{INFINITY}, joint_speed{INFINITY}, jaw_speed{INFINITY};
    Point accepted_target, object_local, alignment_delta_base;
    uint32_t target_sequence{0}, sequence{0}, epoch{0}, target_id{0}, frames{0};
    uint16_t capabilities{0}, profile{0}, source_identity{0};
    float bilateral{0}, support{0}, other{0}, support_arm{0}, fixed_force{0}, moving_force{0};
    float object_height{0}, clearance{0};
};
// Bounded, allocation-free logic. Time, observations and actuation acknowledgement
// are injected; no sleeps, Gazebo/ROS dependencies or joint publications here.
class GraspStateMachine {
public:
    RequestResult request(uint64_t id, uint8_t action, uint16_t profile, uint32_t target, float duration, const Input &in);
    Action step(const Input &in);
    void acknowledge(bool accepted) { if (!accepted && owns()) { fail(Fault::IKRejected); } }
    bool owns() const { return phase != Phase::Idle; }
    bool benchNear(const Input &in) const {
        return in.initialized && in.finite && in.joint_age < .2f && in.position_error <= .015f
            && in.joint_error <= .08f && in.joint_speed <= .05f;
    }
    Phase phase{Phase::Idle}; Result result{Result::None}; Payload payload{Payload::Empty};
    Fault fault{Fault::None}; RequestResult last_request_result{RequestResult::None};
    Phase failed_phase{Phase::Idle};
    uint64_t active_id{0}, last_id{0}, entered{0};
    bool completed_success{false};
    float alignment_error{INFINITY}, motion_shift{0}, hold_drift{0}, actual_lift{0};
private:
    struct Window { uint64_t time{0}; uint32_t frames{0}; float both{0}, support{0}, fixed{0}, moving{0}; };
    Window windows[16]{};
    unsigned window_cursor{0};
    uint32_t last_sequence{0}, epoch{0}; uint16_t source_identity{0};
    uint64_t last_now{0}, qualified_since{0}, last_send{0}, lost_since{0}, aligned_sample{0}, clear_since{0};
    bool identity_set{false}, stage_sent{false}, stop_pending{false};
    unsigned aligned_count{0};
    float hold_seconds{30}, initial_height{0}, hold_min_height{0};
    double hold_frames{0}, hold_both{0}, hold_support{0}, hold_fixed{0}, hold_moving{0};
    Point pickup, cleared, lifted, grasp_reference, hold_reference;
    void enter(Phase next, uint64_t now) { phase=next; entered=now; qualified_since=0; last_send=0; stage_sent=false; }
    void fail(Fault reason) { failed_phase=phase; phase=Phase::Fault; result=Result::Failed; fault=reason; stop_pending=true;
        if (payload != Payload::Empty && payload != Payload::Lost) { payload=Payload::Unknown; } }
    bool dwell(bool good, uint64_t now, uint64_t duration);
    bool clamped(uint64_t now) const;
    Action drive(const Point &goal, float jaw, float max_step, const Input &in, bool &arrived);
};
}
