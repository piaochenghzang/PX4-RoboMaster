#pragma once
#include <cmath>
#include <cstdint>
namespace so101_hover {
enum Denial : uint16_t {
    Missing=1, Stale=2, Invalid=4, Flight=8, Allocation=16, Position=32,
    Speed=64, Angular=128, Tilt=256, Reset=512, Dwelling=1024
};
// LOCAL_FRD EV yaw is locally usable without declaring alignment to true north.
// Keep this separate from PX4's global heading_good_for_control flag.
struct LocalHeading {
    uint64_t now{0},flags_stamp{0},received{0},sample{0};
    bool fused{false},yaw_fault{true},frame_frd{false};
    float heading{NAN},variance{NAN},ev_variance{NAN};
    float q[4]{NAN,NAN,NAN,NAN};
};
inline bool localHeadingReady(const LocalHeading &v) {
    auto fresh=[&v](uint64_t stamp,uint64_t age) {
        return stamp && stamp<=v.now && v.now-stamp<=age;
    };
    // Flags publish on change or at 1 Hz; odometry still has its own short lease.
    if (!v.fused || v.yaw_fault || !v.frame_frd || !fresh(v.flags_stamp,1200000) ||
        !fresh(v.received,300000) || !fresh(v.sample,250000)) return false;
    constexpr float bound=.0872665f; // Existing 5-degree attitude/close bound.
    if (!std::isfinite(v.heading) || !std::isfinite(v.variance) || v.variance<0 ||
        v.variance>bound*bound || !std::isfinite(v.ev_variance) || v.ev_variance<0 ||
        v.ev_variance>bound*bound) return false;
    float norm=0;
    for (float x:v.q) { if (!std::isfinite(x)) return false; norm+=x*x; }
    if (std::fabs(norm-1.f)>.01f) return false;
    const float yaw=std::atan2(2.f*(v.q[0]*v.q[3]+v.q[1]*v.q[2]),
        norm-2.f*(v.q[2]*v.q[2]+v.q[3]*v.q[3]));
    const float error=std::atan2(std::sin(v.heading-yaw),std::cos(v.heading-yaw));
    return std::fabs(error)<=bound;
}
struct Input {
    uint64_t now{0},received{0},sample{0},task{0},reset_key{0};
    uint32_t sequence{0},epoch{0};
    bool valid{false},flight_safe{false},allocation_ok{false};
    float speed{INFINITY},angular_speed{INFINITY},tilt{INFINITY},position_error{INFINITY},motor_max{INFINITY};
    float alignment_error{INFINITY},relative_speed{INFINITY},orientation_error{INFINITY};
};
class Gate {
public:
    void update(const Input &v) {
        start_ready=close_geometry_ready=false; denial=0;
        if (!v.task) { denial=Missing; since=0; return; }
        if (task!=v.task) {
            task=v.task; epoch=v.epoch; reset_key=v.reset_key;
            seen=false; blocked=false; since=0; last_now=0;
        }
        if ((last_now && v.now<last_now) || epoch!=v.epoch || reset_key!=v.reset_key) blocked=true;
        last_now=v.now;
        if (blocked) { denial=Reset; since=0; return; }
        if (!v.received || !v.sample || v.now<v.received || v.now-v.received>300000 ||
            v.sample>v.now+20000 || (v.sample<v.now && v.now-v.sample>100000)) denial|=Stale;
        const float values[]{v.speed,v.angular_speed,v.tilt,v.position_error,v.motor_max,
            v.alignment_error,v.relative_speed,v.orientation_error};
        for (float x:values) if (!std::isfinite(x) || x<0) denial|=Invalid;
        if (!v.valid) denial|=Invalid;
        if (!denial && seen && ((v.sequence==sequence && v.sample!=last_sample) ||
            (v.sequence!=sequence && (int32_t(v.sequence-sequence)<=0 || v.sample<=last_sample)))) {
            blocked=true; denial|=Reset;
        }
        if (denial) { since=0; return; }
        if (!seen || sequence!=v.sequence) { seen=true; sequence=v.sequence; last_sample=v.sample; }
        if (!v.flight_safe) denial|=Flight;
        if (!v.allocation_ok || v.motor_max>=.95f) denial|=Allocation;
        if (v.position_error>.03f) denial|=Position;
        if (v.speed>.05f) denial|=Speed;
        if (v.angular_speed>.10f) denial|=Angular;
        if (v.tilt>.0872665f) denial|=Tilt;
        if (denial) { since=0; return; }
        if (!since) since=v.now;
        if (v.now-since<2000000) { denial=Dwelling; return; }
        start_ready=true;
        close_geometry_ready=v.alignment_error<=.003f && v.relative_speed<=.005f &&
            v.orientation_error<=.0872665f;
    }
    bool start_ready{false},close_geometry_ready{false},blocked{false};
    uint16_t denial{Missing}; uint64_t task{0};
private:
    bool seen{false}; uint32_t epoch{0},sequence{0};
    uint64_t reset_key{0},since{0},last_now{0},last_sample{0};
};
// PREPARE is a revocable observation ticket, never an actuator authorization.
class Preparation {
public:
    void update(uint64_t task,bool eligible) {
        if (prepared_task && (!eligible || task!=prepared_task)) {
            prepared_task=0; result=3;
        }
    }
    void request(uint64_t id,uint64_t requested,uint64_t task,bool eligible,bool request_valid) {
        if (!id || id<=last_request) return;
        last_request=id;
        const bool accepted=request_valid && eligible && task && requested==task;
        prepared_task=accepted ? task : 0; result=accepted ? 1 : 2;
    }
    uint64_t prepared_task{0},last_request{0};
    uint8_t result{0}; // 0 none, 1 accepted, 2 rejected, 3 revoked
};
}
