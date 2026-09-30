// Synthetic inputs verify logic only; these tests never prove physical grasping.
#include "../../grasp/GraspStateMachine.hpp"
#include "../../grasp/GraspFeedbackValidation.hpp"
#include <cassert>
#include <cstdio>
#include <limits>
using namespace so101_grasp;
Input healthy()
{
    Input i{}; i.now=2000000; i.initialized=i.finite=true; i.capabilities=31;
    i.profile=1; i.target_id=1; i.source_identity=456; i.frames=100;
    i.sequence=1; i.sample_stamp=i.now; i.joint_age=i.grasp_age=0;
    i.position_error=i.joint_error=i.joint_speed=i.jaw_speed=0;
    i.accepted_target={.12f,.015f,.03f}; i.object_local={.0025f,-.000218f,-.088f};
    i.object_height=1.f; i.support=1.f; i.bilateral=1.f; i.fixed_force=i.moving_force=.15f; i.clearance=.009f;
    return i;
}
void tick(GraspStateMachine &sm, Input &i)
{
    i.now+=100000; ++i.sequence; i.sample_stamp=i.now;
    if (sm.phase>=Phase::Lift && sm.phase<=Phase::Done) { i.support=0; i.object_height=1.035f; }
    if (sm.phase==Phase::ExitSupport && i.accepted_target.y>.10f) i.support=0;
    const auto action=sm.step(i);
    if (action.type==ActionType::Target) { i.accepted_target=action.target; ++i.target_sequence; sm.acknowledge(true); }
}
void advance(GraspStateMachine &sm, Input &i, Phase wanted)
{
    for (unsigned j=0; j<2000 && sm.phase!=wanted && sm.phase!=Phase::Fault; ++j) tick(sm,i);
    assert(sm.phase==wanted);
}
struct Wire {
    uint8_t source{1}; uint16_t profile_id{1},capabilities{31}; uint32_t target_id{1},source_sequence{1},world_epoch{0},contact_frames{100};
    uint64_t sample_time_us{100000}; float sample_window_s{.1f};
    float fixed_fraction{1},moving_fraction{1},bilateral_fraction{1},support_fraction{0},other_fraction{0},support_arm_fraction{0};
    float fixed_normal_n{.15f},moving_normal_n{.15f},vertical_force_n{.098f};
    float object_center_gripper[3]{0,0,0},object_height_world{1},body_support_clearance{.01f};
};
int main()
{
    unsigned cases=0;
    {
        const Point low{.05f,-.25f,.02f},high{.4f,.25f,.4f},fold{.106f,.012f,-.005f};
        assert(monotonicRecovery(fold,{.118f,.016f,.009f},low,high,.02f));
        assert(!monotonicRecovery(fold,{.106f,.012f,-.006f},low,high,.02f));
        assert(!monotonicRecovery(fold,{.17f,.035f,.05f},low,high,.02f));
        assert(!monotonicRecovery({.12f,0,.03f},{.12f,0,.01f},low,high,.02f));
        ++cases;
    }
    {
        GraspStateMachine s; auto i=healthy(); assert(s.request(1,1,1,1,5,i)==RequestResult::Accepted);
        advance(s,i,Phase::Done); assert(s.result==Result::Succeeded && s.completed_success && s.payload==Payload::Held);
        assert(s.owns()); assert(s.request(2,1,1,1,5,i)==RequestResult::Busy);
        assert(s.request(3,2,1,1,5,i)==RequestResult::Accepted); assert(s.step(i).type==ActionType::Stop);
        assert(s.request(4,3,1,1,5,i)==RequestResult::PayloadLocked); ++cases;
    }
    {
        GraspStateMachine s; auto i=healthy(); s.request(1,1,1,1,5,i);
        advance(s,i,Phase::Approach); s.request(2,2,1,1,5,i);
        assert(s.result==Result::Canceled && s.step(i).type==ActionType::Stop);
        assert(s.request(3,3,1,1,5,i)==RequestResult::Accepted && !s.owns());
        assert(s.request(1,1,1,1,5,i)==RequestResult::Invalid); ++cases;
    }
    {
        GraspStateMachine s; auto i=healthy(); s.request(1,1,1,1,5,i); i.bilateral=0;
        for (unsigned j=0;j<2000 && s.phase!=Phase::Fault;++j) tick(s,i);
        assert(s.fault==Fault::Unconfirmed && !s.completed_success); ++cases;
    }
    for (unsigned fault_case=0;fault_case<7;++fault_case) {
        GraspStateMachine s; auto i=healthy(); s.request(1,1,1,1,5,i); advance(s,i,Phase::Lift);
        const Fault expected[]{Fault::JointStale,Fault::GraspStale,Fault::SourceChanged,Fault::Nonfinite,
            Fault::Separated,Fault::Obstructed,Fault::ClockReset};
        if (fault_case==0) i.joint_age=.4f;
        if (fault_case==1) i.grasp_age=.4f;
        if (fault_case==2) ++i.epoch;
        if (fault_case==3) i.finite=false;
        if (fault_case==4) i.object_local.x+=.05f;
        if (fault_case==5) i.other=.3f;
        if (fault_case==6) i.now=1;
        assert(s.step(i).type==ActionType::Stop && s.fault==expected[fault_case]);
        assert(s.phase==Phase::Fault && s.result==Result::Failed && !s.completed_success); ++cases;
    }
    {
        GraspStateMachine s; auto i=healthy(); s.request(1,1,1,1,5,i);
        advance(s,i,Phase::Hold); i.bilateral=0;
        for (unsigned j=0;j<8;++j) tick(s,i);
        assert(s.fault==Fault::ContactLost); ++cases;
    }
    {
        GraspStateMachine s; auto i=healthy(); s.request(1,1,1,1,5,i); advance(s,i,Phase::Done);
        i.object_local.x+=.05f; s.step(i);
        assert(s.fault==Fault::Separated && s.completed_success && s.result==Result::Failed); ++cases;
    }
    {
        GraspStateMachine s; auto i=healthy(); s.request(1,1,1,1,5,i); s.acknowledge(false);
        assert(s.step(i).type==ActionType::Stop && s.fault==Fault::IKRejected); ++cases;
    }
    {
        FeedbackValidation v; Wire w; assert(v.accept(w)); assert(!v.accept(w));
        ++w.source_sequence; assert(!v.accept(w)); w.sample_time_us+=100000; assert(v.accept(w));
        ++w.source_sequence; w.sample_time_us+=100000; w.bilateral_fraction=1.1f; assert(!v.accept(w));
        w.bilateral_fraction=1.f; w.fixed_normal_n=std::numeric_limits<float>::quiet_NaN(); assert(!v.accept(w));
        w.capabilities=0; assert(v.accept(w)); // Explicit unknown force is not a physical NaN claim.
        ++w.world_epoch; w.source_sequence=1; w.sample_time_us=1; assert(v.accept(w));
        --w.world_epoch; assert(!v.accept(w)); ++cases;
    }
    {
        GraspStateMachine s; auto i=healthy(); s.request(1,1,1,1,5,i);
        for (unsigned j=0;j<100;++j) s.step(i);
        assert(s.phase==Phase::Precheck); ++cases; // Paused simulation time cannot complete dwell.
    }
    std::printf("PASS: %u grasp logic/feedback scenarios (synthetic, not physical grasp proof)\n",cases);
}
