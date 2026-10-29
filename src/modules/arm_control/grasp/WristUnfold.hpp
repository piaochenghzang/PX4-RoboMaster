#pragma once
#include <cmath>

namespace so101_hover {
enum class UnfoldResult { Settled, Move, Wait, Invalid };

// Only the calibrated folded startup selects this branch automatically.
// Arbitrary/already-unfolded targets retain the existing Cartesian approach.
inline bool knownFoldedArm(const float (&anchor)[5],const float (&measured)[5])
{
    constexpr float folded[5]{-.07f,-1.175f,1.45f,1.3f,-1.57f};
    for (unsigned j=0;j<5;++j) {
        if (!std::isfinite(anchor[j]) || !std::isfinite(measured[j]) ||
            std::abs(anchor[j]-folded[j])>=.001f || std::abs(measured[j]-folded[j])>=.03f) return false;
    }
    return true;
}

// Shared fixed-demo wrist unfold. The caller supplies timing, hover/lease,
// pending-motion clearance and workspace checks; this helper grants no motion.
inline UnfoldResult wristUnfoldStep(const float (&anchor)[5],const float (&measured)[5],
    float joint_speed,float max_step,float (&desired)[5],bool inward_recovery=false)
{
    float unfolded[5]{-.07f,-1.175f,1.45f,-1.3f,-1.57f};
    // The folded wrist arc initially dips by <1um when a clearance budget
    // makes its first step smaller than .01rad. A 1mrad elbow co-motion
    // keeps recovery strictly inward. The caller restores the exact original
    // elbow after the TCP is safely inside the workspace.
    if (inward_recovery) unfolded[2]-=.001f;
    if (!std::isfinite(joint_speed) || joint_speed<0 || !std::isfinite(max_step) ||
        max_step<=0 || max_step>.02f) return UnfoldResult::Invalid;
    float command_error=0,feedback_error=0;
    for (unsigned j=0;j<5;++j) {
        if (!std::isfinite(anchor[j]) || !std::isfinite(measured[j])) return UnfoldResult::Invalid;
        const float command=std::abs(unfolded[j]-anchor[j]);
        const float feedback=std::abs(unfolded[j]-measured[j]);
        if (command>command_error) command_error=command;
        if (feedback>feedback_error) feedback_error=feedback;
    }
    if (command_error<.001f && feedback_error<.03f && joint_speed<.05f) return UnfoldResult::Settled;
    if (command_error<.001f) return UnfoldResult::Wait;
    for (unsigned j=0;j<5;++j) {
        const float delta=unfolded[j]-anchor[j];
        desired[j]=anchor[j]+(delta < -max_step ? -max_step : (delta > max_step ? max_step : delta));
    }
    return UnfoldResult::Move;
}
}
