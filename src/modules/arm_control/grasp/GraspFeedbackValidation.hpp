#pragma once
#include <cmath>
#include <cstdint>

namespace so101_grasp {
// Accept only genuinely new source windows. Forwarding an old packet must not
// renew freshness. Invalid packets never consume sequence numbers.
class FeedbackValidation {
public:
    template<typename T> bool accept(const T &v)
    {
        if (v.source != 1 || v.profile_id != 1 || v.target_id == 0
            || (v.capabilities & ~31u) != 0 || v.sample_time_us == 0
            || !std::isfinite(v.sample_window_s) || v.sample_window_s < 0.05f
            || v.sample_window_s > 0.2f || v.contact_frames > 100000) { return false; }
        const float fractions[]{v.fixed_fraction, v.moving_fraction, v.bilateral_fraction,
            v.support_fraction, v.other_fraction, v.support_arm_fraction};
        if (v.capabilities & 1u) {
            if (!v.contact_frames) { return false; }
            for (float f : fractions) { if (!std::isfinite(f) || f < 0.f || f > 1.f) { return false; } }
            if (v.bilateral_fraction > v.fixed_fraction || v.bilateral_fraction > v.moving_fraction) { return false; }
        }
        if ((v.capabilities & 2u) && (!std::isfinite(v.fixed_normal_n) || !std::isfinite(v.moving_normal_n)
            || !std::isfinite(v.vertical_force_n) || v.fixed_normal_n < 0.f || v.moving_normal_n < 0.f)) { return false; }
        if (v.capabilities & 4u) {
            if (!std::isfinite(v.object_height_world)) { return false; }
            for (int i=0; i<3; ++i) { if (!std::isfinite(v.object_center_gripper[i])) { return false; } }
        }
        if ((v.capabilities & 8u) && !std::isfinite(v.body_support_clearance)) { return false; }
        if (seen) {
            if (v.world_epoch == epoch) {
                if (int32_t(v.source_sequence - sequence) <= 0 || v.sample_time_us <= stamp) { return false; }
            } else if (int32_t(v.world_epoch - epoch) <= 0) { return false; }
        }
        seen = true; epoch = v.world_epoch; sequence = v.source_sequence; stamp = v.sample_time_us;
        return true;
    }
private:
    bool seen{false};
    uint32_t epoch{0}, sequence{0};
    uint64_t stamp{0};
};
}
