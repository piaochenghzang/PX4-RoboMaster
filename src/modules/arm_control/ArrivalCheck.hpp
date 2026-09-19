#pragma once
#include <cmath>
#include <cstdint>

// Position-only arrival for the five arm joints; gripper contact is separate.
class ArrivalCheck {
public:
    void reset() { since = 0; arrived = false; }
    bool update(uint64_t now, bool fresh, float position_error, float joint_error, float speed) {
        if (!fresh || !std::isfinite(position_error) || !std::isfinite(joint_error)
            || !std::isfinite(speed) || position_error > 0.005f
            || joint_error > 0.03f || speed > 0.05f) {
            reset();
            return false;
        }
        if (since == 0 || now < since) { since = now; }
        arrived = now - since >= 500000;
        return arrived;
    }
    bool arrived{false};
private:
    uint64_t since{0};
};
