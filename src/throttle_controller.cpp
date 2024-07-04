#include "clifford_drive/throttle_controller.hpp"

ThrottleController::ThrottleController() {}

void ThrottleController::setParams(float max_change_rate, float throttle_scale)
{
    max_change_rate_ = max_change_rate;
    throttle_scale_ = throttle_scale;
}

float ThrottleController::setThrottle(float desired_throttle)
{
    float scaled_throttle = desired_throttle * throttle_scale_;
    scaled_throttle = std::clamp(scaled_throttle, throttle_limit_.first, throttle_limit_.second);
    float delta = scaled_throttle - current_throttle_;

    // Limit the change rate
    if (std::abs(delta) > max_change_rate_) {
        if (delta > 0) {
            scaled_throttle = current_throttle_ + max_change_rate_;
        } else {
            scaled_throttle = current_throttle_ - max_change_rate_;
        }
    }

    current_throttle_ = scaled_throttle; 
    return current_throttle_;   
}