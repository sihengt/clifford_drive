#include "clifford_drive/throttle_controller.hpp"

ThrottleController::ThrottleController(const float max_change_rate) 
    : max_change_rate_(max_change_rate), current_throttle_(0.0) {}

float ThrottleController::setThrottle(float desired_throttle)
{
    float scaled_throttle = desired_throttle * throttle_scale_;
    scaled_throttle = std::clamp(scaled_throttle, -0.5f, 0.5f);
    float delta = scaled_throttle - current_throttle_;

    // Limit the change rate
    if (std::abs(delta) > max_change_rate_) {
        if (delta > 0) {
            scaled_throttle = currentThrottle + max_change_rate_;
        } else {
            scaled_throttle = currentThrottle - max_change_rate_;
        }
    }

    current_throttle_ = scaled_throttle; 
    return currentThrottle;   
}