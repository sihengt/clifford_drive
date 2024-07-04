#ifndef THROTTLE_CONTROLLER_H
#define THROTTLE_CONTROLLER_H

#include <algorithm>

class ThrottleController
{
public:
    ThrottleController();
    float setThrottle(float desired_throttle);
    void setParams(float max_change_rate, float throttle_scale);
private:
    float max_change_rate_ = 0.0f;
    float current_throttle_ = 0.0f;
    float throttle_scale_ = 0.0f;
    std::pair<float, float> throttle_limit_ = std::make_pair(-12000.0f, 12000.0f);
};

#endif //THROTTLE_CONTROLLER_H