#ifndef THROTTLE_CONTROLLER_H
#define THROTTLE_CONTROLLER_H

#include <algorithm>

class ThrottleController
{
public:
    ThrottleController(const float max_change_rate);
    float setThrottle(float desired_throttle);
private:
    float max_change_rate_;
    float current_throttle_;
    const float throttle_scale_;
};

#endif //THROTTLE_CONTROLLER_H