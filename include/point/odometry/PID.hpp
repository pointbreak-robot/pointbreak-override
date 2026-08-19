#ifndef PID_HPP
#define PID_HPP

#pragma once

namespace point {

class PID {
    public:
        PID(float kp, float ki, float kd, float starti);
        float compute(float error);
        void reset();
    private:
        float kp = 0;
        float ki = 0;
        float kd  = 0;
        float starti = 0;

        float integral = 0;
        float prevError = 0;
};

} // namespace point

#endif