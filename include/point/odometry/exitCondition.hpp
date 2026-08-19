#ifndef EXITCONDITION_HPP
#define EXITCONDITION_HPP

#pragma once 

namespace point {

class exitCondition {
    public:
        exitCondition(const float range, const int time);
        bool getExit();
        bool update(const float input);
        void reset();
    private:
        const float range;
        const int time;
        int startTime = -1;
        bool done = false;
};

} // namespace point

#endif 

