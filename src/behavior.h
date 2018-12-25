#ifndef BEHAVIOR
#define BEHAVIOR
#include <math.h>
#include "prediction.h"

class Behavior{
    public:       
        Behavior();

        void set_speed_limit(double value);

        void set_acc(double value);

        void get_behavior(
            LaneStatus status,
            int &lane,
            double &ref_vel
        );
        
    private:
        double _acc = 0.20;
        double _speed_limit  = 50;
};

#endif