#ifndef PREDICT
#define PREDICT
#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
typedef struct{
    bool is_font_safe = false;
    bool is_left_safe = false;
    bool is_right_safe = false;
}LaneStatus;

class Prediction{
    public:       
        Prediction();
        //TODO
        //LANE MAX AND MIN Limit

         
        void set_secure_distance(int value);

        void set_lane_width(int value);

        LaneStatus get_Lane_Status(
            float sensor_d,
            double sensor_s,
            double sensor_vx,
            double sensor_vy,
            int cur_lane, //current car's s 
            int prev_size, // old data size
            double cur_s); //current car's d
    private:
        int _secure_distance = 30;
        int _lane_width = 4;
};

#endif