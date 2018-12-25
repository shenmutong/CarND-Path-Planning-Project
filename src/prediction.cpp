#include "prediction.h"
using namespace std;
Prediction::Prediction(){

}

void Prediction::set_secure_distance(int value){
    this->_secure_distance = value;
}
void Prediction::set_lane_width(int value){
    this->_lane_width = value;
}

LaneStatus Prediction::get_Lane_Status(
    float sensor_d,
    double sensor_s,
    double sensor_vx,
    double sensor_vy,
    int cur_lane,
    int prev_size,
    double cur_s){

    LaneStatus rst;
    rst.is_font_safe = true;
    rst.is_left_safe = true;
    rst.is_right_safe = true;
    float car_d = sensor_d;
    float d = sensor_d;
    double car_s = sensor_s;
    int car_lane = (int)((int)car_d % _lane_width);
    
    if ( d > 0 && d < 4 ) { 
        car_lane = 0;
    } else if ( d > 4 && d < 8 ) {
        car_lane = 1;
    } else if ( d > 8 && d < 12 ) {
        car_lane = 2;
    } 
    
    if(car_lane <0){
        return rst;
    }

    double speed = sqrt(
        sensor_vx*sensor_vx 
        +
        sensor_vy* sensor_vy);
    //get the car actual s
    car_s += ((double)prev_size * 0.02 * speed);

    //check is there close car in the lane
    bool is_car_close = (abs(cur_s - car_s) <= this->_secure_distance);
    #if 0
    if(is_car_close){
        std::cout << "IS CAR CLOSE" << std::endl;
    }
    #endif
    if(cur_lane == car_lane ){  
         
        std::cout << "cur_lane == car_lane " << std::endl;
        //std::cout << "cur_s " << cur_s << std::endl;         
        if(is_car_close && car_s > cur_s){
            rst.is_font_safe = false;
            std::cout << "FONT UNSAFE" << std::endl;
            std::cout << "car_s " << car_s << std::endl;
            std::cout << "cur_s " << cur_s << std::endl;
            std::cout << " cur_lane" << cur_lane << std::endl;
        std::cout << "car_lane" << car_lane << std::endl; 
                     
        }            
    }else if(cur_lane -1 == car_lane ){
        //left
        rst.is_left_safe = !is_car_close;    
         std::cout << " cur_lane" << cur_lane << std::endl;
        std::cout << "car_lane" << car_lane << std::endl;     
    }else if(cur_lane +1 == car_lane){
        //right
        rst.is_right_safe = !is_car_close;  
         std::cout << " cur_lane" << cur_lane << std::endl;
        std::cout << "car_lane" << car_lane << std::endl;                  
    }
    //check road sides
    if(cur_lane <= 0){
        rst.is_left_safe = false;
    }
    if(cur_lane >= 2){
        rst.is_right_safe = false;
    } 
    
    return rst;
}