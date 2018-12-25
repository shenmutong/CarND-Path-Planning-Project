#include "behavior.h"

Behavior::Behavior(){
    this->_speed_limit = 49.5;
    this->_acc = 0.244;
    return;
}
void Behavior::set_speed_limit(double value){
    this->_speed_limit = value;    
}
void Behavior::set_acc(double value){
    this->_acc = value;
}
void Behavior::get_behavior(
    LaneStatus status,
    int &lane,
    double & ref_vel){
        if(!(status.is_font_safe)){ 
            // there is a car in the Sront 
            //ref_vel -= _acc;
            if(status.is_right_safe){
                if(lane < 2){
                    lane++;                
                }
                
            }else if(status.is_left_safe){
                if(lane >0){
                    lane--;
                }            }
            else
            {
                ref_vel -= _acc;
            }            

        }else{
            if(ref_vel <= _speed_limit- _acc ){
                ref_vel += _acc;
            }
            if(lane != 1){
               if(lane < 1 && status.is_right_safe){
                   lane = 1;
               }else if(lane > 1 && status.is_left_safe){
                   lane = 1;
               }
            }            
        }

        

}