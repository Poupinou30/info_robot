#ifndef HEADERS
#include "headers.h"
#define HEADERS
#endif
int speed_FL = 0;
int speed_FR = 0;
int speed_RL = 0;
int speed_RR = 0;



void myOdometry(uint8_t buffer_rear, uint8_t* buffer_front){
    retrieveSpeeds(buffer_front, &speed_FL, &speed_FR);
    retrieveSpeeds(buffer_rear, &speed_RL, &speed_RR);
    float delta_t = 0.1;
    float x = *myFilteredPos.x;
    float y = *myFilteredPos.y;
    float theta = *myFilteredPos.theta;
    float v_x = speed[0];
    float v_y = speed[1];
    float omega = speed[2];
    float x_new = x + v_x * cos(theta) * delta_t - v_y * sin(theta) * delta_t;
    float y_new = y + v_x * sin(theta) * delta_t + v_y * cos(theta) * delta_t;
    float theta_new = theta + omega * delta_t;
    /*output[0] = x_new;
    output[1] = y_new;
    output[2] = theta_new;*/
}

void wheelSpeedToRobotSpeed(){
    
}

void resetOdometry(){
    myOdometryPos.x = myFilteredPos.x;
    myOdometryPos.y = myFilteredPos.y;
    myOdometryPos.theta = myFilteredPos.theta;
}