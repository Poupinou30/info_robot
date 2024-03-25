#ifndef HEADERS
#include "headers.h"
#define HEADERS
#endif



void myOdometry(float* speed){
    float delta_t = 0.1;
    float x = myFilteredPos.x;
    float y = myFilteredPos.y;
    float theta = myFilteredPos.theta;
    float v_x = speed[0];
    float v_y = speed[1];
    float omega = speed[2];
    float x_new = x + v_x * cos(theta) * delta_t - v_y * sin(theta) * delta_t;
    float y_new = y + v_x * sin(theta) * delta_t + v_y * cos(theta) * delta_t;
    float theta_new = theta + omega * delta_t;
    output[0] = x_new;
    output[1] = y_new;
    output[2] = theta_new;
}

void resetOdometry(){
    myOdometryPos.x = myFilteredPos.x;
    myOdometryPos.y = myFilteredPos.y;
    myOdometryPos.theta = myFilteredPos.theta;
}