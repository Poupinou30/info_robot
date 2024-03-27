#ifndef HEADERS
#include "headers.h"
#define HEADERS
#endif
double speed_FL = 0;
double speed_FR = 0;
double speed_RL = 0;
double speed_RR = 0;
double *wheelSpeeds;
double v_x_r = 0;
double v_y_r = 0;
double v_x_f = 0;
double v_y_f = 0;
double omega_r = 0;

struct timeval t1, t2;
double timeDelta;



void myOdometry(uint8_t* buffer_rear, uint8_t* buffer_front){
    double wheelSpeeds[4] = {speed_FL, speed_FR, speed_RL, speed_RR};
    if (t1.tv_sec == 0 && t1.tv_usec == 0) {
        gettimeofday(&t1, NULL);
    }
    else{
    gettimeofday(&t2, NULL);
    timeDelta = (t2.tv_sec - t1.tv_sec) * 1000.0 + (t2.tv_usec - t1.tv_usec)/1000;      // sec to ms

    retrieveSpeeds(buffer_front, &speed_FL, &speed_FR);
    retrieveSpeeds(buffer_rear, &speed_RL, &speed_RR);
    computeSpeedFromOdometry(wheelSpeeds,&v_x_r,&v_y_r,&omega_r);

    v_x_f = v_x_r*cos(degToRad(*myOdometryPos.theta+90)) - v_y_r*sin(degToRad(*myOdometryPos.theta+90));
    v_y_f = v_x_r*sin(degToRad(*myOdometryPos.theta+90)) + v_y_r*cos(degToRad(*myOdometryPos.theta+90));
    *myOdometryPos.x += v_x_f*timeDelta/1000;
    *myOdometryPos.y += v_y_f*timeDelta/1000;
    *myOdometryPos.theta += omega_r*timeDelta/1000;}
}



void resetOdometry(){
    myOdometryPos.x = myFilteredPos.x;
    myOdometryPos.y = myFilteredPos.y;
    myOdometryPos.theta = myFilteredPos.theta;
}