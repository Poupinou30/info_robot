#ifndef HEADERS
#include "headers.h"
#define HEADERS
#endif

double *wheelSpeeds;
double v_x_r = 0;
double v_y_r = 0;
double v_x_f = 0;
double v_y_f = 0;
double omega_r = 0;

struct timeval t1, t2;
double timeDelta;



void myOdometry(){
    double wheelSpeeds[4] = {motorSpeed_FL, motorSpeed_FR, motorSpeed_RL,motorSpeed_RR};
    if (t1.tv_sec == 0 && t1.tv_usec == 0) {
        gettimeofday(&t1, NULL);
    }
    else{
    gettimeofday(&t2, NULL);
    timeDelta = (t2.tv_sec - t1.tv_sec) * 1000.0 + (t2.tv_usec - t1.tv_usec)/1000;      // sec to ms
    gettimeofday(&t1, NULL);
    //printf("timeDelta: %f\n",timeDelta/1000);
    computeSpeedFromOdometry(wheelSpeeds,&v_x_r,&v_y_r,&omega_r);

    v_x_f = v_x_r*cos(degToRad(*myFilteredPos.theta)) - v_y_r*sin(degToRad(*myFilteredPos.theta));
    v_y_f = v_x_r*sin(degToRad(*myFilteredPos.theta)) + v_y_r*cos(degToRad(*myFilteredPos.theta));
    //printf("vxf = %f vyf = %f vxr = %f vyf = %f theta = %f\n",v_x_f,v_y_f,v_x_r,v_y_r,*myFilteredPos.theta);
    *myOdometryPos.x += v_x_f*timeDelta/1000;
    *myOdometryPos.y += v_y_f*timeDelta/1000;
    *myOdometryPos.theta += omega_r*timeDelta/1000;}
}



void resetOdometry(){
    printf("odometry resetted \n");
    *myOdometryPos.x = *myFilteredPos.x;
    *myOdometryPos.y = *myFilteredPos.y;
    *myOdometryPos.theta = *myFilteredPos.theta;
}