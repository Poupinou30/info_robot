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
double totalTime = 0;



void myOdometry(){
    double wheelSpeeds[4] = {motorSpeed_FL, motorSpeed_FR, motorSpeed_RL,motorSpeed_RR};
    if (t1.tv_sec == 0 && t1.tv_usec == 0) {
        gettimeofday(&t1, NULL);
        printf("time start in odometry = %lf \n",t1.tv_sec*1000000.0 + t1.tv_usec);
    }
    else{
    gettimeofday(&t2, NULL);
    timeDelta = (t2.tv_sec - t1.tv_sec) * 1000000.0 + (t2.tv_usec - t1.tv_usec);      // sec to ms
    totalTime+=timeDelta;
    gettimeofday(&t1, NULL);
    //printf("timeDelta: %lf\n",timeDelta/1000);
    computeSpeedFromOdometry(wheelSpeeds,&v_x_r,&v_y_r,&omega_r);

    v_x_f = v_x_r*cos(degToRad(*myFilteredPos.theta)) - v_y_r*sin(degToRad(*myFilteredPos.theta));
    v_y_f = v_x_r*sin(degToRad(*myFilteredPos.theta)) + v_y_r*cos(degToRad(*myFilteredPos.theta));
    //printf("vxf = %f vyf = %f vxr = %f vyf = %f theta = %f\n",v_x_f,v_y_f,v_x_r,v_y_r,*myFilteredPos.theta);
    *myOdometryPos.x += v_x_f*timeDelta/1000000;
    *myOdometryPos.y += v_y_f*timeDelta/1000000;
    *myOdometryPos.theta += 1.015*omega_r*timeDelta/1000000;
    if(*myOdometryPos.theta > 360) *myOdometryPos.theta+=-360;
    if(*myOdometryPos.theta < 0) *myOdometryPos.theta+=360;
    //*myOdometryPos.theta = *myPos.theta;
    }
}



void resetOdometry(){
    //printf("odometry resetted \n");
    pthread_mutex_lock(&lockFilteredPosition);
    pthread_mutex_lock(&lockPosition);
    *myOdometryPos.x = (3*(*myFilteredPos.x)+*myPos.x)/4;
    *myOdometryPos.y = (3*(*myFilteredPos.y)+*myPos.y)/4;
    if(lidarAcquisitionFlag) *myOdometryPos.theta = (0*(*myFilteredPos.theta)+*myPos.theta);
    else *myOdometryPos.theta = (3*(*myFilteredPos.theta)+*myPos.theta)/4;
    pthread_mutex_unlock(&lockFilteredPosition);
    pthread_mutex_unlock(&lockPosition);
}