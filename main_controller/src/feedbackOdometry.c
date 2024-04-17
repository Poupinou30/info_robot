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
    printf("a ");
    double wheelSpeeds[4] = {motorSpeed_FL, motorSpeed_FR, motorSpeed_RL,motorSpeed_RR};
    if (t1.tv_sec == 0 && t1.tv_usec == 0) {
        gettimeofday(&t1, NULL);
        printf("time start in odometry = %lf \n",t1.tv_sec*1000000.0 + t1.tv_usec);
        printf("b1 ");
    }
    
    else{
        printf("b2 ");
    gettimeofday(&t2, NULL);
    timeDelta = (t2.tv_sec - t1.tv_sec) * 1000000.0 + (t2.tv_usec - t1.tv_usec);      // sec to ms
    totalTime+=timeDelta;
    printf("c ");
    gettimeofday(&t1, NULL);
    //printf("timeDelta: %lf\n",timeDelta/1000);
    printf("d ");
    computeSpeedFromOdometry(wheelSpeeds,&v_x_r,&v_y_r,&omega_r);
printf("e ");
    fprintf(stderr,"locking 33\n"); pthread_mutex_lock(&lockFilteredPosition);
    v_x_f = v_x_r*cos(degToRad(*myFilteredPos.theta)) - v_y_r*sin(degToRad(*myFilteredPos.theta));
    v_y_f = v_x_r*sin(degToRad(*myFilteredPos.theta)) + v_y_r*cos(degToRad(*myFilteredPos.theta));
    fprintf(stderr,"unlocking 33\n"); pthread_mutex_unlock(&lockFilteredPosition);
    printf("f ");
    //printf("vxf = %f vyf = %f vxr = %f vyf = %f theta = %f\n",v_x_f,v_y_f,v_x_r,v_y_r,*myFilteredPos.theta);
    *myOdometryPos.x += v_x_f*timeDelta/1000000;
    *myOdometryPos.y += v_y_f*timeDelta/1000000;
    *myOdometryPos.theta += 1.055 *omega_r*timeDelta/1000000;
    printf("g ");
    //printf("omega in odometry = %f \n")
    if(*myOdometryPos.theta > 360) *myOdometryPos.theta+=-360;
    if(*myOdometryPos.theta < 0) *myOdometryPos.theta+=360;
    printf("h ");
    //*myOdometryPos.theta = *myPos.theta;
    }
}



void resetOdometry(){
    //printf("odometry resetted \n");
    fprintf(stderr,"locking 2\n"); pthread_mutex_lock(&lockFilteredPosition);
    fprintf(stderr,"locking 3\n"); pthread_mutex_lock(&lockPosition);
    *myOdometryPos.x = (0*(*myFilteredPos.x)+*myPos.x*4)/4;
    *myOdometryPos.y = (0*(*myFilteredPos.y)+*myPos.y*4)/4;
    /*if(lidarAcquisitionFlag)*/ *myOdometryPos.theta = (0*(*myFilteredPos.theta)+*myPos.theta);
    /*else *myOdometryPos.theta = (3*(*myFilteredPos.theta)+*myPos.theta)/4;*/
    fprintf(stderr,"unlocking 2\n");pthread_mutex_unlock(&lockFilteredPosition);
    fprintf(stderr,"unlocking 3\n");pthread_mutex_unlock(&lockPosition);
}