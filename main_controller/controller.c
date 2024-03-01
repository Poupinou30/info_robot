#ifndef HEADERS
#include "headers.h"
#define HEADERS
#endif

double Kp = 1;
double Ki = 0.00;
double Kd = 0.1;
double Kp_theta = 0.2;
double Ki_theta = 0.00;
double Kd_theta = 0.1;

long now = 0;

double prevErrorX = 0;
double prevErrorY = 0;
double prevErrorTheta = 0;

double errorX = 0;
double errorY = 0;
double errorTheta = 0;

double integralErrorX = 0;
double integralErrorY = 0;
double integralErrorTheta = 0;

double derivativeErrorX = 0;
double derivativeErrorY = 0;
double derivativeErrorTheta = 0;

double targetX = 0;
double targetY = 0;
double targetTheta = 0;
double previousTargetX = 0;
double previousTargetY = 0;
double previousTargetTheta = 0;


double speedX = 0;
double speedY = 0;
double speedTheta = 0;

float windUpLimitXY = 0.2;
float windUpLimitTheta = 0.2; //rad/sec

double speedNorm;

uint16_t counterVerbose = 0;



void myController(double* speedTab, double currentX, double currentY, double currentTheta, double* absoluteSpeedTab, double* targetPosition){
    targetX = targetPosition[0];
    targetY = targetPosition[1];
    targetTheta = targetPosition[2];

    errorX = -(currentX - targetX);
    errorY = -(currentY - targetY);
    errorTheta = -(currentTheta - targetTheta);

    derivativeErrorX = (errorX - prevErrorX -(targetX - previousTargetX))*1000/timeDelay;
    derivativeErrorY = (errorY - prevErrorY - (targetY - previousTargetY))*1000/timeDelay;
    derivativeErrorTheta = (errorTheta - prevErrorTheta -(targetTheta - previousTargetTheta))*1000/timeDelay;

    //Anti WindUp
    if(fabs(speedX) < windUpLimitXY) integralErrorX = integralErrorX + errorX*timeDelay/1000;
    if(fabs(speedY) < windUpLimitXY) integralErrorY = integralErrorY + errorY*timeDelay/1000;
    if(fabs(speedTheta) < windUpLimitTheta) integralErrorTheta = integralErrorTheta + errorTheta * timeDelay/1000;

    speedX = (Kp*errorX + Ki*integralErrorX + Kd*derivativeErrorX);
    speedY = (Kp*errorY + Ki*integralErrorY + Kd*derivativeErrorY);
    speedTheta = (Kp*errorTheta + Ki*integralErrorTheta + Kd*derivativeErrorTheta);

    absoluteSpeedTab[0] = speedX;
    absoluteSpeedTab[1] = speedY;
    absoluteSpeedTab[2] = speedTheta;

    //Limiters
    if(speedX < 0) speedX = fmax(-windUpLimitXY,speedX);
    else speedX = fmin(windUpLimitXY, speedX);
    if(speedY < 0) speedY = fmax(-windUpLimitXY,speedY);
    else speedY = fmin(windUpLimitXY, speedY);
    if(speedTheta < 0) speedTheta = fmax(-windUpLimitTheta,speedTheta);
    else speedTheta = fmin(windUpLimitTheta, speedTheta);

    //Norm speed Limiters
    speedNorm = pow(pow(speedX,2)+pow(speedY,2),1/2);
    if(speedNorm > windUpLimitXY){
        speedX = speedX/speedNorm;
        speedY = speedY/speedNorm;
    }
    convertsVelocity(speedX,speedY,speedTheta,speedTab);


    prevErrorX = errorX;
    prevErrorY = errorY;
    prevErrorTheta = errorTheta;
    previousTargetX = targetX;
    previousTargetY = targetY;

    counterVerbose++;

    if(VERBOSE && counterVerbose == 10){
        
        fprintf(stderr,"speedX = %f \n",speedX);
        fprintf(stderr,"speedY = %f \n",speedY);
        fprintf(stderr,"Omega = %f \n",s
        peedTheta);
        fprintf(stderr,"x-position = %lf and targetX = %lf \n",currentX,targetX);
        fprintf(stderr,"y-position = %lf and targetY = %lf \n",currentY,targetY);
        //fprintf(stderr,"target theta = %f \n", targetTheta);
        //fprintf(stderr,"currentTheta = %f \n", currentTheta);
        counterVerbose = 0;
        }

        

    }