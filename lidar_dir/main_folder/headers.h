#include <time.h>
#include <stdio.h>
#include <vector>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include "sl_lidar.h"
#include "sl_lidar_driver.h"
#include "lidar.h"
#include <rplidar.h>
#include <iostream>
#include <fstream>
#define adjust_value_to_bounds( value , max )  ( ( value > max ) ? max : ( ( value < -max ) ? -max : value ) )  
#define PI          3.141592654
#define cot(x)      ( 1 / tan(x) )
#define RAD2DEG     (180/PI)
#define DEG2RAD     (PI/180)
#define COT_MAX     100000000
#define Cot(x)  cot(x)
typedef struct beaconAbsolutePos{
    float x;
    float y;
} beaconAbsolutePos;



typedef struct lidarPos{
    float x;
    float y;
    float theta;
} lidarPos;


typedef struct {
    double distance;
    float angle;
    float width;
} Beacon;

typedef struct {
    float *distance;
    float *angle;
    int counter;
} lidar_data;

typedef struct {
    uint8_t isDetected;
    float x;
    float y;
} Opponent;

double distance (double a1,double a2,double d1,double d2);
float angleDiff(float a1, float a2);
float triangulationPierlot(float *x, float *y,
						float alpha1, float alpha2, float alpha3,
						float x1, float y1, float x2, float y2, float x3, float y3);

void* beacon_data(void* argument);
void generateLog();
void writeLog(float angle, float distance, float distanceFromCenter, float debugBalises[3][2], float debugPerimetre, float debugIsoceleCondition);
void* filteredPositionAcquisition(void* arg);