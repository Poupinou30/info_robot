//
// Created by adepe on 01-03-24.
//
#ifndef HEADERS
#include "headers.h"
#define HEADERS
#endif
int sizeX = 200; //taille de l'arène en cm
int sizeY = 300; //taille de l'arène en cm
double f_tot_x = 0;
double f_tot_y = 0;
double f_theta = 0;
int refreshCounter = 0;
uint8_t readyToGo = 0;
pthread_t computeKalmanThread;
pthread_mutex_t lockFilteredPosition;
pthread_mutex_t lockPosition;
pthread_mutex_t lockOpponentPosition;
pthread_mutex_t lockDestination;
uint8_t destination_set = 0;
uint8_t pressForward = 0;
int UART_handle;
position myOdometryPos;
uint8_t waitingForReception = 0;
double motorSpeed_FL = 0;
double motorSpeed_FR = 0;
double motorSpeed_RL = 0;
double motorSpeed_RR = 0;
movingState myControllerState = STOPPED;

double measuredSpeedX = 0;
double measuredSpeedY = 0;
double measuredSpeedOmega = 0;

uint8_t startingPoint = 5;

uint8_t changeOfPlan = 1;
uint8_t forksCalibrated = 0;

//Nextion
int UART_handle_nextion;
uint8_t nextionStart = 0;
char myTeam[10] = "";
char myPage[25] = "";
int go = 2;
bool finish = false;
uint8_t score = 0;
uint8_t cameraEnable = 0;
Queue* q; 
struct timeval startInitialization, endQueue;
char receivedChars[NUM_CHARS];