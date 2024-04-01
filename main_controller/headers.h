#include <stdio.h>
#include <pigpio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <sys/time.h>
#include <pthread.h>
#include <sys/wait.h>
#include <sys/types.h>
#define _Pi 3.1415927
#ifndef MAIN_CONTROLLER_HEADERS_H
#define MAIN_CONTROLLER_HEADERS_H

#endif //MAIN_CONTROLLER_HEADERS_H
#define VERBOSE 1
#define timeDelay 100 //ms


extern float* positionReceived;
extern pthread_mutex_t lockPosition;
extern pthread_mutex_t lockRefreshCounter;

typedef struct position{
    float *x;
    float *y;
    float *theta;
} position;

typedef struct obstacle{
    double posX;
    double posY;
    double size;
    uint8_t moving;
    uint8_t isRectangle;
    float x1,x2,y1,y2;
} obstacle;


typedef struct forceVector{
    int obstacleNumber;
    obstacle* obstacleList;
    int movingNumber;
    int* movingIndexes;
    double fx;
    double fy;
    double ftheta;
} forceVector;


typedef struct field{
    double** attractiveField;
    double** repulsiveField;
    double** totalField;
    double** attractiveForceX;
    double** attractiveForceY;
} field;

void createArray(int16_t num1, int16_t num2, uint8_t* output);
double degToRad(double deg);
int initializeSPI(int channel);
void test_pattern(int spi_handle);
void close_spi(int spi_handle);
void SPI_send(uint8_t send_data[4],int spi_handle, uint8_t *received_data);
void print_data(uint8_t *received_data);
void convertsVelocity(double v_x, double v_y, double omega, double* output_speed);
void myController(double* speedTab, double currentX, double currentY, double currentTheta,double* absoluteSpeedTab, double* targetPosition);
double randomDouble(double min, double max);
void processInstruction(float v_x, float v_y, float omega, double* speedTab, int spi_handle_rear,int spi_handle_front,uint8_t *dataFront, uint8_t *dataRear);
void processInstructionNew(float v_x, float v_y, float omega, int i2c_handle_front,int i2c_handle_rear);
void* executeProgram(void* arg);

void extractBytes(uint16_t nombre, uint8_t *octet_haut, uint8_t *octet_bas) ;
void tunePIDOLD(int spi_handle_front,int spi_handle_rear, uint16_t Kp_m, int8_t Kp_e,uint16_t Ki_m, int8_t Ki_e);
void tunePID(float Ki, float Kp, int i2c_handle_front, int i2c_handle_rear);
void* receptionPipe(void* pipefdvoid);
float computeEuclidianDistance(double x1, double y1, double x2, double y2);
float computeRectangleDistance(double x1, double y1, double x2, double y2, double x3, double y3);
void computeAttractiveField(position destination);
void computeInitialRepulsiveField();
void updateRepulsiveField(int x1,int y1, int x2, int y2);
void resetRepulsiveField(int x1,int y1, int x2, int y2);
void computeTotalField(uint8_t mode, int x1, int y1, int x2, int y2);
void print2DArray(int m, int n, double** arr);
void makeHeatmap();
void addRoundObstacle(double posX, double posY, double size, uint8_t moving);
void printObstacleLists();
void removeMovingObstacles();
void computeForceVector();
void myPotentialFieldController();
void* updateKalman(void* args);
position closestPoint(position rect[2], position pos);
void addRectangleObstacle(double x1, double y1, double x2, double y2, uint8_t moving);
void convertsSpeedToRobotFrame(double v_x, double v_y, double omega, double* output_speed);
void retrieveSpeeds(uint8_t* data, double* speed1, double* speed2);
void computeSpeedFromOdometry(double* wheel_speeds, double *v_x, double *v_y, double *omega);
void initializeMainController();

int initializeUART();
void resetOdometry();
void myOdometry();

//UART ET SPI et I2C
void UART_send(int UART_handle, char* data);
uint8_t UART_receive(int UART_handle, char* received);
extern int UARTReception;
extern uint8_t waitingForReception;
int I2C_initialize(int address);
void I2C_send(char* data,char* received, int I2C_handle);
int i2c_handle_front;
int i2c_handle_rear;
//FIN COMMUNICATION

//FORKS - ACTUATORS
int setArmDeployedAngle(int angle);
int setForksDeployedAngle(int angle);
int disableStepperMotor();
int enableStepperMotor();
int setLowerFork(int height);
int setUpperFork(int height);
int retractForks();
int deployForks();
int deployArm();
int setWheelSpeed(int speed);
int setGripperPosition(int position);
int calibrateFork();
int checkCommandReceived(char* expected, char* buffer, int* commandReceivedFlag);
//END ACTUATORS


//STRATEGY
typedef enum {CALIB_FORK,GRAB_PLANTS_INIT, GRAB_PLANTS_MOVE, GRAB_PLANTS_END,UNSTACK_POTS_MOVE,UNSTACK_POT_TAKE,UNSTACK_POT_POSITIONING,UNSTACK_POT_DROP,GRAB_POTS_MOVE,LIFT_POTS,DROP_PLANTS, DROP_ALL, FINISHED } grabbingState;
void manageGrabbing();
extern grabbingState myGrabState;
typedef enum {SENDING_INSTRUCTION,WAITING_ACTUATORS} actuationState;
extern actuationState myActuatorsState;
//END STRATEGY


extern position myPos;
extern field myField;
extern pid_t child_pid;
extern position myFilteredPos;
extern position myOpponent;
extern int sizeX;
extern int sizeY;
extern forceVector myForce;
extern position destination;
extern double f_tot_x;
extern double f_tot_y;
extern int refreshCounter;
extern uint8_t readyToGo;
extern double f_theta;
extern int UART_handle;

extern pthread_t computeKalmanThread;
extern pthread_mutex_t lockFilteredPosition;
extern pthread_mutex_t lockPosition;
extern pthread_mutex_t lockOpponentPosition;
extern pthread_mutex_t lockDestination;
extern uint8_t destination_set;
extern position myOdometryPos;

//Moteurs - odometry
extern double motorSpeed_FL;
extern double motorSpeed_FR;
extern double motorSpeed_RL;
extern double motorSpeed_RR;

