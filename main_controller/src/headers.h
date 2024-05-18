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
#include <fcntl.h>
#include <termios.h>
#include <stdbool.h>			  				

#define _Pi 3.1415927
#define DEG2RAD     (_Pi/180)
#define RAD2DEG     (180/_Pi)
#define POTWIDTH 0.0708
#ifndef MAIN_CONTROLLER_HEADERS_H
#define MAIN_CONTROLLER_HEADERS_H

#define robotLengthX 0.255
#define robotLengthYUndeployed 0.240
#define robotLengthYDeployed 0.340
#define baseSize 0.45
#define jardiniereLength 0.325


#endif //MAIN_CONTROLLER_HEADERS_H
#define VERBOSE 1
#define timeDelay 20 //ms
#define makeLog 1

#define matchDuration 90 //seconds 
extern float MAX_SPEED;
#define NUM_CHARS 100 // for the nextion									

extern float* positionReceived;
extern pthread_mutex_t lockPosition;
extern pthread_mutex_t lockRefreshCounter;


// PlayingField
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
    int obstacleID;
    uint8_t obstacleEnabled;
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

typedef struct plantZone{
    int numberOfPlants;
    int zoneID;
    float posX;
    float posY;
    float targetPositionLowX;
    float targetPositionLowY;
    float targetPositionUpX;
    float targetPositionUpY;
    int obstacleID;
} plantZone;

typedef struct potZone{
    int numberOfPots;
    int zoneID;
    float posX;
    float posY;
    float pos5X;
    float pos5Y;
    float pos6X;
    float pos6Y;
    int obstacleID;
} potZone;

typedef struct jardiniere{
    int numberOfPlants;
    int zoneID;
    float posX;
    float posY;
    int obstacleID;
    int potZoneID;
} jardiniere;

typedef struct solarpanel{
    int panelID;
    float posX;
    float posY;
    int state;
} solarpanel;

typedef struct solarZone{
    int zoneID;
    float posX;
    float posY;
    int stateLeft;
    int stateCenter;
    int stateRight;
    float targetPositionLowX;
    float targetPositionLowY;
    float targetPositionUpX;
    float targetPositionUpY;
} solarZone;   


typedef struct EndZone{
    int zoneID;
    float posX;
    float posY;
    float posTheta;
    float dropPositionX;
    float dropPositionY;
    float dropPositionTheta;
    int numberOfPlants;
    int obstacleIDX; //Partie du mur sur l'axe X
    int obstacleIDY; //Partie du mur sur l'axe Y
} endZone;


plantZone* plantZones;
potZone* potZones;
jardiniere* jardinieres;
solarpanel* solarPanels;
solarZone* solarZones;
endZone* endZones;
endZone* dropZones;

typedef enum {DISPLACEMENT_MOVE, GRABBING_MOVE} moveType;
typedef enum{BLUE, YELLOW} teamColor;
typedef enum{GO_FORWARD_POTS, GO_FORWARD_PLANTS, UNSTACK_MOVE, Y_Align_Pots, X_Align_Pots,GET_ALL_POTS,GET_IN_JARDINIERE,GET_BACK_JARDINIERE,GET_BACK_DROP,SOLARMOVE} movingSubState;

typedef enum{MOVING,STOPPED} movingState;
typedef enum {WAITING_FOR_START, EARNING_POINTS, RETURN_TO_BASE, GAME_OVER} supremeState;
typedef enum {PLANTS_ACTION, PLANTS_POTS_ACTION, SOLAR_PANELS_ACTION} actionChoice;

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


//POTENTIAL FIELD
float computeEuclidianDistance(double x1, double y1, double x2, double y2);
float computeRectangleDistance(double x1, double y1, double x2, double y2, double x3, double y3);
void computeAttractiveField(position destination);
void computeInitialRepulsiveField();
void updateRepulsiveField(int x1,int y1, int x2, int y2);
void resetRepulsiveField(int x1,int y1, int x2, int y2);
void computeTotalField(uint8_t mode, int x1, int y1, int x2, int y2);
void print2DArray(int m, int n, double** arr);
void makeHeatmap();
void addRoundObstacle(double posX, double posY, double size, uint8_t moving, int obstacleID);
void printObstacleLists();
void removeObstacle(int obstacleID);
void enableObstacle(int obstacleID);
void computeForceVector();
void myPotentialFieldController();
void* updateKalman(void* args);
void updateLidarMeasureNoise();
void defineInitialPosition();
position closestPoint(position rect[2], position pos);
void addRectangleObstacle(double x1, double y1, double x2, double y2, uint8_t moving, int obstacleID);
void convertsSpeedToRobotFrame(double v_x, double v_y, double omega, double* output_speed);
void retrieveSpeeds(uint8_t* data, double* speed1, double* speed2);
void computeSpeedFromOdometry(double* wheel_speeds, double *v_x, double *v_y, double *omega);
void initializeObstacles();
void initializeMainController();
void addOpponentObstacle();
void updateOpponentObstacle();
void updateObstaclesStatus(); 
void PrintMapState();




void resetOdometry();
void myOdometry();

//UART ET SPI et I2C
uint8_t UART_send(int UART_handle, char* data);
uint8_t UART_receive(int UART_handle, char* received);
extern int UARTReception;
extern uint8_t waitingForReception;
int I2C_initialize(int address);
void I2C_send(char* data,char* received, int I2C_handle);
int i2c_handle_front;
int i2c_handle_rear;
void initializeLaunchGPIO();
int initializeUART();
//FIN COMMUNICATION

//Communication LIDAR
void sendFilteredPos(int pipefdCL);
//FIN LIDAR

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
extern uint8_t pressForward;
extern position myOdometryPos;

//Moteurs - odometry
extern double motorSpeed_FL;
extern double motorSpeed_FR;
extern double motorSpeed_RL;
extern double motorSpeed_RR;

extern double measuredSpeedY;
extern double measuredSpeedOmega;
extern double measuredSpeedX;
struct timeval lidarAcquisitionTime;
pthread_mutex_t lidarTimeLock;
uint8_t lidarAcquisitionFlag;
pthread_mutex_t lidarFlagLock;

//STATES
extern movingState myControllerState;
//extern uint8_t OurStartingPoint;
uint8_t OurStartingPoint; 
uint8_t OppStrartingPoint;

void handle_sigint(int sig);

extern double totalTime;
FILE* logFile;
FILE* i2cFile;

//LOG AND OTHERS
void generateLog();
void writeLog();

pthread_mutex_t lockFilteredOpponent;
extern position myFilteredOpponent;

//KALMAN
void defineOpponentPosition(float posX, float posY);
						
// STRATEGY
							   
void mainStrategy();
void waitingStrategy();
void pointsStrategy();
void returnToBaseStrategy();
void actionStrategy();
void manageGrabbing(plantZone* bestPlantZone);
uint8_t checkStartSwitch();

typedef enum {MOVE_FRONT_PLANTS, CALIB_FORK,GRAB_PLANTS_INIT, GRAB_PLANTS_MOVE,GRAB_PLANTS_CLOSE, GRAB_PLANTS_END,MOVE_FRONT_POTS,UNSTACK_POTS_MOVE,UNSTACK_POT_TAKE,UNSTACK_POT_POSITIONING,UNSTACK_POT_DROP,GRAB_POTS_MOVE,ALIGN_POTS_MOVE,LIFT_POTS,GRAB_ALL_POTS,MOVE_FRONT_JARDINIERE,MOVE_FORWARD_JARDINIERE,LOWER_PLANTS,DROP_PLANTS, DROP_ALL,MOVE_BACK_JARDINIERE, LOWER_DROP,OPEN_DROP, MOVE_BACK_DROP, MOVE_FRONT_SOLAR,WHEEL_TURN,MOVE_SOLAR,SOLAR_SET,TURN_AROUND,RESET_SOLAR,END_ACTION,FINISHED } grabbingState;
extern grabbingState myGrabState;
typedef enum {SENDING_INSTRUCTION,WAITING_ACTUATORS} actuationState;
extern actuationState myActuatorsState;
typedef enum {JARD,DROP}choice;
choice myChoice;
supremeState mySupremeState;
actionChoice myActionChoice;
movingSubState myMovingSubState;
moveType myMoveType;
uint8_t arrivedAtDestination;
extern uint8_t changeOfPlan;
teamColor myTeamColor;
uint8_t hasPlants;

struct timeval startOfMatch;
extern uint8_t nextionStart;

// fonctions de Jeu
void initializePlantZones();
void initializePotZones();
void initializeJardinieres();
void initializeSolarZones();
void initializeEndZones();
plantZone* computeBestPlantsZone();
potZone* computeBestPotsZone();
endZone* computeBestDropZone();
jardiniere* computeBestJardiniere();
endZone* computeBestStealZone();
solarZone* computeBestSolarZone();
endZone* computeBestEndZone();
void definePlantsDestination(plantZone* bestPlantZone, endZone* computeBestStealZone);
void definePotsDestination(potZone* bestPotZone);
void definedropZoneDestination(endZone* bestDropZone);
void defineJardiniereDestination(jardiniere* bestJardiniere);
void ChooseDropOrJardiniere(endZone* bestDropZone, jardiniere* bestJardiniere);
void defineStealDestination(endZone* bestStealZone);
void defineSolarDestination(solarZone* bestSolarZone);
void defineEndZoneDestination(endZone* bestEndZone);
void defineJardiniereDestination(jardiniere* bestJardiniere);

void defineBestAction();


extern double outputSpeed[3];

double filteredSpeedX, filteredSpeedY, filteredSpeedOmega;
uint8_t forksDeployed;
uint8_t forksCalibrated;

extern float timeFromStartOfMatch;
uint8_t solarDone;
extern uint8_t nbrOfPots;

void resetErrorLists();

double computeTimeElapsed(struct timeval start, struct timeval end);

typedef enum {TURNING_MOVE,MOVE_AWAY_WALL,CLASSIC} forceState;
extern forceState myForceState;
									 
////////////////////////////////////
//Nextion
////////////////////////////////////

extern int UART_handle_nextion;

// Serial input variables
extern char receivedChars[NUM_CHARS];
char withoutCrochet[NUM_CHARS];
extern char myTeam[10];
extern char myPage[25];
int startY;
int startB;
extern int go;
extern bool finish;
extern uint8_t score;

typedef struct Node {
    char* data;
    struct Node* next;
} Node;

typedef struct Queue {
    Node* front;
    Node* rear;
} Queue;

extern Queue* q;

extern struct timeval startInitialization, endQueue;

int initializeUART_nextion();
uint8_t UART_send_commands(int UART_handle, char* data);
void handleCommand(int UART_handle, char *string);
Queue* createQueue();
void enqueue(Queue* q, const char* value);
char* dequeue(Queue* q);
void display(Queue* q);
void map_coordinates(float x_meters, float y_meters, char* x_map, char* y_map, int player);
void nextion_communication(int UART_handle);																												 																														 