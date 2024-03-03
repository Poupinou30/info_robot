#ifndef HEADERS
#include "headers.h"
#define HEADERS
#endif

float* positionReceived;
pthread_mutex_t lockPosition;
position myPos;
field myField;





int mainPattern(){
    gpioInitialise();
    int spi_handle_front = initializeSPI(0);
    int spi_handle_rear = initializeSPI(1);
    uint8_t *dataFront = (uint8_t*) malloc(sizeof(uint8_t)*4);
    uint8_t *dataRear = (uint8_t*) malloc(sizeof(uint8_t)*4);
    uint8_t *receivedData = (uint8_t*) malloc(sizeof(uint8_t)*4);

    double *speedTab = (double*) malloc(sizeof(double)*4);
    double *absoluteSpeedTab = (double*) malloc(sizeof(double)*3);
    int *speedMeasurement1 = (int*) malloc(sizeof(int)*1000);
    // int Kp = 0.03;
    // int Ki = 10;
    int Kp = 10; //reference
    int Ki = 60; //reference
    //Tune PID
    tunePID(spi_handle_front,spi_handle_rear,Kp,0,Ki,0);

    createArray(0,0,dataFront);
    createArray(0,0,dataRear);

    SPI_send(dataFront,spi_handle_front,NULL); //FRONT
    SPI_send(dataRear,spi_handle_rear,NULL);

    sleep(0.5);


    
    // processInstruction(-0.1,-0.05,0.0,speedTab,spi_handle_rear,spi_handle_front,dataFront,dataRear);
    // sleep(3);
    // createArray(0,0,dataFront);
    // createArray(0,0,dataRear);
    // SPI_send(dataFront,spi_handle_front,NULL); //FRONT
    // SPI_send(dataRear,spi_handle_rear,NULL);
    // sleep(1);
    // processInstruction(0,+0.1,-0.0,speedTab,spi_handle_rear,spi_handle_front,dataFront,dataRear);
    // sleep(6);
    // createArray(0,0,dataFront);
    // createArray(0,0,dataRear);
    // SPI_send(dataFront,spi_handle_front,NULL); //FRONT
    // SPI_send(dataRear,spi_handle_rear,NULL);
    // sleep(1);


    //Test PATTERN
    // processInstruction(0.1,0.1,0,speedTab,spi_handle_rear,spi_handle_front,dataFront,dataRear);
    // sleep(6);
    // createArray(0,0,dataFront);
    // createArray(0,0,dataRear);
    // SPI_send(dataFront,spi_handle_front,NULL); //FRONT
    // SPI_send(dataRear,spi_handle_rear,NULL);
    // sleep(1);
    // processInstruction(-0.1,-0.1,0,speedTab,spi_handle_rear,spi_handle_front,dataFront,dataRear);
    
    // sleep(6);
    // createArray(0,0,dataFront);
    // createArray(0,0,dataRear);
    // SPI_send(dataFront,spi_handle_front,NULL); //FRONT
    // SPI_send(dataRear,spi_handle_rear,NULL);

    //Pattern DEMO

    processInstruction(0.2,0,0.0,speedTab,spi_handle_rear,spi_handle_front,dataFront,dataRear);
    sleep(2);//2
    processInstruction(0.5,0,0.0,speedTab,spi_handle_rear,spi_handle_front,dataFront,dataRear);
    sleep(1);
    processInstruction(0.0,0,0.0,speedTab,spi_handle_rear,spi_handle_front,dataFront,dataRear);
    sleep(0.5);
    processInstruction(0,0.2,0.0,speedTab,spi_handle_rear,spi_handle_front,dataFront,dataRear);
    sleep(2);
    processInstruction(0,0.5,0.0,speedTab,spi_handle_rear,spi_handle_front,dataFront,dataRear);
    sleep(1);
    processInstruction(0.0,0,0.0,speedTab,spi_handle_rear,spi_handle_front,dataFront,dataRear);
    sleep(0.5);
    processInstruction(-0.4,-0.4,0.0,speedTab,spi_handle_rear,spi_handle_front,dataFront,dataRear);
    sleep(2.25);
    processInstruction(0,0,0.0,speedTab,spi_handle_rear,spi_handle_front,dataFront,dataRear);
    sleep(0.5);
    processInstruction(0,0,1,speedTab,spi_handle_rear,spi_handle_front,dataFront,dataRear);
    sleep(3);
    processInstruction(0,0,0.0,speedTab,spi_handle_rear,spi_handle_front,dataFront,dataRear);


    


}

int main(){
    //Initialiser la structure de position
    positionReceived = malloc(3*sizeof(float));
    myPos.x = &positionReceived[0];
    myPos.y = &positionReceived[1];
    myPos.theta = &positionReceived[2];
    //initialisation du pipe LIDAR - CONTROLLER
    fprintf(stderr,"program launching \n");
    int pipefd[2];
    pipe(pipefd);

    //Initialiser la structure du field on // on a une arène de

    myField.attractiveField = (double**) malloc(sizeof(double*)*sizeX);
    myField.repulsiveField = (double**) malloc(sizeof(double*)*sizeX);
    myField.totalField = (double**) malloc(sizeof(double*)*sizeX);
    for (int i = 0; i < sizeX; ++i) {
        myField.attractiveField[i] = (double*) calloc(sizeY,sizeof(double));
        myField.repulsiveField[i] = (double*) calloc(sizeY,sizeof(double));
        myField.totalField[i] = (double*) calloc(sizeY,sizeof(double));
    }

     //Buffer pour reprendre les données du Lidar

    //Initialisation thread qui execute le programme lidar
    /*pthread_t lidarExecuteThread;
    pthread_create(&lidarExecuteThread,NULL,executeProgram,&pipefd[1]); // Passage de l'adresse de pipefd[1] à pthread_create
    fprintf(stderr,"Thread for execution launched \n");
    //initialisation thread qui récupère les données du pipe provenant du programme lidar
    pthread_t pipeComThread;
    pthread_create(&pipeComThread,NULL,receptionPipe,&pipefd);
    fprintf(stderr,"Thread for capture launched \n");
    while(1){
        ;
    }




    close(pipefd[0]);
    close(pipefd[1]);*/
    updateRepulsiveField(290,90,300,110);
    position destination;
    *destination.x = 12;
    *destination.y = 127;
    computeAttractiveField(destination);

    return 0;

};

void processInstruction(float v_x, float v_y, float omega, double* speedTab, int spi_handle_rear,int spi_handle_front,uint8_t *dataFront, uint8_t *dataRear){
    convertsVelocity(v_x,v_y,omega,speedTab);
    createArray(speedTab[0]/(2*_Pi) *114688/100,speedTab[1]/(2*_Pi) *114688/100,dataFront);
    createArray(speedTab[2]/(2*_Pi)*114688/100,speedTab[3]/(2*_Pi) *114688/100,dataRear);
    SPI_send(dataRear,spi_handle_rear,NULL);
    SPI_send(dataFront,spi_handle_front,NULL); //FRONT
}


int mainOld() {
    gpioInitialise();
    int spi_handle_front = initializeSPI(0);
    int spi_handle_rear = initializeSPI(1);
    uint8_t *dataFront = (uint8_t*) malloc(sizeof(uint8_t)*4);
    uint8_t *dataRear = (uint8_t*) malloc(sizeof(uint8_t)*4);
    double *speedTab = (double*) malloc(sizeof(double)*4);
    double *absoluteSpeedTab = (double*) malloc(sizeof(double)*3);
    double *targetPosition = (double*) malloc(sizeof(double)*3);
    targetPosition[0] = 5;
    targetPosition[1] = 0;
    targetPosition[2] = 0;
    double currentX = 0;
    double currentY = 0;
    double currentTheta = 0;
    int verboseCount = 0;
    
    double endValue = 0;
    double nowValue = 0;

    struct timeval now, end;
    gettimeofday(&now, NULL);
    nowValue = now.tv_sec*1000+now.tv_usec/1000;

    while(1){
        gettimeofday(&end, NULL);
        endValue = end.tv_sec*1000+end.tv_usec/1000;
    
        if(VERBOSE && 0){
            fprintf(stderr,"Time measurement = %lf \n",(endValue-nowValue));
            fprintf(stderr,"Time measurement now = %f \n",nowValue);
            fprintf(stderr,"Time measurement end = %f \n",endValue);
        } 
        if(endValue - nowValue > timeDelay){
            myController(speedTab,currentX,currentY,currentTheta,absoluteSpeedTab,targetPosition);
            if(VERBOSE && verboseCount == 10){
                fprintf(stderr,"Speed of Front Left = %f \n",speedTab[0]);
                fprintf(stderr,"Speed of Front Right = %f \n",speedTab[1]);
                fprintf(stderr,"Speed of Rear Left = %f \n",speedTab[2]);
                fprintf(stderr,"Speed of Rear Right = %f \n",speedTab[3]);
                verboseCount = 0;
            }
            verboseCount++;
            gettimeofday(&end,NULL);
            endValue = end.tv_sec*1000+end.tv_usec/1000;

            //fprintf(stderr,"Time elapsed between two loops = %f \n", endValue - nowValue);
            currentX = currentX + absoluteSpeedTab[0] * (endValue-nowValue)/1000 * randomDouble(0.95,1.05);
            currentY = currentY + absoluteSpeedTab[1] * (endValue-nowValue)/1000 * randomDouble(0.95,1.05);
            currentTheta = currentTheta + absoluteSpeedTab[2] * (endValue-nowValue)/1000 * randomDouble(0.95,1.05);
            gettimeofday(&now,NULL);
            nowValue = now.tv_sec*1000+now.tv_usec/1000;
            createArray(speedTab[0]/(2*_Pi) *114688/100,speedTab[1]/(2*_Pi) *114688/100,dataFront);
            createArray(speedTab[2]/(2*_Pi)*114688/100,speedTab[3]/(2*_Pi) *114688/100,dataRear);
            SPI_send(dataFront,spi_handle_front,NULL); //FRONT
            SPI_send(dataRear,spi_handle_rear,NULL);
            //if(VERBOSE) fprintf(stderr,"SPI_SENDED \n");
        }
        
        //creer un autre spi handle pour les moteurs  arrières

    }



    /*int16_t num1 = 0;
    int16_t num2 = 10;
    createArray(num1*100,num2*100,data);

    SPI_send(data,spi_handle,NULL);
    sleep(1);
    num2 = 100;
    createArray(num1*100,num2*100,data);
    SPI_send(data,spi_handle,NULL);
    sleep(1);
    num2 = 20;
    createArray(num1*100,num2*100,data);
    SPI_send(data,spi_handle,NULL);
    sleep(1);
    num2 = 50;
    createArray(num1*100,num2*100,data);
    SPI_send(data,spi_handle,NULL);
    sleep(1);
    num2 = 10;
    createArray(num1*100,num2*100,data);
    SPI_send(data,spi_handle,NULL);
    sleep(1);
    num2 = 30;
    createArray(num1*100,num2*100,data);
    SPI_send(data,spi_handle,NULL);
    sleep(1);
    num2 = 0;
    createArray(num1*100,num2*100,data);
    SPI_send(data,spi_handle,NULL);*/
    }


void sendInstruction(float theta,float turn, float speed){
    float theSin = sin(degToRad(theta)- 3.14/4);
    float theCos = cos(degToRad(theta)- 3.14/4);
    float theMax = fmax(fabs(theSin),fabs(theCos));
    float leftFront = speed * theCos/theMax + turn;
    float rightFront = speed * theSin/theMax - turn;
    float leftRear = speed * theSin/theMax + turn;
    float rightRear = speed * theCos/theMax - turn;

    if((speed + fabs(turn)) >1){
;
    }
}

