#ifndef HEADERS
#include "headers.h"
#define HEADERS
#endif

float* positionReceived;

pthread_mutex_t lockRefreshCounter;

field myField;
forceVector myForce;
position destination;
position myPos;
position myFilteredPos;
position myOpponent;
uint8_t *SPI_reception_buffer_rear;
uint8_t *SPI_reception_buffer_front;
int spi_handle_front;
int spi_handle_rear;



int main(){
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

    /*processInstruction(0,0.1,0.0,speedTab,spi_handle_rear,spi_handle_front,dataFront,dataRear);
    sleep(3);//2
    processInstruction(0.0,0,0.0,speedTab,spi_handle_rear,spi_handle_front,dataFront,dataRear);
    sleep(0.5);
    processInstruction(0.1,0,0.0,speedTab,spi_handle_rear,spi_handle_front,dataFront,dataRear);
    sleep(3);
    
    processInstruction(-0.1,-0.1,0.0,speedTab,spi_handle_rear,spi_handle_front,dataFront,dataRear);
    sleep(3);*/

    processInstruction(-0.2,0.2,0.0,speedTab,spi_handle_rear,spi_handle_front,dataFront,dataRear);
    sleep(8);

    /*processInstruction(0.5,0,0.0,speedTab,spi_handle_rear,spi_handle_front,dataFront,dataRear);
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
    sleep(3);*/
    processInstruction(0,0,0.0,speedTab,spi_handle_rear,spi_handle_front,dataFront,dataRear);





}

void initializeMainController(){
    //Initialisation GPIO et interfaces
    gpioInitialise();
    spi_handle_front = initializeSPI(0);
    spi_handle_rear = initializeSPI(1);
    UART_handle = initializeUART();

    //Initialisation variables
    positionReceived = malloc(3*sizeof(float));
    myForce.obstacleNumber = 0;
    myForce.movingNumber = 0;
    destination.x = (float*)malloc(sizeof(float));
    destination.y = (float*)malloc(sizeof(float));
    destination.theta = (float*)malloc(sizeof(float));
    myPos.x = (float*)malloc(sizeof(float));
    myPos.y = (float*)malloc(sizeof(float));
    myPos.theta = (float*)malloc(sizeof(float));
    myFilteredPos.x = (float*)malloc(sizeof(float));
    myFilteredPos.y = (float*)malloc(sizeof(float));
    myFilteredPos.theta = (float*)malloc(sizeof(float));
    myPos.theta = (float*)malloc(sizeof(float));
    myPos.x = &positionReceived[0];
    myPos.y = &positionReceived[1];
    myPos.theta = &positionReceived[2];
    myOpponent.x = (float*)malloc(sizeof(float));
    myOpponent.y = (float*)malloc(sizeof(float));
    myOdometryPos.x = (float*)malloc(sizeof(float));
    myOdometryPos.y = (float*)malloc(sizeof(float));
    myOdometryPos.theta = (float*)malloc(sizeof(float));
    *myOdometryPos.x = 0;
    *myOdometryPos.y = 0;
    *myOdometryPos.theta = 0;
}

int mainFINAL(){

    initializeMainController();
    double* outputSpeed = malloc(sizeof(double)*4);
    uint8_t *dataFront = (uint8_t*) malloc(sizeof(uint8_t)*4);
    uint8_t *dataRear = (uint8_t*) malloc(sizeof(uint8_t)*4);
    struct timeval now, end, endPrint;
    uint8_t *SPI_reception_buffer_1 = (uint8_t*) malloc(sizeof(uint8_t)*4);
    uint8_t *SPI_reception_buffer_2 = (uint8_t*) malloc(sizeof(uint8_t)*4);

    //Assignation des valeurs
    double endValue = 0;
    double nowValue = 0;
    double endValuePrint = 0;
    pthread_mutex_lock(&lockDestination);
    *destination.x = 1.00;
    *destination.y = 2.00;
    *destination.theta = 18;
    destination_set = 0;
    pthread_mutex_unlock(&lockDestination);
    *myPos.x = 0;
    *myPos.y = 0;
    *myPos.theta = 0;
    double speedTabRobotFrame[3] = {0,0,0};


//TEST
    //addObstacle(0,0.10,0.01,0);
    
    addRectangleObstacle(0,0,2,0,0); //Mur du bas
    addRectangleObstacle(0,0,0,3,0); //Mur de gauche
    addRectangleObstacle(2,0,2,3,0); //Mur de droite
    addRectangleObstacle(0,3,2,3,0); //Mur du haut
    addRectangleObstacle(0,1.05,0.145,3-1.05,0); //jardinières gauche
    addRectangleObstacle(2,1.05,2-0.145,3-1.05,0); //jardinières droite
    
    /*while(1){
        fprintf(stderr,"Entrer la position du robot: ");
        scanf("%f %f %f", myPos.x, myPos.y,myPos.theta);
        computeForceVector();
        fprintf(stderr,"Initial force X  = %lf \n",f_tot_x);
        fprintf(stderr,"Initial force Y  = %lf \n",f_tot_y);
        fprintf(stderr,"Initial force Theta  = %lf \n",f_theta);
    }*/
    
//FIN TEST

    //Pipe et thread
    int pipefd[2];
    pipe(pipefd);
    pthread_t lidarExecuteThread;
    
    pthread_create(&lidarExecuteThread,NULL,executeProgram,&pipefd[1]); // Passage de l'adresse de pipefd[1] à pthread_create
    fprintf(stderr,"Thread for execution launched \n");
    //initialisation thread qui récupère les données du pipe provenant du programme lidar
    pthread_t pipeComThread;
    pthread_create(&pipeComThread,NULL,receptionPipe,&pipefd);
    fprintf(stderr,"Thread for capture launched \n");
    while(readyToGo != 1){
        fprintf(stderr,"waiting for position acquisition \n");
        sleep(1);
    }
    fprintf(stderr,"Position acquired \n");
    resetOdometry();
    

    gettimeofday(&end, NULL);
    gettimeofday(&endPrint, NULL);
    endValue = end.tv_sec*1000+end.tv_usec/1000;
    endValuePrint = endPrint.tv_sec*1000+endPrint.tv_usec/1000;
    while(1){
        gettimeofday(&now,NULL);
        nowValue = now.tv_sec*1000+now.tv_usec/1000;
        if(nowValue - endValuePrint > 100){
            if(VERBOSE) fprintf(stderr,"début while\n");
            pthread_mutex_lock(&lockPosition);
            pthread_mutex_lock(&lockFilteredPosition);
            pthread_mutex_lock(&lockOpponentPosition);
            fprintf(stderr,"X = %f \n",*(myPos.x));
            fprintf(stderr,"Y = %f \n",*(myPos.y));
            fprintf(stderr,"Theta = %f \n",*(myPos.theta));
            fprintf(stderr,"filtered X = %f \n",*(myFilteredPos.x));
            fprintf(stderr,"filtered Y = %f \n",*(myFilteredPos.y));
            fprintf(stderr,"filtered Theta = %f \n",*(myFilteredPos.theta));
            fprintf(stderr,"X opponent = %f \n",*(myOpponent.x));
            fprintf(stderr,"Y opponent= %f \n",*(myOpponent.y));
            pthread_mutex_unlock(&lockPosition);
            pthread_mutex_unlock(&lockFilteredPosition);
            pthread_mutex_unlock(&lockOpponentPosition);
            if(VERBOSE) fprintf(stderr,"Avant force vector\n");
            computeForceVector();
            fprintf(stderr,"Initial force X  = %lf \n",f_tot_x);
        fprintf(stderr,"Initial force Y  = %lf \n",f_tot_y);
        fprintf(stderr,"Initial force Theta  = %lf \n",f_theta);

        convertsSpeedToRobotFrame(f_tot_x,f_tot_y,f_theta,speedTabRobotFrame);
        myPotentialFieldController(outputSpeed,dataFront,dataRear,spi_handle_front,spi_handle_rear);

            //fprintf(stderr,"refreshed %d times \n",refreshCounter);

            pthread_mutex_lock(&lockRefreshCounter);
            refreshCounter = 0;
            pthread_mutex_unlock(&lockRefreshCounter);
            if(VERBOSE) fprintf(stderr,"Après lockrefresh\n");
            gettimeofday(&endPrint, NULL);
            endValuePrint = endPrint.tv_sec*1000+endPrint.tv_usec/1000;
            if(VERBOSE) fprintf(stderr,"fin while\n");
        }
        if(nowValue - endValue > 50){
            //myPotentialFieldController(outputSpeed,dataFront,dataRear,spi_handle_front,spi_handle_rear);
            gettimeofday(&end,NULL);
            endValue = end.tv_sec*1000+end.tv_usec/1000;
        }
        

    }


    //Attention, les lignes qui close le pipe doivent être placées tout à la fin du code sinon on a une erreur de lecture!!
    close(pipefd[0]);
    close(pipefd[1]);


}

int mainField(){
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

    myField.attractiveField = (double**) malloc(sizeof(double*)*sizeY);
    myField.repulsiveField = (double**) malloc(sizeof(double*)*sizeY);
    myField.totalField = (double**) malloc(sizeof(double*)*sizeY);
    fprintf(stderr,"First malloc \n");
    for (int i = 0; i < sizeY; ++i) {
        myField.attractiveField[i] = (double*) calloc(sizeX,sizeof(double));
        myField.repulsiveField[i] = (double*) calloc(sizeX,sizeof(double));
        myField.totalField[i] = (double*) calloc(sizeX,sizeof(double));
    }
    fprintf(stderr,"All mallocs done \n");

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

    updateRepulsiveField(50,50,60,60);
    fprintf(stderr,"updateRepulsive done \n");

    destination.x = (float*)malloc(sizeof(float));
    destination.y = (float*)malloc(sizeof(float));
    destination.theta = (float*)malloc(sizeof(float));
    *destination.x = 1.00;
    *destination.y = 2.00;
    fprintf(stderr,"Position set \n");
    computeAttractiveField(destination);
    fprintf(stderr,"Compute attractive done \n");
    makeHeatmap();

    return 0;

}

void processInstruction(float v_x, float v_y, float omega, double* speedTab, int spi_handle_rear,int spi_handle_front,uint8_t *dataFront, uint8_t *dataRear){
    convertsVelocity(v_x,v_y,omega,speedTab);
    createArray(speedTab[0]/(2*_Pi) *114688/100,speedTab[1]/(2*_Pi) *114688/100,dataFront);
    createArray(speedTab[2]/(2*_Pi)*114688/100,speedTab[3]/(2*_Pi) *114688/100,dataRear);
    SPI_send(dataRear,spi_handle_rear,SPI_reception_buffer_rear); //REAR
    SPI_send(dataFront,spi_handle_front,SPI_reception_buffer_front); //FRONT
    
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

