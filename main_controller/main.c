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


int mainUART(){
    initializeMainController();
    printf("UART handle = %d \n",UART_handle);
    //char* myString = "test123";
    //char myString[10] = {};
    //char receivedChar[1000];
    //UART_send(UART_handle,myString,receivedChar);
    //printf("received char = '%s'",receivedChar);
    /*deployForks();
    sleep(1);
    setUpperFork(130);
    sleep(1);
    setLowerFork(10);
    sleep(4);
    retractForks();*/
    myGrabState = CALIB_FORK;
    myActuatorsState = SENDING_INSTRUCTION;
    while(myGrabState != FINISHED){
        
        manageGrabbing();
    }
}

int mainPatternOdometry(){
    initializeMainController();
    struct timeval now, end, endInst;
    gettimeofday(&now, NULL);
    gettimeofday(&end,NULL);
    gettimeofday(&endInst,NULL);
    double nowValue = now.tv_sec*1000+now.tv_usec/1000;
    double endValue = end.tv_sec*1000+end.tv_usec/1000;
    double instValue = endInst.tv_sec*1000+endInst.tv_usec/1000;

    while(1){
        

        gettimeofday(&now, NULL);
        double currentTime = now.tv_sec*1000+now.tv_usec/1000;
        if (currentTime - nowValue >= 3000) {
            break;
        }

        gettimeofday(&endInst,NULL);
        double elapsedTime = endInst.tv_sec*1000+endInst.tv_usec/1000 - instValue;
        if (elapsedTime >= 500) {

            processInstructionNew(-0.0,0.0,1,i2c_handle_front,i2c_handle_rear);
            printf("speed fl = %f and speed fr = %f and speed rl = %f and speed rr = %f \n",motorSpeed_FL,motorSpeed_FR,motorSpeed_RL,motorSpeed_RR);
            double wheelSpeed[4] = {motorSpeed_FL,motorSpeed_FR,motorSpeed_RL,motorSpeed_RR};
            double vx,vy,omega;
            computeSpeedFromOdometry(wheelSpeed,&vx,&vy,&omega);
            printf("vx = %f vy = %f omega = %f \n",vx,vy,omega);

            instValue = endInst.tv_sec*1000+endInst.tv_usec/1000;
        }
    }
    processInstructionNew(0.0,0.0,0,i2c_handle_front,i2c_handle_rear);



}

int main(){
    initializeMainController();
    *myPos.x = 0;
    *myPos.y = 0;
    *myPos.theta = 0;
    *myOdometryPos.x = 0.72;
    *myOdometryPos.y = 1.23;
    while(1){

        updateKalman();
    }
    
}

int mainPatternOdometry(){
    initializeMainController();
    struct timeval now, end, endInst, endSecond;
    gettimeofday(&now, NULL);
    gettimeofday(&end,NULL);
    gettimeofday(&endInst,NULL);
    gettimeofday(&endSecond,NULL);
    double nowValue = now.tv_sec*1000+now.tv_usec/1000;
    double endValue = end.tv_sec*1000+end.tv_usec/1000;
    double instValue = endInst.tv_sec*1000+endInst.tv_usec/1000;
    double secondValue = endSecond.tv_sec*1000+endSecond.tv_usec/1000;
    double vx,vy,omega;
    while(1){
        gettimeofday(&now, NULL);
        double currentTime = now.tv_sec*1000+now.tv_usec/1000;
        if (currentTime - nowValue >= 100000) {
            break;
        }

        gettimeofday(&endInst,NULL);
        double elapsedTime = endInst.tv_sec*1000+endInst.tv_usec/1000 - instValue;
        if (elapsedTime >= 10) {
            processInstructionNew(0.2,0.2,0,i2c_handle_front,i2c_handle_rear);
            double wheelSpeed[4] = {motorSpeed_FL,motorSpeed_FR,motorSpeed_RL,motorSpeed_RR};
            
            computeSpeedFromOdometry(wheelSpeed,&vx,&vy,&omega);
            
            myOdometry();
            if(*myOdometryPos.x > 1 || *myOdometryPos.y > 1) break;
            instValue = endInst.tv_sec*1000+endInst.tv_usec/1000;
        }

        gettimeofday(&endSecond,NULL);
        double secondElapsedTime = endSecond.tv_sec*1000+endSecond.tv_usec/1000 - secondValue;
        if (secondElapsedTime >= 1000) {
            printf("vx = %f vy = %f omega = %f \n",vx,vy,omega);
            printf("pos x = %f, pos y = %f, pos theta = %f",*myOdometryPos.x,*myOdometryPos.y,*myOdometryPos.theta);
            printf("Executing code every second\n");
            secondValue = endSecond.tv_sec*1000+endSecond.tv_usec/1000;
        }
    }
    processInstructionNew(0.0,0.0,0,i2c_handle_front,i2c_handle_rear);
     printf("pos x = %f, pos y = %f, pos theta = %f",*myOdometryPos.x,*myOdometryPos.y,*myOdometryPos.theta);
}

int mainSPI(){ // spi test
    initializeMainController();
    uint8_t myTab[4] = {3};
    uint8_t receivedTab[4];
    SPI_send(myTab,spi_handle_rear,receivedTab);
    for(int i = 0; i<4; i++){
        printf("Received %d \n",receivedTab[i]);
    }
}
int mainPATTERN(){
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
    tunePID(Ki,Kp,i2c_handle_front,i2c_handle_rear);

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
    i2c_handle_front = I2C_initialize(0x40);
    i2c_handle_rear = I2C_initialize(0x41);

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
    *myFilteredPos.x = 0;
    *myFilteredPos.y = 0;
    *myFilteredPos.theta = 0;
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

void processInstruction(float v_x, float v_y, float omega, double* speedTab, int spi_handle_rear,int spi_handle_front,uint8_t *dataFront, uint8_t *dataRear){ //Fonction qui gère l'envoi au moteurs et les calculs
    convertsVelocity(v_x,v_y,omega,speedTab);
    createArray(speedTab[0]/(2*_Pi) *114688/100,speedTab[1]/(2*_Pi) *114688/100,dataFront);
    createArray(speedTab[2]/(2*_Pi)*114688/100,speedTab[3]/(2*_Pi) *114688/100,dataRear);
    SPI_send(dataRear,spi_handle_rear,SPI_reception_buffer_rear); //REAR
    SPI_send(dataFront,spi_handle_front,SPI_reception_buffer_front); //FRONT
    
}

void processInstructionNew(float v_x, float v_y, float omega, int i2c_handle_front,int i2c_handle_rear){
    double speedTab[4];
    convertsVelocity(v_x,v_y,omega,speedTab);
    char toSendFront[100]; char toSendRear[100]; char toReceiveFront[100]; char toReceiveRear[100];
    sprintf(toSendFront,"<setSpeed-%d-%d>",(int) (speedTab[0]/(2*_Pi) *114688),(int) (speedTab[1]/(2*_Pi) *114688)); //Vitesses en ticks par seconde
    sprintf(toSendRear,"<setSpeed-%d-%d>",(int) (speedTab[2]/(2*_Pi) *114688),(int) (speedTab[3]/(2*_Pi) *114688)); //Vitesses en ticks par seconde
    I2C_send(toSendFront,toReceiveFront,i2c_handle_front);
    I2C_send(toSendRear,toReceiveRear,i2c_handle_rear);
    int tempoSpeedFL, tempoSpeedFR, tempoSpeedRL, tempoSpeedRR;
    //if(VERBOSE) printf("received 1 = '%s' and 2 = '%s'", toReceiveFront,toReceiveRear);
    sscanf(toReceiveFront, "<measuredSpeed-%d-%d>", &tempoSpeedFL, &tempoSpeedFR);
    sscanf(toReceiveRear, "<measuredSpeed-%d-%d>", &tempoSpeedRL, &tempoSpeedRR);
    /*if(VERBOSE){
        printf("tempoSpeedFL = %d FR = %d RL = %d RR = %d \n",tempoSpeedFL,tempoSpeedFR,tempoSpeedRL,tempoSpeedRR);
    }*/
    motorSpeed_FL = (double) tempoSpeedFL/114688.0 * 2*_Pi;
    motorSpeed_FR = (double) tempoSpeedFR/114688.0 * 2*_Pi;
    motorSpeed_RL = (double) tempoSpeedRL/114688.0 * 2*_Pi;
    motorSpeed_RR = (double) tempoSpeedRR/114688.0 * 2*_Pi;

}
