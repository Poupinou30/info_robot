#ifndef HEADERS
#include "headers.h"
#define HEADERS
#endif

float* positionReceived;
pthread_mutex_t lockPosition;
position myPos;
field myField;



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

    updateRepulsiveField(90,290,110,300);
    fprintf(stderr,"updateRepulsive done \n");
    position destination;
    destination.x = (float*)malloc(sizeof(float));
    destination.y = (float*)malloc(sizeof(float));
    destination.theta = (float*)malloc(sizeof(float));
    *destination.x = 12;
    *destination.y = 127;
    fprintf(stderr,"Position set \n");
    //computeAttractiveField(destination);
    //fprintf(stderr,"Compute attractive done \n");
    makeHeatmap();

    return 0;

};

void processInstruction(float v_x, float v_y, float omega, double* speedTab, int spi_handle_rear,int spi_handle_front,uint8_t *dataFront, uint8_t *dataRear){
    convertsVelocity(v_x,v_y,omega,speedTab);
    createArray(speedTab[0]/(2*_Pi) *114688/100,speedTab[1]/(2*_Pi) *114688/100,dataFront);
    createArray(speedTab[2]/(2*_Pi)*114688/100,speedTab[3]/(2*_Pi) *114688/100,dataRear);
    SPI_send(dataRear,spi_handle_rear,NULL);
    SPI_send(dataFront,spi_handle_front,NULL); //FRONT
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

