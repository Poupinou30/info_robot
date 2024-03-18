#ifndef HEADERS
#include "headers.h"
#define HEADERS
#endif

float actionDistance = 20; //en cm

float computeEuclidianDistance(double x1, double y1, double x2, double y2){
    return pow(pow(x2-x1,2)+pow(y2-y1,2),0.5);
}

float computeRectangleDistance(double x1, double y1, double x2, double y2, double x3, double y3) {
    double dx = fmax(fmin(x3, x2), x1) - x3;
    double dy = fmax(fmin(y3, y2), y1) - y3;
    return sqrt(dx*dx + dy*dy);
}


position closestPoint(position rect[2], position pos) {
    position closest;
    closest.x = (float*) malloc(sizeof(float));
    closest.y = (float*) malloc(sizeof(float));
    fprintf(stderr, "Entre dans distance \n");
    *closest.x = (*pos.x < *(rect[0].x)) ? *(rect[0].x) : (*pos.x > *(rect[1].x)) ? *(rect[1].x) : *pos.x;
    *closest.y = (*pos.y < *(rect[0].y)) ? *(rect[0].y) : (*pos.y > *(rect[1].y)) ? *(rect[1].y) : *pos.y;
    return closest;
}



void computeAttractiveField(position destination){ //position convertie en cm
    fprintf(stderr,"Entered in computeAttractiveField \n");
    //Cette fonction calcule le champ d'attraction pour le potential field path planning
    //Elle ne sera exécutée qu'à chaque fois qu'on changera l'objectif final en terme de destination (ex: quand on arrive a une plante)
    int posX = (int) *(destination.x)*100;
    int posY = (int) *(destination.y)*100;
    int posTheta = (int) *(destination.theta)*100;
    fprintf(stderr,"posX posY done \n");
    float scalingFactor = 0.01;
    double euclidianDistance;
    fprintf(stderr,"before loop \n");
    for (int i = 0; i < sizeY; ++i) {
        for (int j = 0; j < sizeX; ++j) {
            myField.attractiveField[i][j] = 0.5*scalingFactor* computeEuclidianDistance(j,i,posX,posY);
            //fprintf(stderr,"attractive in x = %d and y = %d is updated to %lf \n",j,i,myField.attractiveField[i][j]);
        }
    }
    fprintf(stderr,"afterloop \n");
    computeTotalField(0,NULL,NULL,NULL,NULL);
}

void computeInitialRepulsiveField(){
    //Cette fonction est censée initialiser le champ répulsif avec les obstacles qu'on connait déja (murs, jardiniaires) et ne sera éxécutée qu'en début de partie
    //Pour l'instant le champ répulsif est à 0
}

void updateRepulsiveField(int x1,int y1, int x2, int y2){

    float scalingFactor = 100;
    int X1 = fmin(x1,x2);
    int X2 = fmax(x1,x2);
    int Y1 = fmin(y1,y2);
    int Y2 = fmax(y1,y2);
    float euclidianDistance;
    int activeX1 = fmax(X1-actionDistance,0);
    int activeY1 = fmax(Y1-actionDistance,0);
    int activeX2 = fmin(X2+actionDistance,sizeX);
    int activeY2 = fmin(Y2+actionDistance,sizeY);//Définit la zone mise à jour

    for (int i = activeY1; i < activeY2; ++i) {
        for (int j = activeX1; j < activeX2; ++j) {
            euclidianDistance = computeRectangleDistance(X1,Y1,X2,Y2,j,i);
            if(j >= X1 && j <= X2 && i >= Y1 && i <= Y2){
                myField.repulsiveField[i][j] = 0.5*pow((1-(1/actionDistance)),2);
                //fprintf(stderr,"repulsive in x = %d and y = %d is updated to %lf \n",j,i,myField.repulsiveField[i][j]);
                //fprintf(stderr,"Computation was %lf \n",0.5*pow((1-(1/actionDistance)),2));
            } //Si on est DANS la zone de l'objet

            else if(euclidianDistance <= actionDistance){
                myField.repulsiveField[i][j] = 0.5*pow(((1/euclidianDistance)-(1/actionDistance)),2);
                //fprintf(stderr,"repulsive in x = %d and y = %d is updated to %lf \n",j,i,myField.repulsiveField[i][j]);
            }
            else myField.repulsiveField[i][j] = 0;

        }
    }
    computeTotalField(1,activeX1,activeY1,activeX2,activeY2);

}

void resetRepulsiveField(int x1,int y1, int x2, int y2){
    int X1 = fmin(x1,x2);
    int X2 = fmax(x1,x2);
    int Y1 = fmin(y1,y2);
    int Y2 = fmax(y1,y2);
    for (int i = Y1; i < Y2; ++i) {
        for (int j = X1; j < X2; ++j) {
            myField.repulsiveField[i][j] = 0;
        }

    }
}

void computeTotalField(uint8_t mode, int x1, int y1, int x2, int y2){
    //Mode 0 = calculer toute l'arène
    //Mode 1 = on précise sur quelle surface on doit mettre à jour le field
    if(mode == 0){
        for (int i = 0; i < sizeY; ++i) {
            for (int j = 0; j < sizeX; ++j) {
                myField.totalField[i][j] = myField.attractiveField[i][j] + myField.repulsiveField[i][j];
            }

        }
    }

    else{
        int X1 = fmin(x1,x2);
        int X2 = fmax(x1,x2);
        int Y1 = fmin(y1,y2);
        int Y2 = fmax(y1,y2);
        for (int i = fmax(Y1-actionDistance,0); i < fmin(Y2+actionDistance,sizeY); ++i) {
            for (int j = fmax(X1-actionDistance,0); j < fmin(X2+actionDistance,sizeX); ++j) {
                myField.totalField[i][j] = myField.attractiveField[i][j] + myField.repulsiveField[i][j];
            }
        }
    }
}

void print2DArray(int m, int n, double** arr) {
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            printf("%lf ", arr[i][j]);
        }
        printf("\n");
    }
}

void makeHeatmap(){
    // Créer un tableau 2D
    double** array = myField.totalField;


    // Écrire les données du tableau dans un fichier
    FILE *file = fopen("data.txt", "w");
    if(file == NULL) {
        printf("Erreur lors de l'ouverture du fichier\n");
        return 1;
    }
    for(int i = 0; i < sizeY; i++) {
        for(int j = 0; j < sizeX; j++) {
            fprintf(file, "%f ", array[i][j]);
        }
        fprintf(file, "\n");
    }
    fclose(file);

    // Utiliser gnuplot pour générer la heatmap
    FILE *gnuplotPipe = popen ("gnuplot -persistent", "w");
    if(gnuplotPipe == NULL) {
        printf("Erreur lors de l'ouverture du pipe vers gnuplot\n");
    }
    fprintf(gnuplotPipe, "set view map\n"); // Pour créer une heatmap
    fprintf(gnuplotPipe, "splot 'data.txt' matrix with image\n"); // Pour lire les données du fichier et générer l'image
    fflush(gnuplotPipe);

    // Libérer la mémoire allouée pour le tableau


}

void addRoundObstacle(double posX, double posY, double size, uint8_t moving){
    obstacle myObstacle;
    myObstacle.posX = posX;
    myObstacle.posY = posY;
    myObstacle.size = size;
    myObstacle.isRectangle = 0;
    myObstacle.moving = moving;
    if(myForce.obstacleNumber == 0){
        myForce.obstacleList = (obstacle*) malloc(sizeof(obstacle));
        myForce.obstacleList[0] = myObstacle;
        myForce.obstacleNumber ++;
        fprintf(stderr,"Obstacle added, size of list was 0 and is 1 now\n");
    }
    else{
        //fprintf(stderr,"Realloc problem 1 with obstacle number = %d \n",myForce.obstacleNumber);
        myForce.obstacleList = realloc(myForce.obstacleList,sizeof(obstacle)*(myForce.obstacleNumber+1));
        myForce.obstacleList[myForce.obstacleNumber] = myObstacle;
        myForce.obstacleNumber ++;
    }
    if(moving){
        if(myForce.movingNumber == 0){
            myForce.movingIndexes = (int*) malloc(sizeof(int)*1);
            myForce.movingIndexes[myForce.movingNumber] = myForce.obstacleNumber-1;
            myForce.movingNumber ++;
        }
        else{
            //fprintf(stderr,"Realloc problem 2 \n");
            myForce.movingIndexes = (int*) realloc(myForce.movingIndexes,sizeof(int)*(myForce.movingNumber+1));
            myForce.movingIndexes[myForce.movingNumber] = myForce.obstacleNumber-1;
            myForce.movingNumber ++;
        }


    }
}

void addRectangleObstacle(double x1, double y1, double x2, double y2, uint8_t moving){
    obstacle myObstacle;
    myObstacle.isRectangle = 1;
    myObstacle.x1 = x1;
    myObstacle.y1 = y1;
    myObstacle.x2 = x2;
    myObstacle.y2 = y2;
    myObstacle.isRectangle = 1;
    myObstacle.moving = moving;
    if(myForce.obstacleNumber == 0){
        myForce.obstacleList = (obstacle*) malloc(sizeof(obstacle));
        myForce.obstacleList[0] = myObstacle;
        myForce.obstacleNumber ++;
        fprintf(stderr,"Obstacle added, size of list was 0 and is 1 now\n");
    }
    else{
        //fprintf(stderr,"Realloc problem 1 with obstacle number = %d \n",myForce.obstacleNumber);
        myForce.obstacleList = realloc(myForce.obstacleList,sizeof(obstacle)*(myForce.obstacleNumber+1));
        myForce.obstacleList[myForce.obstacleNumber] = myObstacle;
        myForce.obstacleNumber ++;
    }
    if(moving){
        if(myForce.movingNumber == 0){
            myForce.movingIndexes = (int*) malloc(sizeof(int)*1);
            myForce.movingIndexes[myForce.movingNumber] = myForce.obstacleNumber-1;
            myForce.movingNumber ++;
        }
        else{
            //fprintf(stderr,"Realloc problem 2 \n");
            myForce.movingIndexes = (int*) realloc(myForce.movingIndexes,sizeof(int)*(myForce.movingNumber+1));
            myForce.movingIndexes[myForce.movingNumber] = myForce.obstacleNumber-1;
            myForce.movingNumber ++;
        }


    }
}

void removeMovingObstacles(){
    int j = 0;
    while (j < myForce.movingNumber) {
        for (int k = myForce.movingIndexes[j]; k < myForce.obstacleNumber-1; ++k) {
            myForce.obstacleList[k] = myForce.obstacleList[k+1];
        }
        myForce.obstacleNumber--;
        for (int k = j; k < myForce.movingNumber; ++k) {
            myForce.movingIndexes[k]--;
        }
        j++;
    }
    myForce.movingNumber = 0;
    myForce.obstacleList = realloc(myForce.obstacleList, sizeof(obstacle)*myForce.obstacleNumber);
}


void printObstacleLists(){
    fprintf(stderr,"There are %d obstacles in the list\n",myForce.obstacleNumber);
    for (int i = 0; i < myForce.obstacleNumber; ++i) {
        fprintf(stderr,"Obstacle %d is at x = %lf, y = %lf and has size of %lf and has moving to %d\n",i,myForce.obstacleList[i].posX,myForce.obstacleList[i].posY,myForce.obstacleList[i].size,myForce.obstacleList[i].moving);
    }
    for (int i = 0; i < myForce.movingNumber; ++i) {
        fprintf(stderr,"index of moving is %d\n",myForce.movingIndexes[i]);
    }
}

void computeForceVector(){
    float k_att_xy = 1;
    float k_att_theta;
    float k_repul = 1;
    double f_att_x = -k_att_xy * (*myPos.x- *destination.x);
    double f_att_y = -k_att_xy * (*myPos.y - *destination.y);
    double f_att_theta = k_att_theta * (*myPos.theta-*destination.theta);
    double f_repul_x = 0;
    double f_repul_y = 0;
    double tempoX;
    double tempoY;
    double distance;
    position tempoRectangle[2];
    position tempoPoint1, tempoPoint2;
    tempoPoint1.x = (float*) malloc(sizeof(float)*1);
    tempoPoint1.y = (float*) malloc(sizeof(float)*1);
    tempoPoint2.x = (float*) malloc(sizeof(float)*1);
    tempoPoint2.y = (float*) malloc(sizeof(float)*1);
    position myClosestPoint;

    obstacle *tempoObstacle;
    //Calcul de la force de répulsion totale
    fprintf(stderr,"Calcul de la force de répulsion totale avant boucle \n");
    for (int i = 0; i < myForce.obstacleNumber; ++i) {
        fprintf(stderr,"Avant assignation pointeur \n");
        tempoObstacle = &myForce.obstacleList[i];
        fprintf(stderr,"Après assignation pointeur \n");
        if(tempoObstacle->isRectangle){
            *tempoPoint1.x = tempoObstacle->x1;
            *tempoPoint1.y = tempoObstacle->y1;
            *tempoPoint2.x = tempoObstacle->x2;
            *tempoPoint2.y = tempoObstacle->y2;
            fprintf(stderr,"Après assignation pointeur 2\n");
            tempoRectangle[0] = tempoPoint1;
            tempoRectangle[1] = tempoPoint2;
            myClosestPoint = closestPoint(tempoRectangle,myPos);
            fprintf(stderr,"Avant calcul distance \n");
            distance = computeEuclidianDistance(*myPos.x, *myPos.y, *myClosestPoint.x, *myClosestPoint.y); //Calcul la distance
            fprintf(stderr,"Après calcul distance \n");
            tempoX = *myClosestPoint.x; //Calcule la position en x
            tempoY = *myClosestPoint.y; //Calcule la position en y
            free(myClosestPoint.x);
            free(myClosestPoint.y);
        }
        else{
            tempoX = tempoObstacle->posX; //Calcule la position en x
            tempoY = tempoObstacle->posY; //Calcule la position en y
            distance = computeEuclidianDistance(tempoX,tempoY,*myPos.x,*myPos.y)-myForce.obstacleList[i].size; //Calcule la distance
        }
        if(distance < actionDistance){
            f_repul_x = f_repul_x + k_repul*(1/distance - 1/actionDistance)*(1/pow(distance,3))*(tempoX - *myPos.x);
            f_repul_y = f_repul_y + k_repul*(1/distance - 1/actionDistance)*(1/pow(distance,3))*(tempoY - *myPos.y);

        }
    }
    free(tempoPoint1.x);
    free(tempoPoint1.y);
    free(tempoPoint2.x);
    free(tempoPoint2.y);
    f_tot_x = f_att_x+f_repul_x;
    f_tot_y = f_att_y + f_repul_y;
    fprintf(stderr,"fin du calcul de la force de répulsion totale avant boucle \n");
    //Calcul de la force de attraction totale
}

void myPotentialFieldController(double* speedTab, uint8_t* dataFront, uint8_t* dataRear, int spi_handle_front, int spi_handle_rear){
    convertsVelocity(f_tot_x,f_tot_y,f_theta,speedTab);
    createArray(speedTab[0]/(2*_Pi) *114688/100,speedTab[1]/(2*_Pi) *114688/100,dataFront);
    createArray(speedTab[2]/(2*_Pi)*114688/100,speedTab[3]/(2*_Pi) *114688/100,dataRear);
    SPI_send(dataFront,spi_handle_front,NULL); //FRONT
    SPI_send(dataRear,spi_handle_rear,NULL);

}