#ifndef HEADERS
#include "headers.h"
#define HEADERS
#endif

#define GRAB_SPEED 0.2

float fixActionDistance = 0.3;
float mobileActionDistance = 0.4;

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
    *closest.x = (*pos.x < *(rect[0].x)) ? *(rect[0].x) : (*pos.x > *(rect[1].x)) ? *(rect[1].x) : *pos.x;
    *closest.y = (*pos.y < *(rect[0].y)) ? *(rect[0].y) : (*pos.y > *(rect[1].y)) ? *(rect[1].y) : *pos.y;
    return closest;
}

position closestPointBetweenRectangles(position rectA[2], position rectB[2]) {
    float minDistance = INFINITY;
    position closestPoint;
    closestPoint.x = (float*) malloc(sizeof(float));
    closestPoint.y = (float*) malloc(sizeof(float));

    // Définition des coins des rectangles A et B
    float cornersAx[4], cornersAy[4], cornersBx[4], cornersBy[4];

    cornersAx[0] = cornersAx[3] = fmin(*rectA[0].x, *rectA[1].x);
    cornersAx[1] = cornersAx[2] = fmax(*rectA[0].x, *rectA[1].x);
    cornersAy[0] = cornersAy[1] = fmin(*rectA[0].y, *rectA[1].y);
    cornersAy[2] = cornersAy[3] = fmax(*rectA[0].y, *rectA[1].y);

    cornersBx[0] = cornersBx[3] = fmin(*rectB[0].x, *rectB[1].x);
    cornersBx[1] = cornersBx[2] = fmax(*rectB[0].x, *rectB[1].x);
    cornersBy[0] = cornersBy[1] = fmin(*rectB[0].y, *rectB[1].y);
    cornersBy[2] = cornersBy[3] = fmax(*rectB[0].y, *rectB[1].y);

    // Comparer chaque coin de A avec chaque coin de B pour trouver le point le plus proche
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            float distance = computeEuclidianDistance(cornersAx[i], cornersAy[i], cornersBx[j], cornersBy[j]);
            if (distance < minDistance) {
                minDistance = distance;
                *closestPoint.x = cornersAx[i];
                *closestPoint.y = cornersAy[i];
            }
        }
    }

    return closestPoint;
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
    float actionDistance = 0.3;
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
    float actionDistance = 0.3;
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
        printf("Erreur lors de l'ouverture du fichier data.txt\n");
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

void addRoundObstacle(double posX, double posY, double size, uint8_t moving, int obstacleID){
    obstacle myObstacle;
    myObstacle.posX = posX;
    myObstacle.posY = posY;
    myObstacle.size = size;
    myObstacle.isRectangle = 0;
    myObstacle.moving = moving;
    myObstacle.obstacleID = obstacleID;
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

void addOpponentObstacle(){
    obstacle myObstacle;
    myObstacle.size = 0.25;
    myObstacle.isRectangle = 0;
    myObstacle.moving = 1;
    myObstacle.obstacleID = 0;
    if(myForce.obstacleNumber == 0){
        myForce.obstacleList = (obstacle*) malloc(sizeof(obstacle));
        myForce.obstacleList[0] = myObstacle;
        myForce.obstacleNumber ++;
        fprintf(stderr,"Opponent obstacle added, size of list was 0 and is 1 now\n");
    }
    else{
        //fprintf(stderr,"Realloc problem 1 with obstacle number = %d \n",myForce.obstacleNumber);
        myForce.obstacleList = realloc(myForce.obstacleList,sizeof(obstacle)*(myForce.obstacleNumber+1));
        myForce.obstacleList[myForce.obstacleNumber] = myObstacle;
        myForce.obstacleNumber ++;
    }
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

void updateOpponentObstacle(){
    pthread_mutex_lock(&lockFilteredOpponent);
    myForce.obstacleList[myForce.movingIndexes[0]].posX = *myFilteredOpponent.x;
    myForce.obstacleList[myForce.movingIndexes[0]].posY = *myFilteredOpponent.y;
    pthread_mutex_unlock(&lockFilteredOpponent);
}

void addRectangleObstacle(double x1, double y1, double x2, double y2, uint8_t moving, int obstacleID){
    obstacle myObstacle;
    myObstacle.isRectangle = 1;
    myObstacle.x1 = x1;
    myObstacle.y1 = y1;
    myObstacle.x2 = x2;
    myObstacle.y2 = y2;
    myObstacle.isRectangle = 1;
    myObstacle.moving = moving;
    myObstacle.obstacleID = obstacleID;
    if(myForce.obstacleNumber == 0){
        myForce.obstacleList = (obstacle*) malloc(sizeof(obstacle));
        myForce.obstacleList[0] = myObstacle;
        myForce.obstacleNumber ++;
        fprintf(stderr,"Obstacle added, size of list was 0 and is %d now\n", myForce.obstacleNumber);
    }
    else{
        //fprintf(stderr,"Realloc problem 1 with obstacle number = %d \n",myForce.obstacleNumber);
        myForce.obstacleList = realloc(myForce.obstacleList,sizeof(obstacle)*(myForce.obstacleNumber+1));
        myForce.obstacleList[myForce.obstacleNumber] = myObstacle;
        myForce.obstacleNumber ++;
        fprintf(stderr,"Obstacle added, size of list was %d and is %d now\n",myForce.obstacleNumber-1, myForce.obstacleNumber);
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
    fprintf(stderr,"Obstacle added, at the end, size of list is %d now\n", myForce.obstacleNumber);
}

void removeObstacle(int obstacleID){
    printf("remove obstacle #%d \n",obstacleID);
    int j = 0;
    while (myForce.obstacleList[j].obstacleID != obstacleID) {
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
    fprintf(stderr,"Obstacle removed, size of list is %d now\n", myForce.obstacleNumber);
}

uint8_t turningMove = 0;

void printObstacleLists(){
    fprintf(stderr,"There are %d obstacles in the list\n",myForce.obstacleNumber);
    for (int i = 0; i < myForce.obstacleNumber; ++i) {
        if(myForce.obstacleList[i].isRectangle) fprintf(stderr,"Obstacle %d is at x1 = %lf, y1 = %lf x2 = %lf y2 = %lf and has size of %lf and has moving to %d\n",i,myForce.obstacleList[i].x1,myForce.obstacleList[i].x2,myForce.obstacleList[i].y1,myForce.obstacleList[i].y2,myForce.obstacleList[i].size,myForce.obstacleList[i].moving);
        else fprintf(stderr,"Obstacle %d is at x = %lf, y = %lf and has size of %lf and has moving to %d\n",i,myForce.obstacleList[i].posX,myForce.obstacleList[i].posY,myForce.obstacleList[i].size,myForce.obstacleList[i].moving);
    }
    for (int i = 0; i < myForce.movingNumber; ++i) {
        fprintf(stderr,"index of moving is %d\n",myForce.movingIndexes[i]);
    }
}
struct timeval startOfArrival, now;

void computeForceVector(){

    double nowTime = now.tv_sec + now.tv_usec/1000000;
    
    double distanceFromDest = computeEuclidianDistance(*myFilteredPos.x,*myFilteredPos.y,*destination.x,*destination.y);
    double k_mult_att;
    
    float k_att_xy = 0.5;
    float k_att_tang = 0.1;
    k_att_xy = k_att_xy * (1+ 1/(0.8+distanceFromDest)); //Rajouté pour booster la force d'attraction lorsqu'on approche de la destination
    float k_att_theta = /*0.3*/ 0.3;
    float k_repul = 0.0005;
    //double theta = *myFilteredPos.theta
    pthread_mutex_lock(&lockDestination);
    pthread_mutex_lock(&lockFilteredPosition); 



    double f_att_x = -destination_set*k_att_xy * (*myFilteredPos.x- *destination.x);
    double f_att_y = -destination_set*k_att_xy * (*myFilteredPos.y - *destination.y);
    
    double theta = *myFilteredPos.theta;
    double desiredTheta = *destination.theta;
    uint8_t sign_f_rep_x;
    uint8_t sign_f_rep_y;
    

    if(theta > 180) theta +=-360;
    if(desiredTheta > 180) desiredTheta+=-360;
    
    double error = theta-desiredTheta;
    if(error<-180){
        error += 360;
    }
    else if(error>180){
        error-=360;
    }
    if(pow(measuredSpeedX*measuredSpeedX + measuredSpeedY*measuredSpeedY,0.5)< 0.05 && fabs(error) > 15 && (*myFilteredPos.x > 0.15 || *myFilteredPos.x <1.85 || *myFilteredPos.y > 0.15 || *myFilteredPos.y <2.85)){
        turningMove = 1;
        
    }
    if(fabs(error) < 2) turningMove = 0;

    if(turningMove){
        f_att_x = 0;
        f_att_y = 0;
        k_att_theta = 0.8;
    }
    

    //if(VERBOSE) printf("theta = %f desired = %f error theta  = %f \n",theta,desiredTheta,error);
    

    if(computeEuclidianDistance(*myFilteredPos.x,*myFilteredPos.y,*destination.x,*destination.y) < 0.01 && fabs(error) < 1){
        if(startOfArrival.tv_sec == 0 && startOfArrival.tv_usec == 0){
            gettimeofday(&startOfArrival,NULL);
        }
        else if(nowTime - (startOfArrival.tv_sec + startOfArrival.tv_usec/1000000) > 0.1){
            printf("arrived at destination \n");
            arrivedAtDestination = 1;
        }
        //myControllerState = STOPPED;
        
    }
    else{
        startOfArrival.tv_sec = 0;
        startOfArrival.tv_usec = 0;
    }
    pthread_mutex_unlock(&lockFilteredPosition); 
    pthread_mutex_unlock(&lockDestination);

    // Ajustement de l'erreur pour tenir compte de la nature circulaire des angles


    double f_att_theta = -destination_set*k_att_theta * error;

    // Calcul de la sortie du contrôleur
    
    double f_repul_x = 0;
    double f_repul_y = 0;
    double tempoX;
    double tempoY;
    double distance;
    double k_reduc_repul;
    float actionDistance = 0.3;
    position tempoRectangle[2];
    position tempoPoint1, tempoPoint2;
    tempoPoint1.x = (float*) malloc(sizeof(float)*1);
    tempoPoint1.y = (float*) malloc(sizeof(float)*1);
    tempoPoint2.x = (float*) malloc(sizeof(float)*1);
    tempoPoint2.y = (float*) malloc(sizeof(float)*1);
    position myClosestObstaclePoint;
    position myClosestRobotPoint;
    myClosestObstaclePoint.x = (float*) malloc(sizeof(float)*1);
    myClosestObstaclePoint.y = (float*) malloc(sizeof(float)*1);
    myClosestRobotPoint.x = (float*) malloc(sizeof(float)*1);
    myClosestRobotPoint.y = (float*) malloc(sizeof(float)*1);

    obstacle *tempoObstacle;
    //Calcul de la force de répulsion totale
    for (int i = 0; i < myForce.obstacleNumber; ++i) {
        tempoObstacle = &myForce.obstacleList[i];
        if(tempoObstacle -> moving) actionDistance = mobileActionDistance;
        else actionDistance = fixActionDistance;


        position robotLowerCorner;
        position robotUpperCorner;
        robotLowerCorner.x = (float*) malloc(sizeof(float));
        robotLowerCorner.y = (float*) malloc(sizeof(float));
        robotUpperCorner.x = (float*) malloc(sizeof(float));
        robotUpperCorner.y = (float*) malloc(sizeof(float));
        pthread_mutex_lock(&lockFilteredPosition);
        if(forksDeployed){
        *robotLowerCorner.x = *myFilteredPos.x + (-robotLengthX/2 * cos((*myFilteredPos.theta)*DEG2RAD) + robotLengthYDeployed/2 * sin((*myFilteredPos.theta)*DEG2RAD));
        *robotLowerCorner.y = *myFilteredPos.y + (-robotLengthX/2 * sin((*myFilteredPos.theta)*DEG2RAD) - robotLengthYDeployed/2 * cos((*myFilteredPos.theta)*DEG2RAD));
        *robotUpperCorner.x = *myFilteredPos.x + (robotLengthX/2 * cos((*myFilteredPos.theta)*DEG2RAD) - robotLengthYDeployed/2 * sin((*myFilteredPos.theta))*DEG2RAD);
        *robotUpperCorner.y = *myFilteredPos.y + (robotLengthX/2 * sin((*myFilteredPos.theta)*DEG2RAD) + robotLengthYDeployed/2 * cos((*myFilteredPos.theta)*DEG2RAD));
        }
        else{
            *robotLowerCorner.x = *myFilteredPos.x + (-robotLengthX/2 * cos((*myFilteredPos.theta)*DEG2RAD) + robotLengthYUndeployed/2 * sin((*myFilteredPos.theta)*DEG2RAD));
            *robotLowerCorner.y = *myFilteredPos.y + (-robotLengthX/2 * sin((*myFilteredPos.theta)*DEG2RAD) - robotLengthYUndeployed/2 * cos((*myFilteredPos.theta)*DEG2RAD));
            *robotUpperCorner.x = *myFilteredPos.x + (robotLengthX/2 * cos((*myFilteredPos.theta)*DEG2RAD) - robotLengthYUndeployed/2 * sin((*myFilteredPos.theta)*DEG2RAD));
            *robotUpperCorner.y = *myFilteredPos.y + (robotLengthX/2 * sin((*myFilteredPos.theta)*DEG2RAD) + robotLengthYUndeployed/2 * cos((*myFilteredPos.theta)*DEG2RAD));
        }
        pthread_mutex_unlock(&lockFilteredPosition);
        position robotCorners[2] = {robotLowerCorner,robotUpperCorner};


        if(tempoObstacle->isRectangle){

            *tempoPoint1.x = tempoObstacle->x1;
            *tempoPoint1.y = tempoObstacle->y1;
            *tempoPoint2.x = tempoObstacle->x2;
            *tempoPoint2.y = tempoObstacle->y2;
            tempoRectangle[0] = tempoPoint1;
            tempoRectangle[1] = tempoPoint2;
            pthread_mutex_lock(&lockFilteredPosition);
            myClosestObstaclePoint = closestPointBetweenRectangles(tempoRectangle,robotCorners);
            myClosestRobotPoint = closestPointBetweenRectangles(robotCorners,tempoRectangle);
            distance = computeEuclidianDistance(*myClosestObstaclePoint.x, *myClosestObstaclePoint.y, *myClosestRobotPoint.x, *myClosestRobotPoint.y); //Calcul la distance
            pthread_mutex_unlock(&lockFilteredPosition);
            tempoX = *myClosestObstaclePoint.x; //Calcule la position en x
            tempoY = *myClosestObstaclePoint.y; //Calcule la position en y
            
        }
        else{
            tempoX = tempoObstacle->posX; //Calcule la position en x
            tempoY = tempoObstacle->posY; //Calcule la position en y
            position obstaclePosition;
            obstaclePosition.x = (float*) malloc(sizeof(float));
            obstaclePosition.y = (float*) malloc(sizeof(float));
            *obstaclePosition.x = tempoX;
            *obstaclePosition.y = tempoY;

            

            myClosestRobotPoint = closestPoint(robotCorners,obstaclePosition);
            
            
            pthread_mutex_lock(&lockFilteredPosition);
            distance = fabs(computeEuclidianDistance(tempoX,tempoY,*myClosestRobotPoint.x,*myClosestRobotPoint.y)-myForce.obstacleList[i].size); //Calcule la distance
            pthread_mutex_unlock(&lockFilteredPosition);
        }
        //printf("distance = %f and actionDistance = %f \n",distance,actionDistance);
        free(robotLowerCorner.x);
        free(robotLowerCorner.y);
        free(robotUpperCorner.x);
        free(robotUpperCorner.y);
        
        
        if(distance < actionDistance){
            pthread_mutex_lock(&lockFilteredPosition);
            if(*myFilteredPos.x > tempoX) sign_f_rep_x = -1;
            else sign_f_rep_x = 1;
            if(*myFilteredPos.y > tempoY) sign_f_rep_y = -1;
            else sign_f_rep_y = 1;
            if(tempoObstacle->moving) k_reduc_repul = 1;
            else{
                if(distanceFromDest < 0.15){
                    k_reduc_repul = 0.3; 
                } 
                else k_reduc_repul = 1;
            } 
            f_repul_x = f_repul_x + k_reduc_repul * k_repul * (1/(distance*distance) - 1/(actionDistance*actionDistance)) * (1/pow(distance, 3)) * (*myFilteredPos.x - tempoX);
            f_repul_y = f_repul_y + k_reduc_repul * k_repul * (1/(distance*distance) - 1/(actionDistance*actionDistance)) * (1/pow(distance, 3)) * (*myFilteredPos.y - tempoY);

            pthread_mutex_unlock(&lockFilteredPosition);

        }
    }
    free(tempoPoint1.x);
    free(tempoPoint1.y);
    free(tempoPoint2.x);
    free(tempoPoint2.y);
    free(myClosestObstaclePoint.x);
    free(myClosestObstaclePoint.y);
    free(myClosestRobotPoint.x);
    free(myClosestRobotPoint.y);
    f_tot_x = f_att_x+f_repul_x;
    f_tot_y = f_att_y + f_repul_y;
    //printf("f_repul_x = %lf f_repul_y = %lf \n",f_repul_x,f_repul_y);
    f_theta = f_att_theta;
    //fprintf(stderr,"fin du calcul de la force de répulsion totale avant boucle \n");
    //Calcul de la force de attraction totale
}

float xStart;
float yStart;
float myX;
float myY;
float myXOpponent;
float myYOpponent;
float opponentDistance;

void myPotentialFieldController(){
    double outputSpeed[3];
    if(myControllerState == MOVING /*&& destination_set == 1*/){ 
        switch(myGrabState)
        {
        case GRABBING_MOVE:
            pthread_mutex_lock(&lockFilteredOpponent);
            pthread_mutex_lock(&lockFilteredPosition);
            myX = *myFilteredPos.x;
            myY = *myFilteredPos.y;
            myXOpponent = *myFilteredOpponent.x;
            myYOpponent = *myFilteredOpponent.y;
            pthread_mutex_unlock(&lockFilteredOpponent);
            pthread_mutex_unlock(&lockFilteredPosition);

            opponentDistance = computeEuclidianDistance(myX,myY,myXOpponent,myYOpponent);
            if(opponentDistance < 0.40 /*|| arrivedAtDestination == 1*/){ 

                //S'arrête si il est bloqué par l'adversaire
                outputSpeed[0] = 0;
                outputSpeed[1] = 0;
                outputSpeed[2] = 0;

            } else{
                if(destination_set == 0){
                    xStart = *myFilteredPos.x;
                    yStart = *myFilteredPos.y;
                    destination_set = 1;
                    arrivedAtDestination = 0;
                }
                switch (myMovingSubState)
                {
                case GO_FORWARD_PLANTS:
                    if(computeEuclidianDistance(xStart,yStart,myX,myY) > 0.25){
                        myControllerState = STOPPED;
                        // destination_set = 0;
                        arrivedAtDestination = 1;
                    }else{
                        outputSpeed[0] = 0;
                        outputSpeed[1] = GRAB_SPEED;
                        outputSpeed[2] = 0;
                    }
                    break;

                case (GO_FORWARD_POTS):
                    if(computeEuclidianDistance(xStart,yStart,myX,myY) > 0.16){
                        myControllerState = STOPPED;
                        // destination_set = 0;
                        arrivedAtDestination = 1;
                        
                    }else{
                        outputSpeed[0] = 0;
                        outputSpeed[1] = GRAB_SPEED;
                        outputSpeed[2] = 0;
                    }                
                    break;
                
                case (UNSTACK_MOVE):
                        if(computeEuclidianDistance(xStart,yStart,myX,myY) > 0.175){
                            destination_set = 0;
                            arrivedAtDestination = 1;
                            myControllerState = STOPPED;
                        }else{
                            outputSpeed[0] = - 0.405 * GRAB_SPEED; 
                            outputSpeed[1] = - 0.914 * GRAB_SPEED;
                            outputSpeed[2] = 0;
                        }
                    break;

                case (Y_Align_Pots):
                    
                    if(computeEuclidianDistance(xStart,yStart,myX,myY) > 0.0904){
                        destination_set = 0;
                        arrivedAtDestination = 1;
                        myControllerState = STOPPED;
                    }else{
                        outputSpeed[0] = 0; 
                        outputSpeed[1] = GRAB_SPEED;
                        outputSpeed[2] = 0;
                    }
                    break;

                case (X_Align_Pots):
                        
                    if(computeEuclidianDistance(xStart,yStart,myX,myY) > 0.0708){
                        destination_set = 0;
                        arrivedAtDestination = 1;
                        myControllerState = STOPPED;
                    }else{
                        outputSpeed[0] = GRAB_SPEED; 
                        outputSpeed[1] = 0;
                        outputSpeed[2] = 0;
                    }
                    break;
                    
                case (GET_ALL_POTS):
            
                    if(computeEuclidianDistance(xStart,yStart,myX,myY) > 0.0708){
                        destination_set = 0;
                        arrivedAtDestination = 1;
                        myControllerState = STOPPED;
                    }else{
                        outputSpeed[0] = 0; 
                        outputSpeed[1] = GRAB_SPEED;
                        outputSpeed[2] = 0;
                    }
                    break;
                    
                case (GET_BACK_JARDINIERE):
                    
                    if(computeEuclidianDistance(xStart,yStart,myX,myY) > 0.16){
                        destination_set = 0;
                        arrivedAtDestination = 1;
                        myControllerState = STOPPED;
                    }else{
                        outputSpeed[0] = 0; 
                        outputSpeed[1] = - GRAB_SPEED;
                        outputSpeed[2] = 0;
                    }
                    break;
                }  
            }

        case (DISPLACEMENT_MOVE):
            computeForceVector();
            convertsSpeedToRobotFrame(f_tot_x,f_tot_y,f_theta,outputSpeed);
            break;
        }
    }
    else{
        outputSpeed[0] = 0;
        outputSpeed[1] = 0;
        outputSpeed[2] = 0;
    }
    processInstructionNew(outputSpeed[0],outputSpeed[1],outputSpeed[2],i2c_handle_front,i2c_handle_rear);
}

void initializeObstacles(){
    addRectangleObstacle(0,0,2,0,0,1); //Mur du bas
    addRectangleObstacle(0,0,0,3,0,2); //Mur de gauche
    addRectangleObstacle(2,0,2,3,0,3); //Mur de droite
    addRectangleObstacle(0,3,2,3,0,4); //Mur du haut
    addRectangleObstacle(0,1.05,0.145,3-1.05,0,5); //jardinières gauche
    addRectangleObstacle(2,1.05,2-0.145,3-1.05,0,6); //jardinières droite
    /*addRoundObstacle(0.5,1.50,0.125,0,11); //Zone plantes f1
    addRoundObstacle(0.7,1,0.125,0,12); //Zone plantes f2
    addRoundObstacle(0.5,2,0.125,0,13); //Zone plantes f3
    addRoundObstacle(1.3,1,0.125,0,14); //Zone plantes f4
    addRoundObstacle(1.3,2,0.125,0,15);  //Zone plantes f5
    addRoundObstacle(1.5,1.5,0.125,0,16); //Zone plantes f6*/

}