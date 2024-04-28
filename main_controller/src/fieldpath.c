#ifndef HEADERS
#include "headers.h"
#define HEADERS
#endif

#define GRAB_SPEED 0.2
#define POT_SPEED 0.1

float fixActionDistance = 0.2;
float mobileActionDistance = 0.6;
float myDistanceList[20] = {INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY};
float myErrorList[20] = {INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY,INFINITY}; //Listes rotatives afin de faire une moyenne temporelle sur l'erreur et la distance avec la destination

void resetErrorLists(){
    for (int i = 0; i < 20; ++i) {
        myDistanceList[i] = INFINITY;
        myErrorList[i] = INFINITY;
    }

}

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
    myObstacle.obstacleEnabled = 1;
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
    myObstacle.obstacleEnabled = 1;
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
    myObstacle.obstacleEnabled = 1;
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
    //printf("Removing obstacle #%d \n",obstacleID);
    int j = 0;
    while (myForce.obstacleList[j].obstacleID != obstacleID) {
        /*for (int k = myForce.movingIndexes[j]; k < myForce.obstacleNumber-1; ++k) {
            myForce.obstacleList[k] = myForce.obstacleList[k+1];
        }
        myForce.obstacleNumber--;
        for (int k = j; k < myForce.movingNumber; ++k) {
            myForce.movingIndexes[k]--;
        }*/
        j++;
    }
    myForce.obstacleList[j].obstacleEnabled = 0;
    //myForce.movingNumber = 0;
    //myForce.obstacleList = realloc(myForce.obstacleList, sizeof(obstacle)*myForce.obstacleNumber);
    //fprintf(stderr,"Obstacle removed\n", myForce.obstacleNumber);
}

void enableObstacle(int obstacleID){
    //printf("Enabling obstacle #%d \n",obstacleID);
    int j = 0;
    while (myForce.obstacleList[j].obstacleID != obstacleID) {
        /*for (int k = myForce.movingIndexes[j]; k < myForce.obstacleNumber-1; ++k) {
            myForce.obstacleList[k] = myForce.obstacleList[k+1];
        }
        myForce.obstacleNumber--;
        for (int k = j; k < myForce.movingNumber; ++k) {
            myForce.movingIndexes[k]--;
        }*/
        j++;
    }
    myForce.obstacleList[j].obstacleEnabled = 1;
    //myForce.movingNumber = 0;
    //myForce.obstacleList = realloc(myForce.obstacleList, sizeof(obstacle)*myForce.obstacleNumber);
    //fprintf(stderr,"Obstacle enabled\n", myForce.obstacleNumber);
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

    /*Pour la force tangentielle quelles règles va-t-on utiliser?
    il faudrait quadriller la zone et définir un sens de rotation en fonction. Pour l'évitement d'un même obstacle, il faut que la force reste LA MEME. 
    Pour bien faire, il faudrait vérifier l'alignement entre l'obstacle et la destination. Si ils ne sont pas alignés, rien ne sert d'activer une force tangentielle!
    */

    pthread_mutex_lock(&lockFilteredPosition);
    float myPosX = *myFilteredPos.x;
    float myPosY = *myFilteredPos.y;
    float myTheta = *myFilteredPos.theta;
    pthread_mutex_unlock(&lockFilteredPosition);

    float distanceFromOpponent;

    double nowTime = now.tv_sec + now.tv_usec/1000000;

    float f_repul_tan_x = 0;
    float f_repul_tan_y = 0;
    float f_repul_norm_x = 0;
    float f_repul_norm_y = 0;
    
    double distanceFromDest = computeEuclidianDistance(myPosX,*myFilteredPos.y,*destination.x,*destination.y);
    float tempoDestX = *destination.x;
    float tempoDestY = *destination.y;
    float tempoDestTheta = *destination.theta;
    double k_mult_att;
    
    float k_att_xy = 0.4;
    float k_rep_tang = 0.01;
    k_att_xy = k_att_xy * (1+ 1/(0.25+distanceFromDest/4)); //Rajouté pour booster la force d'attraction lorsqu'on approche de la destination
    float k_att_theta = /*0.3*/ 0.3;
    
    float k_repul =0.00005 ;
    //double theta = myTheta
    pthread_mutex_lock(&lockDestination);
    


    printf("destination_set = %d myDestination: x = %f y = %f theta = %f \n",destination_set,*destination.x,*destination.y,*destination.theta);

    double f_att_x = -destination_set*k_att_xy * (myPosX- *destination.x);
    double f_att_y = -destination_set*k_att_xy * (myPosY - *destination.y);
    
    double theta = myTheta;
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
    if(pow(measuredSpeedX*measuredSpeedX + measuredSpeedY*measuredSpeedY,0.5)< 0.05 && fabs(error) > 15 && (myPosX > 0.25 || myPosX <1.75 || myPosY > 0.25 || myPosY <2.75)){
        turningMove = 1;
        
    }
    if(fabs(error) < 2) turningMove = 0;

    if(turningMove){
        f_att_x = 0;
        f_att_y = 0;
        k_att_theta = 0.8;
    }

    k_att_theta = k_att_theta * (1+ 3/(0.25+fabs(error)/3)); //Rajouté pour booster la force d'attraction lorsqu'on approche de la destination
    

    //if(VERBOSE) printf("theta = %f desired = %f error theta  = %f \n",theta,desiredTheta,error);


    //ROTATING LIST POUR MOYENNE
    for (int i = 0; i < 19; ++i) {
        myDistanceList[i] = myDistanceList[i+1];
        myErrorList[i] = myErrorList[i+1];
    }
    myDistanceList[19] = distanceFromDest;
    myErrorList[19] = error;

    float averageDistanceFromDest = 0;
    float averageError = 0;
    for(int i = 0; i<20; i++ ){
        averageDistanceFromDest+=myDistanceList[i];
        averageError += myErrorList[i];
    }
    averageDistanceFromDest = averageDistanceFromDest/20;
    averageError = averageError/20;
    

    if(averageDistanceFromDest < 0.0175 && fabs(averageError) < 1){
        arrivedAtDestination = 1;
        
    }
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
    position myClosestPoint;
    myClosestPoint.x = (float*) malloc(sizeof(float)*1);
    myClosestPoint.y = (float*) malloc(sizeof(float)*1);
    myClosestObstaclePoint.x = (float*) malloc(sizeof(float)*1);
    myClosestObstaclePoint.y = (float*) malloc(sizeof(float)*1);
    myClosestRobotPoint.x = (float*) malloc(sizeof(float)*1);
    myClosestRobotPoint.y = (float*) malloc(sizeof(float)*1);

    obstacle *tempoObstacle;
    //Calcul de la force de répulsion totale
    for (int i = 0; i < myForce.obstacleNumber; ++i) {
        if(myForce.obstacleList[i].obstacleEnabled){
            tempoObstacle = &myForce.obstacleList[i];
            if(tempoObstacle -> moving) actionDistance = mobileActionDistance;
            else actionDistance = fixActionDistance;


            position robotLowerCorner;
            position robotUpperCorner;
            robotLowerCorner.x = (float*) malloc(sizeof(float));
            robotLowerCorner.y = (float*) malloc(sizeof(float));
            robotUpperCorner.x = (float*) malloc(sizeof(float));
            robotUpperCorner.y = (float*) malloc(sizeof(float));
            if(forksDeployed){
            *robotLowerCorner.x = myPosX + (-robotLengthX/2 * cos((myTheta)*DEG2RAD) + robotLengthYDeployed/2 * sin((myTheta)*DEG2RAD));
            *robotLowerCorner.y = myPosY + (-robotLengthX/2 * sin((myTheta)*DEG2RAD) - robotLengthYDeployed/2 * cos((myTheta)*DEG2RAD));
            *robotUpperCorner.x = myPosX + (robotLengthX/2 * cos((myTheta)*DEG2RAD) - robotLengthYDeployed/2 * sin((myTheta))*DEG2RAD);
            *robotUpperCorner.y = myPosY + (robotLengthX/2 * sin((myTheta)*DEG2RAD) + robotLengthYDeployed/2 * cos((myTheta)*DEG2RAD));
            }
            else{
                *robotLowerCorner.x = myPosX + (-robotLengthX/2 * cos((myTheta)*DEG2RAD) + robotLengthYUndeployed/2 * sin((myTheta)*DEG2RAD));
                *robotLowerCorner.y = myPosY + (-robotLengthX/2 * sin((myTheta)*DEG2RAD) - robotLengthYUndeployed/2 * cos((myTheta)*DEG2RAD));
                *robotUpperCorner.x = myPosX + (robotLengthX/2 * cos((myTheta)*DEG2RAD) - robotLengthYUndeployed/2 * sin((myTheta)*DEG2RAD));
                *robotUpperCorner.y = myPosY + (robotLengthX/2 * sin((myTheta)*DEG2RAD) + robotLengthYUndeployed/2 * cos((myTheta)*DEG2RAD));
            }
            position robotCorners[2] = {robotLowerCorner,robotUpperCorner};


            if(tempoObstacle->isRectangle){
                /*
                *tempoPoint1.x = tempoObstacle->x1;
                *tempoPoint1.y = tempoObstacle->y1;
                *tempoPoint2.x = tempoObstacle->x2;
                *tempoPoint2.y = tempoObstacle->y2;
                tempoRectangle[0] = tempoPoint1;
                tempoRectangle[1] = tempoPoint2;

                pthread_mutex_lock(&lockFilteredPosition);
                myClosestPoint = closestPoint(tempoRectangle,myFilteredPos);
                pthread_mutex_unlock(&lockFilteredPosition);
                tempoX = *myClosestPoint.x; //Calcule la position en x
                tempoY = *myClosestPoint.y; //Calcule la position en y
                pthread_mutex_lock(&lockFilteredPosition);
                distance = fabs(computeEuclidianDistance(tempoX,tempoY,*myClosestPoint.x,*myClosestPoint.y)); //Calcule la distance
                pthread_mutex_unlock(&lockFilteredPosition);*/

                *tempoPoint1.x = tempoObstacle->x1;
                *tempoPoint1.y = tempoObstacle->y1;
                *tempoPoint2.x = tempoObstacle->x2;
                *tempoPoint2.y = tempoObstacle->y2;
                tempoRectangle[0] = tempoPoint1;
                tempoRectangle[1] = tempoPoint2;
                myClosestPoint = closestPoint(tempoRectangle,myFilteredPos);
                distance = fabs(computeEuclidianDistance(myPosX, myPosY, *myClosestPoint.x, *myClosestPoint.y)-robotLengthYUndeployed); //Calcul la distance
                //printf("distance = %f \n",distance);
                tempoX = *myClosestPoint.x; //Calcule la position en x
                tempoY = *myClosestPoint.y; //Calcule la position en y
                free(myClosestPoint.x);
                free(myClosestPoint.y);

                /*
                pthread_mutex_lock(&lockFilteredPosition);
                myClosestObstaclePoint = closestPointBetweenRectangles(tempoRectangle,robotCorners);
                printf("myClosestObstaclePoint = %f %f \n",*myClosestObstaclePoint.x,*myClosestObstaclePoint.y);
                myClosestRobotPoint = closestPointBetweenRectangles(robotCorners,tempoRectangle);
                printf("myClosestRobotPoint = %f %f \n",*myClosestRobotPoint.x,*myClosestRobotPoint.y);
                distance = computeEuclidianDistance(*myClosestObstaclePoint.x, *myClosestObstaclePoint.y, *myClosestRobotPoint.x, *myClosestRobotPoint.y); //Calcul la distance
                printf("myDistanceFromObstacle = %f \n",distance);
                pthread_mutex_unlock(&lockFilteredPosition);
                tempoX = *myClosestObstaclePoint.x; //Calcule la position en x
                tempoY = *myClosestObstaclePoint.y; //Calcule la position en y
                */
                
            }
            else{
                tempoX = tempoObstacle->posX; //Calcule la position en x
                tempoY = tempoObstacle->posY; //Calcule la position en y
                position obstaclePosition;
                obstaclePosition.x = (float*) malloc(sizeof(float));
                obstaclePosition.y = (float*) malloc(sizeof(float));
                *obstaclePosition.x = tempoX;
                *obstaclePosition.y = tempoY;

                

                //myClosestRobotPoint = closestPoint(robotCorners,obstaclePosition);
                
                
                distance = fabs(computeEuclidianDistance(tempoX,tempoY,myPosX,myPosY)-myForce.obstacleList[i].size - robotLengthYDeployed); //Calcule la distance
            }
            //printf("distance = %f and actionDistance = %f \n",distance,actionDistance);
            free(robotLowerCorner.x);
            free(robotLowerCorner.y);
            free(robotUpperCorner.x);
            free(robotUpperCorner.y);
            
            if(tempoObstacle->obstacleID == 0) distanceFromOpponent = distance;
            
            if(distance < actionDistance){
                
                if(myPosX > tempoX) sign_f_rep_x = -1;
                else sign_f_rep_x = 1;
                if(myPosY > tempoY) sign_f_rep_y = -1;
                else sign_f_rep_y = 1;

                if(tempoObstacle->obstacleID == 0) k_reduc_repul = 2;
                else{
                    if(distanceFromDest < 0.2){
                        k_reduc_repul = (distanceFromDest/0.20); //JAI CHANGE ICI APRES HOMOLOGATION
                    }
                    else k_reduc_repul = 1; 
                    
                } 
                //CALCUL FORCE TANGENTIELLE
                if((tempoObstacle->posX > 0.3&&tempoObstacle->posX < 1.7) && (tempoObstacle->posY>0.3&&tempoObstacle->posY<2.7)){ //Si l'obstacle est un obstacle central, on contourne par le sens le plus "court"
                    //printf("Calculating tangential force\n");
                    double angle = atan2(myPosY-tempoY,myPosX-tempoX);
                    double angleDiff = angle - myTheta*DEG2RAD;
                    if(angleDiff > M_PI) angleDiff -= 2*M_PI;
                    if(angleDiff < -M_PI) angleDiff += 2*M_PI;
                    f_repul_tan_x = f_repul_x + k_rep_tang * k_reduc_repul * sign_f_rep_x * sin(angleDiff) * (1/(distance) - 1/(actionDistance));
                    f_repul_tan_y = f_repul_y + k_rep_tang * k_reduc_repul * sign_f_rep_y * cos(angleDiff) * (1/(distance) - 1/(actionDistance));
                }
                else { // Si l'obstacle est contre un mur, on contourne toujours vers l'intérieur de l'arène
                    // Calculer la direction du centre de l'arène par rapport à la position actuelle du robot
                    double centerX = tempoDestX; // Assurez-vous que ce sont les coordonnées x du centre de l'arène //FINALEMENT j'essaie de me contourner vers la destination, plus smart
                    double centerY = tempoDestY; // Assurez-vous que ce sont les coordonnées y du centre de l'arène
                    double directionToCenterX = centerX - myPosX;
                    double directionToCenterY = centerY - myPosY;

                    // Calculer la tangente à la direction de l'obstacle (perpendiculaire au vecteur obstacle-robot)
                    double obstacleToRobotX = myPosX - tempoX;
                    double obstacleToRobotY = myPosY - tempoY;
                    double tangentX = -obstacleToRobotY; // Rotation de 90 degrés pour obtenir un vecteur perpendiculaire
                    double tangentY = obstacleToRobotX;

                    // Déterminer le signe de la tangente pour qu'elle pointe vers l'intérieur de l'arène
                    double dotProduct = tangentX * directionToCenterX + tangentY * directionToCenterY;
                    if (dotProduct < 0) {
                        tangentX = -tangentX; // Inverser la direction si elle pointe vers l'extérieur
                        tangentY = -tangentY;
                    }

                    // Normaliser la tangente (optionnel, dépend de si vous voulez une magnitude constante ou non)
                    double norm = sqrt(tangentX * tangentX + tangentY * tangentY);
                    tangentX /= norm;
                    tangentY /= norm;

                    // Calcul de la magnitude de la force tangentielle
                    double magnitude = k_rep_tang * (1 / distance - 1 / actionDistance);
                    // Appliquer la force tangentielle
                    f_repul_tan_x = magnitude * tangentX;
                    f_repul_tan_y = magnitude * tangentY;
                }
                //FIN CALCUL TAN

                f_repul_norm_x = k_reduc_repul * k_repul * (1/(distance) - 1/(actionDistance)) * (1/pow(distance, 3)) * (myPosX - tempoX);
                f_repul_norm_y = k_reduc_repul * k_repul * (1/(distance) - 1/(actionDistance)) * (1/pow(distance, 3)) * (myPosY - tempoY);
                f_repul_x = f_repul_x + f_repul_norm_x + f_repul_tan_x;
                f_repul_y = f_repul_y + f_repul_norm_y + f_repul_tan_y;

                printf("\n \nRepulsion force for obstacle #%d at distance = %f and position x = %f y = %f : x_normal = %f y_normal = %f x_tangent = %f y_tangent = %f totalX = %f totalY = %f\n",tempoObstacle->obstacleID,distance,tempoObstacle->posX,tempoObstacle->posY,f_repul_norm_x,f_repul_norm_y,f_repul_tan_x,f_repul_tan_y, f_repul_x,f_repul_y);




            }
    }}
    free(tempoPoint1.x);
    free(tempoPoint1.y);
    free(tempoPoint2.x);
    free(tempoPoint2.y);
    free(myClosestObstaclePoint.x);
    free(myClosestObstaclePoint.y);
    free(myClosestRobotPoint.x);
    free(myClosestRobotPoint.y);
    //Pour que le robot adverse ou les obstacles ne nous pousse pas dans le mur:
    
    if((myPosX < 0.2 && f_repul_x < 0) || (myPosX > 1.8 && f_repul_x > 0)){
        f_repul_x = 0;
    }
    if((myPosY < 0.2 && f_repul_y < 0) || (myPosY > 2.8 && f_repul_y > 0)){
        f_repul_y = 0;
    }


    f_tot_x = f_att_x+f_repul_x;
    f_tot_y = f_att_y + f_repul_y;
    printf("f_repul_x = %lf f_repul_y = %lf f_att_x = %f f_att_y = %f \n",f_repul_x,f_repul_y,f_att_x,f_att_y);
    f_theta = f_att_theta;

    pthread_mutex_lock(&lockOpponentPosition);
    //distanceFromOpponent = computeEuclidianDistance(myPosX,myPosY,*myFilteredOpponent.x,*myFilteredOpponent.y);
    pthread_mutex_unlock(&lockOpponentPosition);
    printf("distanceFromOpponent = %f\n",distanceFromOpponent);

    /*if(distanceFromOpponent < 0.4 && myGrabState == MOVE_FRONT_JARDINIERE && distanceFromDest < 0.5 && mySupremeState == EARNING_POINTS){
        f_tot_x = 0;
        f_tot_y = 0;
        f_theta = 0;
        printf("DANS LE IF DE MERDE\n");
    }*/ //USED TO REMOVE JARDINIERE REPULSION BUT THE LOGIC HAS BEEN CHANGED

}

float xStart;
float yStart;
float myX;
float myY;
float myTheta;
float myXOpponent;
float myYOpponent;
float opponentDistance;

void myPotentialFieldController(){
    double outputSpeed[3];
    if(myControllerState == MOVING){ 
        //printf("myMoveType = %d \n",myMoveType);
        switch(myMoveType)
        {
        case GRABBING_MOVE:
            
            //printf("grabbing move\n");
            pthread_mutex_lock(&lockFilteredOpponent);
            pthread_mutex_lock(&lockFilteredPosition);
            myX = *myFilteredPos.x;
            myY = *myFilteredPos.y;
            myTheta = *myFilteredPos.theta;
            myXOpponent = *myFilteredOpponent.x;
            myYOpponent = *myFilteredOpponent.y;
            pthread_mutex_unlock(&lockFilteredOpponent);
            pthread_mutex_unlock(&lockFilteredPosition);

            opponentDistance = computeEuclidianDistance(myX,myY,myXOpponent,myYOpponent);
            if(destination_set == 0){
                xStart = *myFilteredPos.x;
                yStart = *myFilteredPos.y;
                destination_set = 1;
                resetErrorLists();
                arrivedAtDestination = 0;
            }

            double dx = myXOpponent - myX;
            double dy = myYOpponent - myY;

            // Calcul de l'angle du vecteur par rapport à l'axe x
            double phi = atan2(dy, dx);

            // Conversion de theta pour l'aligner sur l'axe x
            double theta_x = M_PI / 2.0 - myTheta*RAD2DEG;

            // Calcul de l'angle relatif
            double alpha = (phi - theta_x)*RAD2DEG;


            if(opponentDistance < 0.40 /*|| arrivedAtDestination == 1*/ && (alpha > -90||alpha<90)){ 
                printf("opponent too close\n");
                //S'arrête si il est bloqué par l'adversaire
                outputSpeed[0] = 0;
                outputSpeed[1] = 0;
                outputSpeed[2] = 0;

            } else{
                
                switch (myMovingSubState)
                {
                case GO_FORWARD_PLANTS:
                    //printf("goForwardPlant\n");
                    if(computeEuclidianDistance(xStart,yStart,myX,myY) > 0.30){
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
                    if(computeEuclidianDistance(xStart,yStart,myX,myY) > 0.13){
                        myControllerState = STOPPED;
                        // destination_set = 0;
                        arrivedAtDestination = 1;
                        
                    }else{
                        outputSpeed[0] = 0;
                        outputSpeed[1] = POT_SPEED;
                        outputSpeed[2] = 0;
                    }                
                    break;
                
                case (UNSTACK_MOVE):
                        if(computeEuclidianDistance(xStart,yStart,myX,myY) > 2.8*POTWIDTH){
                            //destination_set = 0;
                            arrivedAtDestination = 1;
                            myControllerState = STOPPED;
                            //destination_set = 0;
                            arrivedAtDestination = 1;
                            
                        }else{
                            outputSpeed[0] = + 1.6 * POT_SPEED; 
                            outputSpeed[1] = - 2 * POT_SPEED;
                            outputSpeed[2] = 0;
                        }
                    break;

                case (Y_Align_Pots):
                    
                    if(computeEuclidianDistance(xStart,yStart,myX,myY) > 1.4*POTWIDTH){
                        //destination_set = 0;
                        arrivedAtDestination = 1;
                        myControllerState = STOPPED;
                    }else{
                        outputSpeed[0] = 0; 
                        outputSpeed[1] = POT_SPEED;
                        outputSpeed[2] = 0;
                    }
                    break;

                case (X_Align_Pots):
                        
                    if(computeEuclidianDistance(xStart,yStart,myX,myY) > 0.5 * POTWIDTH){
                        //destination_set = 0;
                        arrivedAtDestination = 1;
                        myControllerState = STOPPED;
                    }else{
                        outputSpeed[0] = -POT_SPEED; 
                        outputSpeed[1] = 0;
                        outputSpeed[2] = 0;
                    }
                    break;
                    
                case (GET_ALL_POTS):
            
                    if(computeEuclidianDistance(xStart,yStart,myX,myY) > 0.0708){
                        //destination_set = 0;
                        arrivedAtDestination = 1;
                        myControllerState = STOPPED;
                    }else{
                        outputSpeed[0] = 0; 
                        outputSpeed[1] = POT_SPEED;
                        outputSpeed[2] = 0;
                    }
                    break;
                case (GET_IN_JARDINIERE):
                        
                        if(computeEuclidianDistance(xStart,yStart,myX,myY) > 0.16){//  ajuster la distance
                            //destination_set = 0;
                            arrivedAtDestination = 1;
                            myControllerState = STOPPED;
                        }else{
                            outputSpeed[0] = 0; 
                            outputSpeed[1] = GRAB_SPEED;
                            outputSpeed[2] = 0;
                        }
                        break;
                    
                case (GET_BACK_JARDINIERE):
                    
                    if(computeEuclidianDistance(xStart,yStart,myX,myY) > 0.16){ // ajouter la distance
                        //destination_set = 0;
                        arrivedAtDestination = 1;
                        myControllerState = STOPPED;
                    }else{
                        outputSpeed[0] = 0; 
                        outputSpeed[1] = - GRAB_SPEED;
                        outputSpeed[2] = 0;
                    }
                    break;
                case (SOLARMOVE):
                    //printf("euclidian distance = %f\n",computeEuclidianDistance(xStart,yStart,myX,myY));
                    if(computeEuclidianDistance(xStart,yStart,myX,myY) > 1.32){
                        //destination_set = 0;
                        arrivedAtDestination = 1;
                        myControllerState = STOPPED;
                        solarDone = 1;
                    }else{
                        if(myTeamColor == 0)
                        {
                            outputSpeed[0] = 0; 
                            outputSpeed[1] = - POT_SPEED/2;
                            outputSpeed[2] = 0;
                        }
                        else
                        {
                            outputSpeed[0] = 0; 
                            outputSpeed[1] = +POT_SPEED/2;
                            outputSpeed[2] = 0;
                        }
                    }
                    break;
                }  
            }
            break;

        case (DISPLACEMENT_MOVE):
            //printf("displacement move\n");
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

    //MUR DU BAS
    addRectangleObstacle(0,0,baseSize,0,0,100); //Mur du bas partie1 base 1
    addRectangleObstacle(baseSize,0,baseSize+jardiniereLength,0,0,101); // Mur du bas jardiniere 1
    addRectangleObstacle(baseSize+jardiniereLength,0,2*baseSize+jardiniereLength,0,0,102); // Mur du bas base jaune centre
    addRectangleObstacle(baseSize+2*jardiniereLength,0,2*baseSize+2*jardiniereLength,0,0,103); //jardiniere2
    addRectangleObstacle(2-baseSize,0,2,0,0,104); //Base de droite

    //MUR DE DROITE
    addRectangleObstacle(2,0,2,baseSize,0,200); //0 et base
    addRectangleObstacle(2,baseSize,2,3-baseSize,0,201); //Entre deux bases
    addRectangleObstacle(2,3-baseSize,2,3,0,202); //base 2

    //MUR DU GAUCHE
    addRectangleObstacle(0,0,0,baseSize,0,300); //0 et base
    addRectangleObstacle(0,baseSize,0,0.6,0,301); //Entre base et jardiniere
    addRectangleObstacle(0,0.6,0,0.6+jardiniereLength,0,302); //jardiniere1
    addRectangleObstacle(0,0.6+jardiniereLength,0,3-(0.6+jardiniereLength),0,303); //Entre jardinieres
    addRectangleObstacle(0,3-(0.6+jardiniereLength),0,3-0.6,0,304); //jardiniere2
    addRectangleObstacle(0,3-0.6,0,3-baseSize,0,305); //Entre jardinière et base haut gauche
    addRectangleObstacle(0,3-baseSize,0,3,0,306); //base haut gauche

    //MUR DU HAUT
    addRectangleObstacle(0,3,baseSize,3,0,400); //Mur du haut base1
    addRectangleObstacle(baseSize,3,baseSize+jardiniereLength,3,0,401); // Mur du haut jardiniere 1
    addRectangleObstacle(baseSize+jardiniereLength,3,2*baseSize+jardiniereLength,3,0,402); // Mur du haut base jaune centre
    addRectangleObstacle(baseSize+2*jardiniereLength,3,2*baseSize+2*jardiniereLength,3,0,403); //MUR DU HAUT JARDINIERE 2
    addRectangleObstacle(2-baseSize,3,2,3,0,404); //Base de droite

    
    addRoundObstacle(0.5,1.50,0.125,0,11); //Zone plantes f1
    addRoundObstacle(0.7,1,0.125,0,12); //Zone plantes f2
    addRoundObstacle(0.5,2,0.125,0,13); //Zone plantes f3
    addRoundObstacle(1.3,1,0.125,0,14); //Zone plantes f4
    addRoundObstacle(1.3,2,0.125,0,15);  //Zone plantes f5
    addRoundObstacle(1.5,1.5,0.125,0,16); //Zone plantes f6*/

    addRoundObstacle(0.6125,0,0.125,0,21); //Zone POT f1
    addRoundObstacle(0.60125,3,0.125,0,22); //Zone POT f2
    addRoundObstacle(1.3875,0,0.125,0,23); //Zone pot f3
    addRoundObstacle(1.3875,3,0.125,0,24); //Zone pot f4
    addRoundObstacle(2,1,0.125,0,25);  //Zone pot f5
    addRoundObstacle(2,2,0.125,0,26); //Zone pot f6*/
    /*removeObstacle(11);
    removeObstacle(12);
    removeObstacle(13);
    removeObstacle(14);removeObstacle(15);
    removeObstacle(16);*/

}