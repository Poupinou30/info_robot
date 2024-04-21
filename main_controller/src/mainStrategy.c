#ifndef HEADERS
#include "headers.h"
#define HEADERS
#endif

struct timeval now;


void mainStrategy(){
    gettimeofday(&now, NULL);
    if(startOfMatch.tv_sec == 0 && startOfMatch.tv_usec == 0){
        gettimeofday(&startOfMatch, NULL);
        timeFromStartOfMatch = now.tv_sec + now.tv_usec/1000000 - startOfMatch.tv_sec - startOfMatch.tv_usec/1000000;
    }
    
    if(timeFromStartOfMatch > matchDuration){
        mySupremeState = GAME_OVER;
    }
    switch (mySupremeState)
    {
    case WAITING_FOR_START:
        waitingStrategy();
        
        break;
    case EARNING_POINTS:
        pointsStrategy();
        break;
    case RETURN_TO_BASE:
        printf("returnToBase\n");
        returnToBaseStrategy();
        break;
    case GAME_OVER:
        gameOverStrategy();
        

    default:
        break;
    }


};

plantZone* bestPlantZone;
potZone* bestPotZone;


void waitingStrategy(){
    myControllerState = STOPPED;
    printf("readyToGo = %d nextionStart = %d, checkstartSwitch = %d\n", readyToGo, nextionStart, checkStartSwitch());
    if(myGrabState != FINISHED){
        myGrabState = CALIB_FORK;
        manageGrabbing(NULL);
    }
    
    else if((!checkStartSwitch()||nextionStart) && readyToGo){
        gettimeofday(&startOfMatch, NULL);
        mySupremeState = EARNING_POINTS;
        defineInitialPosition();
        resetOdometry();
        printf("Match started, going to EARNING_POINTS mode\n");
    }
};

void pointsStrategy(){
    // check if it's time to leave
    float maxSpeed = 0.4;
    //fprintf(stderr,"check1\n");
    float SafetyFactor = 2;
    gettimeofday(&now, NULL);
    //fprintf(stderr,"check1.5\n");
    endZone* bestEndZone = computeBestEndZone();
    //fprintf(stderr,"check1.6\n");
    pthread_mutex_lock(&lockFilteredPosition);
    //fprintf(stderr,"check2\n");
    double x = *myFilteredPos.x;
    double y = *myFilteredPos.y;
    pthread_mutex_unlock(&lockFilteredPosition);
    float distToClosestBase = computeEuclidianDistance(x, y, bestEndZone->posX, bestEndZone->posY);
    //fprintf(stderr,"check3\n");
    float TimeNeededToGetHome = distToClosestBase / maxSpeed * SafetyFactor;
    //fprintf(stderr,"check10\n");
    timeFromStartOfMatch = now.tv_sec + now.tv_usec/1000000 - startOfMatch.tv_sec - startOfMatch.tv_usec/1000000;
    if(timeFromStartOfMatch > matchDuration - TimeNeededToGetHome){
        //fprintf(stderr,"check11\n");
        mySupremeState = RETURN_TO_BASE;
        removeObstacle(11);
        removeObstacle(12);
        removeObstacle(13);
        removeObstacle(14);
        removeObstacle(15);
        removeObstacle(16);
        destination_set = 0;
        if(VERBOSE)
            printf("Match ending, going to RETURN_TO_BASE mode\n");
            printf("Timer = %f\n", now.tv_sec + now.tv_usec/1000000 - startOfMatch.tv_sec - startOfMatch.tv_usec/1000000);
            printf("TimeNeededToGetHome = %f\n", TimeNeededToGetHome);
    }
    
    else{ // let's earn some points
        
        //fprintf(stderr,"check10.1\n");
        
        if(changeOfPlan){
            //fprintf(stderr,"check10.2\n");
            defineBestAction();
            changeOfPlan = 0;
            //fprintf(stderr,"check5\n");
            /*todo: faut set changeOfPlan quand:
            - on a fini une action
            - l'adversaire arrive avant nous à notre target
            - l'aversaire nous bloque le chemin trop longtemps
            */
        }
        //fprintf(stderr,"check10.3\n");
        actionStrategy();
    }
};

void actionStrategy(){
    // encore un peu éclaté, mais ça commence à ressembler à quelque chose
    switch (myActionChoice)
    {
    case PLANTS_ACTION:
        printf("myGrabstate = %d\n", myGrabState);
        if(myGrabState != FINISHED) manageGrabbing(bestPlantZone);//CHANGER  NULL PAR BESTPOTZONE
        else{
            printf("changeOfPlant 1\n");
             changeOfPlan = 1;
        }
        //todo: faut une diff dans manageGrabbing pour savoir si on est en train de prendre des pots ou juste les plantes
        break;
    case PLANTS_POTS_ACTION:
        if(myGrabState != FINISHED) manageGrabbing(bestPlantZone);//CHANGER  NULL PAR BESTPOTZONE
        else{
            printf("changeOfPlant 2\n");
            changeOfPlan = 1;
        }
        //todo: faut une diff dans manageGrabbing pour savoir si on est en train de prendre des pots ou juste les plantes
        break;
    case SOLAR_PANELS_ACTION: 
        if(myGrabState != FINISHED || forksDeployed) manageGrabbing(bestPlantZone);//CHANGER  NULL PAR BESTPOTZONE
        else{
            changeOfPlan = 1;
            printf("changeOfPlant 3\n");
        }
        break;
    default:
        break;
    }
};

void returnToBaseStrategy(){
    if(destination_set != 1){
        endZone* bestEndZone = computeBestEndZone();
        defineEndZoneDestination(bestEndZone);
        destination_set = 1;
        resetErrorLists();
        arrivedAtDestination = 0;
        for(int i = 0; i<6; i++){
            if(potZones[i].numberOfPots == 0) removeObstacle(potZones[i].obstacleID);
            else enableObstacle(potZones[i].obstacleID);
        }}
    else if(arrivedAtDestination){
        mySupremeState = GAME_OVER;
        printf("--------------ARRIVED, GAME OVER---------------\n");
    }
    pthread_mutex_lock(&lockFilteredPosition);
    pthread_mutex_lock(&lockDestination);
    float tempoDistance = computeEuclidianDistance(*myFilteredPos.x, *myFilteredPos.y, *destination.x, *destination.y);
    if(tempoDistance < 0.3){
        removeObstacle(1);
        removeObstacle(2);
        removeObstacle(3);
        removeObstacle(4);
    }
    else{
        enableObstacle(1);
        enableObstacle(2);

    }
    pthread_mutex_unlock(&lockFilteredPosition);
    pthread_mutex_unlock(&lockDestination);

};

void defineBestAction(){
    printf("define best action called\n");
    
    bestPlantZone = computeBestPlantsZone();
    bestPotZone = computeBestPotsZone();
    if((bestPlantZone->numberOfPlants > 2 /*&& timeFromStartOfMatch < 20*/) ){
        printf("ATTENTION, ON REPASSE A MOVE_FRONT_PLANTS\n");
        myActionChoice = PLANTS_POTS_ACTION;
        myGrabState = MOVE_FRONT_PLANTS;
        
    }
    else{
        bestPlantZone = computeBestPlantsZone();
        myActionChoice = PLANTS_ACTION;
        myGrabState = MOVE_FRONT_PLANTS;
        
    /*if(!solarDone){
        myActionChoice = SOLAR_PANELS_ACTION;
        myGrabState = SOLAR_SET;}*/
    }
    //fprintf(stderr,"check7\n");
};

void defineJardiniereDestination(jardiniere* bestJardiniere){
    pthread_mutex_lock(&lockFilteredPosition);
    *destination.x = bestJardiniere->posX;
    *destination.y = bestJardiniere->posY;
    if((bestJardiniere->zoneID == 0) || ( bestJardiniere->zoneID == 3)){
        *destination.theta = 90;
    }else{
        if(*myFilteredPos.y < bestJardiniere->posY){
            *destination.theta = 0;
        } 
        else *destination.theta = 180;
    }
    pthread_mutex_unlock(&lockFilteredPosition);
};


void definePlantsDestination(plantZone* bestPlantZone){
    pthread_mutex_lock(&lockFilteredPosition);
    if(*myFilteredPos.y < bestPlantZone->posY) {
        *destination.x = bestPlantZone->targetPositionLowX;
        *destination.y = bestPlantZone->targetPositionLowY;
        *destination.theta = 0;
    }
    else{
        *destination.x = bestPlantZone->targetPositionUpX;
        *destination.y = bestPlantZone->targetPositionUpY;
        *destination.theta = 180;
    }
    pthread_mutex_unlock(&lockFilteredPosition);
};

void definePotsDestination(potZone* bestPotZone){
    pthread_mutex_lock(&lockFilteredPosition);
    if(nbrOfPots == 6){
        *destination.x = bestPotZone->pos6X;
        *destination.y = bestPotZone->pos6Y;
    }
    else{
        *destination.x = bestPotZone->pos5X;
        *destination.y = bestPotZone->pos5Y;
    }
    if(*myFilteredPos.y < *destination.y) {
        *destination.theta = 0;
    }else{
        *destination.theta = 180;
    }
    pthread_mutex_unlock(&lockFilteredPosition);
}

void defineSolarDestination(solarZone* bestSolarZone){
    printf("before targetPositon\n");
    pthread_mutex_lock(&lockFilteredPosition);
    pthread_mutex_lock(&lockDestination);
    if(*myFilteredPos.y < bestSolarZone->posY) {
        //printf("dans le if 1\n");
        *destination.x = bestSolarZone->targetPositionUpX;
        *destination.y = bestSolarZone->targetPositionUpY;
        *destination.theta = 0;
        //printf("fin le if 1\n");
    }
    else{
        *destination.x = bestSolarZone->targetPositionLowX;
        *destination.y = bestSolarZone->targetPositionLowY;
        *destination.theta = 0;
    }
    pthread_mutex_unlock(&lockDestination);
    pthread_mutex_unlock(&lockFilteredPosition);

};

void defineEndZoneDestination(endZone* bestEndZone){
    pthread_mutex_lock(&lockFilteredPosition);
    *destination.x = bestEndZone->posX;
    *destination.y = bestEndZone->posY;
    if(*myFilteredPos.y < 1.5) {
        *destination.theta = 0;
    }else{
        *destination.theta = 180;
    }
    pthread_mutex_unlock(&lockFilteredPosition);
};

void gameOverStrategy(){
    myControllerState = STOPPED;
    if(myGrabState != FINISHED){
        myGrabState = GRAB_PLANTS_INIT;
        manageGrabbing(NULL);
    }
    
}