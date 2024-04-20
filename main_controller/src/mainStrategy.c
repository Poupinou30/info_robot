#ifndef HEADERS
#include "headers.h"
#define HEADERS
#endif

struct timeval now;


void mainStrategy(){
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
        myControllerState = STOPPED;

    default:
        break;
    }


};

plantZone* bestPlantZone;
potZone* bestPotZone;


void waitingStrategy(){
    myControllerState = STOPPED;
    printf("readyToGo = %d nextionStart = %d, checkstartSwitch = %d\n", readyToGo, nextionStart, checkStartSwitch());
    if(!forksCalibrated){
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
    float SafetyFactor = 1.5;
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
    float TimeNeededToGetHome = distToClosestBase * maxSpeed * SafetyFactor;
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
        /*if(destination_set != 1){
            definePlantsDestination(bestPlantZone);
            printf("destinationX = %f\n", *destination.x);
            printf("destinationY = %f\n", *destination.y);
            // destination_set = 1;
        }*/
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
        arrivedAtDestination = 0;}
    else if(arrivedAtDestination){
        mySupremeState = GAME_OVER;
    }
    

};

void defineBestAction(){
    printf("define best action called\n");
    
    bestPlantZone = computeBestPlantsZone();
    bestPotZone = computeBestPotsZone();
    if((bestPlantZone->numberOfPlants > 2 && timeFromStartOfMatch < 65) /*|| (myGrabState ==  GRAB_PLANTS_MOVE|| myGrabState == GRAB_PLANTS_CLOSE || myGrabState ==  GRAB_PLANTS_END || myGrabState == MOVE_FRONT_POTS || myGrabState == UNSTACK_POTS_MOVE || myGrabState == UNSTACK_POT_TAKE|| myGrabState == UNSTACK_POT_POSITIONING || myGrabState == UNSTACK_POT_DROP || myGrabState == GRAB_POTS_MOVE || myGrabState == ALIGN_POTS_MOVE || myGrabState == LIFT_POTS|| myGrabState == GRAB_ALL_POTS|| myGrabState == MOVE_FRONT_JARDINIERE|| myGrabState == MOVE_FORWARD_JARDINIERE|| myGrabState == DROP_PLANTS|| myGrabState ==  DROP_ALL|| myGrabState == MOVE_BACK_JARDINIERE)*/){
        myActionChoice = PLANTS_POTS_ACTION;
        myGrabState = MOVE_FRONT_PLANTS;
        
    }
    else{
    if(!solarDone){
        myActionChoice = SOLAR_PANELS_ACTION;
        myGrabState = SOLAR_SET;}
    }
    //fprintf(stderr,"check7\n");
};

void defineJardiniereDestination(jardiniere* bestJardiniere){
    pthread_mutex_lock(&lockFilteredPosition);
    *destination.x = bestJardiniere->posX;
    *destination.y = bestJardiniere->posY;
    if(*myFilteredPos.y < bestJardiniere->posY) *destination.theta = 0;
    else *destination.theta = 180;
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
    
    *destination.x = bestPotZone->posX;
    *destination.y = bestPotZone->posY;
    if(*myFilteredPos.y < bestPlantZone->posY) {
        *destination.theta = 180;
    }else{
        *destination.theta = 0;
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
    if(*myFilteredPos.y < bestPlantZone->posY) {
        *destination.theta = 180;
    }else{
        *destination.theta = 0;
    }
    pthread_mutex_unlock(&lockFilteredPosition);
};
