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
        returnToBaseStrategy();
        break;

    default:
        break;
    }


};

plantZone* bestPlantZone;

void waitingStrategy(){
    myControllerState = STOPPED;
    if((checkStartSwitch()||nextionStart) && readyToGo){
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
    float SafetyFactor = 1.5;
    gettimeofday(&now, NULL);
    endZone* bestEndZone = computeBestEndZone();
    pthread_mutex_lock(&lockFilteredPosition);
    double x = *myFilteredPos.x;
    double y = *myFilteredPos.y;
    pthread_mutex_unlock(&lockFilteredPosition);
    float distToClosestBase = computeEuclidianDistance(x, y, bestEndZone->posX, bestEndZone->posY);
    float TimeNeededToGetHome = distToClosestBase * maxSpeed * SafetyFactor;
    if(now.tv_sec + now.tv_usec/1000000 - startOfMatch.tv_sec - startOfMatch.tv_usec/1000000 > matchDuration - TimeNeededToGetHome){
        mySupremeState = RETURN_TO_BASE;
        if(VERBOSE)
            printf("Match ending, going to RETURN_TO_BASE mode\n");
            printf("Timer = %f\n", now.tv_sec + now.tv_usec/1000000 - startOfMatch.tv_sec - startOfMatch.tv_usec/1000000);
            printf("TimeNeededToGetHome = %f\n", TimeNeededToGetHome);
    }
    else{ // let's earn some points
        if(changeOfPlan){
            defineBestAction();
            changeOfPlan = 0;
            /*todo: faut set changeOfPlan quand:
            - on a fini une action
            - l'adversaire arrive avant nous à notre target
            - l'aversaire nous bloque le chemin trop longtemps
            */
        }
        actionStrategy();
    }
};

void actionStrategy(){
    switch (myActionChoice)
    {
    case PLANTS_ACTION:
        if(destination_set != 1){
            definePlantsDestination(bestPlantZone);
            destination_set = 1;
        }
        manageGrabbing(bestPlantZone);
        break;
    case PLANTS_POTS_ACTION:
        if(destination_set != 1){
            definePlantsDestination(bestPlantZone);
            destination_set = 1;
        }
        manageGrabbing(bestPlantZone);
        //todo: faut une diff dans manageGrabbing pour savoir si on est en train de prendre des pots ou juste les plantes
        break;
    case SOLAR_PANELS_ACTION:
        //TODO
        break;
    default:
        break;
    }
};

void returnToBaseStrategy(){
    defineEndZoneDestination(computeBestEndZone());
};
void defineBestAction(){
    bestPlantZone = computeBestPlantsZone();
    if(bestPlantZone->numberOfPlants > 2){
        myActionChoice = PLANTS_POTS_ACTION;
    }
    else{
        myActionChoice = SOLAR_PANELS_ACTION;
    }
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
