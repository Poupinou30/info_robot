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
    }
    else{
        bestPlantZone = computeBestPlantsZone();
        if(bestPlantZone->numberOfPlants > 2){
            myActionChoice = PLANTS_POTS_ACTION;
        }
        else{
            myActionChoice = SOLAR_PANELS_ACTION;
        }
        actionStrategy();
    }
};

void actionStrategy(){
    switch (myActionChoice)
    {
    case PLANTS_ACTION:
        //TODO
        break;
    case PLANTS_POTS_ACTION:
        manageGrabbing(bestPlantZone);
        break;
    case SOLAR_PANELS_ACTION:
        //TODO
        break;
    
    default:
        break;
    }
};

void returnToBaseStrategy(){
    //TODO
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
    if(*myFilteredPos.y < bestPotZone->posY) {
        *destination.x = bestPotZone->targetPositionLowX;
        *destination.y = bestPotZone->targetPositionLowY;
        *destination.theta = 0;
    }
    else{
        *destination.x = bestPotZone->targetPositionUpX;
        *destination.y = bestPotZone->targetPositionUpY;
        *destination.theta = 180;
    }

    pthread_mutex_unlock(&lockFilteredPosition);
}
