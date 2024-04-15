#ifndef HEADERS
#include "headers.h"
#define HEADERS
#endif

struct timeval now;

void mainStrategy(){
    switch (supremeState)
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
    if(checkStartSwitch()){
        gettimeoftheday(&startOfMatch, NULL);
        mySupremeState = EARNING_POINTS;
    }
};

void pointsStrategy(){
    gettimeoftheday(&now, NULL);
    if(now.tv_sec + now.tv_usec/1000000 - startOfMatch.tv_sec - startOfMatch.tv_usec/1000000 > matchDuration - 10){
    /*faut encore prendre en compte le temps pris pour rentrer à la base la plus proche
        -> faut une fonction pour déterminer quelle base est la plus proche parmis celles accessibles (en prennant en compte l'adversaire)
    TimeNeededToGetHome = distToClosestBase*maxSpeed*SafetyFactor 
    if(now.tv_sec + now.tv_usec/1000000 - startOfMatch.tv_sec - startOfMatch.tv_usec/1000000 > matchDuration - 10 - TimeNeededToGetHome){
        mySupremeState = RETURN_TO_BASE;
    }
    */ 
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