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

    default:
        break;
    }


};

plantZone* bestPlantZone;

void waitingStrategy(){
    myControllerState = STOPPED;
    if(!forksCalibrated){
        myGrabState = CALIB_FORK;
        manageGrabbing(NULL,NULL);
    }
    else if((checkStartSwitch()||nextionStart) && readyToGo){
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
    if(now.tv_sec + now.tv_usec/1000000 - startOfMatch.tv_sec - startOfMatch.tv_usec/1000000 > matchDuration - TimeNeededToGetHome){
        //fprintf(stderr,"check11\n");
        mySupremeState = RETURN_TO_BASE;
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
        if(myGrabState != FINISHED) manageGrabbing(bestPlantZone, NULL);//CHANGER  NULL PAR BESTPOTZONE
        else{
            changeOfPlan = 1;
        }
        //todo: faut une diff dans manageGrabbing pour savoir si on est en train de prendre des pots ou juste les plantes
        break;
    case PLANTS_POTS_ACTION:
        if(myGrabState != FINISHED) manageGrabbing(bestPlantZone, NULL);//CHANGER  NULL PAR BESTPOTZONE
        else{
            changeOfPlan = 1;
        }
        //todo: faut une diff dans manageGrabbing pour savoir si on est en train de prendre des pots ou juste les plantes
        break;
    case SOLAR_PANELS_ACTION: 
        printf("Solar\n");
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
        myGrabState = MOVE_FRONT_PLANTS;
        
    }
    else{
        myActionChoice = SOLAR_PANELS_ACTION;
    }
    //fprintf(stderr,"check7\n");
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
