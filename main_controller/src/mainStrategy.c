#ifndef HEADERS
#include "headers.h"
#define HEADERS
#endif

uint8_t matchOver = 0;
struct timeval now;


void mainStrategy(){
    gettimeofday(&now, NULL);
    if(startOfMatch.tv_sec == 0 && startOfMatch.tv_usec == 0){
        gettimeofday(&startOfMatch, NULL);
        
    }
    timeFromStartOfMatch = now.tv_sec + now.tv_usec/1000000 - startOfMatch.tv_sec - startOfMatch.tv_usec/1000000;
    
    if(timeFromStartOfMatch > matchDuration){
        if(mySupremeState != 3){
            printf("======================================================================================\n");
            printf("-------------------------------------MATCH OVER---------------------------------------\n");
            printf("======================================================================================\n");
            mySupremeState = GAME_OVER;
        }
        mySupremeState = GAME_OVER;
    }
    switch (mySupremeState)
    {
    case WAITING_FOR_START:
        waitingStrategy();
        
        break;
    case EARNING_POINTS:
        pointsStrategy();
        updateObstaclesStatus(); //Mets a jour les obstacles en fonction de la position de l'ennemi
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
        printf("=====================================================\n");
        printf("---- Match started, going to EARNING_POINTS mode ----\n");
        printf("=====================================================\n");
    }
};

void pointsStrategy(){
    // check if it's time to leave
    float maxSpeed = 0.4;
    //fprintf(stderr,"check1\n");
    float SafetyFactor = 3.6; // 3.5-3.8 bonne valeur pour 6_pots en zone + 6 en jard + 6en zone
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
        //ON SUPPRIME LES OBSTACLES DES PLANTES PCQ OSEF ET ON REACTIVE TOUS LES MURS
        removeObstacle(11);
        removeObstacle(12);
        removeObstacle(13);
        removeObstacle(14);
        removeObstacle(15);
        removeObstacle(16);
        enableObstacle(100); enableObstacle(101); enableObstacle(102); enableObstacle(103);
        enableObstacle(200); enableObstacle(201);
        enableObstacle(300); enableObstacle(301); enableObstacle(302); enableObstacle(303); enableObstacle(304); enableObstacle(305);
        enableObstacle(400); enableObstacle(401); enableObstacle(402); enableObstacle(403); enableObstacle(404);
        destination_set = 0;
        arrivedAtDestination = 0;
        myMoveType = DISPLACEMENT_MOVE;
        if(VERBOSE)
            printf("======================Match ending, switch to RETURN_TO_BASE mode=====================\n");
            printf("Timer = %f\n", now.tv_sec + now.tv_usec/1000000 - startOfMatch.tv_sec - startOfMatch.tv_usec/1000000);
            printf("TimeNeededToGetHome = %f\n", TimeNeededToGetHome);
            printf("======================================================================================\n");
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
endZone* bestEndZone;

void returnToBaseStrategy(){
    printf(" === RETURN TO BASE ===\n");
    if(destination_set != 1){
        bestEndZone = computeBestEndZone();
        defineEndZoneDestination(bestEndZone);
        destination_set = 1;
        myControllerState = MOVING;
        resetErrorLists();
        arrivedAtDestination = 0;
        for(int i = 0; i<6; i++){
            if(potZones[i].numberOfPots == 0) removeObstacle(potZones[i].obstacleID);
            else enableObstacle(potZones[i].obstacleID);
        }
    }
    else if(arrivedAtDestination){
        mySupremeState = GAME_OVER;
        printf("======================================================================================\n");
        printf("----------------------------------ARRIVED, GAME OVER----------------------------------\n");
        printf("======================================================================================\n");
    }
    pthread_mutex_lock(&lockFilteredPosition);
    pthread_mutex_lock(&lockDestination);
    float tempoDistance = computeEuclidianDistance(*myFilteredPos.x, *myFilteredPos.y, *destination.x, *destination.y);
    if(tempoDistance < 0.2){
        if(bestEndZone->obstacleIDX != NULL) removeObstacle(bestEndZone->obstacleIDX);
        if(bestEndZone->obstacleIDY != NULL) removeObstacle(bestEndZone->obstacleIDY);
    }

    pthread_mutex_unlock(&lockFilteredPosition);
    pthread_mutex_unlock(&lockDestination);

};

void defineBestAction(){
    printf("define best action called\n");
    if(!solarDone){
        printf("ACTION CHOSEN: SOLAR_PANELS\n");
        myActionChoice = SOLAR_PANELS_ACTION;
        myGrabState = SOLAR_SET;
    }
    else{
        bestPlantZone = computeBestPlantsZone();
        if((bestPlantZone->numberOfPlants > 2 /*&& timeFromStartOfMatch < 30*/) ){
            printf("ACTION CHOSEN: PLANTS_POTS_ACTION\n");
            myActionChoice = PLANTS_POTS_ACTION;
            myGrabState = MOVE_FRONT_PLANTS;
        }
        else{
            printf("ACTION CHOSEN: PLANTS_ACTION\n");
            myActionChoice = PLANTS_ACTION;
        }
        myGrabState = MOVE_FRONT_PLANTS;
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
    if(nbrOfPots == 6){
        *destination.x = bestPotZone->pos6X;
        *destination.y = bestPotZone->pos6Y;
    }
    else{
        *destination.x = bestPotZone->pos5X;
        *destination.y = bestPotZone->pos5Y;
    }
    if (bestPotZone->zoneID == 4 || bestPotZone->zoneID == 5){
        *destination.theta = 270;
    }
    else{
        if(*myFilteredPos.y < *destination.y) {
            *destination.theta = 0;
        }else{
            *destination.theta = 180;
        }
    }
    
    pthread_mutex_unlock(&lockFilteredPosition);
}

void ChooseDropOrJardiniere(endZone* bestDropZone, jardiniere* bestJardiniere){
    pthread_mutex_lock(&lockFilteredPosition);
    float myX = *myFilteredPos.x;
    float myY = *myFilteredPos.y;
    pthread_mutex_unlock(&lockFilteredPosition);
    float distToDrop = computeEuclidianDistance(myX, myY, bestDropZone->dropPositionX, bestDropZone->dropPositionY);
    float distToJardiniere = computeEuclidianDistance(myX, myY, bestJardiniere->posX, bestJardiniere->posY);

    if (distToDrop * 2  < distToJardiniere){
        myChoice = DROP;
        defineDropDestination(bestDropZone); // la jardinère est trop loin, on va poser les plantes
    }
    else{
        myChoice = JARD;
        defineJardiniereDestination(bestJardiniere); // la jardinère est suffisamment proche, on préfère aller là
    }
}

void defineDropDestination(endZone* bestDropZone){
    pthread_mutex_lock(&lockFilteredPosition);
    *destination.x = bestDropZone->posX;
    *destination.y = bestDropZone->posY;
    *destination.theta = bestDropZone->dropPositionTheta;
    pthread_mutex_unlock(&lockFilteredPosition);
}
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



void defineSolarDestination(solarZone* bestSolarZone){
    pthread_mutex_lock(&lockFilteredPosition);
    if(*myFilteredPos.y < bestSolarZone->posY) {
        *destination.x = bestSolarZone->targetPositionUpX;
        *destination.y = bestSolarZone->targetPositionUpY;
        *destination.theta = 0;
    }
    else{
        *destination.x = bestSolarZone->targetPositionLowX;
        *destination.y = bestSolarZone->targetPositionLowY;
        *destination.theta = 0;
    }
    pthread_mutex_unlock(&lockFilteredPosition);

};

void defineEndZoneDestination(endZone* bestEndZone){
    pthread_mutex_lock(&lockFilteredPosition);
    *destination.x = bestEndZone->posX;
    *destination.y = bestEndZone->posY;
    *destination.theta = bestEndZone->posTheta;    
    pthread_mutex_unlock(&lockFilteredPosition);
};

void gameOverStrategy(){
    if (matchOver != 0){
        matchOver = 1;
        PrintMapState
    }
    myControllerState = STOPPED;
    setUpperFork(0);
    setGripperPosition(0);
}