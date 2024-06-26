#ifndef HEADERS
#include "headers.h"
#define HEADERS
#endif

char empty[] = "";
char* endMessage = "<stopped>";
char confirmMessage[100] = "<Command received :";


grabbingState myGrabState;
actuationState myActuatorsState;
uint8_t actuator_reception;
int done = 0;
uint8_t done0 = 0, done1 = 0, done2 = 0, done3 = 0, done4 = 0;
char receivedData[255];
potZone* bestPotZone;
jardiniere* bestJardiniere;
endZone* bestDropZone;

struct timeval commandSended;
struct timeval actualTime;
uint8_t timeStarted = 0;

double computeTimeElapsed(struct timeval start, struct timeval end){ //en ms
    double timeElapsed = (end.tv_sec - start.tv_sec) * 1000.0;      // sec to ms
    timeElapsed += (end.tv_usec - start.tv_usec) / 1000.0;   // us to ms
    return timeElapsed;
}


void manageGrabbing(plantZone* bestPlantZone){

    //fprintf(stderr,"myGrabState = %d and actuatorsState = %d \n", myGrabState,myActuatorsState);
    
    switch (myGrabState)
    {  
    case CALIB_FORK:
        printf("Calibrating\n");
        switch (myActuatorsState)
        {
        case SENDING_INSTRUCTION:
            done = calibrateFork();
            if(!timeStarted){
                timeStarted = 1;
                gettimeofday(&commandSended,NULL);
            }
            else{
                gettimeofday(&actualTime,NULL);
                if(computeTimeElapsed(commandSended,actualTime) > 200){
                    done = 1;
                    timeStarted = 0;
                }
            }
            if(done) myActuatorsState = WAITING_ACTUATORS;
            done = 0;
            break;
    
        case WAITING_ACTUATORS:
            //fprintf(stderr,"received message = %s \n",receivedData);
            if(!actuator_reception){
                //printf("dans if actuator reception\n");
                actuator_reception = UART_receive(UART_handle,receivedData);}
            if(strlen(receivedData) != 0) fprintf(stderr,"condition = %d and string1 = %s and string 2 = %s \n", (strcmp(receivedData,endMessage) == 0),receivedData,endMessage);
            if(strcmp(receivedData,endMessage) == 0){
                if(VERBOSE) fprintf(stderr,"End message received from actuator\n");
                actuator_reception = 0;
                myActuatorsState = SENDING_INSTRUCTION;
                myGrabState = FINISHED;
                receivedData[0] = '\0';
                forksCalibrated = 1;
            } 
            break;
        }
        break;

    case MOVE_FRONT_PLANTS:
        
        if(destination_set != 1){
            printf("moveFrontPlants started\n");
            definePlantsDestination(bestPlantZone);
            removeObstacle(bestPlantZone->obstacleID);
            destination_set = 1;
            resetErrorLists();
            myMoveType = DISPLACEMENT_MOVE;
            myControllerState = MOVING;
        }
        if (myActuatorsState == SENDING_INSTRUCTION){
            if(!done1) done1 = deployForks();
            if (myActionChoice == PLANTS_ACTION){
                if(!done2 && done1) done2 = setLowerFork(74);
            }
            else if(myActionChoice == PLANTS_POTS_ACTION){
                if(nbrOfPots == 6){
                    if(!done2 && done1) done2 = setLowerFork(68);
                }
                else{
                    if(!done2 && done1) done2 = setLowerFork(30);
                }
            }
            if(!done3 && done2) done3 = done2 && setUpperFork(0);
            if (!done4 && done3) done4 = setGripperPosition(0);
            if(done1 && done2 && done3 && done4) myActuatorsState = WAITING_ACTUATORS;
            break;
        }
        if(arrivedAtDestination && lidarAcquisitionFlag){
            myGrabState = GRAB_PLANTS_INIT;
            myControllerState = STOPPED;
            destination_set = 0;
            arrivedAtDestination = 0;
        } 
        break;

    case GRAB_PLANTS_INIT:
        if (myActuatorsState == WAITING_ACTUATORS){
            printf("grabplantInit started\n");
            if(!actuator_reception){
                actuator_reception = UART_receive(UART_handle,receivedData);
            }
            if(actuator_reception && strcmp(receivedData,endMessage) == 0){
                if(VERBOSE) fprintf(stderr,"End message received from actuator\n");
                myActuatorsState = SENDING_INSTRUCTION;
                if(mySupremeState == GAME_OVER) myGrabState = FINISHED;
                else myGrabState = GRAB_PLANTS_MOVE;
                destination_set = 0;
                done1=0; done2=0; done3 = 0;
                receivedData[0] = '\0';
                actuator_reception = 0;   
            } 
        }
        break;

    case GRAB_PLANTS_MOVE:
        if(destination_set == 0){ 
            printf("grabplantMove started\n");
            myMoveType = GRABBING_MOVE;
            myMovingSubState = GO_FORWARD_PLANTS;
            myControllerState = MOVING;
            //destination_set = 1;
        }
        else if (destination_set == 1 && arrivedAtDestination == 0){
            myGrabState = GRAB_PLANTS_MOVE;
            
        }
        else{
            // myControllerState = STOPPED;
            myGrabState = GRAB_PLANTS_CLOSE;
            destination_set = 0;
            arrivedAtDestination = 0;
        }
        break;

    case GRAB_PLANTS_CLOSE:
        switch (myActuatorsState)
        {
        case SENDING_INSTRUCTION:
            printf("grabplantClose started\n");
            if(!done1) done1 = setGripperPosition(1);
            //printf("done1 = %d done2 =%d \n",done1,done2);
            if(done1) myActuatorsState = WAITING_ACTUATORS;
            break;
        
        case WAITING_ACTUATORS:
            //printf("waiting actuators\n");
            if(!actuator_reception){
               
                actuator_reception = UART_receive(UART_handle,receivedData);
                }
            //else printf("actuators_reception = %d\n",actuator_reception);
            if(actuator_reception && strcmp(receivedData,endMessage) == 0 || 1){ //J'ai rajouté le ou 1 car parfois le stopped est envoyé trop vite et on ne le recois pas, on en a pas besoin dans ce cas
                if(VERBOSE) fprintf(stderr,"End message received from actuator\n");
                myActuatorsState = SENDING_INSTRUCTION;
                myGrabState = GRAB_PLANTS_END;
                done1 = 0; done2 = 0; done3 = 0;
                receivedData[0] = '\0';
                actuator_reception = 0;    
            } 
            break;
        }
        
        break;
        
    case GRAB_PLANTS_END:
        
        bestPlantZone->numberOfPlants = 0; 
        switch (myActuatorsState)
        {
        case SENDING_INSTRUCTION:
            printf("grabplantEnd started\n");
            if(!done1){
                done1 = setUpperFork(142);

            } 
            if(done1) myActuatorsState = WAITING_ACTUATORS;
            break;
        
        case WAITING_ACTUATORS:
            //printf("waiting actuators\n");
            if(!actuator_reception){
               
                actuator_reception = UART_receive(UART_handle,receivedData);
                }
            //else printf("actuators_reception = %d\n",actuator_reception);
            if(actuator_reception && strcmp(receivedData,endMessage) == 0){
                if(VERBOSE) fprintf(stderr,"End message received from actuator\n");
                myActuatorsState = SENDING_INSTRUCTION;
                if (myActionChoice == PLANTS_ACTION){
                    myGrabState = MOVE_FRONT_JARDINIERE;
                }else if ( myActionChoice == PLANTS_POTS_ACTION){
                    myGrabState = MOVE_FRONT_POTS;
                }
                destination_set = 0;
                done1 = 0; done2 = 0; done3 = 0;
                receivedData[0] = '\0';
                actuator_reception = 0;
                
            } 
            break;
        }
        
        break;

    case MOVE_FRONT_POTS:
        
        if(destination_set == 0){
            printf("moveFrontPots started\n");
            resetErrorLists();
            bestPotZone = computeBestPotsZone();
            definePotsDestination(bestPotZone);
            removeObstacle(bestPotZone->obstacleID);
            destination_set = 1;
            myControllerState = MOVING;
            myMoveType = DISPLACEMENT_MOVE;
        }
        else if(arrivedAtDestination){
            if (nbrOfPots ==6){
                myGrabState = UNSTACK_POTS_MOVE;
            }
            else{
                myGrabState = GRAB_POTS_MOVE;
            }
            myControllerState = STOPPED;
            arrivedAtDestination = 0;
            destination_set = 0;
        } 
        else{
            myGrabState = MOVE_FRONT_POTS;
        }
        break;

    case UNSTACK_POTS_MOVE: // ça c'est de front vers captured sur le ppt
        
        if(destination_set == 0){
            printf("unstackPotsMove started\n");
            myMoveType = GRABBING_MOVE;
            myMovingSubState = GO_FORWARD_POTS;
            myControllerState = MOVING;
            
        }
        else if (destination_set == 1 && arrivedAtDestination == 0){
            myGrabState = UNSTACK_POTS_MOVE;
            
        }
        else{

            myGrabState = UNSTACK_POT_TAKE;
            destination_set = 0;
            arrivedAtDestination = 0;
        }
        break;


    case UNSTACK_POT_TAKE: 
        
        switch (myActuatorsState)
        {
        case SENDING_INSTRUCTION:
            printf("unstackPotTake started\n");
            if(!done1) done1 = setLowerFork(125);
            if (done1) myActuatorsState = WAITING_ACTUATORS; //RAJOUTER UN TIMEOUT
            break;

        case WAITING_ACTUATORS:
            if(!actuator_reception){
                actuator_reception = UART_receive(UART_handle,receivedData);}
            if(actuator_reception && strcmp(receivedData,endMessage) == 0){
                if(VERBOSE) fprintf(stderr,"End message received from actuator\n");
                myActuatorsState = SENDING_INSTRUCTION;
                myGrabState = UNSTACK_POT_POSITIONING;
                receivedData[0] = '\0';
                actuator_reception = 0;
                done1 = 0;
            } 
            break;
        }
        break;

    case UNSTACK_POT_POSITIONING: // captured vers unstacked sur le ppt DIAGONALE
        
        if(destination_set == 0){
            printf("unstackPotPositioning started\n");        
            myMoveType = GRABBING_MOVE;
            myMovingSubState = UNSTACK_MOVE;
            myControllerState = MOVING;
        }
        else if (destination_set == 1 && arrivedAtDestination == 0){
            myGrabState = UNSTACK_POT_POSITIONING;
        }
        else{
            myGrabState = UNSTACK_POT_DROP;
            destination_set = 0;
            arrivedAtDestination = 0;
        }
        break;

    case UNSTACK_POT_DROP: 
        
        switch (myActuatorsState)
        {
        case SENDING_INSTRUCTION:
            printf("unstackPotDrop started\n");
            if(!done1) done1 = setLowerFork(30);
            if(done1) myActuatorsState = WAITING_ACTUATORS;
            break;

        case WAITING_ACTUATORS:
            if(!actuator_reception){
                actuator_reception = UART_receive(UART_handle,receivedData);}
            if(actuator_reception && strcmp(receivedData,endMessage) == 0){
                if(VERBOSE) fprintf(stderr,"End message received from actuator\n");
                myActuatorsState = SENDING_INSTRUCTION;
                myGrabState = GRAB_POTS_MOVE;
                receivedData[0] = '\0';
                actuator_reception = 0;
                done1 = 0; done2 = 0; done3 = 0;
                
            } 
            break;
        }
        break;

    case GRAB_POTS_MOVE: // unstacked vers 1st row sur le ppt
        if(destination_set == 0){
            printf("grabPotsMove started\n");
            myMoveType = GRABBING_MOVE;
            myMovingSubState = Y_Align_Pots;
            myControllerState = MOVING;
        }
        else if (destination_set == 1 && arrivedAtDestination == 0){
            myGrabState = GRAB_POTS_MOVE;
        }
        else{
            myGrabState = ALIGN_POTS_MOVE;
            destination_set = 0;
            arrivedAtDestination = 0;

        }
        break;

    case ALIGN_POTS_MOVE: // 1st row vers aligned sur le ppt
         if(destination_set == 0){
            printf("alignPotsMove started\n");
            myMoveType = GRABBING_MOVE;
            myMovingSubState = X_Align_Pots;
            myControllerState = MOVING;
        }
        else if (destination_set == 1 && arrivedAtDestination == 0){
            myGrabState = ALIGN_POTS_MOVE;
        }
        else{
            myGrabState = GRAB_ALL_POTS;
            destination_set = 0;
            arrivedAtDestination = 0;
        }
        break;

    case GRAB_ALL_POTS:
        if(destination_set == 0){
            printf("grabALLPots started\n");
            myMoveType = GRABBING_MOVE;
            myMovingSubState = GET_ALL_POTS;
            myControllerState = MOVING;
        }
        else if (destination_set == 1 && arrivedAtDestination == 0){
            myGrabState = GRAB_ALL_POTS;
        }
        else{
            myGrabState = LIFT_POTS;
            destination_set = 0;
            arrivedAtDestination = 0;
        }
        break;

    case LIFT_POTS:

        bestPotZone->numberOfPots = 0; 
        
        switch (myActuatorsState)
        {
        case SENDING_INSTRUCTION:
            printf("liftPots started\n");
            if(!done1) done1 = setLowerFork(135);
            if(done1) myActuatorsState = WAITING_ACTUATORS;
            break;

        case WAITING_ACTUATORS:
            if(!actuator_reception){
                actuator_reception = UART_receive(UART_handle,receivedData);}
            if(actuator_reception && strcmp(receivedData,endMessage) == 0){
                if(VERBOSE) fprintf(stderr,"End message received from actuator\n");
                myActuatorsState = SENDING_INSTRUCTION;
                myGrabState = MOVE_FRONT_JARDINIERE;
                receivedData[0] = '\0';
                actuator_reception = 0;
                done1 = 0; done2 = 0; done3 = 0;
            } 
            break;
        }
        break;

    case MOVE_FRONT_JARDINIERE: // choisis entre une jardiniere ou une zone de dépot !!!!!!
        if(destination_set != 1){
            bestJardiniere = computeBestJardiniere();
            bestDropZone = computeBestDropZone();
            bestStealZone = computeBestStealZone();
            ChooseDropOrJardiniere(bestDropZone,bestJardiniere);
            if (changeOfPlan){
                break;
            }
            destination_set = 1;
            resetErrorLists();
            myMoveType = DISPLACEMENT_MOVE;
            myControllerState = MOVING;
            removeObstacle(bestJardiniere->obstacleID); // On retire l'obstacle de la jardiniere afin de pouvoir s'y rendre
            removeObstacle(bestJardiniere->obstacleID-1);
            removeObstacle(bestJardiniere->obstacleID+1);
        }
        if(arrivedAtDestination /*&& lidarAcquisitionFlag*/){
            if (myChoice == DROP){
                myGrabState = LOWER_DROP;
            }
            else{
                if (myActionChoice == PLANTS_ACTION){
                    myGrabState = LOWER_PLANTS;
                }
                else{
                    myGrabState = DROP_PLANTS;
                } 
                enableObstacle(bestJardiniere->obstacleID); //On reactive la force de répulsion de ce mur
                enableObstacle(bestJardiniere->obstacleID-1);
                enableObstacle(bestJardiniere->obstacleID+1);
            }
            myControllerState = STOPPED;
            arrivedAtDestination = 0;
            destination_set = 0;
            printf("move<frontJardiniere> done\n");
        } 
        break;

    case MOVE_FORWARD_JARDINIERE:
        if(destination_set == 0){
            printf("moveForwardJardiniere started\n");
            myMoveType = GRABBING_MOVE;
            myMovingSubState = GET_IN_JARDINIERE;
            myControllerState = MOVING;
        }
        else if (destination_set == 1 && arrivedAtDestination == 0){
            myGrabState = MOVE_FORWARD_JARDINIERE;
        }
        else{
            if (nbrOfPots == 6){
                myGrabState = DROP_PLANTS;
            }
            else{
                myGrabState = LOWER_PLANTS;
            }
            destination_set = 0;
            arrivedAtDestination = 0;
        }

    case LOWER_PLANTS:
        switch (myActuatorsState)
        {
        case SENDING_INSTRUCTION:
            printf("LowerPlants");
            if(!done1) done1 = setUpperFork(75);
            if(done1) myActuatorsState = WAITING_ACTUATORS;
            break;

        case WAITING_ACTUATORS:
            if(!actuator_reception){
                actuator_reception = UART_receive(UART_handle,receivedData);}
            if(actuator_reception && strcmp(receivedData,endMessage) == 0){
                if(VERBOSE) fprintf(stderr,"End message received from actuator\n");
                myActuatorsState = SENDING_INSTRUCTION;
                myGrabState = DROP_PLANTS;
                receivedData[0] = '\0';
                actuator_reception = 0;
                done1 = 0; done2 = 0; done3 = 0;
                
            } 
            break;
        }
        break;
    

    case DROP_PLANTS:
        bestJardiniere->numberOfPlants = 6;
        switch (myActuatorsState)
        {
        case SENDING_INSTRUCTION:
            printf("dropPlants started\n");
            if(!done1) done1 = setGripperPosition(0);
            if(done1) myActuatorsState = WAITING_ACTUATORS;
            break;
        
        case WAITING_ACTUATORS:
            if(!actuator_reception){
                actuator_reception = UART_receive(UART_handle,receivedData);}
            if(actuator_reception && strcmp(receivedData,endMessage) == 0){
                if(VERBOSE) fprintf(stderr,"End message received from actuator\n");
                myActuatorsState = SENDING_INSTRUCTION;
                if (myActionChoice == PLANTS_ACTION){
                    myGrabState = MOVE_BACK_JARDINIERE;
                }else{
                    myGrabState = DROP_ALL;
                }
                receivedData[0] = '\0';
                actuator_reception = 0;
                done1 = 0; done2 = 0; done3 = 0;
                
            } 
            break;
        }       
        break;

    case DROP_ALL:
        switch (myActuatorsState)
        {
        case SENDING_INSTRUCTION:
            printf("dropAll started\n");
            if(!done1) done1 = setLowerFork(75);
            if(!done2) done2 = done1 && setUpperFork(80);
            if(done1 && done2) myActuatorsState = WAITING_ACTUATORS;
            break;

        case WAITING_ACTUATORS:
            if(!actuator_reception){
                actuator_reception = UART_receive(UART_handle,receivedData);}
            if(actuator_reception && strcmp(receivedData,endMessage) == 0){
                if(VERBOSE) fprintf(stderr,"End message received from actuator\n");
                myActuatorsState = SENDING_INSTRUCTION;
                myGrabState = MOVE_BACK_JARDINIERE;
                receivedData[0] = '\0';
                actuator_reception = 0;
                done1 = 0; done2 = 0; done3 = 0;
            } 
            break;
        }
        break;
    case MOVE_BACK_JARDINIERE:
        if(destination_set == 0){
            printf("moveBackJardiniere started\n");
            myMoveType = GRABBING_MOVE;
            myMovingSubState = GET_BACK_JARDINIERE;
            myControllerState = MOVING;
        }
        else if (destination_set == 1 && arrivedAtDestination == 0){
            myGrabState = MOVE_BACK_JARDINIERE;
        }
        else{
            myGrabState = FINISHED;
            destination_set = 0;
            arrivedAtDestination = 0;
            
        }
        break; 

    case LOWER_DROP:
        printf("lowerDrop\n");
        switch (myActuatorsState)
        {
        case SENDING_INSTRUCTION:
            printf("lowerDrop started\n");
            if(nbrOfPots == 6){
                if(!done1) done1 = setUpperFork(58);
            }else{
                if(!done1) done1 = setUpperFork(68);
            }
            if(!done2) done2 = done1 && setLowerFork(0);
            if(done1 && done2) myActuatorsState = WAITING_ACTUATORS;
            break;

        case WAITING_ACTUATORS:
            if(!actuator_reception){
                actuator_reception = UART_receive(UART_handle,receivedData);}
            if(actuator_reception && strcmp(receivedData,endMessage) == 0){
                if(VERBOSE) fprintf(stderr,"End message received from actuator\n");
                myActuatorsState = SENDING_INSTRUCTION;
                myGrabState = OPEN_DROP;
                receivedData[0] = '\0';
                actuator_reception = 0;
                done1 = 0; done2 = 0;
            } 
            break;
        }
        break;
        
    case OPEN_DROP:
        printf("openDrop\n");
        switch (myActuatorsState)
        {
        case SENDING_INSTRUCTION:
            printf("openDrop started\n");
            if(!done1) done1 = setGripperPosition(0);
            if(done1) myActuatorsState = WAITING_ACTUATORS;
            break;

        case WAITING_ACTUATORS:
            if(!actuator_reception){
                actuator_reception = UART_receive(UART_handle,receivedData);}
            if(actuator_reception && strcmp(receivedData,endMessage) == 0){
                if(VERBOSE) fprintf(stderr,"End message received from actuator\n");
                myActuatorsState = SENDING_INSTRUCTION;
                myGrabState = MOVE_BACK_DROP;
                receivedData[0] = '\0';
                actuator_reception = 0;
                done1 = 0; 
            } 
            break;
        }
        break;

    case MOVE_BACK_DROP:
        printf("moveBackDrop\n");
        if(destination_set == 0){
            printf("moveBackDrop started\n");
            myMoveType = GRABBING_MOVE;
            myMovingSubState = GET_BACK_DROP;
            myControllerState = MOVING;
        }
        else if (destination_set == 1 && arrivedAtDestination == 0){
            myGrabState = MOVE_BACK_DROP;
        }
        else{
            myGrabState = FINISHED;
            destination_set = 0;
            arrivedAtDestination = 0;
        }
        break;


    case MOVE_FRONT_SOLAR:
        if(destination_set != 1){
            printf("moveFrontSolar started\n");
            bestSolarZone = computeBestSolarZone();
            defineSolarDestination(bestSolarZone);
            if (bestSolarZone->zoneID == 2 || bestSolarZone->zoneID == 1){
                removeObstacle(26);
                removeObstacle(201);
                removeObstacle(202);
            }
            if (bestSolarZone->zoneID == 0 || bestSolarZone->zoneID == 1){
                removeObstacle(25);
                removeObstacle(200);
                removeObstacle(201);
            }
            destination_set = 1;
            resetErrorLists();
            myMoveType = DISPLACEMENT_MOVE;
            myControllerState = MOVING;
        }
        if(arrivedAtDestination && lidarAcquisitionFlag){
            myGrabState = WHEEL_TURN;
            myControllerState = STOPPED;
            arrivedAtDestination = 0;
            destination_set = 0;
            printf("moveFrontZolar done\n");
        } 
        break;

    case SOLAR_SET:
        switch (myActuatorsState)
        {
        case SENDING_INSTRUCTION:
            printf("solarSet started\n");
            if(!done) done = deployArm();
            if(done) myActuatorsState = WAITING_ACTUATORS;
            break;
        
        case WAITING_ACTUATORS:
            if(!actuator_reception){
                actuator_reception = UART_receive(UART_handle,receivedData);}
            if(strcmp(receivedData,endMessage) == 0){
                if(VERBOSE) fprintf(stderr,"End message received from actuator\n");
                actuator_reception = 0;
                myActuatorsState = SENDING_INSTRUCTION;
                myGrabState = MOVE_FRONT_SOLAR;
                receivedData[0] = '\0';
                done = 0;
            } 
            break;
        }
        break;

    case WHEEL_TURN:
        if(!done){
            if (myTeamColor == 0) done = setWheelSpeed(-50); 
            else done = setWheelSpeed(+50);
        } 
        if(done){
            myGrabState = MOVE_SOLAR;
            done = 0;
        } 
        break;

    case MOVE_SOLAR:
        if(destination_set == 0){
            printf("moveSolar started\n");
            myMoveType = GRABBING_MOVE;
            myMovingSubState = SOLARMOVE;
            myControllerState = MOVING;
            
        }
        else if (destination_set == 1 && arrivedAtDestination == 0){
            myGrabState = MOVE_SOLAR;
        }
        else{
            myGrabState = TURN_AROUND;
            myControllerState = STOPPED;
            arrivedAtDestination = 0;
            destination_set = 0;
            //setWheelSpeed(0);
        }

        break;
    
    case TURN_AROUND:
        if(destination_set == 0){
            printf("TurnAround started\n");
            pthread_mutex_lock(&lockFilteredPosition);
            if (myTeamColor){
                *destination.x = 1.6; /// à ajuster plus tard en "juste écarte toi"
                *destination.y = 2.6;
                *destination.theta = 0;
            }else{
                *destination.x = 1.6;
                *destination.y = 0.4;
                *destination.theta = 0;
            }
            
            pthread_mutex_unlock(&lockFilteredPosition);
            destination_set = 1;

            resetErrorLists();
            myMoveType = DISPLACEMENT_MOVE;
            myControllerState = MOVING;
        }
        else if(destination_set == 1 && arrivedAtDestination == 0){
            myGrabState = TURN_AROUND;
        }
        else{
            bestSolarZone->stateCenter = myTeamColor;
            myGrabState = END_ACTION;
            myControllerState = STOPPED;
            arrivedAtDestination = 0;
            destination_set = 0;
        }
        break;

    case RESET_SOLAR:
        if(destination_set == 0){
            printf("resetSolar Started\n");
            pthread_mutex_lock(&lockFilteredPosition);
            *destination.x = 2 - 0.225; /// à ajuster plus tard en "juste écarte toi"
            *destination.y = 3 - 0.2;
            *destination.theta = 170;
            pthread_mutex_unlock(&lockFilteredPosition);
            destination_set = 1;

            resetErrorLists();
            myMoveType = DISPLACEMENT_MOVE;
            myControllerState = MOVING;
        }
        else if(destination_set == 1 && arrivedAtDestination == 0){
            myGrabState = RESET_SOLAR;
        }
        else{
            myGrabState = FINISHED;
            arrivedAtDestination = 0;
            destination_set = 0;
        }
        break;

    case END_ACTION:
        switch (myActuatorsState)
        {
        case SENDING_INSTRUCTION:
            if(!done) done = retractForks();
            if(done) myActuatorsState = WAITING_ACTUATORS;
            break;
        
        case WAITING_ACTUATORS:
            if(!actuator_reception){
                actuator_reception = UART_receive(UART_handle,receivedData);}
            if(strcmp(receivedData,endMessage) == 0){
                if(VERBOSE) fprintf(stderr,"End message received from actuator\n");
                actuator_reception = 0;
                myActuatorsState = SENDING_INSTRUCTION;
                myGrabState = FINISHED;
                receivedData[0] = '\0';
                done = 0;
            } 
            break;
        }
        break;
    

    case FINISHED:
        printf("rentre dans finished\n");
        //setWheelSpeed(0);


        break;

    default:
        printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        printf("---------------------------------------------------------------\n");
        printf("===============================================================\n");
        printf("URGENCE URGENCE DEFAULT STATE REACHED IN FORK_STRATEGY!!!!!!!!!\n");
        printf("===============================================================\n");
        printf("---------------------------------------------------------------\n");
        printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    }
}

/*
STATE 1: --------------------------------
Deploy forks
Lift lower forks at 65mm forks plantsssss
Lower the upper fork at 0
STATE 2: --------------------------------
AVANCER POUR CHOPPER LES PLANTES 
STATE 3: --------------------------------
Close the gripper
Lift the upper fork at 142mm
STATE 4: --------------------------------
Move forward to take the stacked pot
STATE 5: --------------------------------
Lift lower fork to 125 // Lever pour prendre le pot stacké
STATE 6: 
Move to drop pot correctly
STATE 7:
Lower lower fork to 30mm // Descendre pour lacher le pot 

STATE 8:
Move to grab the other pots
STATE 9:
Lift lower fork to 135 to lift the pots under the plants
Open the gripper to release the plants in the pots
STATE 10:
Lower the lower fork at 75mm and the upper fork at 80mm drop pots
 */