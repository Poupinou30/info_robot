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
uint8_t done0 = 0, done1 = 0, done2 = 0, done3 = 0;
char receivedData[255];
potZone* bestPotZone;
jardiniere* bestJardiniere;
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
                    if(!done2 && done1) done2 = setLowerFork(69);
                }
                else{
                    if(!done2 && done1) done2 = setLowerFork(30);
                }
            }
            if(!done3 && done2) done3 = done2 && setUpperFork(0);
            if(done1 && done2 && done3) myActuatorsState = WAITING_ACTUATORS;
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
        //printf("grabplantMove started\n");
        if(destination_set == 0){ 
            myMoveType = GRABBING_MOVE;
            myMovingSubState = GO_FORWARD_PLANTS;
            myControllerState = MOVING;
            //destination_set = 1;
        }
        else if (destination_set == 1 && arrivedAtDestination == 0){
            myGrabState = GRAB_PLANTS_MOVE;
            
        }
        else{
            myControllerState = STOPPED;
            myGrabState = GRAB_PLANTS_CLOSE;
            destination_set = 0;
            arrivedAtDestination = 0;
        }
        break;

    case GRAB_PLANTS_CLOSE:
        //printf("grabplantClose started\n");
        switch (myActuatorsState)
        {
        case SENDING_INSTRUCTION:
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
    
        //printf("grabplantEnd started\n");
        bestPlantZone->numberOfPlants = 0; 
        switch (myActuatorsState)
        {
        case SENDING_INSTRUCTION:
            //receivedData[0] = '\0';
            if(!done1){
                //printf("rentre dans le if setUpperFork\n");
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

    case MOVE_FRONT_POTS: // else *****
        printf("moveFrontPots started\n");
        if(destination_set == 0){
            printf("dans if 1 \n");
            resetErrorLists();
            computeBestPotsZone();
            definePotsDestination(bestPotZone);
            removeObstacle(bestPotZone->obstacleID);
            destination_set = 1;
            myControllerState = MOVING;
            myMoveType = DISPLACEMENT_MOVE;
        }
        else if(arrivedAtDestination){
            printf("dans if 2 \n");
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
            printf("dans if 3 \n");
            myGrabState = MOVE_FRONT_POTS;
        }
        break;

    case UNSTACK_POTS_MOVE: // ça c'est de front vers captured sur le ppt
        printf("unstackPotsMove started, destination_set = %d, arrivedAtDestination = %d \n",destination_set,arrivedAtDestination);
        if(destination_set == 0){ 
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
        printf("unstackPotTake started\n");
        switch (myActuatorsState)
        {
        case SENDING_INSTRUCTION:
            printf("dans unstack pots and done = %d\n",done1);
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
                done1 = 0; done2 = 0; done3 = 0;
                actuator_reception = 0;
                //sleep(3);
                
            } 
            break;
        }
        break;

    case UNSTACK_POT_POSITIONING: // captured vers unstacked sur le ppt DIAGONALE
        printf("unstackPotPositioning started, destination_set = %d arrivedAtDestination = %d\n",destination_set,arrivedAtDestination);
        
        if(destination_set == 0){
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
        printf("unstackPotDrop started\n");
        switch (myActuatorsState)
        {
        case SENDING_INSTRUCTION:
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
        printf("grabPotsMove started\n");
        if(destination_set == 0){
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
        printf("alignPotsMove started and destination_set = %d arrivedAtDestination = %d\n", destination_set,arrivedAtDestination);
        if(destination_set == 0){
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
        printf("grabAllPots started\n");
        if(destination_set == 0){
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
        printf("liftPots started\n");
        switch (myActuatorsState)
        {
        case SENDING_INSTRUCTION:
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

    case MOVE_FRONT_JARDINIERE:
        if(destination_set != 1){
            bestJardiniere = computeBestJardiniere();
            defineJardiniereDestination(bestJardiniere);
            printf("Destination jardiniere defined at x = %f and y = %f \n",bestJardiniere->posX,bestJardiniere->posY);
            pthread_mutex_lock(&lockFilteredPosition);
            printf("my position is x = %f and y = %f \n",*myFilteredPos.x, *myFilteredPos.y);
            pthread_mutex_unlock(&lockFilteredPosition);
            destination_set = 1;
            resetErrorLists();
            myMoveType = DISPLACEMENT_MOVE;
            fprintf(stderr, "Destination jardiniere defined at x = %f and y = %f \n",bestJardiniere->posX,bestJardiniere->posY);
            myControllerState = MOVING;
            removeObstacle(bestJardiniere->obstacleID); // On retire l'obstacle de la jardiniere afin de pouvoir s'y rendre
            removeObstacle(bestJardiniere->obstacleID-1);
            removeObstacle(bestJardiniere->obstacleID+1);
        }
        if(arrivedAtDestination /*&& lidarAcquisitionFlag*/){
            if (myActionChoice == PLANTS_ACTION){
                myGrabState = LOWER_PLANTS;
            }
            else{
                myGrabState = DROP_PLANTS;
            } 
            myControllerState = STOPPED;
            enableObstacle(bestJardiniere->obstacleID); //On reactive la force de répulsion de ce mur
            enableObstacle(bestJardiniere->obstacleID-1);
            enableObstacle(bestJardiniere->obstacleID+1);
            arrivedAtDestination = 0;
            destination_set = 0;
            printf("move<frontJardiniere> done\n");
        } 
        break;

    case MOVE_FORWARD_JARDINIERE:
        printf("moveForwardJardiniere started\n");
        if(destination_set == 0){
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
        printf("LowerPlants");
        switch (myActuatorsState)
        {
        case SENDING_INSTRUCTION:
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
        printf("dropPlants started\n");
        bestJardiniere->numberOfPlants = 6;
        switch (myActuatorsState)
        {
        case SENDING_INSTRUCTION:
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
        printf("dropAll started\n");
        switch (myActuatorsState)
        {
        case SENDING_INSTRUCTION:
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
                done = 0;
                done1 = 0; done2 = 0; done3 = 0;
            } 
            break;
        }
        break;
    case MOVE_BACK_JARDINIERE:
        printf("moveBackJardiniere started destination_set = %d et arrivedAtDestination = %d\n", destination_set,arrivedAtDestination);
        if(destination_set == 0){
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

    case MOVE_FRONT_SOLAR:
        printf("Dans move front solar panels et arrivedAtDestination = %d \n",arrivedAtDestination);
        if(destination_set != 1){
            defineSolarDestination(computeBestSolarZone());
            destination_set = 1;
            resetErrorLists();
            myMoveType = DISPLACEMENT_MOVE;
            fprintf(stderr, "before moving \n");
            myControllerState = MOVING;
        }
        if(arrivedAtDestination && lidarAcquisitionFlag){
            myGrabState = WHEEL_TURN;
            myControllerState = STOPPED;
            arrivedAtDestination = 0;
            destination_set = 0;
            printf("move<frontPlant> done\n");
        } 
        break;
    case SOLAR_SET:
        switch (myActuatorsState)
        {
        case SENDING_INSTRUCTION:
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
                if (myTeamColor == 0) done = setWheelSpeed(-18); 
                else done = setWheelSpeed(+18);
            } 
            if(done){
                myGrabState = MOVE_SOLAR;
                done = 0;
            } 
            break;
        


    case MOVE_SOLAR:
        if(destination_set == 0){
            myMoveType = GRABBING_MOVE;
            myMovingSubState = SOLARMOVE;
            myControllerState = MOVING;
        }
        else if (destination_set == 1 && arrivedAtDestination == 0){
            myGrabState = MOVE_SOLAR;
        }
        else{
            myGrabState = FINISHED;
            destination_set = 0;
        }
        break;
        
    case FINISHED:
        printf("rentre dans finished\n");
        setWheelSpeed(0);
        retractForks();
        break;

    default:
        printf("URGENCE URGENCE !!!!!!!!!\n");
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