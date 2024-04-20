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

void manageGrabbing(plantZone* bestPlantZone, potZone* bestPotZone){

    //fprintf(stderr,"myGrabState = %d and actuatorsState = %d \n", myGrabState,myActuatorsState);
    
    switch (myGrabState)
    {
    case MOVE_FRONT_PLANTS:

        if(destination_set != 1){
            definePlantsDestination(bestPlantZone);
            destination_set = 1;
            myMoveType = DISPLACEMENT_MOVE;
            fprintf(stderr, "before moving \n");
            myControllerState = MOVING;
        }
        if(arrivedAtDestination){
            myGrabState = GRAB_PLANTS_INIT;
            myControllerState = STOPPED;
            arrivedAtDestination = 0;
            printf("move<frontPlant> done\n");
        } 
        break;
        
    case CALIB_FORK:
        switch (myActuatorsState)
        {
        case SENDING_INSTRUCTION:
            done = calibrateFork();

            //sleep(5);
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
    case GRAB_PLANTS_INIT:
        switch (myActuatorsState)
        {
        case SENDING_INSTRUCTION:
            if(!done0) done0 = setGripperPosition(0);
            if(!done1&&done0) done1 = deployForks();
            if(!done2 && done1) done2 = done1 && setLowerFork(65);
            if(!done3 && done2) done3 = done2 && setUpperFork(0);
            if(done0 && done1 && done2 && done3) myActuatorsState = WAITING_ACTUATORS;
            break;
        
        case WAITING_ACTUATORS:
        fprintf(stderr,"actuators reception = %d\n",actuator_reception);
        
            if(!actuator_reception){
                actuator_reception = UART_receive(UART_handle,receivedData);

            }
            if(actuator_reception && strcmp(receivedData,endMessage) == 0){
                if(VERBOSE) fprintf(stderr,"End message received from actuator\n");
                myActuatorsState = SENDING_INSTRUCTION;
                myGrabState = GRAB_PLANTS_MOVE;
                destination_set = 0;
                done1=0; done2=0; done3 = 0;
                receivedData[0] = '\0';
                actuator_reception = 0;   
            } 
            break;
        }
        break;

    case GRAB_PLANTS_MOVE:
        printf("grabplantMove started\n");
        if(destination_set == 0){ 
            myMoveType = GRABBING_MOVE;
            myMovingSubState = GO_FORWARD_PLANTS;
            myControllerState = MOVING;
        }
        else if (destination_set == 1 && arrivedAtDestination == 0){
            myGrabState = GRAB_PLANTS_MOVE;
            
        }
        else{
            myGrabState = GRAB_PLANTS_CLOSE;
            destination_set = 0;
            arrivedAtDestination = 0;
        }
        break;

    case GRAB_PLANTS_CLOSE:
        printf("grabplantClose started\n");
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
    
        printf("grabplantEnd started\n");
        bestPlantZone->numberOfPlants = 0; 
        switch (myActuatorsState)
        {
        case SENDING_INSTRUCTION:
            receivedData[0] = '\0';
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
                myGrabState = MOVE_FRONT_POTS;
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
            
            definePotsDestination(bestPotZone);
            destination_set = 1;
            myControllerState = MOVING;
            myMoveType = DISPLACEMENT_MOVE;
        }
        if(arrivedAtDestination){
            myGrabState = UNSTACK_POTS_MOVE;
            myControllerState = STOPPED;
            arrivedAtDestination = 0;
            destination_set = 0;
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
            if (done1) myActuatorsState = WAITING_ACTUATORS;
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
                sleep(3);
                
            } 
            break;
        }
        break;

    case UNSTACK_POT_POSITIONING: // captured vers unstacked sur le ppt
        printf("unstackPotPositioning started\n");
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
        }
        break;

    // faut ajouter la transistions ALIGN_POTS_MOVE (1st row vers aligned)
    case ALIGN_POTS_MOVE: // 1st row vers aligned sur le ppt
        printf("alignPotsMove started\n");
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
        }
        break;

    // faut ajouter la transistions GRAB_ALL_POTS (aligned vers pots ready)
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
        }
        break;

    case LIFT_POTS:
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
                myGrabState = DROP_PLANTS; // changer vers un Move_Front_jardiniere
                receivedData[0] = '\0';
                actuator_reception = 0;
                done1 = 0; done2 = 0; done3 = 0;
            } 
            break;
        }
        break;

    // ajouter le Move_Front_jardiniere (aller jusqu'en jardiniere)
    /*
    case MOVE_FRONT_JARDINIERE:
    */

    case DROP_PLANTS:
        printf("dropPlants started\n");
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
                myGrabState = DROP_ALL;
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
                myGrabState = FINISHED;
                receivedData[0] = '\0';
                actuator_reception = 0;   
            } 
            break;
        }
        break;
    case MOVE_BACK_JARDINIERE:
        if(destination_set == 0){
            myMoveType = GRABBING_MOVE;
            myMovingSubState = GET_BACK_JARDINIERE;
            myControllerState = MOVING;
        }
        else if (destination_set == 1 && arrivedAtDestination == 0){
            myGrabState = MOVE_BACK_JARDINIERE;
        }
        else{
            myGrabState = LIFT_POTS;
            destination_set = 0;
        }
        break; 

    case MOVE_FRONT_SOLAR:
        if(destination_set != 1){
            defineSolarDestination(computeBestSolarZone());
            destination_set = 1;
            myMoveType = DISPLACEMENT_MOVE;
            fprintf(stderr, "before moving \n");
            myControllerState = MOVING;
        }
        if(arrivedAtDestination){
            myGrabState = MOVE_SOLAR;
            myControllerState = STOPPED;
            arrivedAtDestination = 0;
            destination_set = 0;
            printf("move<frontPlant> done\n");
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
        break;

    default:
        break;
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