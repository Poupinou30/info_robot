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
uint8_t done1 = 0, done2 = 0, done3 = 0;

void manageGrabbing(plantZone* bestPlantZone, potZone* bestPotZone){ // pourquoi tu passes bestPlantZone en argument? je vois pas où tu l'utilises

    //fprintf(stderr,"myGrabState = %d and actuatorsState = %d \n", myGrabState,myActuatorsState);
    char receivedData[255];
    switch (myGrabState)
    {
    case MOVE_FRONT_PLANTS:
        if(destination_set != 1){
            definePlantsDestination(bestPlantZone);
            destination_set = 1;
        }
        myMoveType = DISPLACEMENT_MOVE;
        if(!arrivedAtDestination) myControllerState = MOVING;
        else{
            myGrabState = GRAB_PLANTS_INIT;
            myControllerState = STOPPED;
            arrivedAtDestination = 0;
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
            //fprintf(stderr,"actuators reception = %d\n",actuator_reception);
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
                sleep(3);                
            } 
            break;
        }
        break;
    case GRAB_PLANTS_INIT:
        switch (myActuatorsState)
        {
        case SENDING_INSTRUCTION:
            if(!done1) done1 = deployForks();
            if(!done2) done2 = done1 && setLowerFork(65);
            if(!done3) done3 = done2 && setUpperFork(0);
            if(done1 && done2 && done3) myActuatorsState = WAITING_ACTUATORS;
            break;
        
        case WAITING_ACTUATORS:
            if(!actuator_reception){
                actuator_reception = UART_receive(UART_handle,receivedData);}
            if(actuator_reception && strcmp(receivedData,endMessage) == 0){
                if(VERBOSE) fprintf(stderr,"End message received from actuator\n");
                myActuatorsState = SENDING_INSTRUCTION;
                myGrabState = GRAB_PLANTS_MOVE;
                done1=0; done2=0; done3 = 0;
                receivedData[0] = '\0';
                actuator_reception = 0;
                
            } 
            break;
        }
        break;

    case GRAB_PLANTS_MOVE:
        /*
        on mettrait pas des pressForward à la place des destination_set pour faire la dif entre quand tu as un objectif en potentialField 
        et quand tu fais un move hardcodé, comme ça actionStrategy nous recherche pas une nouvelle destination ?
        */   
        if(destination_set == 0){ 
            myMoveType = GRABBING_MOVE;
            myMovingSubState = GO_FORWARD_PLANTS;
            destination_set = 1;
            arrivedAtDestination = 0;
        }
        else if (pressForward == 1 && arrivedAtDestination == 0){
            myGrabState = GRAB_PLANTS_MOVE;
            
        }
        else{
            myGrabState = GRAB_PLANTS_CLOSE;
            destination_set = 0;
            // arrivedAtDestination = 0; // jsp si il faut le mettre, en fct de comment tu le gères pour l'instant
        }
        break;

    case GRAB_PLANTS_CLOSE:
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
            if(actuator_reception && strcmp(receivedData,endMessage) == 0){
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
                myGrabState = UNSTACK_POTS_MOVE; // changer pour un MOVE_FRONT_POTS vu qu'on doit tenir en compte les move types
                done1 = 0; done2 = 0; done3 = 0;
                receivedData[0] = '\0';
                actuator_reception = 0;
                
            } 
            break;
        }
        
        break;

    case MOVE_FRONT_POTS:
        if(destination_set != 1){
            definePotsDestination(bestPotZone);
            destination_set = 1;
        }
        myMoveType = DISPLACEMENT_MOVE;
        if(!arrivedAtDestination) myControllerState = MOVING;
        else{
            myGrabState = UNSTACK_POTS_MOVE;
            myControllerState = STOPPED;
            arrivedAtDestination = 0;
        } 
        break;

    case UNSTACK_POTS_MOVE: // ça c'est de front vers captured sur le ppt
        if(destination_set == 0){ 
            myMoveType = GRABBING_MOVE;
            myMovingSubState = GO_FORWARD_POTS;
            //
            destination_set = 1;
            arrivedAtDestination = 0;
        }
        else if (destination_set == 1 && arrivedAtDestination == 0){
            myGrabState = UNSTACK_POTS_MOVE;
            
        }
        else{
            myGrabState = UNSTACK_POT_TAKE;
            destination_set = 0;
        }
        break;


    case UNSTACK_POT_TAKE: 
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
        if(destination_set == 0){
            myMoveType = GRABBING_MOVE;
            myMovingSubState = UNSTACK_MOVE;
            destination_set = 1;
            arrivedAtDestination = 0;
        }
        else if (destination_set == 1 && arrivedAtDestination == 0){
            myGrabState = UNSTACK_POT_POSITIONING;
        }
        else{
            myGrabState = UNSTACK_POT_DROP;
            destination_set = 0;
        }
        break;

    case UNSTACK_POT_DROP:
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
        if(destination_set == 0){
            myMoveType = GRABBING_MOVE;
            myMovingSubState = Y_Align_Pots;
            destination_set = 1;
            arrivedAtDestination = 0;
        }
        else if (destination_set == 1 && arrivedAtDestination == 0){
            myGrabState = GRAB_POTS_MOVE;
        }
        else{
            myGrabState = ALIGN_POTS_MOVE;
            destination_set = 0;
        }

    // faut ajouter la transistions ALIGN_POTS_MOVE (1st row vers aligned)
    case ALIGN_POTS_MOVE: // 1st row vers aligned sur le ppt
        if(destination_set == 0){
            myMoveType = GRABBING_MOVE;
            myMovingSubState = X_Align_Pots;
            destination_set = 1;
            arrivedAtDestination = 0;
        }
        else if (destination_set == 1 && arrivedAtDestination == 0){
            myGrabState = ALIGN_POTS_MOVE;
        }
        else{
            myGrabState = GRAB_ALL_POTS;
            destination_set = 0;
        }
    // faut ajouter la transistions GRAB_ALL_POTS (aligned vers pots ready)
    case GRAB_ALL_POTS:
        if(destination_set == 0){
            myMoveType = GRABBING_MOVE;
            myMovingSubState = GET_ALL_POTS;
            destination_set = 1;
            arrivedAtDestination = 0;
        }
        else if (destination_set == 1 && arrivedAtDestination == 0){
            myGrabState = GRAB_ALL_POTS;
        }
        else{
            myGrabState = LIFT_POTS;
            destination_set = 0;
        }

    case LIFT_POTS:
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
    // ajouter le MOVE_BACK_JARDINIERE (marche arrière pour faire tomber les plantes)
    case MOVE_BACK_JARDINIERE:
        if(destination_set == 0){
            myMoveType = GRABBING_MOVE;
            myMovingSubState = GET_BACK_JARDINIERE;
            destination_set = 1;
            arrivedAtDestination = 0;
        }
        else if (destination_set == 1 && arrivedAtDestination == 0){
            myGrabState = MOVE_BACK_JARDINIERE;
        }
        else{
            myGrabState = LIFT_POTS;
            destination_set = 0;
        }
        

    
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