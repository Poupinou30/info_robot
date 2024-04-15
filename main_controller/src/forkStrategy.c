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

void manageGrabbing(*plantZone bestPlantZone){
    
    pthread_mutex_lock(&filteredPositionLock);
    if(*myFilteredPos.y < bestPlantZone.posY) {
        *destination.x = bestPlantZone.targetPositionLowX;
        *destination.y = bestPlantZone.targetPositionLowY;
        *destination.theta = 0;
    }
    else{
        *destination.x = bestPlantZone.targetPositionHighX;
        *destination.y = bestPlantZone.targetPositionHighY;
        *destination.theta = 180;
    }

    pthread_mutex_unlock(&filteredPositionLock);

    //fprintf(stderr,"myGrabState = %d and actuatorsState = %d \n", myGrabState,myActuatorsState);
    char receivedData[255];
    switch (myGrabState)
    {
    case MOVE_FRONT_PLANTS:
        if(!arrivedAtDestination) myControllerState = MOVING;
        else{
            myGrabState = GRAB_PLANTS_INIT;
            myControllerState = STOPPED;
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
        // Move forward to grab the plants
        myGrabState = GRAB_PLANTS_CLOSE;
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
                myGrabState = UNSTACK_POTS_MOVE;
                done1 = 0; done2 = 0; done3 = 0;
                receivedData[0] = '\0';
                actuator_reception = 0;
                
            } 
            break;
        }
        
        break;

    case UNSTACK_POTS_MOVE:
        sleep(3);
        printf("dans unstack move\n");
        myGrabState = UNSTACK_POT_TAKE;
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

    case UNSTACK_POT_POSITIONING:
        
        myGrabState = UNSTACK_POT_DROP;
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

    case GRAB_POTS_MOVE:
        sleep(3);
        myGrabState = LIFT_POTS;

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
                myGrabState = DROP_PLANTS;
                receivedData[0] = '\0';
                actuator_reception = 0;
                done1 = 0; done2 = 0; done3 = 0;
            } 
            break;
        }
        break;

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