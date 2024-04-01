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

void manageGrabbing(){
    //fprintf(stderr,"myGrabState = %d and actuatorsState = %d \n", myGrabState,myActuatorsState);
    char receivedData[255];
    switch (myGrabState)
    {
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
                myActuatorsState = SENDING_INSTRUCTION;
                myGrabState = GRAB_PLANTS_INIT;
                
            } 
            break;
        }
        break;
    case GRAB_PLANTS_INIT:
        switch (myActuatorsState)
        {
        case SENDING_INSTRUCTION:
            done = deployForks();
            done = done && setLowerFork(65);
            done = done && setUpperFork(0);
            myActuatorsState = WAITING_ACTUATORS;
            break;
        
        case WAITING_ACTUATORS:
            if(!actuator_reception){
                actuator_reception = UART_receive(UART_handle,receivedData);}
            if(actuator_reception && strcmp(receivedData,endMessage) == 0){
                if(VERBOSE) fprintf(stderr,"End message received from actuator\n");
                myActuatorsState = SENDING_INSTRUCTION;
                myGrabState = GRAB_PLANTS_MOVE;
                
            } 
            break;
        }
        break;

    case GRAB_PLANTS_MOVE:
        // Move forward to grab the plants
        myGrabState = GRAB_PLANTS_END;
        break;

    case GRAB_PLANTS_END:
        switch (myActuatorsState)
        {
        case SENDING_INSTRUCTION:
            setGripperPosition(1);
            setUpperFork(142);
            myActuatorsState = WAITING_ACTUATORS;
            break;
        
        case WAITING_ACTUATORS:
         
            if(!actuator_reception){
               
                actuator_reception = UART_receive(UART_handle,receivedData);
                }
            //else printf("actuators_reception = %d\n",actuator_reception);
            if(actuator_reception && strcmp(receivedData,endMessage) == 0){
                if(VERBOSE) fprintf(stderr,"End message received from actuator\n");
                myActuatorsState = SENDING_INSTRUCTION;
                myGrabState = UNSTACK_POTS_MOVE;
                
            } 
            break;
        }
        
        break;

    case UNSTACK_POTS_MOVE:
        
        myGrabState = UNSTACK_POT_TAKE;
        break;


    case UNSTACK_POT_TAKE:
        switch (myActuatorsState)
        {
        case SENDING_INSTRUCTION:
            setLowerFork(125);
            sleep(3);
            myActuatorsState = WAITING_ACTUATORS;
            break;

        case WAITING_ACTUATORS:
            if(!actuator_reception){
                actuator_reception = UART_receive(UART_handle,receivedData);}
            if(actuator_reception && strcmp(receivedData,endMessage) == 0){
                if(VERBOSE) fprintf(stderr,"End message received from actuator\n");
                myActuatorsState = SENDING_INSTRUCTION;
                myGrabState = UNSTACK_POT_POSITIONING;
                
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
            setLowerFork(30);
            sleep(3);
            myActuatorsState = WAITING_ACTUATORS;
            break;

        case WAITING_ACTUATORS:
            if(!actuator_reception){
                actuator_reception = UART_receive(UART_handle,receivedData);}
            if(actuator_reception && strcmp(receivedData,endMessage) == 0){
                if(VERBOSE) fprintf(stderr,"End message received from actuator\n");
                myActuatorsState = SENDING_INSTRUCTION;
                myGrabState = GRAB_POTS_MOVE;
                
            } 
            break;
        }
        break;

    case GRAB_POTS_MOVE:

        myGrabState = LIFT_POTS;

    case LIFT_POTS:
        switch (myActuatorsState)
        {
        case SENDING_INSTRUCTION:
            setLowerFork(135);
            sleep(3);
            myActuatorsState = WAITING_ACTUATORS;
            break;

        case WAITING_ACTUATORS:
            if(!actuator_reception){
                actuator_reception = UART_receive(UART_handle,receivedData);}
            if(actuator_reception && strcmp(receivedData,endMessage) == 0){
                if(VERBOSE) fprintf(stderr,"End message received from actuator\n");
                myActuatorsState = SENDING_INSTRUCTION;
                myGrabState = DROP_PLANTS;
                
            } 
            break;
        }
        break;

    case DROP_PLANTS:
        switch (myActuatorsState)
        {
        case SENDING_INSTRUCTION:
            setGripperPosition(0);
            break;
        
        case WAITING_ACTUATORS:
            if(!actuator_reception){
                actuator_reception = UART_receive(UART_handle,receivedData);}
            if(actuator_reception && strcmp(receivedData,endMessage) == 0){
                if(VERBOSE) fprintf(stderr,"End message received from actuator\n");
                myActuatorsState = SENDING_INSTRUCTION;
                myGrabState = DROP_ALL;
                
            } 
            break;
        }       
        break;

    case DROP_ALL:
        switch (myActuatorsState)
        {
        case SENDING_INSTRUCTION:
            setLowerFork(75);
            setUpperFork(80);
            sleep(3);
            myActuatorsState = WAITING_ACTUATORS;
            break;

        case WAITING_ACTUATORS:
            if(!actuator_reception){
                actuator_reception = UART_receive(UART_handle,receivedData);}
            if(actuator_reception && strcmp(receivedData,endMessage) == 0){
                if(VERBOSE) fprintf(stderr,"End message received from actuator\n");
                myActuatorsState = SENDING_INSTRUCTION;
                myGrabState = FINISHED;
                
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