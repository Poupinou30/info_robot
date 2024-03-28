#ifndef HEADERS
#include "headers.h"
#define HEADERS
#endif

char empty[] = "";
char* endMessage = "<stopped>";


grabbingState myGrabState;
actuationState myActuatorsState;
uint8_t actuator_reception;

void manageGrabbing(){
    char receivedData[255];
    switch (myGrabState)
    {
    case CALIB_FORK:
        switch (myActuatorsState)
        {
        case SENDING_INSTRUCTION:
            calibrateFork();
            //sleep(5);
            myActuatorsState = WAITING_ACTUATORS;
            break;
        
        case WAITING_ACTUATORS:
            if(!actuator_reception){
                actuator_reception = UART_receive(UART_handle,receivedData);}
                //printf("end message = %s \n",endMessage);}
                //waitingForReception = 1;}
            if(receivedData == endMessage){
                if(VERBOSE) printf("End message received from actuator\n");
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
            deployForks();
            setLowerFork(65);
            setUpperFork(0);
            myActuatorsState = WAITING_ACTUATORS;
            break;
        
        case WAITING_ACTUATORS:
            if(!waitingForReception){
                actuator_reception = UART_receive(UART_handle,receivedData);
                waitingForReception = 1;}
            if(actuator_reception && receivedData == endMessage){
                if(VERBOSE) printf("End message received from actuator\n");
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
            if(!waitingForReception){
                actuator_reception = UART_receive(UART_handle,receivedData);
                waitingForReception = 1;}
            if(actuator_reception && receivedData == endMessage){
                if(VERBOSE) printf("End message received from actuator\n");
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
            if(!waitingForReception){
                actuator_reception = UART_receive(UART_handle,receivedData);
                waitingForReception = 1;}
            if(actuator_reception && receivedData == endMessage){
                if(VERBOSE) printf("End message received from actuator\n");
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
            if(!waitingForReception){
                actuator_reception = UART_receive(UART_handle,receivedData);
                waitingForReception = 1;}
            if(actuator_reception && receivedData == endMessage){
                if(VERBOSE) printf("End message received from actuator\n");
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
            if(!waitingForReception){
                actuator_reception = UART_receive(UART_handle,receivedData);
                waitingForReception = 1;}
            if(actuator_reception && receivedData == endMessage){
                if(VERBOSE) printf("End message received from actuator\n");
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
            if(!waitingForReception){
                actuator_reception = UART_receive(UART_handle,receivedData);
                waitingForReception = 1;}
            if(actuator_reception && receivedData == endMessage){
                if(VERBOSE) printf("End message received from actuator\n");
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
            if(!waitingForReception){
                actuator_reception = UART_receive(UART_handle,receivedData);
                waitingForReception = 1;}
            if(actuator_reception && receivedData == endMessage){
                if(VERBOSE) printf("End message received from actuator\n");
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