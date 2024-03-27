#ifndef HEADERS
#include "headers.h"
#define HEADERS
#endif

char receivedData[255];

void setArmDeployedAngle(int angle){ //Bras du panneau solaire
    char buffer[10];
    sprintf(buffer, "<A-%d>", (int)angle);
    UART_send(UART_handle, buffer,receivedData);
    if(VERBOSE) printf("Debug from fork = %s \n",receivedData);
}

void setForksDeployedAngle(int angle){ //Bras du panneau solaire
    char buffer[10];
    sprintf(buffer, "<T-%d>", (int)angle);
    UART_send(UART_handle, buffer,receivedData);
    if(VERBOSE) printf("Debug from fork = %s \n",receivedData);
}

void disableStepperMotor(){
    char buffer[10];
    sprintf(buffer, "<D-0>");
    UART_send(UART_handle, buffer,receivedData);
    if(VERBOSE) printf("Debug from fork = %s \n",receivedData);
}

void enableStepperMotor(){
    char buffer[10];
    sprintf(buffer, "<E-0>");
    UART_send(UART_handle, buffer,receivedData);
    if(VERBOSE) printf("Debug from fork = %s \n",receivedData);
}

void setLowerFork(int height){
    char buffer[10];
    sprintf(buffer, "<F-%d>", (int)height);
    UART_send(UART_handle, buffer,receivedData);
    if(VERBOSE) printf("Debug from fork = %s \n",receivedData);
}

void setUpperFork(int height){
    char buffer[10];
    sprintf(buffer, "<f-%d>", (int)height);
    UART_send(UART_handle, buffer,receivedData);
    if(VERBOSE) printf("Debug from fork = %s \n",receivedData);
}

void retractForks(){
    char buffer[10];
    sprintf(buffer, "<S-0>");
    UART_send(UART_handle, buffer,receivedData);
    if(VERBOSE) printf("Debug from fork = %s \n",receivedData);
}

void deployForks(){
    char buffer[10];
    sprintf(buffer, "<S-1>");
    UART_send(UART_handle, buffer,receivedData);
    if(VERBOSE) printf("Debug from fork = %s \n",receivedData);
}

void deployArm(){
    char buffer[10];
    sprintf(buffer, "<S-2>");
    UART_send(UART_handle, buffer,receivedData);
    if(VERBOSE) printf("Debug from fork = %s \n",receivedData);
}


void setWheelSpeed(int speed){
    char buffer[10];
    sprintf(buffer, "<W-%d>",speed);
    UART_send(UART_handle, buffer,receivedData);
    if(VERBOSE) printf("Debug from fork = %s \n",receivedData);
}

void setGripperPosition (int position){
    /*  x = 0 : gripper open (0°)
        x = 1 : gripper closed (180°)
        2 <= x <= 180 : set servo angle
    */
    char buffer[10];
    sprintf(buffer, "<G-%d>",position);
    UART_send(UART_handle, buffer,receivedData);
    if(VERBOSE) printf("Debug from fork = %s \n",receivedData);
}


