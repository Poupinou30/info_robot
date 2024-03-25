#ifndef HEADERS
#include "headers.h"
#define HEADERS
#endif

void setLowerFork(int height){
    char buffer[10];
    sprintf(buffer, "<F-%d>", (int)height);
    UART_send(UART_handle, buffer,NULL);
}

void setUpperFork(int height){
    char buffer[10];
    sprintf(buffer, "<f-%d>", (int)height);
    UART_send(UART_handle, buffer,NULL);
}

void retractForks(){
    char buffer[10];
    sprintf(buffer, "<S-0>");
    UART_send(UART_handle, buffer,NULL);
}

void deployForks(){
    char buffer[10];
    sprintf(buffer, "<S-1>");
    UART_send(UART_handle, buffer,NULL);
}

void deployArm(){
    char buffer[10];
    sprintf(buffer, "<S-2>");
    UART_send(UART_handle, buffer,NULL);
}


void setWheelSpeed(int speed){
    char buffer[10];
    sprintf(buffer, "<W-%d>",speed);
    UART_send(UART_handle, buffer,NULL);
}

