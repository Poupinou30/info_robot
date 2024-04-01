#ifndef HEADERS
#include "headers.h"
#define HEADERS
#endif

char dataToSend[255];
char setArmDeployedAngleBuffer[100];
char setForksDeployedAngleBuffer[100];
char disableStepperMotorBuffer[100];
char calibrateForkBuffer[100];
char enableStepperMotorBuffer[100];
char setLowerForkBuffer[100];
char setUpperForkBuffer[100];
char retractForksBuffer[100];
char deployForksBuffer[100];
char deployArmBuffer[100];
char setWheelSpeedBuffer[100];
char setGripperPositionBuffer[100];

int armDeployedAngleCommandReceivedFlag = 0;
int forksDeployedAngleCommandReceivedFlag = 0;
int disableStepperMotorCommandReceivedFlag = 0;
int calibrateForkCommandReceivedFlag = 0;
int enableStepperMotorCommandReceivedFlag = 0;
int setLowerForkCommandReceivedFlag = 0;
int setUpperForkCommandReceivedFlag = 0;
int retractForksCommandReceivedFlag = 0;
int deployForksCommandReceivedFlag = 0;
int deployArmCommandReceivedFlag = 0;
int setWheelSpeedCommandReceivedFlag = 0;
int setGripperPositionCommandReceivedFlag = 0;

int checkCommandReceived(char* expected, char* buffer, int* commandReceivedFlag) {
    char expectedData[255];
    char withoutCrochet[20];
    sscanf(expected, "<%[^>]>", withoutCrochet);
    sprintf(expectedData, "<Command received : %s>", withoutCrochet);
    printf("expected data = %s and buffer = %s \n",expectedData, buffer);
    if (strcmp(expectedData, buffer) == 0) {
        printf("buffer erased\n");
        buffer[0] = '\0'; // Erase the buffer
        *commandReceivedFlag = 1;
        return 1;
    }
    return 0;
}

int setArmDeployedAngle(int angle) { //Bras du panneau solaire
    sprintf(dataToSend, "<A-%d>", (int)angle);
    if (armDeployedAngleCommandReceivedFlag == 0) {
        UART_send(UART_handle, dataToSend);
        armDeployedAngleCommandReceivedFlag = 1; // Set flag to 1
    }
    if (!UART_receive(UART_handle, setArmDeployedAngleBuffer) || !checkCommandReceived(dataToSend,setArmDeployedAngleBuffer, &armDeployedAngleCommandReceivedFlag)) {
        return 0;
    }
    return armDeployedAngleCommandReceivedFlag;
}

int setForksDeployedAngle(int angle) { //Bras du panneau solaire
    sprintf(dataToSend, "<T-%d>", (int)angle);
    if (forksDeployedAngleCommandReceivedFlag == 0) {
        UART_send(UART_handle, dataToSend);
        forksDeployedAngleCommandReceivedFlag = 1; // Set flag to 1
    }
    if (!UART_receive(UART_handle, setForksDeployedAngleBuffer) || !checkCommandReceived(dataToSend,setForksDeployedAngleBuffer, &forksDeployedAngleCommandReceivedFlag)) {
        return 0;
    }
    return forksDeployedAngleCommandReceivedFlag;
}

int disableStepperMotor() {
    sprintf(dataToSend, "<D-0>");
    if (disableStepperMotorCommandReceivedFlag == 0) {
        UART_send(UART_handle, dataToSend);
        disableStepperMotorCommandReceivedFlag = 1; // Set flag to 1
    }
    if (!UART_receive(UART_handle, disableStepperMotorBuffer) || !checkCommandReceived(dataToSend, disableStepperMotorBuffer, &disableStepperMotorCommandReceivedFlag)) {
        return 0;
    }
    return disableStepperMotorCommandReceivedFlag;
}

int calibrateFork() {
    sprintf(dataToSend, "<H-0>");
    if (calibrateForkCommandReceivedFlag == 0) {
        UART_send(UART_handle, dataToSend);
        calibrateForkCommandReceivedFlag = 1; // Set flag to 1
    }
    if (!UART_receive(UART_handle, calibrateForkBuffer) || !checkCommandReceived(dataToSend, calibrateForkBuffer, &calibrateForkCommandReceivedFlag)) {
        return 0;
    }
    return calibrateForkCommandReceivedFlag;
}

int enableStepperMotor() {
    sprintf(dataToSend, "<E-0>");
    if (enableStepperMotorCommandReceivedFlag == 0) {
        UART_send(UART_handle, dataToSend);
        enableStepperMotorCommandReceivedFlag = 1; // Set flag to 1
    }
    if (!UART_receive(UART_handle, enableStepperMotorBuffer) || !checkCommandReceived(dataToSend, enableStepperMotorBuffer, &enableStepperMotorCommandReceivedFlag)) {
        return 0;
    }
    return enableStepperMotorCommandReceivedFlag;
}

int setLowerFork(int height) {
    sprintf(dataToSend, "<F-%d>", (int)height);
    if (setLowerForkCommandReceivedFlag == 0) {
        UART_send(UART_handle, dataToSend);
        setLowerForkCommandReceivedFlag = 1; // Set flag to 1
    }
    if (!UART_receive(UART_handle, setLowerForkBuffer) || !checkCommandReceived(dataToSend, setLowerForkBuffer, &setLowerForkCommandReceivedFlag)) {
        return 0;
    }
    return setLowerForkCommandReceivedFlag;
}

int setUpperFork(int height) {
    sprintf(dataToSend, "<f-%d>", (int)height);
    if (setUpperForkCommandReceivedFlag == 0) {
        UART_send(UART_handle, dataToSend);
        setUpperForkCommandReceivedFlag = 1; // Set flag to 1
    }
    if (!UART_receive(UART_handle, setUpperForkBuffer) || !checkCommandReceived(dataToSend, setUpperForkBuffer, &setUpperForkCommandReceivedFlag)) {
        return 0;
    }
    return setUpperForkCommandReceivedFlag;
}

int retractForks() {
    sprintf(dataToSend, "<S-0>");
    if (retractForksCommandReceivedFlag == 0) {
        UART_send(UART_handle, dataToSend);
        retractForksCommandReceivedFlag = 1; // Set flag to 1
    }
    if (!UART_receive(UART_handle, retractForksBuffer) || !checkCommandReceived(dataToSend, retractForksBuffer, &retractForksCommandReceivedFlag)) {
        return 0;
    }
    return retractForksCommandReceivedFlag;
}

int deployForks() {
    sprintf(dataToSend, "<S-1>");
    if (deployForksCommandReceivedFlag == 0) {
        UART_send(UART_handle, dataToSend);
        deployForksCommandReceivedFlag = 1; // Set flag to 1
    }
    if (!UART_receive(UART_handle, deployForksBuffer) || !checkCommandReceived(dataToSend, deployForksBuffer, &deployForksCommandReceivedFlag)) {
        return 0;
    }
    return deployForksCommandReceivedFlag;
}

int deployArm() {
    sprintf(dataToSend, "<S-2>");
    if (deployArmCommandReceivedFlag == 0) {
        UART_send(UART_handle, dataToSend);
        deployArmCommandReceivedFlag = 1; // Set flag to 1
    }
    if (!UART_receive(UART_handle, deployArmBuffer) || !checkCommandReceived(dataToSend, deployArmBuffer, &deployArmCommandReceivedFlag)) {
        return 0;
    }
    return deployArmCommandReceivedFlag;
}

int setWheelSpeed(int speed) {
    sprintf(dataToSend, "<W-%d>", speed);
    if (setWheelSpeedCommandReceivedFlag == 0) {
        UART_send(UART_handle, dataToSend);
        setWheelSpeedCommandReceivedFlag = 1; // Set flag to 1
    }
    if (!UART_receive(UART_handle, setWheelSpeedBuffer) || !checkCommandReceived(dataToSend, setWheelSpeedBuffer, &setWheelSpeedCommandReceivedFlag)) {
        return 0;
    }
    return setWheelSpeedCommandReceivedFlag;
}

int setGripperPosition(int position) {
    /*  x = 0 : gripper open (0°)
        x = 1 : gripper closed (180°)
        2 <= x <= 180 : set servo angle
    */
    sprintf(dataToSend, "<G-%d>", position);
    if (setGripperPositionCommandReceivedFlag == 0) {
        UART_send(UART_handle, dataToSend);
        setGripperPositionCommandReceivedFlag = 1; // Set flag to 1
    }
    if (!UART_receive(UART_handle, setGripperPositionBuffer) || !checkCommandReceived(dataToSend, setGripperPositionBuffer, &setGripperPositionCommandReceivedFlag)) {
        return 0;
    }
    return setGripperPositionCommandReceivedFlag;
}
