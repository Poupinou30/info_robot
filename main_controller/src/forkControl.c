#ifndef HEADERS
#include "headers.h"
#define HEADERS
#endif

#define COMMAND_TIMEOUT 500

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

struct timeval armDeployedTime;
struct timeval forksDeployedTime;
struct timeval stepperMotorDisabledTime;
struct timeval forkCalibratedTime;
struct timeval stepperMotorEnabledTime;
struct timeval lowerForkSetTime;
struct timeval upperForkSetTime;
struct timeval forksRetractedTime;
struct timeval forksDeployedTime;
struct timeval armDeployedTime;
struct timeval wheelSpeedSetTime;
struct timeval gripperPositionSetTime;
struct timeval currentTime;

int checkCommandReceived(char* expected, char* buffer, int* commandReceivedFlag) {
    printf("Dans checkCommand et expected = %s", expected);
    char expectedData[255];
    char withoutCrochet[20];
    sscanf(expected, "<%[^>]>", withoutCrochet);
    sprintf(expectedData, "<Command received : %s>", withoutCrochet);
    printf("expected data = %s and buffer = %s \n", expectedData, buffer);

    // Recherche et suppression de "<stopped>" dans buffer
    char* found = strstr(buffer, "<stopped>");
    while (found != NULL) {
        memmove(found, found + strlen("<stopped>"), strlen(found + strlen("<stopped>")) + 1);
        found = strstr(buffer, "<stopped>");
    }

    if (strcmp(expectedData, buffer) == 0) {
        printf("buffer erased\n");
        buffer[0] = '\0'; // Effacer le buffer
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
        gettimeofday(&armDeployedTime, NULL);
    }
    gettimeofday(&currentTime, NULL);
    if(computeTimeElapsed(armDeployedTime, currentTime) > COMMAND_TIMEOUT){
        armDeployedAngleCommandReceivedFlag = 0;
    }
    if (!UART_receive(UART_handle, setArmDeployedAngleBuffer) || !checkCommandReceived(dataToSend,setArmDeployedAngleBuffer, &armDeployedAngleCommandReceivedFlag)) {
        return 0;
    }
    else{
        armDeployedAngleCommandReceivedFlag =0;
        return 1;
    }
    
}

int setForksDeployedAngle(int angle) {
    sprintf(dataToSend, "<T-%d>", (int)angle);
    if (forksDeployedAngleCommandReceivedFlag == 0) {
        UART_send(UART_handle, dataToSend);
        forksDeployedAngleCommandReceivedFlag = 1;
        gettimeofday(&forksDeployedTime, NULL);
    }
    gettimeofday(&currentTime, NULL);
    if (computeTimeElapsed(forksDeployedTime, currentTime) > COMMAND_TIMEOUT) {
        forksDeployedAngleCommandReceivedFlag = 0;
    }
    if (!UART_receive(UART_handle, setForksDeployedAngleBuffer) || !checkCommandReceived(dataToSend, setForksDeployedAngleBuffer, &forksDeployedAngleCommandReceivedFlag)) {
        return 0;
    }
    else {
        forksDeployedAngleCommandReceivedFlag = 0;
        return 1;
    }
}


int disableStepperMotor() {
    sprintf(dataToSend, "<D-0>");
    if (disableStepperMotorCommandReceivedFlag == 0) {
        UART_send(UART_handle, dataToSend);
        disableStepperMotorCommandReceivedFlag = 1;
        gettimeofday(&stepperMotorDisabledTime, NULL);
    }
    gettimeofday(&currentTime, NULL);
    if (computeTimeElapsed(stepperMotorDisabledTime, currentTime) > COMMAND_TIMEOUT) {
        disableStepperMotorCommandReceivedFlag = 0;
    }
    if (!UART_receive(UART_handle, disableStepperMotorBuffer) || !checkCommandReceived(dataToSend, disableStepperMotorBuffer, &disableStepperMotorCommandReceivedFlag)) {
        return 0;
    }
    else {
        disableStepperMotorCommandReceivedFlag = 0;
        return 1;
    }
}


int calibrateFork() {
    sprintf(dataToSend, "<H-0>");
    printf("fork to calibrate\n");
    if (calibrateForkCommandReceivedFlag == 0) {
        UART_send(UART_handle, dataToSend);
        calibrateForkCommandReceivedFlag = 1;
    }
    if (!UART_receive(UART_handle, calibrateForkBuffer) || !checkCommandReceived(dataToSend, calibrateForkBuffer, &calibrateForkCommandReceivedFlag)) {
        return 0;
    }
    else{
        calibrateForkCommandReceivedFlag = 0;
        return 1;
    }

}

int enableStepperMotor() {
    sprintf(dataToSend, "<E-0>");
    if (enableStepperMotorCommandReceivedFlag == 0) {
        UART_send(UART_handle, dataToSend);
        enableStepperMotorCommandReceivedFlag = 1;
        gettimeofday(&stepperMotorEnabledTime, NULL);
    }
    gettimeofday(&currentTime, NULL);
    if (computeTimeElapsed(stepperMotorEnabledTime, currentTime) > COMMAND_TIMEOUT) {
        enableStepperMotorCommandReceivedFlag = 0;
    }
    if (!UART_receive(UART_handle, enableStepperMotorBuffer) || !checkCommandReceived(dataToSend, enableStepperMotorBuffer, &enableStepperMotorCommandReceivedFlag)) {
        return 0;
    }
    else {
        enableStepperMotorCommandReceivedFlag = 0;
        return 1;
    }
}


int setLowerFork(int height) {
    sprintf(dataToSend, "<F-%d>", (int)height);
    if (setLowerForkCommandReceivedFlag == 0) {
        UART_send(UART_handle, dataToSend);
        setLowerForkCommandReceivedFlag = 1;
        gettimeofday(&lowerForkSetTime, NULL);
    }
    gettimeofday(&currentTime, NULL);
    if (computeTimeElapsed(lowerForkSetTime, currentTime) > COMMAND_TIMEOUT) {
        setLowerForkCommandReceivedFlag = 0;
    }
    if (!UART_receive(UART_handle, setLowerForkBuffer) || !checkCommandReceived(dataToSend, setLowerForkBuffer, &setLowerForkCommandReceivedFlag)) {
        return 0;
    }
    else {
        setLowerForkCommandReceivedFlag = 0;
        return 1;
    }
}


int setUpperFork(int height) {
    sprintf(dataToSend, "<f-%d>", (int)height);
    if (setUpperForkCommandReceivedFlag == 0) {
        UART_send(UART_handle, dataToSend);
        setUpperForkCommandReceivedFlag = 1;
        gettimeofday(&upperForkSetTime, NULL);
    }
    gettimeofday(&currentTime, NULL);
    if (computeTimeElapsed(upperForkSetTime, currentTime) > COMMAND_TIMEOUT) {
        setUpperForkCommandReceivedFlag = 0;
    }
    if (!UART_receive(UART_handle, setUpperForkBuffer) || !checkCommandReceived(dataToSend, setUpperForkBuffer, &setUpperForkCommandReceivedFlag)) {
        return 0;
    }
    else {
        setUpperForkCommandReceivedFlag = 0;
        return 1;
    }
}


int retractForks() {
    sprintf(dataToSend, "<S-0>");
    if (retractForksCommandReceivedFlag == 0) {
        UART_send(UART_handle, dataToSend);
        retractForksCommandReceivedFlag = 1;
        gettimeofday(&forksRetractedTime, NULL);
    }
    gettimeofday(&currentTime, NULL);
    if (computeTimeElapsed(forksRetractedTime, currentTime) > COMMAND_TIMEOUT) {
        retractForksCommandReceivedFlag = 0;
    }
    if (!UART_receive(UART_handle, retractForksBuffer) || !checkCommandReceived(dataToSend, retractForksBuffer, &retractForksCommandReceivedFlag)) {
        return 0;
    }
    else {
        retractForksCommandReceivedFlag = 0;
        return 1;
    }
}


int deployForks() {
    sprintf(dataToSend, "<S-1>");
    if (deployForksCommandReceivedFlag == 0) {
        UART_send(UART_handle, dataToSend);
        deployForksCommandReceivedFlag = 1;
        gettimeofday(&forksDeployedTime, NULL);
    }
    gettimeofday(&currentTime, NULL);
    if (computeTimeElapsed(forksDeployedTime, currentTime) > COMMAND_TIMEOUT) {
        deployForksCommandReceivedFlag = 0;
    }
    if (!UART_receive(UART_handle, deployForksBuffer) || !checkCommandReceived(dataToSend, deployForksBuffer, &deployForksCommandReceivedFlag)) {
        return 0;
    }
    else {
        deployForksCommandReceivedFlag = 0;
        return 1;
    }
}


int deployArm() {
    sprintf(dataToSend, "<S-2>");
    if (deployArmCommandReceivedFlag == 0) {
        UART_send(UART_handle, dataToSend);
        deployArmCommandReceivedFlag = 1;
        gettimeofday(&armDeployedTime, NULL); // Note: This reuses the variable for arm deployment, ensure it's intended or use a different variable.
    }
    gettimeofday(&currentTime, NULL);
    if (computeTimeElapsed(armDeployedTime, currentTime) > COMMAND_TIMEOUT) {
        deployArmCommandReceivedFlag = 0;
    }
    if (!UART_receive(UART_handle, deployArmBuffer) || !checkCommandReceived(dataToSend, deployArmBuffer, &deployArmCommandReceivedFlag)) {
        return 0;
    }
    else {
        deployArmCommandReceivedFlag = 0;
        return 1;
    }
}


int setWheelSpeed(int speed) {
    sprintf(dataToSend, "<W-%d>", speed);
    if (setWheelSpeedCommandReceivedFlag == 0) {
        UART_send(UART_handle, dataToSend);
        setWheelSpeedCommandReceivedFlag = 1;
        gettimeofday(&wheelSpeedSetTime, NULL);
    }
    gettimeofday(&currentTime, NULL);
    if (computeTimeElapsed(wheelSpeedSetTime, currentTime) > COMMAND_TIMEOUT) {
        setWheelSpeedCommandReceivedFlag = 0;
    }
    if (!UART_receive(UART_handle, setWheelSpeedBuffer) || !checkCommandReceived(dataToSend, setWheelSpeedBuffer, &setWheelSpeedCommandReceivedFlag)) {
        return 0;
    }
    else {
        setWheelSpeedCommandReceivedFlag = 0;
        return 1;
    }
}


int setGripperPosition(int position) {
    sprintf(dataToSend, "<G-%d>", position);
    if (setGripperPositionCommandReceivedFlag == 0) {
        UART_send(UART_handle, dataToSend);
        setGripperPositionCommandReceivedFlag = 1;
        gettimeofday(&gripperPositionSetTime, NULL);
    }
    gettimeofday(&currentTime, NULL);
    if (computeTimeElapsed(gripperPositionSetTime, currentTime) > COMMAND_TIMEOUT) {
        setGripperPositionCommandReceivedFlag = 0;
    }
    if (!UART_receive(UART_handle, setGripperPositionBuffer) || !checkCommandReceived(dataToSend, setGripperPositionBuffer, &setGripperPositionCommandReceivedFlag)) {
        return 0;
    }
    else {
        setGripperPositionCommandReceivedFlag = 0;
        return 1;
    }
}

