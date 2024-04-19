#ifndef HEADERS
#include "headers.h"
#define HEADERS
#endif

#include <fcntl.h>

double radius = 0.03;
double l_y = 0.175;
double l_x = 0.21;

double v_max = 0.2;
double omega_max_running = 200


;
double omega_max_fixed = 200;

void retrieveSpeeds(uint8_t* data, double* speed1, double* speed2) {
    int16_t num1 = (data[0] << 8) | data[1]; // Récupère le premier nombre
    int16_t num2 = (data[2] << 8) | data[3]; // Récupère le deuxième nombre

    *speed1 = num1 * 2 * M_PI * 100 / 114688;
    *speed2 = num2 * 2 * M_PI * 100 / 114688;
}




void createArray(int16_t num1, int16_t num2, uint8_t* output) {
    output[0] = (num1 >> 8) & 0xFF; // Premier octet du premier nombre
    output[1] = num1 & 0xFF;        // Deuxième octet du premier nombre
    output[2] = (num2 >> 8) & 0xFF; // Premier octet du deuxième nombre
    output[3] = num2 & 0xFF;        // Deuxième octet du deuxième nombre
}

double degToRad(double deg) {
    return deg * 3.14 / 180.0;
}

void convertsVelocity(double v_x, double v_y, double omega, double* output_speed){
    //Speed limitation
    double omega_max;
    if(v_x  == 0 && v_y == 0){
        omega_max = omega_max_fixed;
    }
    else omega_max = omega_max_running;
    struct timeval currentTime;
    gettimeofday(&currentTime,NULL);
    pthread_mutex_lock(&lidarTimeLock);
    double lidarElapsedTime = -(lidarAcquisitionTime.tv_sec - currentTime.tv_sec) * 1000.0; // Convert to milliseconds
    lidarElapsedTime -= (lidarAcquisitionTime.tv_usec - currentTime.tv_usec) / 1000.0; // Convert to milliseconds
    pthread_mutex_unlock(&lidarTimeLock);
    pthread_mutex_lock(&lockFilteredOpponent);
    pthread_mutex_lock(&lockFilteredPosition);
    double distanceFromOpponent = computeEuclidianDistance(*myFilteredPos.x,*myFilteredPos.y,*myFilteredOpponent.x,*myFilteredOpponent.y);

    if(lidarElapsedTime > 500) v_max = 0.3;
    else if (distanceFromOpponent<0.7){
        v_max = 0.5*distanceFromOpponent/0.7;
    }
    else v_max = 0.5;
    pthread_mutex_unlock(&lockFilteredOpponent);
    pthread_mutex_unlock(&lockFilteredPosition);

    if (fabs(omega) > omega_max) {
        omega = (omega > 0 ? omega_max : -omega_max);
    }

    omega = omega/112.5;
    // Calculate the magnitude of the velocity vector
    double v_magnitude = sqrt(pow(v_x, 2) + pow(v_y, 2));
    
    // Check if the magnitude is greater than the max allowed speed
    if (v_magnitude > v_max) {
        // Calculate the scaling factor
        double scaling_factor = v_max / v_magnitude;
        
        // Apply the scaling factor to limit the speeds while maintaining the direction
        v_x *= scaling_factor;
        v_y *= scaling_factor;
    }

    
    double omega_fl = 1.0/radius *(v_y+v_x-(l_x+l_y)*omega); //front left
    double omega_fr = 1.0/radius *(v_y-v_x+(l_x+l_y)*omega); //front right
    double omega_rl = 1.0/radius *(v_y-v_x-(l_x+l_y)*omega); //rear left
    double omega_rr = 1.0/radius *(v_y+v_x+(l_x+l_y)*omega); //rear right*/

    output_speed[0] = omega_fl;
    output_speed[1] = omega_fr;
    output_speed[2] = omega_rl;
    output_speed[3] = omega_rr;
    //if(VERBOSE) printf("Omega fl = %f omega_fr = %f omega_rl = %f omega_rr = %f \n",omega_fl,omega_fr,omega_rl,omega_rr);

}

void computeSpeedFromOdometry(double* wheel_speeds, double *v_x, double *v_y, double *omega) {
    *v_x = 1.02*1.04*radius/1.0744 / 4 * (wheel_speeds[0] - wheel_speeds[1] - wheel_speeds[2] + wheel_speeds[3]);

    *v_y = 1.0167*radius / 4 * (wheel_speeds[0] + wheel_speeds[1] + wheel_speeds[2] + wheel_speeds[3]);

    *omega = 112.5*/*1.043**/radius / (4 * (l_x + l_y)) * (-wheel_speeds[0] + wheel_speeds[1] - wheel_speeds[2] + wheel_speeds[3]);

    measuredSpeedX = *v_x;
    measuredSpeedY = *v_y;
    measuredSpeedOmega = *omega;

}

int I2C_initialize(int address){

    int bus = 1;
    int I2C_handle = i2cOpen(bus,address,0);
    if(I2C_handle){
        fprintf(stderr,"");
    }
    return I2C_handle;
}
void I2C_send(char* data,char* received, int I2C_handle){
    int err_code = i2cWriteDevice(I2C_handle,data,strlen(data));
    if(err_code < 0){
        fprintf(stderr,"error %d while sending I2C \n",err_code);}
    int read_data = i2cReadDevice(I2C_handle,received,100);
}

int initializeUART(){
    //int send_PIN = 15;
    //int receive_PIN = 16;
    int baud_rate = 115200;
    
    int UART_handle = serOpen("/dev/ttyS0",baud_rate,0); //ttyS0 est le port UART, 0 est le flag du mode à utiliser
    if (UART_handle < 0)
    {
        fprintf(stderr, "Erreur lors de l'ouverture de la connexion UART.\n");
    }
    return UART_handle;
}
uint8_t UART_send(int UART_handle, char* data){
    if (VERBOSE) printf("Sending '%s' by UART\n", data);
    if(serWrite(UART_handle, data, strlen(data))!=0){
        fprintf(stderr,"Error while writing to UART\n");
        return 0;
    }
    else if (VERBOSE) printf("UART data sent successfully\n");
    return 1;
}

uint8_t UART_receive(int UART_handle, char* received){
    char tempoChar[100] = "";
    
    int bytesRead = serRead(UART_handle, tempoChar, 255);
    if (bytesRead < 0) {
        fprintf(stderr, "Error reading from UART.\n");
        return 0; // Return 0 to indicate failure
    } else if (bytesRead > 0) {
        strcat(received,tempoChar);
        fprintf(stderr,"%d received bytes : '%s' \n",bytesRead,tempoChar);
		
        if(tempoChar[bytesRead-1] == '>') {
            tempoChar[bytesRead] = '\0'; // Null-terminate the string
            return 1;
        }
    }
	printf("received: '%s' \n",received);
    return 0;
}

void initializeLaunchGPIO(){
    int gpio = 25; // Remplacez par le numéro de votre broche GPIO

    // Configure la broche en entrée
    gpioSetMode(gpio, PI_INPUT);

    // Active le pull-up
    gpioSetPullUpDown(gpio, PI_PUD_UP);

}

double randomDouble(double min, double max) {
    double range = (max - min); 
    double div = RAND_MAX / range;
    return min + (rand() / div);
}

void extractBytes(uint16_t nombre, uint8_t *octet_haut, uint8_t *octet_bas) {
    *octet_haut = (nombre >> 8) & 0xFF;
    *octet_bas = nombre & 0xFF;
}

void tunePIDOLD(int spi_handle_front,int spi_handle_rear, uint16_t Kp_m, int8_t Kp_e,uint16_t Ki_m, int8_t Ki_e){ //ki_m = mantissa et Ki_e = exposant
    uint8_t *PIDTab = (uint8_t*) malloc(sizeof(uint8_t)*4);
    PIDTab[3] = 254;
    extractBytes(Kp_m,&PIDTab[2],&PIDTab[1]);
    PIDTab[0] = Kp_e;
    SPI_send(PIDTab,spi_handle_front,NULL);

    SPI_send(PIDTab,spi_handle_rear,NULL);

    extractBytes(Ki_m,&PIDTab[2],&PIDTab[1]);
    PIDTab[0] = Ki_e;
    PIDTab[3] = 255;

    SPI_send(PIDTab,spi_handle_front,NULL);

    SPI_send(PIDTab,spi_handle_rear,NULL);
    free(PIDTab);

}

void tunePID(float Ki, float Kp, int i2c_handle_front, int i2c_handle_rear){
    char toSend[100]; char toReceiveFront[100]; char toReceiveRear[100];
    sprintf(toSend,"<setCoeff-%f-%f>",Kp,Ki); //Vitesses en ticks par seconde
    I2C_send(toSend,toReceiveFront,i2c_handle_front);
    I2C_send(toSend,toReceiveRear,i2c_handle_rear);
}


pid_t child_pid = 0;

/*void handle_sigint(int sig) {
    killpg(getpgid(child_pid), SIGINT);
}


void* executeProgram(void* arg){
    fprintf(stderr,"Entered correcyly in executeProgram function\n");
    int pipefd = *((int*)arg); // Récupération du descripteur de fichier à partir du pointeur
    fprintf(stderr,"int pipefd wtfq\n");
    char cmd[256];
    //sprintf(cmd,"/home/pi/Documents/lab_git_augu/info_robot/lidar_dir/output/Linux/Release/main_folder %d", pipefd);
    fprintf(stderr,"sprintf wtfq\n");
    sprintf(cmd,"/home/pi/Documents/bumblebot/info_robot/sender_test/build/sender_test %d", pipefd);
    fprintf(stderr,"Before signal function\n");
    signal(SIGINT, handle_sigint);
    fprintf(stderr,"After signal function\n");
    fprintf(stderr,"Before fork function\n");
    child_pid = fork();
    fprintf(stderr,"After fork function\n");
    if (child_pid == 0) {
        fprintf(stderr,"Entered correcyly in childpid==0\n");
        setpgid(0, 0);  // Crée un nouveau groupe de processus avec le PID du processus enfant
        execl("/bin/sh", "sh", "-c", cmd, (char *)NULL);
        _exit(EXIT_FAILURE);
    } else if (child_pid < 0) {
        fprintf(stderr,"Error occured \n");
    } else {
        int status;
        fprintf(stderr,"Entered in else waitpid \n");
        waitpid(child_pid, &status, 0);
    }

    fprintf(stderr,"Lidar program correctly launched \n");
    return NULL;*/
void handle_sigsegv(int sig) {
    gpioTerminate();
    fprintf(stderr, "Erreur de segmentation capturée, terminaison du programme.\n");
    if (child_pid > 0) {
        killpg(getpgid(child_pid), SIGINT);
    }
    exit(1);
}

void handle_sigint(int sig) {
    processInstructionNew(0.0,0,0,i2c_handle_front,i2c_handle_rear);
    gpioTerminate();
    fprintf(stderr,"SIGINT handled \n");
    if (child_pid > 0) {
        killpg(getpgid(child_pid), SIGINT);
    }
    signal(SIGINT, SIG_DFL);  // Restaure le comportement par défaut du signal SIGINT
    raise(SIGINT);  // Envoie un signal SIGINT au processus parent
}

void* executeProgram(void* arg){
    int *pipesfd = (int*) arg;
    int pipefdLC = pipesfd[0];
    int pipefdCL = pipesfd[1];
    char cmd[256];
    sprintf(cmd,"/home/pi/Documents/lastGit/info_robot/lidar_dir/output/Linux/Release/main_folder %d %d %d", pipefdLC, pipefdCL, startingPoint);
    //sprintf(cmd,"/home/student/Documents/lab_git_augu/info_robot/lidar_dir/output/Linux/Release/main_folder %d", pipefd);

    child_pid = fork();
    if (child_pid == 0) {
        setpgid(0, 0);
        execl("/bin/sh", "sh", "-c", cmd, (char *)NULL);
        _exit(EXIT_FAILURE);
    } else if (child_pid < 0) {
        fprintf(stderr,"Error occured \n");
    } else {
        signal(SIGINT, handle_sigint);  // Déplacez cette ligne ici
        signal(SIGSEGV, handle_sigsegv);  // Ajoutez cette ligne pour gérer SIGSEGV
        int status;
        waitpid(child_pid, &status, 0);
    }

    fprintf(stderr,"Lidar program stopped\n");
    return NULL;
}

uint8_t kalmanLaunched = 0;

void sendFilteredPos(int pipefdCL){
    float buffer[3];
    buffer[0] = *myFilteredPos.x;
    buffer[1] = *myFilteredPos.y;
    buffer[2] = *myFilteredPos.theta;
    int writeDebugger = write(pipefdCL,(void*) (&buffer),3*sizeof(float));
    if(writeDebugger == -1){
        perror("Error while writing RIP");
    }
    //printf("FilteredPositionWritten and write returned %d for pipefdCL = %d\n",writeDebugger,pipefdCL);
    int flags = fcntl(pipefdCL, F_GETFL);
    if (flags == -1) {
        perror("fcntl");
        // Gérer l'erreur de récupération des drapeaux du descripteur de fichier
    } else {
        // Le descripteur de fichier est valide et correspond à un tube
        //printf("Le descripteur de fichier est valide et correspond à un tube.\n");
    }

}



void* receptionPipe(void* pipefdvoid){
    uint8_t opponentInitialized = 0;
    float *buffer = (float*) malloc(5*sizeof(float));
    uint8_t first = 1;
    fprintf(stderr,"waiting for reading \n");
    //float *buffer = positionReceived; //Position stored in positionRecevived and buffer
    int* pipefd = (int*) pipefdvoid;
    fd_set set;
    while(1){
        
        int ret;
        FD_ZERO(&set); // Initialiser le set à zéro
        FD_SET(pipefd[0], &set); // Ajouter le descripteur de fichier de lecture du pipe au set

        struct timeval timeout;
        timeout.tv_sec = 1; // Timeout de 1 seconde
        timeout.tv_usec = 0;

        ret = select(pipefd[0] + 1, &set, NULL, NULL, &timeout);
        if (ret == -1) {
            perror("select");
            exit(EXIT_FAILURE);
        } else if (ret == 0) {
            //printf("No data within one seconds.\n");
        } //DES DONNEES SONT DISPONIBLES
        else {
            
            read(pipefd[0], buffer, 5*sizeof(float));

            //OPPONENT
            pthread_mutex_lock(&lockOpponentPosition);
            *myOpponent.x = buffer[3]; //A SORTIR DE LA CONDITION WALLAH
            *myOpponent.y = buffer[4];
            pthread_mutex_unlock(&lockOpponentPosition);
            //POSITION
            //printf("conditions: %d %d %d %d %d %d\n",buffer[0] > 0 , buffer[1] > 0 , buffer[0] < 2 , buffer[1]< 3 , (computeEuclidianDistance(*myFilteredPos.x,*myFilteredPos.y,buffer[0],buffer[1]) < 0.40),first);
            if((buffer[0] > 0 && buffer[1] > 0 && buffer[0] < 2 && buffer[1]< 3 && computeEuclidianDistance(*myFilteredPos.x,*myFilteredPos.y,buffer[0],buffer[1]) < 0.40 && fabs(*myFilteredPos.theta - buffer[2])<30)||(first && buffer[0] > 0.01 && buffer[1] > 0.01 && buffer[0] < 2 && buffer[1]< 3) ){
            first = 0;
            pthread_mutex_lock(&lockPosition);
            
            if(*myPos.x != buffer[0] && *myPos.y != buffer[1] && *myPos.theta != buffer[2]){
                pthread_mutex_lock(&lidarTimeLock);
                gettimeofday(&lidarAcquisitionTime,NULL);
                pthread_mutex_unlock(&lidarTimeLock);
            }
            *myPos.x = buffer[0];
            *myPos.y = buffer[1];
            *myPos.theta = buffer[2];
            
            pthread_mutex_unlock(&lockPosition);
            

            if(buffer[3] != 0 && !opponentInitialized){
                opponentInitialized = 1;
                defineOpponentPosition(buffer[3],buffer[4]);
                if(VERBOSE) printf("define opponent position called\n");
            }
            
            
            pthread_mutex_lock(&lockRefreshCounter);
            
            refreshCounter ++;
            readyToGo = 1;

            pthread_mutex_unlock(&lockRefreshCounter);
            
            /*if(kalmanLaunched){
                pthread_join(computeKalmanThread,NULL);
                kalmanLaunched = 0;
            }
            pthread_create(&computeKalmanThread,NULL,updateKalman,NULL);
            kalmanLaunched = 1;*/
            
            }
            //else if(VERBOSE) printf("Condition = %d %d %d %d %d OR %d \n",buffer[0] > 0 , buffer[1] > 0 , buffer[0] < 2 , buffer[1]< 3 , (computeEuclidianDistance(*myFilteredPos.x,*myFilteredPos.y,buffer[0],buffer[1]) < 0.20),first);
            
            /*
            if(VERBOSE){
                fprintf(stderr,"X = %f \n",*(myPos.x));
                fprintf(stderr,"Y = %f \n",*(myPos.y));
                fprintf(stderr,"OPPX = %f \n",buffer[3]);
                fprintf(stderr,"OPPY = %f \n",buffer[4]);
                fprintf(stderr,"Theta = %f \n",*(myPos.theta));

            }*/

    }
    }
}

void convertsSpeedToRobotFrame(double v_x, double v_y, double omega, double* output_speed){
    double theta = *myFilteredPos.theta;
    double cos_theta = cos(theta* _Pi / 180);
    double sin_theta = sin(theta*_Pi / 180);

    double v_x_robot = v_x * cos_theta + v_y * sin_theta;
    double v_y_robot = -v_x * sin_theta + v_y * cos_theta;

    output_speed[0] = v_x_robot;
    output_speed[1] = v_y_robot;
    output_speed[2] = omega;

}

void generateLog(){
    int variable = 0;

    logFile = fopen("../logFiles/logPosition.txt", "w");
    if (logFile == NULL) {
        printf("Erreur lors de l'ouverture du fichier\n");
        return;
    }
    fprintf(logFile, "lidarPos ; odometryPos ; filteredPos ; opponentPos ; opponentFilteredPos ; measuredOmega ; filteredOmega\n");
    printf("File generated i guess\n");
}

void writeLog(){
    fprintf(logFile, "%f %f %f ; %f %f %f ; %f %f %f ; %f %f ; %f %f ; %f ; %f \n", *myPos.x, *myPos.y, *myPos.theta,*myOdometryPos.x,*myOdometryPos.y,*myOdometryPos.theta,*myFilteredPos.x,*myFilteredPos.y,*myFilteredPos.theta, *myOpponent.x, *myOpponent.y, *myFilteredOpponent.x, *myFilteredOpponent.y, measuredSpeedOmega, filteredSpeedOmega);
}

uint8_t checkStartSwitch(){
    return gpioRead(25);
}

plantZone* computeBestPlantsZone(){

    plantZone* bestPlantZone = &plantZones[0];
    int numberOfPlants = bestPlantZone->numberOfPlants;
    pthread_mutex_lock(&lockFilteredPosition);
    float x = *myFilteredPos.x;
    float y = *myFilteredPos.y;
    pthread_mutex_unlock(&lockFilteredPosition);
    float smallestDistance = computeEuclidianDistance(x, y, plantZones[0].posX, plantZones[0].posY);

    for (int i = 1; i < 6; i++) {
        if(plantZones[i].numberOfPlants > numberOfPlants){
            bestPlantZone = &plantZones[i];
        }else if(plantZones[i].numberOfPlants == numberOfPlants){
            double distanceFromPlant = computeEuclidianDistance(x, y, plantZones[i].posX, plantZones[i].posY);
            if (distanceFromPlant < smallestDistance) {
                fprintf(stderr,"Distance = %f and sma\n",computeEuclidianDistance(x, y, plantZones[i].posX, plantZones[i].posY));
                bestPlantZone = &plantZones[i];
                smallestDistance = distanceFromPlant;
            }
        }
    }
    return bestPlantZone;
}

potZone* computeBestPotsZone(){
    potZone* bestPotZone = &potZones[0];
    int numberOfPots = bestPotZone->numberOfPots;
    pthread_mutex_lock(&lockFilteredPosition);
    float x = *myFilteredPos.x;
    float y = *myFilteredPos.y;
    pthread_mutex_unlock(&lockFilteredPosition);
    float smallestDistance = computeEuclidianDistance(x, y, potZones[0].posX, potZones[0].posY);

    for (int i = 1; i < 6; i++) {
        if(potZones[i].numberOfPots > numberOfPots){
            bestPotZone = &potZones[i];
        }else if(potZones[i].numberOfPots = numberOfPots){
            if (computeEuclidianDistance(x, y, potZones[i].posX, potZones[i].posY) < smallestDistance) {
                bestPotZone = &potZones[i];
            }
        }
    }
    return bestPotZone;
}

endZone* computeBestDropZone(){
    endZone* bestDropZone = &dropZones[3*myTeamColor];
    pthread_mutex_lock(&lockFilteredPosition);
    float x = *myFilteredPos.x;
    float y = *myFilteredPos.y;
    pthread_mutex_unlock(&lockFilteredPosition);
    float smallestDistance = computeEuclidianDistance(x, y, dropZones[3*myTeamColor].posX, dropZones[3*myTeamColor].posY);
    

    for (int i = 1; i < 3; i++) {
        if(computeEuclidianDistance(x, y, dropZones[3*myTeamColor+i].posX, dropZones[3*myTeamColor+i].posY) < smallestDistance){
            bestDropZone = &dropZones[i];
        }
    }
    return bestDropZone;
}

jardiniere* computeBestJardiniere(){
    jardiniere* bestJardiniere = &jardinieres[3*myTeamColor];
    int number_of_plants = jardinieres[3*myTeamColor].numberOfPlants;
    pthread_mutex_lock(&lockFilteredPosition);
    float x = *myFilteredPos.x;
    float y = *myFilteredPos.y;
    pthread_mutex_unlock(&lockFilteredPosition);
    float smallestDistance = computeEuclidianDistance(x, y, jardinieres[3*myTeamColor].posX, jardinieres[3*myTeamColor].posY);

    for (int i = 1; i < 3; i++) {
        if(jardinieres[i].numberOfPlants < number_of_plants){
            bestJardiniere = &jardinieres[i];
        }else if (jardinieres[i].numberOfPlants = number_of_plants){
            if(computeEuclidianDistance(x, y, jardinieres[3*myTeamColor+i].posX, jardinieres[3*myTeamColor+i].posY) < smallestDistance){
                bestJardiniere = &jardinieres[i];
            }
        }
    }
    return bestJardiniere;
}

solarpanel* computeBestSolarPanel(){
    solarpanel* bestSolarPanel = &solarPanels[0];
    int state = bestSolarPanel->state;
    pthread_mutex_lock(&lockFilteredPosition);
    float x = *myFilteredPos.x;
    float y = *myFilteredPos.y;
    pthread_mutex_unlock(&lockFilteredPosition);
    float smallestDistance = computeEuclidianDistance(x, y, solarPanels[0].posX, solarPanels[0].posY);

    for (int i = 1; i < 9; i++) {
        if(solarPanels[i].state = myTeamColor){
            if(computeEuclidianDistance(x, y, solarPanels[i].posX, solarPanels[i].posY) < smallestDistance){
                bestSolarPanel = &solarPanels[i];
            }
        }
    }
    return bestSolarPanel;
}

endZone* computeBestEndZone(){
    endZone* bestEndZone = &endZones[3*myTeamColor];
    pthread_mutex_lock(&lockFilteredPosition);
    float x = *myFilteredPos.x;
    float y = *myFilteredPos.y;
    pthread_mutex_unlock(&lockFilteredPosition);
    float smallestDistance = computeEuclidianDistance(x, y, endZones[3*myTeamColor].posX, endZones[3*myTeamColor].posY);
        
    for (int i = 1; i < 3; i++) {
        if(computeEuclidianDistance(x, y, endZones[3*myTeamColor+i].posX, endZones[3*myTeamColor+i].posY) < smallestDistance){
           bestEndZone = &endZones[i];
        }
    }
    return bestEndZone;
}

/*
plantType* cameraConfirmation(){
    plantType *plantType = (plantType*)malloc(sizeof(plantType)) 
    for(int i = 1; i < numberPlants; i++){
        if (plants[i].standing == 1){
            if (plants[i].color == purple){
                plantType.purple += 1;
            }else{
                plantType.white += 1; 
            }
        }
    }
    return plantType;
}

void computePoints(){
    if(myActionChoice == PLANTS_ACTION){
        if(myMovingSubState == CHECK_UP_JARD){
            if(cameraConfirmation() != NULL){
                plantType *plantType = cameraConfirmation();
                score += 4*(plantType->purple + plantType->white);
            }else{
                score += 24;
            }
            return;

        }else if(myMovingSubState == CHECK_UP_DROP){
            if(cameraConfirmation() != NULL){
                plantType *plantType = cameraConfirmation();
                score += 3*(plantType->purple);
            }else{
                score += 6;
            }
            return;
        }

    }else if(myActionChoice == PLANTS_POTS_ACTION){
        if(myMovingSubState == CHECK_UP_JARD){
            if(cameraConfirmation() != NULL){
                plantType *plantType = cameraConfirmation();
                score += 5*(plantType->purple + plantType->white);
            }else{
                score += 30;
            }
            return;

        }else if(myMovingSubState == CHECK_UP_DROP){
            if(cameraConfirmation() != NULL){
                plantType *plantType = cameraConfirmation();
                score += 4*(plantType->purple + plantType->white);
            }else{
                score += 24;
            }
            return;
        }
        
    }else if(myActionChoice == SOLAR_PANELS_ACTION){
        score += 5;
        return;
    }else if(mySupremeState == GAME_OVER){
        score += 10; 
        return;
    }
}

float tempoOppX, tempoOppY;
float oldOppX, oldOppY; 
struct timeval lastCheckTime, currentTime;

void missingPlants(){
    pthread_mutex_lock(&lockFilteredOpponent);
    tempoOppX = *myFilteredOpponent.x;
    tempoOppY = *myFilteredOpponent.y;
    pthread_mutex_unlock(&lockFilteredOpponent);

    gettimeofday(&currentTime, NULL);
    if ((currentTime - (lastCheckTime.tv_sec*1000 + lastCheckTime.tv_usec/1000) > 500)){
        oldOppX = tempoOppX;
        oldOppY = tempoOppY;
        gettimeofday(&lastCheckTime, NULL);
    }

    for(int i, i++, i<6){
        if (isInPlantZone(tempoOppX, tempoOppY, plantZones[i].posX, plantZones[i].posY) && plantZones[i].numberOfPlants != 0){
            plantZones[i].numberOfPlants=0;
        }else if(areOverlapping(plantZones[i].posX, plantZones[i].posY, tempoOppX, tempoOppY, radiusOpponent) && distance(tempoOppX, tempoOppY, oldOppX, oldOppY)<0.05 && plantZones[i].numberOfPlants != 0){
            plantZones[i].numberOfPlants=0;
        }
    }
}

bool isInPlantZone(float x, float y, float xc, float yc) {
    float distance_squared = (x - xc) * (x - xc) + (y - yc) * (y - yc);
    float radius_squared = 0.125 * 0.125;
    return distance_squared <= radius_squared;
}

float distance(float x1, float y1, float x2, float y2) {
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

// Fonction pour calculer la longueur d'empiètement entre deux cercles
uint8_t areOverlapping(float xc, float yc, float x, float y, float r2, float *overlap) {
    float d = distance(xc, yc, x, y);
    if (d > 0.125 + r2 || d < fabs(0.125 - r2) || d == 0) {
        return 0; // Pas d'intersection
    }

    float a = (0.125 * 0.125 - r2 * r2 + d * d) / (2 * d);
    float overlapLength = 2*(0.125-a);

    if (overlapLength > 0) {
        *overlap = overlapLength;
        return 1;
    }

    return 0;
}

*/