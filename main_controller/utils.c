#ifndef HEADERS
#include "headers.h"
#define HEADERS
#endif


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
    double radius = 0.029;
    double l_x = 0.175;
    double l_y = 0.21;
    double omega_fl = 1.0/radius *(v_x-v_y-(l_x+l_y)*omega); //front left
    double omega_fr = 1.0/radius *(v_x+v_y+(l_x+l_y)*omega); //front right
    double omega_rl = 1.0/radius *(v_x+v_y-(l_x+l_y)*omega); //rear left
    double omega_rr = 1.0/radius *(v_x-v_y+(l_x+l_y)*omega); //rear right
    output_speed[0] = omega_fl;
    output_speed[1] = omega_fr;
    output_speed[2] = omega_rl;
    output_speed[3] = omega_rr;

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

void tunePID(int spi_handle_front,int spi_handle_rear, uint16_t Kp_m, int8_t Kp_e,uint16_t Ki_m, int8_t Ki_e){
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

void handle_sigint(int sig) {
    fprintf(stderr,"signit handeled \n");
    killpg(getpgid(child_pid), SIGINT);
    signal(SIGINT, SIG_DFL);  // Restaure le comportement par défaut du signal SIGINT
    raise(SIGINT);  // Envoie un signal SIGINT au processus parent
}


void* executeProgram(void* arg){
    int pipefd = *((int*)arg);
    char cmd[256];
    sprintf(cmd,"/home/pi/Documents/bumblebot/info_robot/sender_test/build/sender_test %d", pipefd);
    sprintf(cmd,"/home/pi/Documents/lab_git_augu/info_robot/lidar_dir/output/Linux/Release/main_folder %d", pipefd);

    child_pid = fork();
    signal(SIGINT, handle_sigint);  // Déplacez cette ligne ici
    if (child_pid == 0) {

        setpgid(0, 0);
        execl("/bin/sh", "sh", "-c", cmd, (char *)NULL);
        _exit(EXIT_FAILURE);
    } else if (child_pid < 0) {
        fprintf(stderr,"Error occured \n");
    } else {
        int status;
        waitpid(child_pid, &status, 0);
    }

    fprintf(stderr,"Lidar program correctly launched \n");
    return NULL;
}

void* receptionPipe(void* pipefdvoid){
    fprintf(stderr,"waiting for reading \n");
    float *buffer = positionReceived; //Position stored in positionRecevived and buffer
    int* pipefd = (int*) pipefdvoid;
    while(1){
        fd_set set;
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
            printf("No data within one seconds.\n");
        } else {
            // Des données sont disponibles, lire les données
            pthread_mutex_lock(&lockPosition);
            read(pipefd[0], buffer, 3*sizeof(buffer));
            pthread_mutex_unlock(&lockPosition);
            fprintf(stderr,"Readed \n");
            for (int i = 0; i < 3; ++i) {
                fprintf(stderr,"%f \n", positionReceived[i]);
            }
            fprintf(stderr,"X = %f \n",*(myPos.x));
            fprintf(stderr,"Y = %f \n",*(myPos.y));
            fprintf(stderr,"Theta = %f \n",*(myPos.theta));

        }
    }


}