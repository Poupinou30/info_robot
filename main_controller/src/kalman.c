#ifndef HEADERS
#include "headers.h"
#define HEADERS
#endif

// Déclaration initiale des variables globales
double x[8]; // Vecteur d'état initial [x, y, theta, opponent_x, opponent_y, velocity_x, velocity_y]
double P[8][8] = {
    {0.01, 0, 0, 0, 0, 0, 0, 0},
    {0, 0.01, 0, 0, 0, 0, 0, 0},
    {0, 0, 0.01, 0, 0, 0, 0, 0},
    {0, 0, 0, 0.01, 0, 0, 0, 0},
    {0, 0, 0, 0, 0.01, 0, 0, 0},
    {0, 0, 0, 0, 0, 0.0001, 0, 0},
    {0, 0, 0, 0, 0, 0, 0.0001, 0},
    {0, 0, 0, 0, 0, 0, 0, 0.0001}
}; // Covariance initiale de l'état

// Matrices constantes pour le filtre de Kalman
double F[8][8] = {
    {1, 0, 0, 0, 0, timeDelay/1000, 0, 0}, //La position x est influencée par le vitesse en x
    {0, 1, 0, 0, 0, 0, timeDelay/1000, 0}, //Idem Y
    {0, 0, 1, 0, 0, 0, 0, timeDelay/1000}, //Idem omega
    {0, 0, 0, 1, 0, 0, 0, 0},
    {0, 0, 0, 0, 1, 0, 0, 0},
    {0, 0, 0, 0, 0, 1, 0, 0},
    {0, 0, 0, 0, 0, 0, 1, 0},
    {0, 0, 0, 0, 0, 0, 0, 1}
}; // Matrice de transition d'état
double H[11][8] = {
    {1, 0, 0, 0, 0, 0, 0, 0}, // myPos.x from sensor 1
    {0, 1, 0, 0, 0, 0, 0, 0}, // myPos.y from sensor 1
    {0, 0, 1, 0, 0, 0, 0, 0}, // myPos.theta from sensor 1
    {1, 0, 0, 0, 0, 0, 0, 0}, // myPos.x from sensor 2
    {0, 1, 0, 0, 0, 0, 0, 0}, // myPos.y from sensor 2
    {0, 0, 1, 0, 0, 0, 0, 0}, // myPos.theta from sensor 2
    {0, 0, 0, 1, 0, 0, 0, 0}, // opponentX
    {0, 0, 0, 0, 1, 0, 0, 0}, // opponentY
    {0, 0, 0, 0, 0, 1, 0, 0}, // speedX
    {0, 0, 0, 0, 0, 0, 1, 0},  // speedY
    {0, 0, 0, 0, 0, 0, 0, 1} //speedOmega
};

double Q[8][8] = {
    {0.01, 0, 0, 0, 0, 0, 0, 0},
    {0, 0.01, 0, 0, 0, 0, 0, 0},
    {0, 0, 0.01, 0, 0, 0, 0, 0},
    {0, 0, 0, 0.01, 0, 0, 0, 0},
    {0, 0, 0, 0, 0.01, 0, 0, 0},
    {0, 0, 0, 0, 0, 0.1, 0, 0},
    {0, 0, 0, 0, 0, 0, 0.1, 0},
    {0, 0, 0, 0, 0, 0, 0, 0.1}
}; // Bruit de processus
double R[11] = {0.8, 0.8, 0.5,0.8, 0.8, 0.5, 0.5, 0.5, 0.1, 0.1,0.1}; // Bruit de mesure pour chaque variable d'état
double oldTheta;
double meanTheta = 0;
uint8_t thetaForcedFlag = 0;

// Fonction de mise à jour du filtre de Kalman
void* updateKalman(void* args) {
    double y;
    pthread_mutex_lock(&lockPosition);
    pthread_mutex_lock(&lockFilteredOpponent);
    double measurements[8] = {*(myPos.x), *(myPos.y), *(myPos.theta), *(myOpponent.x), *(myOpponent.y), measuredSpeedX, measuredSpeedY, measuredSpeedOmega}; // Obtention des mesures
    pthread_mutex_unlock(&lockFilteredOpponent);
    pthread_mutex_unlock(&lockPosition);
    double measurementsCombined[11];

    struct timeval currentTime;
    gettimeofday(&currentTime,NULL);
    pthread_mutex_lock(&lidarTimeLock);
    double lidarElapsedTime = -(lidarAcquisitionTime.tv_sec - currentTime.tv_sec) * 1000.0; // Convert to milliseconds
    lidarElapsedTime -= (lidarAcquisitionTime.tv_usec - currentTime.tv_usec) / 1000.0; // Convert to milliseconds
    pthread_mutex_unlock(&lidarTimeLock);
    

    // Obtention des mesures du deuxième capteur (à remplacer par les vraies valeurs)
    double secondSensorMeasurement[3] = {*myOdometryPos.x, *myOdometryPos.y, *myOdometryPos.theta}; // Remplacez par les vraies valeurs

    // Combinaison des mesures des deux capteurs
    pthread_mutex_lock(&lidarTimeLock);
    if(0){ //On prend que l'odo pour la caractérisation de l'odo
        for(int i = 0; i < 3; i++){
            measurementsCombined[i] = measurements[i];
            measurementsCombined[i+3] = secondSensorMeasurement[i];
        }
    }
    else{ //On prend que l'odométrie
        for(int i = 0; i < 3; i++){
            measurementsCombined[i] = secondSensorMeasurement[i];
            measurementsCombined[i+3] = secondSensorMeasurement[i];
        }
    }
    measurementsCombined[6] = measurements[3];
    measurementsCombined[7] = measurements[4];
    measurementsCombined[8] = measurements[5];
    measurementsCombined[9] = measurements[6];
    measurementsCombined[10] = measurements[7];
    pthread_mutex_unlock(&lidarTimeLock);

    // Étape de prédiction
    double x_pred[8];
    double P_pred[8][8];
    for (int i = 0; i < 8; i++) {
        x_pred[i] = 0;
        for (int j = 0; j < 8; j++) {
            x_pred[i] += F[i][j] * x[j]; // Prédiction de l'état
            P_pred[i][j] = 0;
            for (int k = 0; k < 8; k++) {
                P_pred[i][j] += F[i][k] * P[k][j]; // Prédiction de la covariance de l'état
            }
            P_pred[i][j] += Q[i][j]; // Ajout du bruit de processus
        }
    }

    // Étape de mise à jour
    for (int j = 0; j < 8; j++) {
        double innovationSum = 0;
        double covarianceSum = 0;
        for (int i = 0; i < 11; i++) {
            if (i == j || ((i-3) == j )) { // Utiliser seulement les mesures pertinentes pour chaque mise à jour d'état
                double diff = measurementsCombined[i] - x_pred[j];
                // Ajustement de l'innovation pour les transitions entre 0 et 360 degrés
                if (j == 2) {
                    if (diff > 180) {
                        diff -= 360;
                    } else if (diff < -180) {
                        diff += 360;
                    }
                }
                y = diff; // Innovation ajustée
                double S = H[i][0] * P_pred[0][0] * H[i][0] + H[i][1] * P_pred[1][1] * H[i][1] + H[i][2] * P_pred[2][2] * H[i][2] + H[i][3] * P_pred[3][3] * H[i][3] + H[i][4] * P_pred[4][4] * H[i][4] + H[i][5] * P_pred[5][5] * H[i][5] + H[i][6] * P_pred[6][6] * H[i][6] + H[i][7] * P_pred[7][7] * H[i][7]+ R[i]; // Covariance de l'innovation
                //printf("Impression de chaque termes de S: H[i][0] = %f P_pred[0][0] = %f H[i][1] = %f P_pred[1][1] = %f H[i][2] = %f P_pred[2][2] = %f H[i][3] = %f P_pred[3][3] = %f H[i][4] = %f P_pred[4][4] = %f H[i][5] = %f P_pred[5][5] = %f H[i][6] = %f P_pred[6][6] = %f R[i] = %f \n",H[i][0],P_pred[0][0],H[i][1],P_pred[1][1],H[i][2],P_pred[2][2],H[i][3],P_pred[3][3],H[i][4],P_pred[4][4],H[i][5],P_pred[5][5],H[i][6],P_pred[6][6],R[i]   );
                double K = P_pred[j][j] * H[i][j] / S; // Gain de Kalman
                innovationSum += K * y;
                //printf("Gain de kalman pour j = %d, i = %d K = %f avec P_pred = %f et H = %f et S = %f \n",j,i,K,P_pred[j][j],H[i][j],S);
                covarianceSum += K * H[i][j] * P_pred[j][j];
            }
        }
        x[j] = x_pred[j] + innovationSum; // Mise à jour de l'état
        P[j][j] = P_pred[j][j] - covarianceSum; // Mise à jour de la covariance de l'état
    }

    // Mise à jour de la position filtrée
    pthread_mutex_lock(&lockFilteredPosition);
    *(myFilteredPos.x) = x[0];
    *(myFilteredPos.y) = x[1];
    double filteredTheta = x[2];
    while(filteredTheta > 360) filteredTheta -= 360;
    while (filteredTheta < 0) filteredTheta += 360;
    *myFilteredPos.theta = filteredTheta;
    pthread_mutex_unlock(&lockFilteredPosition);

    // Mise à jour de la position de l'opposant filtrée
    pthread_mutex_lock(&lockFilteredOpponent);
    *(myFilteredOpponent.x) = x[3];
    *(myFilteredOpponent.y) = x[4];
    pthread_mutex_unlock(&lockFilteredOpponent);
    //printf("filteredOMega = %f et realOmega = %f \n",x[7],measuredSpeedOmega);
    filteredSpeedX = x[5];
    filteredSpeedY = x[6];
    filteredSpeedOmega = x[7];
    //printf("filteredSpeedX = %f et filteredSpeedY = %f et filteredSpeedOmega = %f \n",filteredSpeedX,filteredSpeedY,filteredSpeedOmega);

    return NULL;
}


void defineInitialPosition(){
    x[0] = *myPos.x; x[1] = *myPos.y; x[2] = *myPos.theta; x[3] = -1; x[4] = -1; x[5] = 0; x[6] = 0;
    pthread_mutex_lock(&lockFilteredPosition);
    *(myFilteredPos.x) = x[0];
    *(myFilteredPos.y) = x[1];
    *(myFilteredPos.theta) = x[2];
    oldTheta = x[2];
    pthread_mutex_unlock(&lockFilteredPosition);
}

void defineOpponentPosition(float posX, float posY){
    x[3] = posX; x[4] = posY;
}

