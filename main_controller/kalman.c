#ifndef HEADERS
#include "headers.h"
#define HEADERS
#endif

// Déclaration initiale des variables globales
double x[3]; // Vecteur d'état initial [x, y, theta]
double P[3][3] = {{0.01, 0, 0}, {0, 0.01, 0}, {0, 0, 0.01}}; // Covariance initiale de l'état

// Matrices constantes pour le filtre de Kalman
double F[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}; // Matrice de transition d'état
double H[6][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}}; // Matrice d'observation

double Q[3][3] = {{0.005, 0, 0}, {0, 0.005, 0}, {0, 0, 0.01}}; // Bruit de processus
double R[6] = {1, 1, 0.01, 0.4, 0.4, 0.1}; // Bruit de mesure pour chaque variable d'état
double oldTheta;
double meanTheta = 0;
uint8_t thetaForcedFlag = 0;

// Fonction de mise à jour du filtre de Kalman
void* updateKalman(void* args) {
    pthread_mutex_lock(&lockPosition);
    double measurements[3] = {*(myPos.x), *(myPos.y), *(myPos.theta)}; // Obtention des mesures
    pthread_mutex_unlock(&lockPosition);
    double measurementsCombined[6];

    // Obtention des mesures du deuxième capteur (à remplacer par les vraies valeurs)
    double secondSensorMeasurement[3] = {*myOdometryPos.x, *myOdometryPos.y, *myOdometryPos.theta}; // Remplacez par les vraies valeurs

    // Combinaison des mesures des deux capteurs
    pthread_mutex_lock(&lidarFlagLock);
    if(lidarAcquisitionFlag){
        measurementsCombined[0] = measurements[0];
        measurementsCombined[1] = measurements[1];
        measurementsCombined[2] = measurements[2];
        measurementsCombined[3] = secondSensorMeasurement[0];
        measurementsCombined[4] = secondSensorMeasurement[1];
        measurementsCombined[5] = secondSensorMeasurement[2];
    }
    else{
        measurementsCombined[0] = secondSensorMeasurement[0];
        measurementsCombined[1] = secondSensorMeasurement[1];
        measurementsCombined[2] = secondSensorMeasurement[2];
        measurementsCombined[3] = secondSensorMeasurement[0];
        measurementsCombined[4] = secondSensorMeasurement[1];
        measurementsCombined[5] = secondSensorMeasurement[2];
    }
    meanTheta = (measurementsCombined[2] + measurementsCombined[5])/2;
    pthread_mutex_unlock(&lidarFlagLock);
    
    // Étape de prédiction
    double x_pred[3];
    double P_pred[3][3];
    for (int i = 0; i < 3; i++) {
        x_pred[i] = 0;
        for (int j = 0; j < 3; j++) {
            x_pred[i] += F[i][j] * x[j]; // Prédiction de l'état
            P_pred[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                P_pred[i][j] += F[i][k] * P[k][j]; // Prédiction de la covariance de l'état
            }
            P_pred[i][j] += Q[i][j]; // Ajout du bruit de processus
        }
    }

    // Étape de mise à jour
// Étape de mise à jour
for (int j = 0; j < 3; j++) {
    double innovationSum = 0;
    double covarianceSum = 0;
    for (int i = 0; i < 6; i++) {
        double y;
        if (j == 2) {
            double diff = measurementsCombined[i] - x_pred[j];
            // Ajustement de l'innovation pour les transitions entre 0 et 360 degrés
            if (diff > 180) {
                diff -= 360;
            } else if (diff < -180) {
                diff += 360;
            }
            y = diff; // Innovation ajustée
        } else {
            y = measurementsCombined[i] - (H[i][0] * x_pred[0] + H[i][1] * x_pred[1] + H[i][2] * x_pred[2]); // Innovation
        }
        double S = H[i][0] * P_pred[0][0] * H[i][0] + H[i][1] * P_pred[1][1] * H[i][1] + H[i][2] * P_pred[2][2] * H[i][2] + R[i]; // Covariance de l'innovation
        
        double K = P_pred[j][j] * H[i][j] / S; // Gain de Kalman
        innovationSum += K * y;
        covarianceSum += K * H[i][j] * P_pred[j][j];
    }
    x[j] = x_pred[j] + innovationSum; // Mise à jour de l'état
    P[j][j] = P_pred[j][j] - covarianceSum; // Mise à jour de la covariance de l'état
}


    pthread_mutex_lock(&lockFilteredPosition);
    *(myFilteredPos.x) = x[0];
    *(myFilteredPos.y) = x[1];
    if(x[2]> 360) *myFilteredPos.theta = x[2] - 360;
    else *myFilteredPos.theta = x[2];
    *(myFilteredPos.theta) = fmod(x[2],360);
    pthread_mutex_unlock(&lockFilteredPosition);

    return NULL;
}

void defineInitialPosition(){
    x[0] = *myPos.x; x[1] = *myPos.y; x[2] = *myPos.theta;
    pthread_mutex_lock(&lockFilteredPosition);
    *(myFilteredPos.x) = x[0];
    *(myFilteredPos.y) = x[1];
    *(myFilteredPos.theta) = x[2];
    oldTheta = x[2];
    pthread_mutex_unlock(&lockFilteredPosition);

}