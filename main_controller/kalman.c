#ifndef HEADERS
#include "headers.h"
#define HEADERS
#endif

// Déclaration initiale des variables globales
double x[3] = {1, 1.5, 0}; // Vecteur d'état initial [x, y, theta]
double P[3][3] = {{0.01, 0, 0}, {0, 0.01, 0}, {0, 0, 0.01}}; // Covariance initiale de l'état

// Matrices constantes pour le filtre de Kalman
double F[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}; // Matrice de transition d'état
double H[6][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}}; // Matrice d'observation

double Q[3][3] = {{0.005, 0, 0}, {0, 0.005, 0}, {0, 0, 0.01}}; // Bruit de processus
double R[6] = {0.4, 0.4, 0.01, 0.4, 0.4, 0.01}; // Bruit de mesure pour chaque variable d'état

// Fonction de mise à jour du filtre de Kalman
void* updateKalman(void* args) {
    pthread_mutex_lock(&lockPosition);
    double measurements[3] = {*(myPos.x), *(myPos.y), *(myPos.theta)}; // Obtention des mesures
    pthread_mutex_unlock(&lockPosition);

    // Obtention des mesures du deuxième capteur (à remplacer par les vraies valeurs)
    double secondSensorMeasurement[3] = {*myOdometryPos.x, *myOdometryPos.y, *myOdometryPos.theta}; // Remplacez par les vraies valeurs

    // Combinaison des mesures des deux capteurs
    double measurementsCombined[6] = {measurements[0], measurements[1], measurements[2], 
                                      secondSensorMeasurement[0], secondSensorMeasurement[1], secondSensorMeasurement[2]};

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
    for (int j = 0; j < 3; j++) {
        double innovationSum = 0;
        double covarianceSum = 0;
        for (int i = 0; i < 6; i++) {
            double y = measurementsCombined[i] - (H[i][0] * x_pred[0] + H[i][1] * x_pred[1] + H[i][2] * x_pred[2]); // Innovation
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
    *(myFilteredPos.theta) = x[2];
    pthread_mutex_unlock(&lockFilteredPosition);

    return NULL;
}
