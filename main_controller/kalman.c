#ifndef HEADERS
#include "headers.h"
#define HEADERS
#endif

// Déclaration initiale des variables globales
double x[3] = {0, 0, 0}; // Vecteur d'état initial [x, y, theta]
double P[3][3] = {{0.01, 0, 0}, {0, 0.01, 0}, {0, 0, 0.01}}; // Covariance initiale de l'état

// Matrices constantes pour le filtre de Kalman
double F[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}; // Matrice de transition d'état
double H[6][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}}; // Matrice d'observation
double Q[3][3] = {{0.005, 0, 0}, {0, 0.005, 0}, {0, 0, 0.001}}; // Bruit de processus
double R[6] = {0.4, 0.4, 0.001, 0.4, 0.4, 0.001}; // Bruit de mesure pour chaque variable d'état

// Fonction de mise à jour du filtre de Kalman
void* updateKalman(void* args) {
    pthread_mutex_lock(&lockPosition);
    double measurements[3] = {*(myPos.x), *(myPos.y), *(myPos.theta)}; // Obtention des mesures
    pthread_mutex_unlock(&lockPosition);

    // Obtention des mesures du deuxième capteur (à remplacer par les vraies valeurs)
    double secondSensorMeasurement[3] = {0, 0, 0}; // Remplacez par les vraies valeurs

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
    for (int i = 0; i < 6; i++) {
        double y = measurementsCombined[i] - (H[i][0] * x_pred[0] + H[i][1] * x_pred[1] + H[i][2] * x_pred[2]); // Innovation
        double S = H[i][0] * P_pred[0][0] * H[i][0] + H[i][1] * P_pred[1][1] * H[i][1] + H[i][2] * P_pred[2][2] * H[i][2] + R[i]; // Covariance de l'innovation
        double K[3];
        for (int j = 0; j < 3; j++) {
            K[j] = P_pred[j][0] * H[i][0] / S; // Gain de Kalman
            x[j] = x_pred[j] + K[j] * y; // Mise à jour de l'état
            P[j][0] = P_pred[j][0] - K[j] * H[i][0] * P_pred[j][0]; // Mise à jour de la covariance de l'état
            // Mise à jour des autres éléments de la matrice de covariance de l'état
            for (int k = 1; k < 3; k++) {
                P[j][k] = P_pred[j][k] - K[j] * H[i][k] * P_pred[j][k];
            }
        }
    }

    pthread_mutex_lock(&lockFilteredPosition);
    //fprintf(stderr, "x filtered test = %lf \n", x[0]);
    *(myFilteredPos.x) = x[0];
    *(myFilteredPos.y) = x[1];
    *(myFilteredPos.theta) = x[2];
    pthread_mutex_unlock(&lockFilteredPosition);

    return NULL; // Assurez-vous que la fonction correspond à la signature attendue
}
