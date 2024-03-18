#ifndef HEADERS
#include "headers.h"
#define HEADERS
#endif

double x[3] = {0.125, 0.125, 0}; // Vecteur d'état initial [x, y, theta]
double P[3][3] = {{10, 0, 0}, {0, 10, 0}, {0, 0, 360}}; // Matrice de covariance initiale
double F[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}; // Matrice de transition d'état
double H[3] = {1, 1, 1}; // Matrice d'observation
double R[3] = {10, 10, 0.01}; // Bruit de mesure
double Q[3][3] = {{0.1, 0, 0}, {0, 0.1, 0}, {0, 0, 0.0001}}; // Bruit de processus
double measurements[3];

void* updateKalman(void* args){
    position *myPos = (position*) args;
    pthread_mutex_lock(&lockPosition);
    measurements[0] = *(myPos->x);
    measurements[1] = *(myPos->y);
    measurements[2] = *(myPos->theta);
    pthread_mutex_unlock(&lockPosition);
    double x_pred[3] = {F[0][0]*x[0] + F[0][1]*x[1] + F[0][2]*x[2], 
                        F[1][0]*x[0] + F[1][1]*x[1] + F[1][2]*x[2],
                        F[2][0]*x[0] + F[2][1]*x[1] + F[2][2]*x[2]};
    // Mise à jour de la matrice de covariance
    for(int j=0; j<3; j++)
        for(int k=0; k<3; k++)
            P[j][k] = F[j][0]*P[0][k] + F[j][1]*P[1][k] + F[j][2]*P[2][k];
    for(int j=0; j<3; j++)
        P[j][j] += Q[j][j];

    // Mise à jour
    for(int j=0; j<3; j++) {
        double y = measurements[j] - (H[0]*x_pred[0] + H[1]*x_pred[1] + H[2]*x_pred[2]); // Innovation
        double S = H[j]*H[j]*P[j][j] + R[j]; // Innovation covariance
        double K[3] = {P[0][j]*H[j]/S, P[1][j]*H[j]/S, P[2][j]*H[j]/S}; // Gain de Kalman

        // Mise à jour de l'état et de la matrice de covariance
        for(int k=0; k<3; k++) {
            x[k] = x_pred[k] + K[k]*y;
            P[k][j] = (1 - K[k]*H[j])*P[k][j];
        }
    }
    fprintf(stderr,"resultat du filtre de kalman x = %lf y = %lf theta = %lf \n",x[0],x[1],
    x[2]);
    pthread_mutex_lock(&lockFilteredPosition);
     *(myFilteredPos.x) = measurements[0];
     *(myFilteredPos.y) = measurements[1];
     *(myFilteredPos.theta) = measurements[2];
    pthread_mutex_unlock(&lockFilteredPosition);
}