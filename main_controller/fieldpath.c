#ifndef HEADERS
#include "headers.h"
#define HEADERS
#endif

float actionDistance = 10; //en cm

float computeEuclidianDistance(int x1, int y1, int x2, int y2){
    return pow(pow(x2-x1,2)+pow(y2-y1,2),0.5);
}

float computeRectangleDistance(double x1, double y1, double x2, double y2, double x3, double y3) {
    double dx = max(min(x3, x2), x1) - x3;
    double dy = max(min(y3, y2), y1) - y3;
    return sqrt(dx*dx + dy*dy);
}

void computeAttractiveField(position destination){ //position convertie en cm
    //Cette fonction calcule le champ d'attraction pour le potential field path planning
    //Elle ne sera exécutée qu'à chaque fois qu'on changera l'objectif final en terme de destination (ex: quand on arrive a une plante)
    int posX = (int) *(destination.x)*100;
    int posY = (int) *(destination.y)*100;
    int posTheta = (int) *(destination.theta)*100;
    float scalingFactor = 1;
    double euclidianDistance;
    for (int i = 0; i < sizeY; ++i) {
        for (int j = 0; j < sizeX; ++j) {
            myField.attractiveField[i][j] = 0.5*scalingFactor* computeEuclidianDistance(i,j,posX,posY);
        }
    }
}

void computeInitialRepulsiveField(){
    //Cette fonction est censée initialiser le champ répulsif avec les obstacles qu'on connait déja (murs, jardiniaires) et ne sera éxécutée qu'en début de partie
    //Pour l'instant le champ répulsif est à 0
}

void updateRepulsiveField(int x1,int y1, int x2, int y2){

    float scalingFactor = 1;
    int X1 = min(x1,x2);
    int X2 = max(x1,x2);
    int Y1 = min(y1,y2);
    int Y2 = max(y1,y2);
    float euclidianDistance;
    int activeX1 = max(X1-actionDistance,0);
    int activeY1 = max(Y1-actionDistance,0);
    int activeX2 = min(X2+actionDistance,sizeX);
    int activeY2 = min(Y2+actionDistance,sizeY);//Définit la zone mise à jour

    for (int i = activeY1; i < activeY2; ++i) {
        for (int j = activeX1; j < activeX2; ++j) {
            euclidianDistance = computeRectangleDistance(X1,Y1,X2,Y2,j,i);
            if(j >= X1 && j <= X2 && i >= Y1 && i <= Y2){
                myField.repulsiveField[i][j] = 1/2 * pow((1-1/actionDistance),2);
            } //Si on est DANS la zone de l'objet

            else if(euclidianDistance <= actionDistance){
                myField.repulsiveField[i][j] = 1/2 *pow((1/euclidianDistance - 1/actionDistance),2);
            }
            else myField.repulsiveField[i][j] = 0;

        }
    }
    computeTotalField(1,activeX1,activeY1,activeX2,activeY2);

}

void resetRepulsiveField(int x1,int y1, int x2, int y2){
    int X1 = min(x1,x2);
    int X2 = max(x1,x2);
    int Y1 = min(y1,y2);
    int Y2 = max(y1,y2);
    for (int i = Y1; i < Y2; ++i) {
        for (int j = X1; j < X2; ++j) {
            myField.repulsiveField[i][j] = 0;
        }

    }
}

void computeTotalField(uint8_t mode, int x1, int y1, int x2, int y2){
    //Mode 0 = calculer toute l'arène
    //Mode 1 = on précise sur quelle surface on doit mettre à jour le field
    if(mode == 0){
        for (int i = 0; i < sizeY; ++i) {
            for (int j = 0; j < sizeX; ++j) {
                myField.totalField[i][j] = myField.attractiveField[i][j] + myField.repulsiveField[i][j];
            }

        }
    }

    else{
        int X1 = min(x1,x2);
        int X2 = max(x1,x2);
        int Y1 = min(y1,y2);
        int Y2 = max(y1,y2);
        for (int i = max(Y1-actionDistance,0); i < min(Y2+actionDistance,sizeY); ++i) {
            for (int j = max(X1-actionDistance,0); j < min(X2+actionDistance,sizeX); ++j) {
                myField.totalField[i][j] = myField.attractiveField[i][j] + myField.repulsiveField[i][j];
            }
        }
    }
}

void print2DArray(int m, int n, int** arr) {
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            printf("%d ", arr[i][j]);
        }
        printf("\n");
    }
}