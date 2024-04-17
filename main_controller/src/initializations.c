#ifndef HEADERS
#include "headers.h"
#define HEADERS
#endif

void initializePlantZones(){
    plantZones = (plantZone*)malloc(sizeof(plantZone)*6);
    plantZones[0].zoneID = 1;
    plantZones[0].numberOfPlants = 6;
    plantZones[0].posX = 0.5;
    plantZones[0].posY = 1.5;
    plantZones[0].targetPositionLowX = plantZones[0].posX;
    plantZones[0].targetPositionLowY = plantZones[0].posY - 0.3;
    plantZones[0].targetPositionUpX = plantZones[0].posX;
    plantZones[0].targetPositionUpY = plantZones[0].posY + 0.3;
    plantZones[1].zoneID = 2;
    plantZones[1].numberOfPlants = 6;
    plantZones[1].posX = 0.7;
    plantZones[1].posY = 1;
    plantZones[1].targetPositionLowX = plantZones[1].posX;
    plantZones[1].targetPositionLowY = plantZones[1].posY - 0.3;
    plantZones[1].targetPositionUpX = plantZones[1].posX;
    plantZones[1].targetPositionUpY = plantZones[1].posY + 0.3;
    plantZones[2].zoneID = 3;
    plantZones[2].numberOfPlants = 6;
    plantZones[2].posX = 0.5;
    plantZones[2].posY = 2;
    plantZones[2].targetPositionLowX = plantZones[2].posX;
    plantZones[2].targetPositionLowY = plantZones[2].posY - 0.3;
    plantZones[2].targetPositionUpX = plantZones[2].posX;
    plantZones[2].targetPositionUpY = plantZones[2].posY + 0.3;   
    plantZones[3].zoneID = 4;
    plantZones[3].numberOfPlants = 6;
    plantZones[3].posX = 1.3;
    plantZones[3].posY = 1;
    plantZones[3].targetPositionLowX = plantZones[3].posX;
    plantZones[3].targetPositionLowY = plantZones[3].posY - 0.3;
    plantZones[3].targetPositionUpX = plantZones[3].posX;
    plantZones[3].targetPositionUpY = plantZones[3].posY + 0.3;
    plantZones[4].zoneID = 5;
    plantZones[4].numberOfPlants = 6;
    plantZones[4].posX = 1.3;
    plantZones[4].posY = 2;
    plantZones[4].targetPositionLowX = plantZones[4].posX;
    plantZones[4].targetPositionLowY = plantZones[4].posY - 0.3;
    plantZones[4].targetPositionUpX = plantZones[4].posX;
    plantZones[4].targetPositionUpY = plantZones[4].posY + 0.3;
    plantZones[5].zoneID = 6;
    plantZones[5].numberOfPlants = 6;
    plantZones[5].posX = 1.5;
    plantZones[5].posY = 1.5;
    plantZones[5].targetPositionLowX = plantZones[5].posX;
    plantZones[5].targetPositionLowY = plantZones[5].posY - 0.3;
    plantZones[5].targetPositionUpX = plantZones[5].posX;
    plantZones[5].targetPositionUpY = plantZones[5].posY + 0.3;

}


void initializeEndZones(){
    EndZones = (endZone*)malloc(sizeof(endZone)*6);
    EndZones[0].zoneID = 1;
    EndZones[0].posX =  0.225;
    EndZones[0].posY =  0.225;

    EndZones[1].zoneID = 2;
    EndZones[1].posX = 1;
    EndZones[1].posY = 2.775;

    EndZones[2].zoneID = 3;
    EndZones[2].posX = 1.775;
    EndZones[2].posY = 0.225;

    EndZones[3].zoneID = 4;
    EndZones[3].posX = 0.225;
    EndZones[3].posY = 2.775;

    EndZones[4].zoneID = 5;
    EndZones[4].posX = 1;
    EndZones[4].posY = 0.225;

    EndZones[5].zoneID = 6;
    EndZones[5].posX = 1.775;
    EndZones[5].posY = 2.775;
}
