#ifndef HEADERS
#include "headers.h"
#define HEADERS
#endif

void initializePlantZones(){
    plantZones = (plantZone*)malloc(sizeof(plantZone)*6);
    plantZones[0].zoneID = 1;
    plantsZone[0].numbrereOfPlants = 6;
    plantZones[0].posX = 0.5;
    plantZones[0].posY = 1.5;
    plantZones[0].targetPositionLowX = plantZones[0].posX;
    plantZones[0].targetPositionLowY = plantZones[0].posY - 0.3;
    plantZones[0].targetPositionHighX = plantZones[0].posX;
    plantZones[0].targetPositionHighY = plantZones[0].posY + 0.3;
    plantZones[1].zoneID = 2;
    plantsZone[1].numbrereOfPlants = 6;
    plantZones[1].posX = 0.7;
    plantZones[1].posY = 1;
    plantZones[1].targetPositionLowX = plantZones[1].posX;
    plantZones[1].targetPositionLowY = plantZones[1].posY - 0.3;
    plantZones[1].targetPositionHighX = plantZones[1].posX;
    plantZones[1].targetPositionHighY = plantZones[1].posY + 0.3;
    plantZones[2].zoneID = 3;
    plantsZone[2].numbrereOfPlants = 6;
    plantZones[2].posX = 0.5;
    plantZones[2].posY = 2;
    plantZones[2].targetPositionLowX = plantZones[2].posX;
    plantZones[2].targetPositionLowY = plantZones[2].posY - 0.3;
    plantZones[2].targetPositionHighX = plantZones[2].posX;
    plantZones[2].targetPositionHighY = plantZones[2].posY + 0.3;   
    plantZones[3].zoneID = 4;
    plantsZone[3].numbrereOfPlants = 6;
    plantZones[3].posX = 1.3;
    plantZones[3].posY = 1;
    plantZones[3].targetPositionLowX = plantZones[3].posX;
    plantZones[3].targetPositionLowY = plantZones[3].posY - 0.3;
    plantZones[3].targetPositionHighX = plantZones[3].posX;
    plantZones[3].targetPositionHighY = plantZones[3].posY + 0.3;
    plantZones[4].zoneID = 5;
    plantsZone[4].numbrereOfPlants = 6;
    plantZones[4].posX = 1.3;
    plantZones[4].posY = 2;
    plantZones[4].targetPositionLowX = plantZones[4].posX;
    plantZones[4].targetPositionLowY = plantZones[4].posY - 0.3;
    plantZones[4].targetPositionHighX = plantZones[4].posX;
    plantZones[4].targetPositionHighY = plantZones[4].posY + 0.3;
    plantZones[5].zoneID = 6;
    plantsZone[5].numbrereOfPlants = 6;
    plantZones[5].posX = 1.5;
    plantZones[5].posY = 1.5;
    plantZones[5].targetPositionLowX = plantZones[5].posX;
    plantZones[5].targetPositionLowY = plantZones[5].posY - 0.3;
    plantZones[5].targetPositionHighX = plantZones[5].posX;
    plantZones[5].targetPositionHighY = plantZones[5].posY + 0.3;

}