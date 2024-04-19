#ifndef HEADERS
#include "headers.h"
#define HEADERS
#endif

void initializePlantZones(){
    /*
    LAYOUT:
               0
          1         2

          3         4
               5
    
    */
    plantZones = (plantZone*)malloc(sizeof(plantZone)*6);
    
    plantZones[0].zoneID = 0;
    plantZones[0].numberOfPlants = 6;
    plantZones[0].posX = 0.5;
    plantZones[0].posY = 1.5;
    plantZones[0].targetPositionLowX = plantZones[0].posX;
    plantZones[0].targetPositionLowY = plantZones[0].posY - 0.39;
    plantZones[0].targetPositionUpX = plantZones[0].posX;
    plantZones[0].targetPositionUpY = plantZones[0].posY + 0.39;

    plantZones[1].zoneID = 1;
    plantZones[1].numberOfPlants = 6;
    plantZones[1].posX = 0.7;
    plantZones[1].posY = 1;
    plantZones[1].targetPositionLowX = plantZones[1].posX;
    plantZones[1].targetPositionLowY = plantZones[1].posY - 0.39;
    plantZones[1].targetPositionUpX = plantZones[1].posX;
    plantZones[1].targetPositionUpY = plantZones[1].posY + 0.39;

    plantZones[2].zoneID = 2;
    plantZones[2].numberOfPlants = 6;
    plantZones[2].posX = 0.7;
    plantZones[2].posY = 2;
    plantZones[2].targetPositionLowX = plantZones[2].posX;
    plantZones[2].targetPositionLowY = plantZones[2].posY - 0.39;
    plantZones[2].targetPositionUpX = plantZones[2].posX;
    plantZones[2].targetPositionUpY = plantZones[2].posY + 0.39;  

    plantZones[3].zoneID = 3;
    plantZones[3].numberOfPlants = 6;
    plantZones[3].posX = 1.3;
    plantZones[3].posY = 1;
    plantZones[3].targetPositionLowX = plantZones[3].posX;
    plantZones[3].targetPositionLowY = plantZones[3].posY - 0.39;
    plantZones[3].targetPositionUpX = plantZones[3].posX;
    plantZones[3].targetPositionUpY = plantZones[3].posY + 0.39;

    plantZones[4].zoneID = 4;
    plantZones[4].numberOfPlants = 6;
    plantZones[4].posX = 1.3;
    plantZones[4].posY = 2;
    plantZones[4].targetPositionLowX = plantZones[4].posX;
    plantZones[4].targetPositionLowY = plantZones[4].posY - 0.39;
    plantZones[4].targetPositionUpX = plantZones[4].posX;
    plantZones[4].targetPositionUpY = plantZones[4].posY + 0.39;

    plantZones[5].zoneID = 5;
    plantZones[5].numberOfPlants = 6;
    plantZones[5].posX = 1.5;
    plantZones[5].posY = 1.5;
    plantZones[5].targetPositionLowX = plantZones[5].posX;
    plantZones[5].targetPositionLowY = plantZones[5].posY - 0.39;
    plantZones[5].targetPositionUpX = plantZones[5].posX;
    plantZones[5].targetPositionUpY = plantZones[5].posY + 0.39;
}

void initializePotZones(){
    /*
    LAYOUT:
    0                      1

    2                      3

    4                      5

	0 P0 =0. 6125, 0.4305
	1 P1 = 0.6125, 2.5695
	2 P2 = 1.3875, 0.4305
	3 P3 = 1.3875, 2.5695
	4 P4 = 1.5695, 1
	5 P5 = 1.5695, 2


    */
    potZones = (potZone*)malloc(sizeof(potZone)*6);
    potZones[0].zoneID = 0;
    potZones[0].posX = 0.6125;
    potZones[0].posY = 0.4305;
    potZones[0].numberOfPots = 6;

    potZones[1].zoneID = 1;
    potZones[1].posX = 0.6125;
    potZones[1].posY = 2.5695 ;   
    potZones[1].numberOfPots = 6;

    potZones[2].zoneID = 2;
    potZones[2].posX = 1.3875;
    potZones[2].posY = 0.4305;
    potZones[2].numberOfPots = 6;

    potZones[3].zoneID = 3;
    potZones[3].posX = 1.3875;
    potZones[3].posY = 2.5695;
    potZones[3].numberOfPots = 6;

    potZones[4].zoneID = 4;
    potZones[4].posX = 1.5695;
    potZones[4].posY = 1;
    potZones[4].numberOfPots = 6;

    potZones[5].zoneID = 5;
    potZones[5].posX = 1.5695;
    potZones[5].posY = 2;
    potZones[5].numberOfPots = 6;
}
void initializeJardinieres(){
    /*
    LAYOUT:
          0         3
    1                     4

    5                     2 

    0 (JB0) = 0.12, 0.7625
	1 (JB1)= 0.6125, 0.12
    2 (JB2)= 138.75, 2.88
    3 (JJ0) = 0.12, 2.2375
	4 (JJ1) = 0.6125, 2.88
	5 (JJ2) = 138.75, 0.12

    */	
    jardinieres = (jardiniere*)malloc(sizeof(jardiniere)*6);
    jardinieres[0].zoneID = 0;
    jardinieres[0].posX = 0.12;
    jardinieres[0].posY = 0.7625;
    jardinieres[0].numberOfPlants = 0;

    jardinieres[1].zoneID = 1;
    jardinieres[1].posX = 0.6125;
    jardinieres[1].posY = 0.12;
    jardinieres[1].numberOfPlants = 0;

    jardinieres[2].zoneID = 2;
    jardinieres[2].posX = 1.3875;
    jardinieres[2].posY = 2.88;
    jardinieres[2].numberOfPlants = 0;

    jardinieres[3].zoneID = 3;
    jardinieres[3].posX = 0.12;
    jardinieres[3].posY = 2.2375;
    jardinieres[3].numberOfPlants = 0;

    jardinieres[4].zoneID = 4;
    jardinieres[4].posX = 0.6125;
    jardinieres[4].posY = 2.88;
    jardinieres[4].numberOfPlants = 0;

    jardinieres[5].zoneID = 5;
    jardinieres[5].posX = 1.3875;
    jardinieres[5].posY = 0.12;
    jardinieres[5].numberOfPlants = 0;
    
}

void initializeSolarPanels(){
    /*
    LAYOUT:
        0 1 2   3 4 5    6 7 8   

	SP0 = 1.7824, 0.275
	SP1 = 1.7824, 0.5
	SP2 = 1.7824, 0.725
	SP3 = 1.7824, 1.275
	SP4 = 1.7824, 1.50
	SP5 = 1.7824, 1.725
	SP6 = 1.7824, 2.275
	SP7 = 1.7824, 2.5
	SP8 = 1.7824, 2.725

    */

    solarPanels = (solarpanel*)malloc(sizeof(solarpanel)*9);
    solarPanels[0].panelID = 0;
    solarPanels[0].posX = 1.7824;
    solarPanels[0].posY = 0.275;
    solarPanels[0].state = 2;

    solarPanels[1].panelID = 1;
    solarPanels[1].posX = 1.7824;
    solarPanels[1].posY = 0.5;
    solarPanels[1].state = 2;

    solarPanels[2].panelID = 2;
    solarPanels[2].posX = 1.7824;
    solarPanels[2].posY = 0.725;
    solarPanels[2].state = 2;

    solarPanels[3].panelID = 3;
    solarPanels[3].posX = 1.7824;
    solarPanels[3].posY = 1.275;
    solarPanels[3].state = 2;

    solarPanels[4].panelID = 4;
    solarPanels[4].posX = 1.7824;
    solarPanels[4].posY = 1.5;
    solarPanels[4].state = 2;

    solarPanels[5].panelID = 5;
    solarPanels[5].posX = 1.7824;
    solarPanels[5].posY = 1.725;
    solarPanels[5].state = 2;

    solarPanels[6].panelID = 6;
    solarPanels[6].posX = 1.7824;
    solarPanels[6].posY = 2.275;
    solarPanels[6].state = 2;

    solarPanels[7].panelID = 7;
    solarPanels[7].posX = 1.7824;
    solarPanels[7].posY = 2.5;
    solarPanels[7].state = 2;

    solarPanels[8].panelID = 8;
    solarPanels[8].posX = 1.7824;
    solarPanels[8].posY = 2.725;
    solarPanels[8].state = 2;
}

void initializeEndZones(){
    /*
    LAYOUT:
    0               3

    4               1

    2               5

	0 SB0 = 0.225, 0.225
    1 SB1 = 1, 2.775
    2 SB2 = 1.775, 0.225
	3 SJ0 = 0.225, 2.775
	4 SJ1 = 1, 0.225
	5 SJ2 = 1.775, 2.775

    */

    endZones = (endZone*)malloc(sizeof(endZone)*6);
    endZones[0].zoneID = 0;
    endZones[0].posX =  0.225;
    endZones[0].posY =  0.225;
    endZones[0].numberOfPlants = 0;

    endZones[1].zoneID = 1;
    endZones[1].posX = 1;
    endZones[1].posY = 2.775;
    endZones[1].numberOfPlants = 0;

    endZones[2].zoneID = 2;
    endZones[2].posX = 1.775;
    endZones[2].posY = 0.225;
    endZones[2].numberOfPlants = 0;

    endZones[3].zoneID = 3;
    endZones[3].posX = 0.225;
    endZones[3].posY = 2.775;
    endZones[3].numberOfPlants = 0;

    endZones[4].zoneID = 4;
    endZones[4].posX = 1;
    endZones[4].posY = 0.225;
    endZones[4].numberOfPlants = 0;

    endZones[5].zoneID = 5;
    endZones[5].posX = 1.775;
    endZones[5].posY = 2.775;
    endZones[5].numberOfPlants = 0;
}

