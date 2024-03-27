#ifndef HEADERS
#include "headers.h"
#define HEADERS
#endif


grabbingState myGrabState;

void manageGrabbing(){

    switch (myGrabState)
    {
    case GRAB_PLANTS_INIT:
        deployForks();
        setLowerFork(65);
        setUpperFork(0);
        sleep(3);
        myGrabState = GRAB_PLANTS_MOVE;
        break;
    case GRAB_PLANTS_MOVE:
        //Move forward to grab the plants
        myGrabState = GRAB_PLANTS_END;
        break;
    case GRAB_PLANTS_END:
        setGripperPosition(1);
        setUpperFork(142);
        sleep(3);
        myGrabState = UNSTACK_POTS_MOVE;
        break;
    case UNSTACK_POTS_MOVE:
        myGrabState = UNSTACK_POT_TAKE;
        break;
    case UNSTACK_POT_TAKE:
        setLowerFork(125);
        sleep(3);
        myGrabState = UNSTACK_POT_POSITIONING;
        break;
    case UNSTACK_POT_POSITIONING:
        myGrabState = UNSTACK_POT_DROP;
        break;
    case UNSTACK_POT_DROP:
        setLowerFork(30);
        sleep(3);
        myGrabState = GRAB_POTS_MOVE;
        break;
    case GRAB_POTS_MOVE:
        myGrabState = DROP_PLANTS;
        sleep(3);
        myGrabState = LIFT_POTS;
        break;
    case LIFT_POTS:
        setLowerFork(135);
        sleep(3);
        myGrabState = DROP_PLANTS;
        break;
    case DROP_PLANTS:
        setGripperPosition(0);
        sleep(3);
        myGrabState = DROP_ALL;
        break;
    case DROP_ALL:
        setLowerFork(75);
        setUpperFork(80);
        sleep(3);
        myGrabState = FINISHED;
        break;
    case FINISHED:
        break;

    default:
        break;
    }
}

/*
STATE 1: --------------------------------
Deploy forks
Lift lower forks at 65mm forks plantsssss
Lower the upper fork at 0
STATE 2: --------------------------------
AVANCER POUR CHOPPER LES PLANTES 
STATE 3: --------------------------------
Close the gripper
Lift the upper fork at 142mm
STATE 4: --------------------------------
Move forward to take the stacked pot
STATE 5: --------------------------------
Lift lower fork to 125 // Lever pour prendre le pot stacké
STATE 6: 
Move to drop pot correctly
STATE 7:
Lower lower fork to 30mm // Descendre pour lacher le pot 

STATE 8:
Move to grab the other pots
STATE 9:
Lift lower fork to 135 to lift the pots under the plants
Open the gripper to release the plants in the pots
STATE 10:
Lower the lower fork at 75mm and the upper fork at 80mm drop pots
 */