#ifndef ROBOTSTATES_H
#define ROBOTSTATES_H

enum RobotState
{   //Move this
    RobotStateNormal,         //Robot is communicating to us, but being controlled by someone else (or nobody)
    RobotStateDisconnected,   //Robot stopped communicating
    RobotStateConnected,      //We are controlling this robot
    RobotStateDisappeared     //Robot no longer communicates and is considered lost (not displayed on map)
};

#endif // ROBOTSTATES_H
