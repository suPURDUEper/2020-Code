#include "commonVariables.h"
/*
    Description: A function to detect when balls are coming in and out
    Pre: No balls in the robot
    Post: Balls coming in and out while the robot is running, but never exceeding 5 balls total 
*/
double conveyor()
{
    double ConveyorSpeed;
    if ((!breakBeamTop || breakBeamBottom) && !rightTrigger0) // if bottom beam is broken and top beam not, then increase ball count by 1
    {
        ConveyorSpeed = 0;
    }
    else if (!breakBeamBottom && breakBeamTop) //took out not triggered b/c robot needs to run coneyor and shoot at the same time
    {
        ConveyorSpeed = 0.5;
    }

    return ConveyorSpeed;
}