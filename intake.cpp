#include "commonVariables.h"
/*
    Description: A function to detect when balls are coming in and out
    Pre: No balls in the robot
    Post: Balls coming in and out while the robot is running, but never exceeding 5 balls total 
*/
void intakeMacro()
{
    if (breakBeamTop && breakBeamTop)
    {
        // turn on intake motor
        // turn on indexer
    }
    else if (!breakBeamTop && breakBeamBottom) // if bottom beam is broken and top beam not, then increase ball count by 1
    {
        ballCount++;
    }
    else if (!breakBeamBottom && !breakBeamTop) //took out not triggered b/c robot needs to run coneyor and shoot at the same time
    {
        // turn off the motor
        // turn of the indexer
    }
    else if (breakBeamBottom && !breakBeamTop) //
    {
        ballCount--;
    }
    /*else if (breakBeamBottom)
    {
        notTriggered = true;
    }don't know what this means*/ 
}
