#include "commonVariables.h"
/*
    Description: A function to detect when balls are coming in and out
    Pre: No balls in the robot
    Post: Balls coming in and out while the robot is running, but never exceeding 5 balls total 
*/
void intakeMacro()
{
    if (breakBeamBottom && breakBeamTop /* &&  motors not already on (should I inclde something like this) */)
    {
        // turn on intake motor
        // turn on indexer
    }
    else if (!breakBeamTop && breakBeamBottom && firstBottom) //added first so that we don't infinitely increase ball count // if bottom beam is broken and top beam not, then increase ball count by 1
    {
        ballCount++;
        firstBottom = false;
    }
    else if (!breakBeamBottom && !breakBeamTop) //took out not triggered b/c robot needs to run coneyor and shoot at the same time
    {
        // turn off the motor
        // turn off the indexer
    }
    else if (breakBeamBottom && !breakBeamTop && firstTop) // As soon as we get 4 balls, it'll reduce it to negative infinity
    {
        ballCount--;
        firstTop = false;
    }
    
    if (breakBeamTop)
    {
        firstTop = true;
    }

    if (breakBeamBottom)
    {
        firstBottom = true;
    }
    /*else if (breakBeamBottom)
    {
        first = true;
    }don't know what this means*/
}