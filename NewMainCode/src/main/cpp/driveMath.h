#ifndef DRIVEMATH_H_
#define DRIVEMATH_H_

#include <math.h>

double straightMath(float driveMinPower, float modValue, double leftAxisY0)
{

    double maxDriveStraight;
    if (rightBumper0)
    {
        maxDriveStraight = 0.9;
    }
    else
    {
        maxDriveStraight = 0.75;
    }
    
    double driveStraight;

    if (leftAxisY0 > 0.05) // Take the Drive Input and delinearize it for the Left Axis
        driveStraight = (driveMinPower + (maxDriveStraight - driveMinPower) * (modValue * (pow(leftAxisY0, 3)) + (1 - modValue) * leftAxisY0));
    else if (leftAxisY0 < -0.05)
        driveStraight = ((-1 * driveMinPower) + (maxDriveStraight - driveMinPower) * (modValue * (pow(leftAxisY0, 3)) + (1 - modValue) * leftAxisY0));
    else
        driveStraight = 0;

    return driveStraight;
}

double turnMath(float turnMinPower, float modValueTurn, double rightAxisX0)
{
    double driveTurn;
    double maxDriveTurn = 0.6;
    if (rightAxisX0 > 0.05) // Take the Drive Input and delinearize it for the Right Axis
        driveTurn = (turnMinPower + (maxDriveTurn - turnMinPower) * (modValueTurn * (pow(rightAxisX0, 3)) + (1 - modValueTurn) * rightAxisX0));
    else if (rightAxisX0 < -0.05)
        driveTurn = ((-1 * turnMinPower) + (maxDriveTurn - turnMinPower) * (modValueTurn * (pow(rightAxisX0, 3)) + (1 - modValueTurn) * rightAxisX0));
    else
        driveTurn = 0;

    return driveTurn;
}
#endif