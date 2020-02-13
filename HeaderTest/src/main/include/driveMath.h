#ifndef DRIVEMATH_H_
#define DRIVEMATH_H_

#include <math.h>

double straightMath(float driveMinPower, float modValue, double leftAxisY0)
{
    double driveStraight;
    if (leftAxisY0 > 0.05) // Take the Drive Input and delinearize it for the Left Axis
        driveStraight = (driveMinPower + (1 - driveMinPower) * (modValue * (pow(leftAxisY0, 3) + (1 - modValue) * leftAxisY0)));
    else if (leftAxisY0 < -0.05)
        driveStraight = ((-1 * driveMinPower) + (1 - driveMinPower) * (modValue * (pow(leftAxisY0, 3) + (1 - modValue) * leftAxisY0)));
    else
        driveStraight = 0;

    if (driveStraight > 0.7) 
        driveStraight = 0.7;

    if (driveStraight < -0.7) 
        driveStraight = -0.7;

    return driveStraight;
}

double turnMath(float turnMinPower, float modValueTurn, double rightAxisX0)
{
    double driveTurn;
    if (rightAxisX0 > 0.05) // Take the Drive Input and delinearize it for the Right Axis
        driveTurn = (turnMinPower + (1 - turnMinPower) * (modValueTurn * (pow(rightAxisX0, 3) + (1 - modValueTurn) * rightAxisX0)));
    else if (rightAxisX0 < -0.05)
        driveTurn = ((-1 * turnMinPower) + (1 - turnMinPower) * (modValueTurn * (pow(rightAxisX0, 3) + (1 - modValueTurn) * rightAxisX0)));
    else
        driveTurn = 0;

    if (driveTurn > 0.7) 
        driveTurn = 0.7;

    if (driveTurn < -0.7)
        driveTurn = -0.7;

    return driveTurn;
}
#endif