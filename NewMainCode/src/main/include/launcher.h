#ifndef LAUNCHER_H_
#define LAUNCHER_H_
#include "commonVariables.h"

double launcher(double trenchSpeed, double initSpeed, double wallSpeed, double limelightSpeed)
{
    double flyWheelDesiredSpeed;
    if (btnY1 /*&& flyWheelL.GetSelectedSensorVelocity() < 0.9 && flyWheelR.GetSelectedSensorVelocity() < 0.9 || flyWheelL.GetSelectedSensorVelocity() > 0.9 && flyWheelR.GetSelectedSensorVelocity() > 0.9*/)
    { //temporarily got rid of if !btnX0 for scrimmage
        flyWheelDesiredSpeed = trenchSpeed * 3.4133;
    }
    else if (btnX1 /*&& flyWheelL.GetSelectedSensorVelocity() < 0.7 && flyWheelR.GetSelectedSensorVelocity() < 0.7 || flyWheelL.GetSelectedSensorVelocity() > 0.7 && flyWheelR.GetSelectedSensorVelocity() > 0.7*/)
    {
        flyWheelDesiredSpeed = initSpeed * 3.4133;
    }
    else if (btnA1 /*&& flyWheelL.GetSelectedSensorVelocity() < 0.5 && flyWheelR.GetSelectedSensorVelocity() < 0.5 || flyWheelL.GetSelectedSensorVelocity() > 0.5 && flyWheelR.GetSelectedSensorVelocity() > 0.5*/)
    {
        flyWheelDesiredSpeed = wallSpeed * 3.4133;
    }
    else if (btnB1)
    {
        flyWheelDesiredSpeed = limelightSpeed * 3.4133;
    }
    else
    {
        flyWheelDesiredSpeed = 0;
    }

    return flyWheelDesiredSpeed;
}

/* if (rightTrigger0) {
        if (btnY1 && flyWheelL.GetSelectedSensorVelocity() < 0.9 && flyWheelR.GetSelectedSensorVelocity() < 0.9 || flyWheelL.GetSelectedSensorVelocity() > 0.9 && flyWheelR.GetSelectedSensorVelocity() > 0.9) { //temporarily got rid of if !btnX0 for scrimmage
            std::cout << ".9 power" << endl;
            flyWheelL.Set(0.9);
            flyWheelR.Set(0.9);
        } else if (btnX1 && flyWheelL.GetSelectedSensorVelocity() < 0.7 && flyWheelR.GetSelectedSensorVelocity() < 0.7 || flyWheelL.GetSelectedSensorVelocity() > 0.7 && flyWheelR.GetSelectedSensorVelocity() > 0.7) {
            std::cout << ".7 power" << endl;
            flyWheelL.Set(0.7);
            flyWheelR.Set(0.7);
        } else if (btnA1 && flyWheelL.GetSelectedSensorVelocity() < 0.5 && flyWheelR.GetSelectedSensorVelocity() < 0.5 || flyWheelL.GetSelectedSensorVelocity() > 0.5 && flyWheelR.GetSelectedSensorVelocity() > 0.5) {
            std::cout << ".5 power" << endl;
            flyWheelL.Set(0.5);
            flyWheelR.Set(0.5);
        }
    } else if (!rightTrigger0) {
        flyWheelL.Set(0);
        flyWheelR.Set(0);
    }*/

/*
    Comment left by Zach:
    I recommend using a while loop rather than an if statement, for example.

    while(rightTrigger)
        {
            limelight will get the distance to the target;
            calculate the target rpm;

            while(velocity != rpm)
                {
                    check the target rpm
                }
            shoot ball
        }
*/
#endif