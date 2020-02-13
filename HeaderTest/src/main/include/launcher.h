#include <ctre/Phoenix.h>
#include <iostream>
#include <rev/SparkMax.h>
#include "commonVariables.h"

double launcher() {
    double flyWheelDesiredSpeed;
    if (btnY1 /*&& flyWheelL.GetSelectedSensorVelocity() < 0.9 && flyWheelR.GetSelectedSensorVelocity() < 0.9 || flyWheelL.GetSelectedSensorVelocity() > 0.9 && flyWheelR.GetSelectedSensorVelocity() > 0.9*/) { //temporarily got rid of if !btnX0 for scrimmage
        std::cout << ".9 power" << endl;
        flyWheelDesiredSpeed = 0.9;
    } else if (btnX1 /*&& flyWheelL.GetSelectedSensorVelocity() < 0.7 && flyWheelR.GetSelectedSensorVelocity() < 0.7 || flyWheelL.GetSelectedSensorVelocity() > 0.7 && flyWheelR.GetSelectedSensorVelocity() > 0.7*/) {
        std::cout << ".7 power" << endl;
        flyWheelDesiredSpeed = 0.7;
    } else if (btnA1 /*&& flyWheelL.GetSelectedSensorVelocity() < 0.5 && flyWheelR.GetSelectedSensorVelocity() < 0.5 || flyWheelL.GetSelectedSensorVelocity() > 0.5 && flyWheelR.GetSelectedSensorVelocity() > 0.5*/) {
        std::cout << ".5 power" << endl;
        flyWheelDesiredSpeed = 0.5;
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
    I recommend using a while loop rather than an if statement, fore example.

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
