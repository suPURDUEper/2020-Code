#include <ctre/Phoenix.h>
#include <iostream>
#include <rev/SparkMax.h>
#include "commonVariables.h"
/*
SparkMax flyWheelL(1);
SparkMax flyWheelR(2);

void launcher() {
    //if (rightTrigger) {
        if (!btnX/*flyWheelL.GetSpeed() < 0.8 && flyWheelR.GetSpeed() < 0.8*//* ) {
            std::cout << "0";// flyWheelL.Set(1);
            //flyWheelR.Set(1);
        } else if (btnX/*flyWheelL.GetSpeed() > 0.8 && flyWheelR.GetSpeed() > 0.8*///) {
            //std::cout << "1";// flyWheelL.Set(0.6);
            //flyWheelR.Set(0.6);
        //} 
    //} else if (!leftTrigger) {
        //flyWheelL.Set(0);
        //flyWheelR.Set(0);
    //}

//}

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
