#include <ctre/Phoenix.h>
#include <iostream>
#include <rev/SparkMax.h>
#include "commonVariables.h"


rev::SparkMax flyWheelL(1);
rev::SparkMax flyWheelR(2);

void launcher() {
//if (btnX) {
    if (!btnX/*flyWheelL.GetSpeed() < .8*/) {
        std::cout << "0";// flyWheelL.Set(1);
    } else if (btnX/*flyWheelL.GetSpeed() > .8*/) {
        std::cout << "1";// flyWheelL.Set(.6);
    }
else {
   // flyWheelL.Set(0);
}

}