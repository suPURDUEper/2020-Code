#include "commonVariables.h"
#include "motorDefine.h"

SparkMax flyWheelL(1);
SparkMax flyWheelR(2);

void launcher() {
    if (btnX) {
        if (flyWheelL.GetSpeed() < .8 || flyWheelR.GetSpeed() < .8) {
            flyWheelL.Set(1);
            flyWheelR.Set(1);
        } else if (flyWheelL.GetSpeed() > .8 || flyWheelR.GetSpeed() > .8) {
            flyWheelL.Set(.6);
            flyWheelR.Set(.6);
        } else {
            flyWheelL.Set(0);
            flyWheelR.Set(0);
        }
    }
}
