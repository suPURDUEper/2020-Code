#include <frc/Victor.h>
#include "commonVariables.h"


Victor sliderMotor1(4);
Victor sliderMotor2(5);

void slider() {
    if(btnY0) {
        sliderMotor1.Set(0.5);
        sliderMotor2.Set(0.5);
    } else if (btnB0) {
        sliderMotor1.Set(-0.5);
        sliderMotor2.Set(-0.5);
    } else {
        sliderMotor1.Set(0);
        sliderMotor2.Set(0);
    }
}