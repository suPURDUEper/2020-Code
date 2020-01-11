#include <frc/Victor.h>
#include "commonVariables.h"

void slider() {

    if(btnY) {
        sliderMotor1.Set(0.5);
        sliderMotor2.Set(0.5);
    } else if (btnB) {
        sliderMotor1.Set(-0.5);
        sliderMotor2.Set(-0.5);
    } else {
        sliderMotor1.Set(0);
        sliderMotor2.Set(0);
    }

}