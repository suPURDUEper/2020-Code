#include "commonVariables.h"
#include "motorDefine.h"

Victor sliderMotor1(4);
Victor sliderMotor2(5);

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
