#include "commonVariables.h"
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>



void controlPanel() {
    if (hat = 90) {
        controlPanelMotor.Set(0.5);
    } else if (hat = 270) {
        controlPanelMotor.Set(-0.5);
    } else {
        controlPanelMotor.Set(0);
    }
}

