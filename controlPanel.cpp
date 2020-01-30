  
#include "commonVariables.h"
#include "motorDefine.h"

WPI_TalonSRX controlPanelMotor(3);

void controlPanel() {
    if ((hat = 90)) {
        controlPanelMotor.Set(0.5);
    } else if ((hat = 270)) {
        controlPanelMotor.Set(-0.5);
    } else {
        controlPanelMotor.Set(0);
    }
}
