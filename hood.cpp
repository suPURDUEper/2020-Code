#include "commonVariables.h"
#include "motorDefine.h"

DoubleSolenoid hoodSolenoid(7, 8);

void hood() {

if (btnA) {
    hoodSolenoid.Set(DoubleSolenoid::Value::kForward);
} else if (!btnA) {
    hoodSolenoid.Set(DoubleSolenoid::Value::kReverse);
}

}
