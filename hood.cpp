#include <iostream>
#include "frc/DoubleSolenoid.h"
#include "commonVariables.h"

using namespace frc;

void hood() {

if (btnA) {
    hoodSolenoid.Set(DoubleSolenoid::Value::kForward);
} else if (!btnA) {
    hoodSolenoid.Set(DoubleSolenoid::Value::kReverse);
}

}