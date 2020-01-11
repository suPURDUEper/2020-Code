#pragma once
#include <frc/DoubleSolenoid.h>
#include <frc/Victor.h>
#include <rev/SparkMax.h>
#include <ctre/Phoenix.h>

using namespace frc;
using namespace rev;


//      XBAY Button Defines     //    
extern bool btnA;
extern bool btnB;
extern bool btnX;
extern bool btnY;
//      XY Axes Defines      //
extern int leftAxisX;
extern int leftAxisY;
extern int rightAxisX;
extern int rightAxisY;
//      Bumper and Trigger Defines        //
extern bool leftBumper;
extern bool rightBumper;
extern bool leftTrigger;
extern bool rightTrigger;
//      Misc. Button Defines      //
extern bool btnBack;
extern bool btnStart;
extern bool leftJoystickClick;
extern bool rightJoystickClick;
//      flightStickDefines      //
extern int hat;
//      Solenoids       //
frc::DoubleSolenoid hoodSolenoid;
//      Motors      //
Victor sliderMotor1;
Victor sliderMotor2;
SparkMax flyWheelL(1);
SparkMax flyWheelR(2);
WPI_TalonSRX controlPanelMotor(3);
