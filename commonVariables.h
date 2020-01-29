#pragma once
#ifndef commonVariables
#define commonVariables

#include <frc/DoubleSolenoid.h>
#include <frc/Victor.h>
#include <rev/SparkMax.h>
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>

using namespace frc;
using namespace rev;
using namespace ctre;
using namespace std;

//      XBAY Button Defines     //    
extern bool btnA;
extern bool btnB;
extern bool btnX;
extern bool btnY;

//      XY Axes Defines      //
extern double leftAxisX;
extern double leftAxisY;
extern double rightAxisX;
extern double rightAxisY;

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

//      Flight Stick Defines      //
extern int hat;

#endif
