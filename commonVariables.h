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
extern bool btnA0;
extern bool btnB0;
extern bool btnX0;
extern bool btnY0;
extern bool btnA1;
extern bool btnB1;
extern bool btnX1;
extern bool btnY1;

//      XY Axes Defines      //
extern int leftAxisX0;
extern int leftAxisY0;
extern int rightAxisX0;
extern int rightAxisY0;
extern int leftAxisX1;
extern int leftAxisY1;
extern int rightAxisX1;
extern int rightAxisY1;

//      Bumper and Trigger Defines        //
extern bool leftBumper0;
extern bool rightBumper0;
extern bool leftTrigger0;
extern bool rightTrigger0;
extern bool leftBumper1;
extern bool rightBumper1;
extern bool leftTrigger1;
extern bool rightTrigger1;

//      Misc. Button Defines      //
extern bool btnBack0;
extern bool btnStart0;
extern bool leftJoystickClick0;
extern bool rightJoystickClick0;
extern bool btnBack1;
extern bool btnStart1;
extern bool leftJoystickClick1;
extern bool rightJoystickClick1;

//      Break Beam Defines       //
extern bool breakBeamBottom;
extern bool breakBeamTop;

//      Misc. Defines       //
extern int ballCount;
extern bool firstBottom;
extern bool firstTop;
extern int timer;

#endif