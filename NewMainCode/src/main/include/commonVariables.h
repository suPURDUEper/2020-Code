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
extern double leftAxisX0;
extern double leftAxisY0;
extern double rightAxisX0;
extern double rightAxisY0;
extern double leftAxisX1;
extern double leftAxisY1;
extern double rightAxisX1;
extern double rightAxisY1;

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

//      Misc. Defines       //
extern bool firstBottom;
extern bool firstTop;
extern int clicks;


#endif