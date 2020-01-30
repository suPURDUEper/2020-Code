#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/drive/DifferentialDrive.h>
#include <networktables/NetworkTable.h>
#include <cameraserver/CameraServer.h>
#include <frc/DigitalInput.h>
#include <frc/DigitalOutput.h>
#include <rev/CANSparkMax.h>
#include <frc/Joystick.h>
#include <frc/Solenoid.h>
#include <frc/Talon.h>
#include <iostream>
#include "Robot.h"

#include "commonVariables.h"
#include "subSystems.h"

using namespace rev;
using namespace frc;
using namespace std;

double clamp(double in, double minval, double maxval)
{
  if (in > maxval)
    return maxval;
  if (in < minval)
    return minval;
  return in;
}

Joystick controller{0};
Joystick flightStick{1};

//      Motors      //

WPI_TalonSRX intakeMotor{0};
CANSparkMax LDriveMotor{1, CANSparkMax::MotorType::kBrushless};
CANSparkMax LDriveMotor2{2, CANSparkMax::MotorType::kBrushless};
CANSparkMax RDriveMotor{3, CANSparkMax::MotorType::kBrushless};
CANSparkMax RDriveMotor2{4, CANSparkMax::MotorType::kBrushless};
WPI_TalonSRX conveyorMotor{10};

CANPIDController LpidController = LDriveMotor.GetPIDController();
CANEncoder Lencoder = LDriveMotor.GetEncoder();
CANPIDController RpidController = RDriveMotor.GetPIDController();
CANEncoder Rencoder = RDriveMotor.GetEncoder();

// default PID coefficients
double kP = 5e-5, kI = 1e-6, kD = 0, kIz = 0, kFF = 0.000156, kMaxOutput = 1, kMinOutput = -1;

// default smart motion coefficients
double kMaxVel = 2000, kMinVel = 0, kMaxAcc = 1500, kAllErr = 0;

// motor max RPM
const double MaxRPM = 5700;

int currentLimit{50};

//      Joysticks     //
double leftAxisX{controller.GetRawAxis(0)};
double leftAxisY{controller.GetRawAxis(1)};
double rightAxisX{controller.GetRawAxis(2)};
double rightAxisY{controller.GetRawAxis(3)};

//      ABXY Buttons      //
bool btnX{controller.GetRawButton(1)};
bool btnA{controller.GetRawButton(2)};
bool btnB{controller.GetRawButton(3)};
bool btnY{controller.GetRawButton(4)};

//      Bumpers && Triggers     //
bool leftBumper{controller.GetRawButton(5)};
bool rightBumper{controller.GetRawButton(6)};
bool leftTrigger{controller.GetRawButton(7)};
bool rightTrigger{controller.GetRawButton(8)};

//      Misc. Buttons     //
bool btnBack{controller.GetRawButton(9)};
bool btnStart{controller.GetRawButton(10)};

//      Flight Stick      //
bool trigger{flightStick.GetRawButton(1)};
bool btnSafety{flightStick.GetRawButton(11)};
double switchEnable{flightStick.GetRawAxis(3)};
int hat{flightStick.GetPOV(0)};

//      Lift Variables      //
bool lifting;
bool liftTrigger;

//      Sensors     //
DigitalInput breakBeamBottomIn(0);
DigitalOutput breakBeamBottomOut(1);
DigitalInput breakBeamTopIn(2);
DigitalOutput breakBeamTopOut(3);

//      Drive Variables     //
double driveStraight{0};
double driveTurn{0};
float driveMinPower{0.3};
float turnMinPower{0.4};
float modValue{0.5};
float modValueTurn{0.5};
bool nitros{false};

// Drive Train
DifferentialDrive ArcadeDrive(LDriveMotor, RDriveMotor);

//      Solenoids     //

void Robot::RobotInit()
{
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  frc::CameraServer::GetInstance()->StartAutomaticCapture(0);

  LDriveMotor.RestoreFactoryDefaults();
  RDriveMotor.RestoreFactoryDefaults();
  LDriveMotor2.Follow(LDriveMotor);
  RDriveMotor2.Follow(RDriveMotor);

  LpidController.SetP(kP);
  LpidController.SetI(kI);
  LpidController.SetD(kD);
  LpidController.SetIZone(kIz);
  LpidController.SetFF(kFF);
  LpidController.SetOutputRange(kMinOutput, kMaxOutput);
  RpidController.SetP(kP);
  RpidController.SetI(kI);
  RpidController.SetD(kD);
  RpidController.SetIZone(kIz);
  RpidController.SetFF(kFF);
  RpidController.SetOutputRange(kMinOutput, kMaxOutput);

  LpidController.SetSmartMotionMaxVelocity(kMaxVel);
  LpidController.SetSmartMotionMinOutputVelocity(kMinVel);
  LpidController.SetSmartMotionMaxAccel(kMaxAcc);
  LpidController.SetSmartMotionAllowedClosedLoopError(kAllErr);
  RpidController.SetSmartMotionMaxVelocity(kMaxVel);
  RpidController.SetSmartMotionMinOutputVelocity(kMinVel);
  RpidController.SetSmartMotionMaxAccel(kMaxAcc);
  RpidController.SetSmartMotionAllowedClosedLoopError(kAllErr);

  // display PID coefficients on SmartDashboard
  SmartDashboard::PutNumber("P Gain", kP);
  SmartDashboard::PutNumber("I Gain", kI);
  SmartDashboard::PutNumber("D Gain", kD);
  SmartDashboard::PutNumber("I Zone", kIz);
  SmartDashboard::PutNumber("Feed Forward", kFF);
  SmartDashboard::PutNumber("Max Output", kMaxOutput);
  SmartDashboard::PutNumber("Min Output", kMinOutput);

  // display Smart Motion coefficients
  SmartDashboard::PutNumber("Max Velocity", kMaxVel);
  SmartDashboard::PutNumber("Min Velocity", kMinVel);
  SmartDashboard::PutNumber("Max Acceleration", kMaxAcc);
  SmartDashboard::PutNumber("Allowed Closed Loop Error", kAllErr);
  SmartDashboard::PutNumber("Set Position", 0);
  SmartDashboard::PutNumber("Set Degrees", 0);
  SmartDashboard::PutNumber("Set Velocity", 0);

  // button to toggle between velocity and smart motion modes
  SmartDashboard::PutBoolean("Mode", true);
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit()
{
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom)
  {
    // Custom Auto goes here
  }
  else
  {
    // Default Auto goes here
  }
}
int clicks = 21;

void forwardFunction(float distance)
{
  float desiredDistance{distance / 2};
  while (Rencoder.GetPosition() != desiredDistance && Lencoder.GetPosition() != desiredDistance)
  {
    LpidController.SetReference(desiredDistance, ControlType::kSmartMotion);
    RpidController.SetReference(-desiredDistance, ControlType::kSmartMotion);
  }
}

void turnFunction(float distance)
{
  float desiredDistance{distance * clicks};
  while (Lencoder.GetPosition() < desiredDistance && Rencoder.GetPosition() < desiredDistance)
  {
    RpidController.SetP(desiredDistance);
    LpidController.SetP(desiredDistance);
    LpidController.SetReference(desiredDistance, ControlType::kSmartMotion);
    RpidController.SetReference(desiredDistance, ControlType::kSmartMotion);
  }
}

void Robot::AutonomousPeriodic()
{
  double p = SmartDashboard::GetNumber("P Gain", 0);
  double i = SmartDashboard::GetNumber("I Gain", 0);
  double d = SmartDashboard::GetNumber("D Gain", 0);
  double iz = SmartDashboard::GetNumber("I Zone", 0);
  double ff = SmartDashboard::GetNumber("Feed Forward", 0);
  double max = SmartDashboard::GetNumber("Max Output", 0);
  double min = SmartDashboard::GetNumber("Min Output", 0);
  double maxV = SmartDashboard::GetNumber("Max Velocity", 0);
  double minV = SmartDashboard::GetNumber("Min Velocity", 0);
  double maxA = SmartDashboard::GetNumber("Max Acceleration", 0);
  double allE = SmartDashboard::GetNumber("Allowed Closed Loop Error", 0);

  // if PID coefficients on SmartDashboard have changed, write new values to controller
  if ((p != kP))
  {
    LpidController.SetP(p);
    kP = p;
    RpidController.SetP(p);
    kP = p;
  }
  if ((i != kI))
  {
    LpidController.SetI(i);
    kI = i;
    RpidController.SetI(i);
    kI = i;
  }
  if ((d != kD))
  {
    LpidController.SetD(d);
    kD = d;
    RpidController.SetD(d);
    kD = d;
  }
  if ((iz != kIz))
  {
    LpidController.SetIZone(iz);
    kIz = iz;
    RpidController.SetIZone(iz);
    kIz = iz;
  }
  if ((ff != kFF))
  {
    LpidController.SetFF(ff);
    kFF = ff;
    RpidController.SetFF(ff);
    kFF = ff;
  }
  if ((max != kMaxOutput) || (min != kMinOutput))
  {
    LpidController.SetOutputRange(min, max);
    RpidController.SetOutputRange(min, max);
    kMinOutput = min;
    kMaxOutput = max;
  }
  if ((maxV != kMaxVel))
  {
    LpidController.SetSmartMotionMaxVelocity(maxV);
    RpidController.SetSmartMotionMaxVelocity(maxV);
    kMaxVel = maxV;
  }
  if ((minV != kMinVel))
  {
    LpidController.SetSmartMotionMinOutputVelocity(minV);
    RpidController.SetSmartMotionMinOutputVelocity(minV);
    kMinVel = minV;
  }
  if ((maxA != kMaxAcc))
  {
    LpidController.SetSmartMotionMaxAccel(maxA);
    RpidController.SetSmartMotionMaxAccel(maxA);
    kMaxAcc = maxA;
  }
  if ((allE != kAllErr))
  {
    LpidController.SetSmartMotionAllowedClosedLoopError(allE);
    RpidController.SetSmartMotionAllowedClosedLoopError(allE);
    allE = kAllErr;
  }

  float distance;
  float desiredDistance{distance * clicks};
  while (Rencoder.GetPosition() != desiredDistance && Lencoder.GetPosition() != desiredDistance)
  {
    LpidController.SetReference(desiredDistance, ControlType::kSmartMotion);
    RpidController.SetReference(-desiredDistance, ControlType::kSmartMotion);
  }
  distance = 0;
  Rencoder.SetPosition(0);
  Lencoder.SetPosition(0);
  forwardFunction(20);
  //turnFunction(7);
  /*
    double SetPoint, ProcessVariable, RProcessVariable;
    bool mode = SmartDashboard::GetBoolean("Mode", false);
    if (mode)
    {
      SetPoint = SmartDashboard::GetNumber("Set Degrees", 0) * clicks;
      LpidController.SetReference(SetPoint, ControlType::kSmartMotion);
      RpidController.SetReference(SetPoint, ControlType::kSmartMotion);
      ProcessVariable = Lencoder.GetPosition();
      RProcessVariable = Rencoder.GetPosition();
    }
    else
    {
      SetPoint = SmartDashboard::GetNumber("Set Position", 0) * clicks;
      /*
       * As with other PID modes, Smart Motion is set by calling the
       * SetReference method on an existing pid object and setting
       * the control type to kSmartMotion
       */
  /*
      LpidController.SetReference(SetPoint, ControlType::kSmartMotion);
      RpidController.SetReference(-SetPoint, ControlType::kSmartMotion);
      ProcessVariable = Lencoder.GetPosition();
      RProcessVariable = Rencoder.GetPosition();
    }

    SmartDashboard::PutNumber("Set Point", SetPoint);
    SmartDashboard::PutNumber("Process Variable", ProcessVariable);
    SmartDashboard::PutNumber("Output", LDriveNeo.GetAppliedOutput());
*/
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic()
{
  Update_Limelight_Tracking();

  // Deadzone for drive
  if (rightAxisX > -0.05 && rightAxisX < 0.05)
    rightAxisX = 0;

  if (leftAxisY > -0.05 && leftAxisY < 0.05)
    leftAxisY = 0;

  //  Non-linear drive math
  driveMinPower = .15;
  modValue = 1;

  if (leftAxisY > 0)
    driveStraight = (driveMinPower + (1 - driveMinPower) * (modValue * (pow(leftAxisY, 3) + (1 - modValue) * leftAxisY)));
  else if (leftAxisY < 0)
    driveStraight = ((-1 * driveMinPower) + (1 - driveMinPower) * (modValue * (pow(leftAxisY, 3) + (1 - modValue) * leftAxisY)));
  else
    driveStraight = 0;

  if (rightAxisX > 0)
    driveTurn = (turnMinPower + (1 - turnMinPower) * (modValueTurn * (pow(rightAxisX, 3) + (1 - modValueTurn) * rightAxisX)));
  else if (rightAxisX < 0)
    driveTurn = ((-1 * turnMinPower) + (1 - turnMinPower) * (modValueTurn * (pow(rightAxisX, 3) + (1 - modValueTurn) * rightAxisX)));
  else
    driveTurn = 0;

  ArcadeDrive.ArcadeDrive(-1 * driveStraight, driveTurn);

  std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  double ta = table->GetNumber("ta", 0.0);

  SmartDashboard::PutBoolean("Beam", breakBeamBottomIn.Get());

  /*
  if (!breakBeamTopIn.Get() && leftTrigger)
    intakeMotor.Set(100);
  else
    intakeMotor.Set(0);

  if (/*btnStart && !breakBeamBottomIn.Get())
    conveyorMotor.Set(100);
  else
    conveyorMotor.Set(0);*/
}

void Robot::TestPeriodic() {}

void Robot::Update_Limelight_Tracking()
{
  //Proportional Steering Constant
  const double STEER_K = 0.05;

  //Proportional Drive Constant
  const double DRIVE_K = 0.26;

  //Area of tape when robot has reached the goal
  const double DESIRED_TARGET_AREA = 49.0; //changed from 13.0
  const double MAX_DRIVE = 0.25;
  const double MAX_STEER = 0.4f;

  std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  double tx = table->GetNumber("tx", 0.0);
  double ty = table->GetNumber("ty", 0.0);
  double ta = table->GetNumber("ta", 0.0);
  double tv = table->GetNumber("tv", 0.0);

  if (tv < 1.0)
  {
    m_LimelightHasTarget = false;
    m_LimelightDriveCmd = 0.0;
    m_LimelightTurnCmd = 0.0;
  }
  else
  {
    m_LimelightHasTarget = true;
    // Proportional steering
    m_LimelightTurnCmd = tx * STEER_K;
    m_LimelightTurnCmd = clamp(m_LimelightTurnCmd, -MAX_STEER, MAX_STEER);
    // drive forward until the target area reaches our desired area
    if (ta > 49.0)             //
      m_LimelightDriveCmd = 0; //change 11.5 to new value for dual target
    else
      m_LimelightDriveCmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

    m_LimelightDriveCmd = clamp(m_LimelightDriveCmd, -MAX_DRIVE, MAX_DRIVE);
  }
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
