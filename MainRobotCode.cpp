#include "Robot.h"
#include <iostream>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/phoenix.h>
#include <cameraserver/CameraServer.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMax.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/DoubleSolenoid.h>
#include <frc/DigitalInput.h>
#include <frc/Joystick.h>
#include <frc/Solenoid.h>
#include <frc/Victor.h>
#include <frc/Talon.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>

double clamp(double in, double minval, double maxval)
{
  if (in > maxval)
    return maxval;
  if (in < minval)
    return minval;
  return in;
}

using namespace frc;
using namespace std;
using namespace rev;

Joystick controller{0};
Joystick flightStick{1};

int currentLimit{50};

//      Drive Motors      //
CANSparkMax LDriveNeo2{1, CANSparkMax::MotorType::kBrushless};
CANSparkMax LDriveNeo{2, CANSparkMax::MotorType::kBrushless}; //first number is CAN ID of motor
CANSparkMax RDriveNeo{3, CANSparkMax::MotorType::kBrushless};
CANSparkMax RDriveNeo2{4, CANSparkMax::MotorType::kBrushless};

//      Non-Drive Motors      //
WPI_TalonSRX intakeMotor(0);
WPI_TalonSRX conveyorMotor(4);

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
frc::DigitalInput breakBeamBottom(0);
frc::DigitalInput breakBeamTop(1);
frc::DigitalInput breakBeamShoot(3);

//      Intake Variables      //
int ballCounter = 0;

//      Drive Variables     //
double driveStraight{0};
double driveTurn{0};
float driveMinPower{0.3};
float turnMinPower{0.4};
float modValue{0.5};
float modValueTurn{0.5};
bool nitros{false};

// Drive Train
DifferentialDrive ArcadeDrive(LDriveNeo, RDriveNeo);

//      Solenoids     //
frc::DoubleSolenoid hoodSolenoid(7, 8);

//      Motors      //
rev::SparkMax flyWheelL(1);
rev::SparkMax flyWheelR(2);
ctre::phoenix::motorcontrol::can::WPI_TalonSRX controlPanelMotor7(3);
frc::Victor sliderMotor2(4);
frc::Victor sliderMotor1(5);

void lift()
{
  if (liftTrigger && !lifting && switchEnable < -0.8) {}
}

void launcher()
{
  if (btnX)
  {
    if (flyWheelL.GetSpeed() < .8)
    {
      flyWheelL.Set(1);
    }
    else if (flyWheelL.GetSpeed() > .8)
    {
      flyWheelL.Set(.6);
    }
    else
    {
      flyWheelL.Set(0);
    }
  }
}

void slider()
{
  if (btnY)
  {
    sliderMotor1.Set(0.5);
    sliderMotor2.Set(0.5);
  }
  else if (btnB)
  {
    sliderMotor1.Set(-0.5);
    sliderMotor2.Set(-0.5);
  }
  else
  {
    sliderMotor1.Set(0);
    sliderMotor2.Set(0);
  }
}

void hood()
{
  if (btnA)
  {
    hoodSolenoid.Set(DoubleSolenoid::Value::kForward);
  }
  else if (!btnA)
  {
    hoodSolenoid.Set(DoubleSolenoid::Value::kReverse);
  }
}

void controlPanel()
{
  if ((hat = 90))
  {
    controlPanelMotor7.Set(0.5);
  }
  else if ((hat = 270))
  {
    controlPanelMotor7.Set(-0.5);
  }
  else
  {
    controlPanelMotor7.Set(0);
  }
}

// Sub Systems //

void Robot::RobotInit()
{
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  frc::CameraServer::GetInstance()->StartAutomaticCapture(0);
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

void Robot::AutonomousPeriodic()
{
  if (m_autoSelected == kAutoNameCustom)
  {
    // Custom Auto goes here
  }
  else
  {
    // Default Auto goes here
  }
}

void Robot::TeleopInit()
{
  LDriveNeo2.Follow(LDriveNeo);
  RDriveNeo2.Follow(RDriveNeo);

  LDriveNeo.SetSmartCurrentLimit(currentLimit);
  RDriveNeo.SetSmartCurrentLimit(currentLimit);
}

void Robot::TeleopPeriodic()
{
  Update_Limelight_Tracking();
  leftAxisX = controller.GetRawAxis(0);
  leftAxisY = controller.GetRawAxis(1);
  rightAxisX = controller.GetRawAxis(2);
  rightAxisY = controller.GetRawAxis(3);

  leftTrigger = controller.GetRawButton(7);

  //safety button not held
  // Deadzone for drive
  if (rightAxisX > -0.05 && rightAxisX < 0.05)
  {
    rightAxisX = 0;
  }

  if (leftAxisY > -0.05 && leftAxisY < 0.05)
  {
    leftAxisY = 0;
  }

  //  Non-linear drive math
  driveMinPower = .15;
  modValue = 1;

  if (leftAxisY > 0)
  {
    driveStraight = (driveMinPower + (1 - driveMinPower) * (modValue * (pow(leftAxisY, 3) + (1 - modValue) * leftAxisY)));
  } else if (leftAxisY < 0) {
    driveStraight = ((-1 * driveMinPower) + (1 - driveMinPower) * (modValue * (pow(leftAxisY, 3) + (1 - modValue) * leftAxisY)));
  } else {
    driveStraight = 0;
  }

  if (rightAxisX > 0)
  {
    driveTurn = (turnMinPower + (1 - turnMinPower) * (modValueTurn * (pow(rightAxisX, 3) + (1 - modValueTurn) * rightAxisX)));
  } else if (rightAxisX < 0) {
    driveTurn = ((-1 * turnMinPower) + (1 - turnMinPower) * (modValueTurn * (pow(rightAxisX, 3) + (1 - modValueTurn) * rightAxisX)));
  } else {
    driveTurn = 0;
  }

  ArcadeDrive.ArcadeDrive(-1 * driveStraight, driveTurn);

  std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  double ta = table->GetNumber("ta", 0.0);

  if (/*!breakBeamTop.Get() &&*/ leftTrigger)
  {
    intakeMotor.Set(100);
  } else {
    intakeMotor.Set(0);
  }

  if (btnStart /*&& !breakBeamBottom.Get()*/)
  {
    conveyorMotor.Set(100);
  }
  else
  {
    conveyorMotor.Set(0);
  }
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
    if (ta > 49.0)
    { //change 11.5 to new value for dual target
      m_LimelightDriveCmd = 0;
    }
    else
    {
      m_LimelightDriveCmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;
    }
    m_LimelightDriveCmd = clamp(m_LimelightDriveCmd, -MAX_DRIVE, MAX_DRIVE);
  }
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
