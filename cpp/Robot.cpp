#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/drive/DifferentialDrive.h>
#include <networktables/NetworkTable.h>
#include <cameraserver/CameraServer.h>
#include <frc/DoubleSolenoid.h>
#include <frc/DigitalOutput.h>
#include <frc/DigitalInput.h>
#include <rev/CANSparkMax.h>
#include <frc/Joystick.h>
#include <ctre/Phoenix.h>
#include <frc/Solenoid.h>
#include <frc/Talon.h>
#include <iostream>
#include "Robot.h"

#include "commonVariables.h"
#include "Constants.h"
#include "subSystems.h"

using namespace rev;
using namespace frc;
using namespace std;
using namespace ctre;

double clamp(double in, double minval, double maxval)
{
  if (in > maxval)
    return maxval;
  if (in < minval)
    return minval;
  return in;
}

DigitalInput breakBeamTopIn(0);
DigitalOutput breakBeamTopOut(1);
DigitalInput breakBeamBottomIn(2);
DigitalOutput breakBeamBottomOut(3);

Joystick driveController{0};
Joystick operatorController{1};

//      Motors      //

WPI_TalonSRX intakeMotor{0};
CANSparkMax LDriveMotor{1, CANSparkMax::MotorType::kBrushless};
CANSparkMax LDriveMotor2{2, CANSparkMax::MotorType::kBrushless};
CANSparkMax RDriveMotor{3, CANSparkMax::MotorType::kBrushless};
CANSparkMax RDriveMotor2{4, CANSparkMax::MotorType::kBrushless};
WPI_TalonFX flyWheelL{5};
WPI_TalonFX flyWheelR{6};
CANSparkMax Neo1{7, CANSparkMax::MotorType::kBrushless};
CANSparkMax Neo2{8, CANSparkMax::MotorType::kBrushless};
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

//      XY Axes Defines Driver      //
double leftAxisX0{driveController.GetRawAxis(0)};
double leftAxisY0{driveController.GetRawAxis(1)}; //took out * -1
double rightAxisX0{driveController.GetRawAxis(2)};
double rightAxisY0{driveController.GetRawAxis(3)}; //took out * -1

//      XABY Button Defines Driver     //
bool btnX0{driveController.GetRawButton(0)};
bool btnA0{driveController.GetRawButton(1)};
bool btnB0{driveController.GetRawButton(2)};
bool btnY0{driveController.GetRawButton(3)};

//      Bumper and Trigger Defines Driver        //
bool leftBumper0{driveController.GetRawButton(4)};
bool rightBumper0{driveController.GetRawButton(5)};
bool leftTrigger0{driveController.GetRawButton(6)};
bool rightTrigger0{driveController.GetRawButton(7)};

//      Misc. Button Defines Driver      //
bool btnBack0{driveController.GetRawButton(8)};
bool btnStart0{driveController.GetRawButton(9)};
bool leftJoystickClick0{driveController.GetRawButton(10)};
bool rightJoystickClick0{driveController.GetRawButton(11)};

//      XY Operator Axes Defines      //
double leftAxisX1{operatorController.GetRawAxis(0)};
double leftAxisY1{operatorController.GetRawAxis(1)}; //took out * -1
double rightAxisX1{operatorController.GetRawAxis(2)};
double rightAxisY1{operatorController.GetRawAxis(3)}; //too out * -1

//      XABY Operator Button Defines     //
bool btnX1{operatorController.GetRawButton(0)};
bool btnA1{operatorController.GetRawButton(1)};
bool btnB1{operatorController.GetRawButton(2)};
bool btnY1{operatorController.GetRawButton(3)};

//      Operator Bumper and Trigger Defines        //
bool leftBumper1{operatorController.GetRawButton(4)};
bool rightBumper1{operatorController.GetRawButton(5)};
bool leftTrigger1{operatorController.GetRawButton(6)};
bool rightTrigger1{operatorController.GetRawButton(7)};

//     Operator Misc. Button Defines      //
bool btnBack1{operatorController.GetRawButton(8)};
bool btnStart1{operatorController.GetRawButton(9)};
bool leftJoystickClick1{operatorController.GetRawButton(10)};
bool rightJoystickClick1{operatorController.GetRawButton(11)};

//      Break Beam Variables      //
bool breakBeamBottom{breakBeamBottomIn.Get()};
bool breakBeamTop{breakBeamTopIn.Get()};

//      Misc. Defines       //
int ballCount;
bool firstTop{true};
bool firstBottom{true};
int timer;

//      Drive Variables     //
double driveStraight{0};
double driveTurn{0};
float driveMinPower{0.3};
float turnMinPower{0.4};
float modValue{0.5};
float modValueTurn{0.5};
bool nitros{false};

//      Drive Train     //
DifferentialDrive ArcadeDrive(LDriveMotor, RDriveMotor);

string _sb;
int _loops = 0;

void Robot::RobotInit()
{
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  SmartDashboard::PutData("Auto Modes", &m_chooser);
  CameraServer::GetInstance()->StartAutomaticCapture(0);

  flyWheelL.ConfigFactoryDefault();
  /* first choose the sensor */

  flyWheelL.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, kTimeoutMs);
  flyWheelL.SetSensorPhase(true);

  /* set the peak and nominal outputs */
  flyWheelL.ConfigNominalOutputForward(0, kTimeoutMs);
  flyWheelL.ConfigNominalOutputReverse(0, kTimeoutMs);
  flyWheelL.ConfigPeakOutputForward(1, kTimeoutMs);
  flyWheelL.ConfigPeakOutputReverse(-1, kTimeoutMs);
  /* set closed loop gains in slot0 */
  flyWheelL.Config_kF(kPIDLoopIdx, 0.1097, kTimeoutMs);
  flyWheelL.Config_kP(kPIDLoopIdx, 0.22, kTimeoutMs);
  flyWheelL.Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
  flyWheelL.Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);

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

void Robot::RobotPeriodic() { frc2::CommandScheduler::GetInstance().Run(); }

void Robot::AutonomousInit()
{
  m_autoSelected = m_chooser.GetSelected();
  //m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  cout << "Auto selected: " << m_autoSelected << endl;

  if (m_autoSelected == kAutoNameCustom)
  {
    // Custom Auto goes here
  }
  else
  {
    // Default Auto goes here
  }

  if (m_autonomousCommand != nullptr)
  {
    m_autonomousCommand->Schedule();
  }
}
int clicks = 21;

void forwardFunction(float distance)
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

void turnFunction(float distance)
{
  float desiredDistance{distance / 2};
  while (Rencoder.GetPosition() != desiredDistance && Lencoder.GetPosition() != desiredDistance)
  {
    LpidController.SetReference(desiredDistance, ControlType::kSmartMotion);
    RpidController.SetReference(-desiredDistance, ControlType::kSmartMotion);
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

  // if PID coefficients on the SmartDashboard have changed, write new values to controller //

  if ((p != kP)) // Reset the Proportional if it has changed
  {
    LpidController.SetP(p);
    kP = p;
    RpidController.SetP(p);
    kP = p;
  }

  if ((i != kI)) // Reset the Integral if it has changed
  {
    LpidController.SetI(i);
    kI = i;
    RpidController.SetI(i);
    kI = i;
  }

  if ((d != kD)) // Reset the Derivative if it has changed
  {
    LpidController.SetD(d);
    kD = d;
    RpidController.SetD(d);
    kD = d;
  }

  if ((iz != kIz)) // Reset the Integral zone if it has changed
  {
    LpidController.SetIZone(iz);
    kIz = iz;
    RpidController.SetIZone(iz);
    kIz = iz;
  }

  if ((ff != kFF)) // Reset the feed-forward if incorrect
  {
    LpidController.SetFF(ff);
    kFF = ff;
    RpidController.SetFF(ff);
    kFF = ff;
  }

  if ((max != kMaxOutput) || (min != kMinOutput)) // If the max output or the min output are wrong, reset the range
  {
    LpidController.SetOutputRange(min, max);
    RpidController.SetOutputRange(min, max);
    kMinOutput = min;
    kMaxOutput = max;
  }

  if ((maxV != kMaxVel)) // Reset the max velocity if it has changed
  {
    LpidController.SetSmartMotionMaxVelocity(maxV);
    RpidController.SetSmartMotionMaxVelocity(maxV);
    kMaxVel = maxV;
  }

  if ((minV != kMinVel)) // Reset the min output if it has changed
  {
    LpidController.SetSmartMotionMinOutputVelocity(minV);
    RpidController.SetSmartMotionMinOutputVelocity(minV);
    kMinVel = minV;
  }

  if ((maxA != kMaxAcc)) // Reset the max acceleration if it has changed
  {
    LpidController.SetSmartMotionMaxAccel(maxA);
    RpidController.SetSmartMotionMaxAccel(maxA);
    kMaxAcc = maxA;
  }

  if ((allE != kAllErr)) // Reset the allowed error if it has changed
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
      LpidController.SetReferencce(SetPoint, ControlType::kSmartMotion);
      RpidController.SetReference(SetPoint, ControlType::kSmartMotion);
      ProcessVariable = Lencoder.GetPosition();
      RProcessVariable = Rencoder.GetPosition();
    }
    else
    {
      SetPoint = SmartDashboard::GetNumber("Set Position", 0) * clicks
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

//double FalconVelocity;

void Robot::TeleopInit()
{
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand != nullptr)
  {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
  }
}
int counter;
void ball() // This will increment the number of balls
{
  counter++;
  cout << "ball " << counter << endl;
}

void Robot::TeleopPeriodic()
{
  Update_Limelight_Tracking();

  btnX0 = driveController.GetRawButton(0);

  SmartDashboard::PutBoolean("Beam", breakBeamBottomIn.Get());
  SmartDashboard::PutNumber("Velocity", flyWheelL.GetSelectedSensorVelocity(kPIDLoopIdx));
  SmartDashboard::PutNumber("Balls", ballCount);

  double motorOutput{flyWheelL.GetMotorOutputPercent()};

  _sb.append("\tout:");
  _sb.append(to_string(motorOutput));
  _sb.append("\tspd:");
  _sb.append(to_string(flyWheelL.GetSelectedSensorVelocity(kPIDLoopIdx)));

  if (btnA0)
  {
    double targetVelocity_UnitsPer100ms = leftAxisY0 * 500 * 4096 / 600;

    flyWheelL.Set(ControlMode::Velocity, targetVelocity_UnitsPer100ms);

    _sb.append("\terrNative:");
    _sb.append(to_string(flyWheelL.GetClosedLoopError(kPIDLoopIdx)));
    _sb.append("\ttrg");
    _sb.append(to_string(targetVelocity_UnitsPer100ms));
  }
  else
  {
    flyWheelL.Set(ControlMode::PercentOutput, leftAxisY0);
  }

  if (++_loops >= 10) // print every 10 times
  {
    _loops = 0;
    printf("%s\n", _sb.c_str());
  }
  _sb.clear();

  // Deadzone for drive
  if (rightAxisX0 > -0.05 && rightAxisX0 < 0.05)
    rightAxisX0 = 0;

  if (leftAxisY0 > -0.05 && leftAxisY0 < 0.05)
    leftAxisY0 = 0;

  // Non-linear drive math
  driveMinPower = .15;
  modValue = 1;

  if (leftAxisY0 > 0)
    driveStraight = (driveMinPower + (1 - driveMinPower) * (modValue * (pow(leftAxisY0, 3) + (1 - modValue) * leftAxisY0)));
  else if (leftAxisY0 < 0)
    driveStraight = ((-1 * driveMinPower) + (1 - driveMinPower) * (modValue * (pow(leftAxisY0, 3) + (1 - modValue) * leftAxisY0)));
  else
    driveStraight = 0;

  if (rightAxisX0 > 0)
    driveTurn = (turnMinPower + (1 - turnMinPower) * (modValueTurn * (pow(rightAxisX0, 3) + (1 - modValueTurn) * rightAxisX0)));
  else if (rightAxisX0 < 0)
    driveTurn = ((-1 * turnMinPower) + (1 - turnMinPower) * (modValueTurn * (pow(rightAxisX0, 3) + (1 - modValueTurn) * rightAxisX0)));
  else
    driveTurn = 0;

  ArcadeDrive.ArcadeDrive(-1 * driveStraight, driveTurn);

  shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

  double ta = table->GetNumber("ta", 0.0);

  if (btnY0)
  {
    Neo1.Set(1);
    Neo2.Set(1);
  }
  else
  {
    Neo1.Set(0);
    Neo2.Set(0);
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
  const double DESIRED_TARGET_AREA = 49.0;
  const double MAX_DRIVE = 0.25;
  const double MAX_STEER = 0.4f;

  shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  double tx = table->GetNumber("tx", 0.0);
  double ty = table->GetNumber("ty", 0.0);
  double ta = table->GetNumber("ta", 0.0);
  double tv = table->GetNumber("tv", 0.0);

  if (tv < 1.0)
  {
    m_LimelightHasTarget = false;
    m_LimelightDriveCmd = 0;
    m_LimelightTurnCmd = 0;
  }
  else
  {
    m_LimelightHasTarget = true;
    // Proportional steering
    m_LimelightTurnCmd = tx * STEER_K;
    m_LimelightTurnCmd = clamp(m_LimelightTurnCmd, -MAX_STEER, MAX_STEER);
    // drive forward until the target area reaches our desired area
    if (ta > 49.0)
    {
      m_LimelightDriveCmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;
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