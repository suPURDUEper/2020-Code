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
#include "launcher.h"

#include "commonVariables.h"
#include "driveMath.h"
#include "Autonomous.h"
#include "Constants.h"

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

DigitalInput breakBeamFull(3);
DigitalInput breakBeamNewBall(2);
DigitalInput breakBeamFourthBall(1);
DigitalInput breakBeamFifthBall(0);

Joystick driveController{0};
Joystick operatorController{1};

//      Motors      //

WPI_TalonFX flyWheelL{0};
WPI_TalonFX flyWheelR{1};
WPI_TalonFX climbL{2};
WPI_TalonFX climbR{3};
WPI_TalonSRX intakeMotor{5};
WPI_TalonSRX conveyorMotor{4};
WPI_VictorSPX vMotor1{0};
WPI_VictorSPX vMotor2{1};
WPI_VictorSPX intakeFollower{2};
CANSparkMax indexMotor{5, CANSparkMax::MotorType::kBrushless};

//      Solenoids     //
Solenoid brakeLift{4};
DoubleSolenoid intakeSolenoid{2, 3};
DoubleSolenoid hoodSolenoid{0, 1};

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
bool btnX0{driveController.GetRawButton(1)};
bool btnA0{driveController.GetRawButton(2)};
bool btnB0{driveController.GetRawButton(3)};
bool btnY0{driveController.GetRawButton(4)};

//      Bumper and Trigger Defines Driver        //
bool leftBumper0{driveController.GetRawButton(5)};
bool rightBumper0{driveController.GetRawButton(6)};
bool leftTrigger0{driveController.GetRawButton(7)};
bool rightTrigger0{driveController.GetRawButton(8)};

//      Misc. Button Defines Driver      //
bool btnBack0{driveController.GetRawButton(9)};
bool btnStart0{driveController.GetRawButton(10)};
bool leftJoystickClick0{driveController.GetRawButton(11)};
bool rightJoystickClick0{driveController.GetRawButton(12)};

//      XY Operator Axes Defines      //
double leftAxisX1{operatorController.GetRawAxis(0)};
double leftAxisY1{operatorController.GetRawAxis(1)}; //took out * -1
double rightAxisX1{operatorController.GetRawAxis(2)};
double rightAxisY1{operatorController.GetRawAxis(3)}; //too out * -1

//      XABY Operator Button Defines     //
bool btnX1{operatorController.GetRawButton(1)};
bool btnA1{operatorController.GetRawButton(2)};
bool btnB1{operatorController.GetRawButton(3)};
bool btnY1{operatorController.GetRawButton(4)};

//      Operator Bumper and Trigger Defines        //
bool leftBumper1{operatorController.GetRawButton(5)};
bool rightBumper1{operatorController.GetRawButton(6)};
bool leftTrigger1{operatorController.GetRawButton(7)};
bool rightTrigger1{operatorController.GetRawButton(8)};

//     Operator Misc. Button Defines      //
bool btnBack1{operatorController.GetRawButton(9)};
bool btnStart1{operatorController.GetRawButton(10)};
bool leftJoystickClick1{operatorController.GetRawButton(11)};
bool rightJoystickClick1{operatorController.GetRawButton(12)};

//      Misc. Defines       //

bool firstTop{true};
bool firstBottom{true};
Timer timer;
int clicks = 21;
bool intaketoggled = false;
bool lastRightJoyClick = false;

//      Drive Variables     //
bool nitros{false};

//      Drive Train     //
DifferentialDrive ArcadeDrive(LDriveMotor, RDriveMotor);

string _sb;
int _loops = 0;

void Robot::RobotInit()
{
  intakeSolenoid.Set(DoubleSolenoid::Value::kForward);
  intakeMotor.ConfigContinuousCurrentLimit(5, 0);
  flyWheelL.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 40, 40, 0), 0);
  flyWheelR.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 40, 40, 0), 0);
  conveyorMotor.ConfigContinuousCurrentLimit(5, 0);
  indexMotor.SetSmartCurrentLimit(5);
  climbL.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 40, 40, 0), 0);
  climbR.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 40, 40, 0), 0);
  conveyorMotor.SetInverted(true);
  conveyorMotor.SetNeutralMode(Brake);
  indexMotor.SetInverted(true);
  intakeFollower.Follow(intakeMotor);

  brakeLift.Set(true);
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  SmartDashboard::PutData("Auto Modes", &m_chooser);
  CameraServer::GetInstance()->StartAutomaticCapture(0);

  flyWheelL.ConfigFactoryDefault();
  /* first choose the sensor */

  flyWheelL.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, kTimeoutMs);
  flyWheelL.SetSensorPhase(true);

  /* set the peak and nominal outputs */
  /*
  flyWheelL.ConfigNominalOutputForward(0, kTimeoutMs);
  flyWheelL.ConfigNominalOutputReverse(0, kTimeoutMs);
  flyWheelL.ConfigPeakOutputForward(1, kTimeoutMs);
  flyWheelL.ConfigPeakOutputReverse(-1, kTimeoutMs);
  */
  flyWheelL.SetInverted(true);

  // set motor follow //
  flyWheelR.Follow(flyWheelL);
  flyWheelR.SetInverted(false);

  flyWheelL.Config_kF(kPIDLoopIdx, 0.04, kTimeoutMs);
  flyWheelL.Config_kP(kPIDLoopIdx, 0.22, kTimeoutMs);
  flyWheelL.Config_kI(kPIDLoopIdx, 0, kTimeoutMs);
  flyWheelL.Config_kD(kPIDLoopIdx, 0, kTimeoutMs);

  flyWheelL.SetNeutralMode(Coast);
  flyWheelR.SetNeutralMode(Coast);

  double trenchSpeed = SmartDashboard::PutNumber("Trench Speed", 6000);
  double initSpeed = SmartDashboard::PutNumber("Init Line Speed", 4500);
  double wallSpeed = SmartDashboard::PutNumber("Wall Speed", 3000);

  climbR.SetInverted(true);

  LDriveMotor.RestoreFactoryDefaults();
  RDriveMotor.RestoreFactoryDefaults();
  LDriveMotor2.Follow(LDriveMotor);
  RDriveMotor2.Follow(RDriveMotor);

  //        SparkMAX PID defines        //

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

  //    Display PID for DriveTrain      //

  /*SmartDashboard::PutNumber("P Gain", kP);
  SmartDashboard::PutNumber("I Gain", kI);
  SmartDashboard::PutNumber("D Gain", kD);
  SmartDashboard::PutNumber("I Zone", kIz);
  SmartDashboard::PutNumber("Feed Forward", kFF);
  SmartDashboard::PutNumber("Max Output", kMaxOutput);
  SmartDashboard::PutNumber("Min Output", kMinOutput); */

  SmartDashboard::PutNumber("Fly Wheel Velocity", flyWheelL.GetSelectedSensorVelocity() / 3.4133);
}

void Robot::RobotPeriodic()
{
  frc2::CommandScheduler::GetInstance().Run();
}

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
  timer.Reset();
  timer.Start();

  SmartDashboard::PutNumber("SetF", 0.1027);
  SmartDashboard::PutNumber("SetP", 0.22);
  SmartDashboard::PutNumber("SetI", 0);
  SmartDashboard::PutNumber("SetD", 0);

  double flyWheelF = SmartDashboard::GetNumber("SetF", 0.0453);
  double flyWheelP = SmartDashboard::GetNumber("SetP", 0.15);
  double flyWheelI = SmartDashboard::GetNumber("SetI", 0);
  double flyWheelD = SmartDashboard::GetNumber("SetD", 1.5);

  flyWheelL.Config_kF(kPIDLoopIdx, flyWheelF, kTimeoutMs);
  flyWheelL.Config_kP(kPIDLoopIdx, flyWheelP, kTimeoutMs);
  flyWheelL.Config_kI(kPIDLoopIdx, flyWheelI, kTimeoutMs);
  flyWheelL.Config_kD(kPIDLoopIdx, flyWheelD, kTimeoutMs);
}

void Robot::AutonomousPeriodic()
{
  float phase1 = 1.500;
  float phase2 = 2.000 + phase1;
  float phase3 = 0.500 + phase2;
  float phase4 = 2.500 + phase3;
  if (m_autoSelected == kAutoNameCustom)
  {
    if (0 < timer.Get() && timer.Get() < phase1)
    {
      cout << "phase1 " << timer.Get() << endl;
      flyWheelL.Set(ControlMode::Velocity, 5000 * 3.4133);
      ArcadeDrive.ArcadeDrive(0, 0);
    }
    else if (phase1 < timer.Get() && timer.Get() < phase2)
    {
      cout << "phase2 " << timer.Get() << endl;
      conveyorMotor.Set(0.5);
      indexMotor.Set(0.7);
      ArcadeDrive.ArcadeDrive(0, 0);
    }
    else if (phase2 < timer.Get() && timer.Get() < phase3)
    {
      cout << "phase3 " << timer.Get() << endl;
      flyWheelL.Set(0);
      conveyorMotor.Set(0);
      indexMotor.Set(0);
      ArcadeDrive.ArcadeDrive(0.6, 0);
    }
    else if (phase3 < timer.Get() && timer.Get() < phase4)
    {
      ArcadeDrive.ArcadeDrive(-0.3, 0);
    }
    else
    {
      cout << "phase4 " << timer.Get() << endl;
      ArcadeDrive.ArcadeDrive(0, 0);
      indexMotor.Set(0);
      conveyorMotor.Set(0);
      flyWheelL.Set(0);
    }
  }
}

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

  SmartDashboard::PutNumber("Set FlyWheelV", 0);
}

void Robot::TeleopPeriodic()
{
  btnStart0 = driveController.GetRawButton(10);
  Update_Limelight_Tracking();

  SmartDashboard::PutBoolean("Lined up?", limelightAligned);

  //--------------- PID for SparkMax ------------------------------------//

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

  //-------------- if PID coefficients on the SmartDashboard have changed, write new values to controller, for SPARKMAX -----------------------------//

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

  if ((allE != kAllErr)) // Reset the allowed error if it has changed, SPARKMAX PID
  {
    LpidController.SetSmartMotionAllowedClosedLoopError(allE);
    RpidController.SetSmartMotionAllowedClosedLoopError(allE);
    allE = kAllErr;
  }

  /* --------------- Update Controller Values ----------------*/

  leftBumper1 = operatorController.GetRawButton(5);
  rightBumper1 = operatorController.GetRawButton(6);
  leftTrigger1 = operatorController.GetRawButton(7);
  rightTrigger1 = operatorController.GetRawButton(8);

  leftBumper0 = driveController.GetRawButton(5);
  leftTrigger0 = driveController.GetRawButton(7);
  rightTrigger0 = driveController.GetRawButton(8);

  btnX1 = operatorController.GetRawButton(1);
  btnA1 = operatorController.GetRawButton(2);
  btnB1 = operatorController.GetRawButton(3);
  btnY1 = operatorController.GetRawButton(4);

  leftAxisY0 = driveController.GetRawAxis(1);
  rightAxisX0 = driveController.GetRawAxis(2);

  btnBack1 = operatorController.GetRawButton(9);
  btnStart1 = operatorController.GetRawButton(10);
  leftJoystickClick1 = operatorController.GetRawButton(11);
  rightJoystickClick1 = operatorController.GetRawButton(12);

  //--------------------------------------------------------------------------------------------------//

  SmartDashboard::PutNumber("Current FlyWheelV", flyWheelL.GetSelectedSensorVelocity(kPIDLoopIdx) / 3.4133);

  //            Falcon PID configs              //
  SmartDashboard::PutNumber("SetF", 0.1027);
  SmartDashboard::PutNumber("SetP", 0.22);
  SmartDashboard::PutNumber("SetI", 0);
  SmartDashboard::PutNumber("SetD", 0);

  double flyWheelF = SmartDashboard::GetNumber("SetF", 0.0453);
  double flyWheelP = SmartDashboard::GetNumber("SetP", 0.15);
  double flyWheelI = SmartDashboard::GetNumber("SetI", 0);
  double flyWheelD = SmartDashboard::GetNumber("SetD", 1.5);

  double trenchSpeed = SmartDashboard::GetNumber("Trench Speed", 6000);
  double initSpeed = SmartDashboard::GetNumber("Init Line Speed", 4500);
  double wallSpeed = SmartDashboard::GetNumber("Wall Speed", 3000);

  flyWheelL.Config_kF(kPIDLoopIdx, flyWheelF, kTimeoutMs);
  flyWheelL.Config_kP(kPIDLoopIdx, flyWheelP, kTimeoutMs);
  flyWheelL.Config_kI(kPIDLoopIdx, flyWheelI, kTimeoutMs);
  flyWheelL.Config_kD(kPIDLoopIdx, flyWheelD, kTimeoutMs);

  // Three Speed Shooter Control
  if (launcher(trenchSpeed, initSpeed, wallSpeed) == 0)
    flyWheelL.Set(0);
  else
    flyWheelL.Set(ControlMode::Velocity, launcher(trenchSpeed, initSpeed, wallSpeed));

  SmartDashboard::PutNumber("Returned Value", launcher(trenchSpeed, initSpeed, wallSpeed));
  SmartDashboard::PutBoolean("Spun Up?", spunUp);

  if (rightTrigger0)
  {
    conveyorMotor.Set(.5);
    indexMotor.Set(.9);
  }
  else if (leftTrigger0)
  {
    intakeMotor.Set(.6);
    vMotor1.Set(0.7);
    vMotor2.Set(0.9);

    if (!breakBeamFull.Get() && !breakBeamNewBall.Get() && !breakBeamFourthBall.Get() && !breakBeamFifthBall.Get())
      indexMotor.Set(0);
    else
      indexMotor.Set(0.5); //changed from .25

    if (!breakBeamFull.Get())
      conveyorMotor.Set(0);
    else if (!breakBeamNewBall.Get())
      conveyorMotor.Set(0.3);
    else
      conveyorMotor.Set(0);
  }
  else if (leftBumper0)
  {
    intakeMotor.Set(0);
    vMotor1.Set(0.7);
    vMotor2.Set(0.9);
    indexMotor.Set(0.5);

    if (!breakBeamFull.Get() && !breakBeamNewBall.Get() && !breakBeamFourthBall.Get() && !breakBeamFifthBall.Get())
      indexMotor.Set(0);
    else
      indexMotor.Set(0.25);

    if (!breakBeamFull.Get())
      conveyorMotor.Set(0);
    else if (!breakBeamNewBall.Get())
      conveyorMotor.Set(0.3);
    else
      conveyorMotor.Set(0);
  }
  else
  {
    intakeMotor.Set(0);
    vMotor1.Set(0);
    vMotor2.Set(0);
    indexMotor.Set(0);
    conveyorMotor.Set(0);
  }

  // Making Climb Move
  if (rightBumper1)
  {
    climbL.Set(0.5);
    climbR.Set(0.5);
  }
  else if (rightTrigger1)
  {
    climbL.Set(-0.8);
    climbR.Set(-0.8);
  }
  else
  {
    climbL.Set(0);
    climbR.Set(0);
  }

  if (leftJoystickClick1)
    brakeLift.Set(false);
  else
    brakeLift.Set(true);

  if (btnBack1)
    hoodSolenoid.Set(DoubleSolenoid::Value::kReverse);
  else if (btnStart1)
    hoodSolenoid.Set(DoubleSolenoid::Value::kForward);

  if (rightJoystickClick1 && !lastRightJoyClick)
  {
    if (intaketoggled)
    {
      intakeSolenoid.Set(DoubleSolenoid::Value::kForward);
      intaketoggled = false;
    }
    else if (!intaketoggled)
    {
      intakeSolenoid.Set(DoubleSolenoid::Value::kReverse);
      intaketoggled = true;
    }
  }
  lastRightJoyClick = rightJoystickClick1;

  cout << "DriveStraight " << straightMath(0.15, 0.5, leftAxisY0) << ", ";
  cout << "Turn " << turnMath(0.18, 0.5, rightAxisX0) << endl;

  if (btnStart0)
  {
    if (m_LimelightHasTarget)
    {
      ArcadeDrive.ArcadeDrive(-1 * straightMath(0.15, 0.5, leftAxisY0), m_LimelightTurnCmd);
    }
    else
    {
      ArcadeDrive.ArcadeDrive(0, 0);
    }
  }
  else
  {
    ArcadeDrive.ArcadeDrive(-1 * straightMath(0.15, .5, leftAxisY0), turnMath(0.18, 0.5, rightAxisX0));
  }

  shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

  double ta = table->GetNumber("ta", 0.0);
}

void Robot::TestPeriodic() {}

bool limelightAligned = false;

void Robot::Update_Limelight_Tracking()
{
  //Proportional Steering Constant
  const double STEER_K = 0.14;
  float ki = 0.003;
  //Proportional Drive Constant
  const double DRIVE_K = 0.26;

  //Area of tape when robot has reached the goal
  const double DESIRED_TARGET_AREA = 49.0;
  const double MAX_DRIVE = 0.25;
  const double MAX_STEER = 0.33f;
  float min_command = 0.14f;

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
    float steering_adjust = 0.0f;
    if (tx > 2.5 || tx < -2.5)
    {
      limelightIntegral = 0;
    }
    else
    {
      limelightIntegral = limelightIntegral + tx;
    }

    if (tx > 0.2)
    {
      cout << limelightIntegral << endl;
      m_LimelightTurnCmd = tx * STEER_K + limelightIntegral * ki - min_command;
      limelightAligned = false;
    }
    else if (tx < 0.2)
    {
      cout << limelightIntegral << endl;
      m_LimelightTurnCmd = tx * STEER_K + limelightIntegral * ki + min_command;
      limelightAligned = false;
    }
    else
    {
      limelightAligned = true;
    }

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