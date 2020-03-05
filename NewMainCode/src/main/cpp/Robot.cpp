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

// default PID coefficients //kp default 5e-5 kff default 0.000156
double kP = 7e-5, kI = 1e-6, kD = 8e-4, kIz = 0, kFF = 0.000156, kMaxOutput = 1, kMinOutput = -1;

// default smart motion coefficients
double kMaxVel = 1800, kMinVel = 0, kMaxAcc = 2000, kAllErr = 1;

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
int ledMode;

void Robot::RobotInit()
{
  ledMode = 1;
  Update_Limelight_Tracking(ledMode);

  intakeSolenoid.Set(DoubleSolenoid::Value::kForward);
  intakeMotor.ConfigContinuousCurrentLimit(5, 0);
  flyWheelL.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 40, 40, 0), 0);
  flyWheelR.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 40, 40, 0), 0);
  conveyorMotor.ConfigContinuousCurrentLimit(5, 0);
  indexMotor.SetSmartCurrentLimit(5);
  climbL.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 40, 40, 0), 0);
  climbR.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 40, 40, 0), 0);

  conveyorMotor.SetInverted(false); //was true, then motor got flipped physically
  conveyorMotor.SetNeutralMode(Brake);
  indexMotor.SetInverted(true);
  intakeFollower.Follow(intakeMotor);

  brakeLift.Set(true);
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  //SmartDashboard::PutData("Auto Modes", &m_chooser);
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

  flyWheelL.SetNeutralMode(Coast);
  flyWheelR.SetNeutralMode(Coast);

  //      Speed Variables      //
  /*
  double trenchSpeed = SmartDashboard::PutNumber("Trench Speed", 6000);
  double initSpeed = SmartDashboard::PutNumber("Init Line Speed", 5500);
  double wallSpeed = SmartDashboard::PutNumber("Wall Speed", 2700);
  */

  double fastConveyor = 1.0;
  double medConveyor = 0.7;
  double slowConveyor = 0.5;

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

  SmartDashboard::PutNumber("Fly Wheel Velocity", flyWheelL.GetSelectedSensorVelocity() / 3.4133);
}

void Robot::RobotPeriodic()
{
  frc2::CommandScheduler::GetInstance().Run();
}

int phase;

void auton1()
{
  float phase1 = 1.000;
  float phase2 = 1.000 + phase1;
  float phase3 = 0.500 + phase2;
  float phase4 = 2.500 + phase3;
  if (0 < timer.Get() && timer.Get() < phase1)
  {
    //      set flywheel velocity     //
    cout << "phase1 " << timer.Get() << endl;
    flyWheelL.Set(ControlMode::Velocity, 5000 * 3.4133);
    //ArcadeDrive.ArcadeDrive(0, 0);
  }
  else if (phase1 < timer.Get() && timer.Get() < phase2)
  {
    //      turn on conveyor belt and index motor     //
    cout << "phase2 " << timer.Get() << endl;
    conveyorMotor.Set(0.5);
    indexMotor.Set(0.7);
    //ArcadeDrive.ArcadeDrive(0, 0);
  }
  else if (phase2 < timer.Get() && timer.Get() < phase3)
  {
    //      turn off flywheel, indexer, and conveyor      //
    cout << "phase3 " << timer.Get() << endl;
    flyWheelL.Set(0);
    conveyorMotor.Set(0);
    indexMotor.Set(0);
    //ArcadeDrive.ArcadeDrive(0.6, 0);
  }
  else if (phase3 < timer.Get() && timer.Get() < phase4)
  {
    //ArcadeDrive.ArcadeDrive(-0.3, 0);
  }
  else
  {
    cout << "phase4 " << timer.Get() << endl;
    //ArcadeDrive.ArcadeDrive(0, 0);
    indexMotor.Set(0);
    conveyorMotor.Set(0);
    flyWheelL.Set(0);
  }
}

void auton2(float m_LimelightTurnCmd)
{
  float phase1 = 0.25;            //spin up
  float phase2 = 0.500 + phase1;  //unload
  float phase3 = 0.025 + phase2;  //turn off shooter
  float phase4 = 0.025 + phase3;  //back up // eliminated for now
  float phase5 = 1.0 + phase4;    //turn //was 0.75
  float phase6 = 2.0 + phase5;    //back up to line up with trench balls
  float phase7 = 1.5 + phase6;    //turn to face trench balls
  float phase8 = 0.5 + phase7;    //lower intake (already done in last phase)
  float phase9 = 2.5 + phase8;    // drive forward to intake balls
  float phase10 = 0.5 + phase9;   //short turn before backing up
  float phase11 = 2.25 + phase10; //backing up
  float phase12 = 1.75 + phase11; //turning to face goal
  float phase13 = 1.2 + phase12;  //limelight time
  float phase14 = 1 + phase13;    //shooting
  float phase15 = .025 + phase14;
  ArcadeDrive.FeedWatchdog();

  shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  double tx = table->GetNumber("tx", 0.0);

  LpidController.SetReference(leftDriveGoal, ControlType::kSmartMotion);
  RpidController.SetReference(rightDriveGoal, ControlType::kSmartMotion);

  //Turn on Flywheel and let it get up to speed
  if (0 < timer.Get() && timer.Get() < phase1)
  {
    if (phase == 0)
    {

      phase = 1;
    }
    flyWheelL.Set(ControlMode::Velocity, 6000 * 3.4133); //change in auton1 if this works
    cout << "phase1 " << timer.Get() << endl;
  }
  // Launch 3 preload balls
  else if (phase1 < timer.Get() && timer.Get() < phase2)
  {
    if (phase == 1)
    {

      phase = 2;
    }
    conveyorMotor.Set(1);
    indexMotor.Set(0.7);
    cout << "phase2 " << timer.Get() << endl;
  }
  //Turn off motors from launching preloads
  else if (phase2 < timer.Get() && timer.Get() < phase3)
  {
    if (phase == 2)
    {

      phase = 3;
    }
    flyWheelL.Set(0);
    conveyorMotor.Set(0);
    indexMotor.Set(0);
    cout << "phase3 " << timer.Get() << endl;
  }
  //Backup from initiation line
  else if (phase3 < timer.Get() && timer.Get() < phase4)
  {
    if (phase == 3)
    {
      phase = 4;
    }
    cout << "phase4 " << timer.Get() << endl;
  }
  //First 90 degree backing up turn
  else if (phase4 < timer.Get() && timer.Get() < phase5)
  {
    if (phase == 4)
    {
      rightSweep(-50);
      phase = 5;
    }
    cout << "phase5 " << timer.Get() << endl;
  }
  //Backup to trench
  else if (phase5 < timer.Get() && timer.Get() < phase6)
  {
    if (phase == 5)
    {
      driveStraight(-80);
      phase = 6;
    }
    cout << "phase6 " << timer.Get() << endl;
  }
  //2nd 90 degree backing up turn
  else if (phase6 < timer.Get() && timer.Get() < phase7)
  {
    if (phase == 6)
    {
      turn(-131);
      phase = 7;
    }
    cout << "phase7 " << timer.Get() << endl;
  }
  //Lower intake to prepare to take in 3 trench balls
  else if (phase7 < timer.Get() && timer.Get() < phase8)
  {
    if (phase == 7)
    {

      driveStraight(0);
      phase = 8;
    }

    intakeSolenoid.Set(DoubleSolenoid::Value::kReverse);

    cout << "phase8 " << timer.Get() << endl;
  }
  //Drive forward to get trench balls... should be 90? (testing at less)
  else if (phase8 < timer.Get() && timer.Get() < phase9)
  {
    if (phase == 8)
    {
      driveStraight(66);
      phase = 9;
    }
    cout << "phase9 " << timer.Get() << endl;
  }
  //Turning before leaving trench
  else if (phase9 < timer.Get() && timer.Get() < phase10)
  {
    if (phase == 9)
    {
      rightSweep(-20);
      phase = 10;
    }
    cout << "phase10 " << timer.Get() << endl;
  }
  //Backing up out of trench... should be 90?
  else if (phase10 < timer.Get() && timer.Get() < phase11)
  {
    if (phase == 10)
    {

      driveStraightFast(-124);
      phase = 11;
    }
    intakeSolenoid.Set(DoubleSolenoid::Value::kForward);
    indexMotor.Set(0);
    cout << "phase11 " << timer.Get() << endl;
  }
  //Turn around to face goal
  else if (phase11 < timer.Get() && timer.Get() < phase12)
  {
    if (phase == 11)
    {
      turn(-160);
      phase = 12;
    }
    cout << "phase12 " << timer.Get() << endl;
  }
  //Limelight aim at goal
  else if (phase12 < timer.Get() && timer.Get() < phase13)
  {
    if (phase == 12)
    {

      phase = 13;
    }
    ledMode = 3;
    ArcadeDrive.ArcadeDrive(0, m_LimelightTurnCmd);
    flyWheelL.Set(ControlMode::Velocity, 5500 * 3.4133);
    cout << "phase13 " << timer.Get() << endl;
  }
  //Fire 3 trench balls
  else if (phase13 < timer.Get() && timer.Get() < phase14)
  {
    if (phase == 13)
    {

      phase = 14;
    }

    ArcadeDrive.ArcadeDrive(0, m_LimelightTurnCmd);
    conveyorMotor.Set(1);
    indexMotor.Set(0.7);
    cout << "phase13 " << timer.Get() << endl;
  }
  //Turn off motors at end.
  else
  {
    // ArcadeDrive.ArcadeDrive(0, 0);
    ArcadeDrive.ArcadeDrive(0, m_LimelightTurnCmd);
    indexMotor.Set(0);
    conveyorMotor.Set(0);
    flyWheelL.Set(0);
  }
  //Breakbeam logic during these intaking phases
  if (phase == 8 || phase == 9 || phase == 10 || phase == 11)
  {
    if (phase == 11)
    {
      intakeMotor.Set(0);
    }
    else
    {
      intakeMotor.Set(1);
    }

    vMotor1.Set(0.7);
    vMotor2.Set(0.9);

    cout << breakBeamNewBall.Get() << endl;

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
  else if (phase == 10)
  {
    conveyorMotor.Set(0);
    intakeMotor.Set(0);
  }
}

void auton3(double m_LimelightTurnCmd)
{
  float phase1 = 2;              // Drive Forward lower hood
  float phase2 = 1 + phase1;     // Turn toward 2nd ball
  float phase3 = 0.025 + phase2; // Drive forward slightly
  float phase4 = 2.5 + phase3;   // Get outta there
  float phase5 = 2 + phase4;     // Turn left
  float phase6 = 2 + phase5;     // Drive across field
  float phase7 = 2 + phase6;     // Turn toward goal
  float phase8 = 1 + phase7;     // Limelight
  float phase9 = 1.5 + phase8;   // Shoot

  ArcadeDrive.FeedWatchdog();

  shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  double tx = table->GetNumber("tx", 0.0);

  LpidController.SetReference(leftDriveGoal, ControlType::kSmartMotion);
  RpidController.SetReference(rightDriveGoal, ControlType::kSmartMotion);

  //Drive into the trench and intake balls
  if (0 < timer.Get() && timer.Get() < phase1)
  {
    if (phase == 0)
    {
      driveStraightFast(91);
      phase = 1;
    }
    intakeSolenoid.Set(DoubleSolenoid::Value::kReverse);
    //change in auton1 if this works
    cout << "phase1 " << timer.Get() << endl;
  }
  // Drive away from trench
  else if (phase1 < timer.Get() && timer.Get() < phase2)
  {
    if (phase == 1)
    {
      turn(30);
      phase = 2;
    }

    cout << "phase2 " << timer.Get() << endl;
  }
  else if (phase2 < timer.Get() && timer.Get() < phase3)
  {
    if (phase == 2)
    {
      //driveStraight(6);
      phase = 3;
    }

    cout << "phase3 " << timer.Get() << endl;
  }
  else if (phase3 < timer.Get() && timer.Get() < phase4)
  {
    if (phase == 3)
    {
      driveStraightFast(-88);
      phase = 4;
    }

    cout << "phase3 " << timer.Get() << endl;
  }
  //Turn left 90 degrees
  else if (phase4 < timer.Get() && timer.Get() < phase5)
  {
    if (phase == 4)
    {
      turn(-118);
      phase = 5;
    }
    intakeSolenoid.Set(DoubleSolenoid::Value::kForward);
    cout << "phase4 " << timer.Get() << endl;
  }
  //race across the field
  else if (phase5 < timer.Get() && timer.Get() < phase6)
  {
    if (phase == 5)
    {
      driveStraight(90);
      phase = 6;
    }
    cout << "phase5 " << timer.Get() << endl;
  }
  //Turn 60 degrees to the left
  else if (phase6 < timer.Get() && timer.Get() < phase7)
  {
    if (phase == 6)
    {
      turn(-70);
      phase = 7;
    }
    cout << "phase7 " << timer.Get() << endl;
  }
  //Spin up
  else if (phase7 < timer.Get() && timer.Get() < phase8)
  {
    if (phase == 7)
    {
      ledMode = 3;
      phase = 8;
    }
    cout << "phase6 " << timer.Get() << endl;
    ArcadeDrive.ArcadeDrive(0, m_LimelightTurnCmd);
    flyWheelL.Set(ControlMode::Velocity, 5500 * 3.4133);
  }
  else if (phase8 < timer.Get() && timer.Get() < phase9)
  {
    if (phase == 8)
    {
      phase = 9;
    }

    ArcadeDrive.ArcadeDrive(0, m_LimelightTurnCmd);
    conveyorMotor.Set(1);
    indexMotor.Set(0.7);
    cout << "phase8 " << timer.Get() << endl;
  }
  //Breakbeam logic during these intaking phases
  if (phase == 1 || phase == 2 || phase == 3 || phase == 4 || phase == 5)
  {
    intakeMotor.Set(.8);
    vMotor1.Set(0.7);
    vMotor2.Set(0.9);

    cout << breakBeamNewBall.Get() << endl;

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
  else if (phase == 6)
  {
    conveyorMotor.Set(0);
    intakeMotor.Set(0);
    intakeSolenoid.Set(DoubleSolenoid::Value::kForward);
  }
}

void auton4()
{
  float phase1 = 3.00;
  float phase2 = 3.00 + phase1;
  ArcadeDrive.FeedWatchdog();

  LpidController.SetReference(leftDriveGoal, ControlType::kSmartMotion);
  RpidController.SetReference(rightDriveGoal, ControlType::kSmartMotion);
  if (0 < timer.Get() && timer.Get() < phase1)
  {
    if (phase == 0)
    {
      driveStraight(-72);
      phase = 1;
    }
    //change in auton1 if this works
    cout << "phase1 " << timer.Get() << endl;
  }
  else if (phase1 < timer.Get() && timer.Get() < phase2)
  {
    if (phase == 1)
    {
      turn(-90);
      phase = 2;
    }
    //change in auton1 if this works
    cout << "phase1 " << timer.Get() << endl;
  }
}

void Robot::AutonomousInit()
{
  m_autoSelected = m_chooser.GetSelected();
  //m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  cout << "Auto selected: " << m_autoSelected << endl;
  phase = 0;
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

  SmartDashboard::PutNumber("SetF", 0.0453);
  SmartDashboard::PutNumber("SetP", 0.15);
  SmartDashboard::PutNumber("SetI", 0);
  SmartDashboard::PutNumber("SetD", 1.5);

  double flyWheelF = SmartDashboard::GetNumber("SetF", 0.0453);
  double flyWheelP = SmartDashboard::GetNumber("SetP", 0.15);
  double flyWheelI = SmartDashboard::GetNumber("SetI", 0);
  double flyWheelD = SmartDashboard::GetNumber("SetD", 1.5);
  double wallP = SmartDashboard::GetNumber("WallP", 0.125);

  flyWheelL.Config_kF(kPIDLoopIdx, flyWheelF, kTimeoutMs);
  flyWheelL.Config_kP(kPIDLoopIdx, flyWheelP, kTimeoutMs);
  flyWheelL.Config_kI(kPIDLoopIdx, flyWheelI, kTimeoutMs);
  flyWheelL.Config_kD(kPIDLoopIdx, flyWheelD, kTimeoutMs);

  cout << L_encoder.GetPosition() << endl;
  cout << R_encoder.GetPosition() << endl;
  leftDriveGoal = L_encoder.GetPosition();
  rightDriveGoal = R_encoder.GetPosition();

  intakeSolenoid.Set(DoubleSolenoid::Value::kReverse);
}

void Robot::AutonomousPeriodic()
{
  Update_Limelight_Tracking(ledMode);
  auton2(m_LimelightTurnCmd);
  //auton3(m_LimelightTurnCmd);
  //auton4();
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
  bool firstLoop = true;
  //SmartDashboard::PutNumber("Set FlyWheelV", 0);
  SmartDashboard::PutNumber("SetF", 0.047);
  SmartDashboard::PutNumber("SetP", 0.11);
  SmartDashboard::PutNumber("SetI", 0);
  SmartDashboard::PutNumber("SetD", 6.0);
}

void Robot::TeleopPeriodic()
{

  Update_Limelight_Tracking(ledMode);

  /* --------------- Update Controller Values ----------------*/

  leftBumper1 = operatorController.GetRawButton(5);
  rightBumper1 = operatorController.GetRawButton(6);
  leftTrigger1 = operatorController.GetRawButton(7);
  rightTrigger1 = operatorController.GetRawButton(8);

  leftBumper0 = driveController.GetRawButton(5);
  rightBumper0 = driveController.GetRawButton(6);
  leftTrigger0 = driveController.GetRawButton(7);
  rightTrigger0 = driveController.GetRawButton(8);

  btnStart0 = driveController.GetRawButton(10);

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
  btnA0 = driveController.GetRawButton(2);

  //--------------------------------------------------------------------------------------------------//

  SmartDashboard::PutNumber("Current FlyWheelV", flyWheelL.GetSelectedSensorVelocity(kPIDLoopIdx) / 3.4133);

  //            Falcon PID configs              //

  double flyWheelF = SmartDashboard::GetNumber("SetF", 0.047); // Generally 0.0453 // current test.047
  double flyWheelP = SmartDashboard::GetNumber("SetP", 0.125); // Generally 0.15 // current test .11
  double flyWheelI = SmartDashboard::GetNumber("SetI", 0);
  double flyWheelD = SmartDashboard::GetNumber("SetD", 10); // Generally 1.5 // current test 6

  //          Modify Shooter Speed              //
  /*
  double trenchSpeed = SmartDashboard::GetNumber("Trench Speed", 6000);
  double initSpeed = SmartDashboard::GetNumber("Init Line Speed", 5500);
  double wallSpeed = SmartDashboard::GetNumber("Wall Speed", 2700);
  */

  if (btnA1) // Wall PID
  {
    /*
    cout << "BtnA" << endl;
    cout << "flyWheelF: " << flyWheelF << endl;
    cout << "flyWheelP: " << flyWheelP << endl;
    cout << "flyWheelD: " << flyWheelD << endl;
    */

    flyWheelL.Config_kF(kPIDLoopIdx, flyWheelF, kTimeoutMs); // atm 0.047
    flyWheelL.Config_kP(kPIDLoopIdx, flyWheelP, kTimeoutMs); // atm .125 // current test .11
    flyWheelL.Config_kI(kPIDLoopIdx, flyWheelI, kTimeoutMs); // atm 0
    flyWheelL.Config_kD(kPIDLoopIdx, flyWheelD, kTimeoutMs); // atm 10 // current test 6
  }
  else // General PID
  {
    /*
    cout << "Else " << endl;
    cout << "flyWheelF: " << flyWheelF << endl;
    cout << "flyWheelP: " << flyWheelP << endl;
    cout << "flyWheelD: " << flyWheelD << endl;
    */
    flyWheelL.Config_kF(kPIDLoopIdx, 0.0453, kTimeoutMs);
    flyWheelL.Config_kP(kPIDLoopIdx, 0.15, kTimeoutMs);
    flyWheelL.Config_kI(kPIDLoopIdx, 0, kTimeoutMs);
    flyWheelL.Config_kD(kPIDLoopIdx, 1.5, kTimeoutMs);
  }

  // Three Speed Shooter Control
  if (launcher(trenchSpeed, initSpeed, wallSpeed) == 0)
    flyWheelL.Set(0);
  else
    flyWheelL.Set(ControlMode::Velocity, launcher(trenchSpeed, initSpeed, wallSpeed));

  //SmartDashboard::PutNumber("Returned Value", launcher(trenchSpeed, initSpeed, wallSpeed));

  if (btnBack1)
  {
    hoodSolenoid.Set(DoubleSolenoid::Value::kForward);
  }
  else if (btnStart1)
  {
    hoodSolenoid.Set(DoubleSolenoid::Value::kReverse);
  }

  if (rightTrigger0)
  {
    if (btnY1) // High
    {
      conveyorMotor.Set(1);
      indexMotor.Set(.9);
    }
    else if (btnX1) // Medium
    {
      conveyorMotor.Set(0.7);
      indexMotor.Set(.9);
    }
    else if (btnA1) // Low
    {
      conveyorMotor.Set(0.5);
      indexMotor.Set(.9);
    }
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
  else if (leftBumper0) //Intake minus the intake motor
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
  else if (leftTrigger1) // Purge the system
  {
    intakeMotor.Set(-0.5);
    conveyorMotor.Set(-0.5);
    indexMotor.Set(-1);
  }
  else if (rightTrigger1) // reverse the "v"
  {
    vMotor1.Set(-0.5);
    vMotor2.Set(-0.5);
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

  climbL.Set(-rightAxisY1);
  climbR.Set(-rightAxisY1);

  if (leftBumper1)
    brakeLift.Set(false);
  else
    brakeLift.Set(true);

  if (rightBumper1 && !lastRightJoyClick)
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
  lastRightJoyClick = rightBumper1;

  //cout << "DriveStraight " << straightMath(0.15, 0.5, leftAxisY0) << ", ";
  //cout << "Turn " << turnMath(0.18, 0.5, rightAxisX0) << endl;

  if (btnA0)
  {
    ledMode = 3;
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
    ledMode = 1;
    ArcadeDrive.ArcadeDrive(-1 * straightMath(0.15, .5, leftAxisY0), turnMath(0.18, 0.5, rightAxisX0));
  }

  shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

  double ta = table->GetNumber("ta", 0.0);
}

void Robot::TestPeriodic() {}

float txavg = 0;

void Robot::Update_Limelight_Tracking(int ledMode)
{
  //Proportional Steering Constant
  const double STEER_K = 0.03;
  float ki = 0.00125;
  //Proportional Drive Constant
  const double DRIVE_K = 0.26;

  //Area of tape when robot has reached the goal
  const double DESIRED_TARGET_AREA = 49.0;
  const double MAX_DRIVE = 0.25;
  const double MAX_STEER = 0.3f;
  float min_command = 0.07f;

  shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  double tx = table->GetNumber("tx", 0.0);
  double ty = table->GetNumber("ty", 0.0);
  double ta = table->GetNumber("ta", 0.0);
  double tv = table->GetNumber("tv", 0.0);
  ledMode = table->PutNumber("ledMode", ledMode);

  if (tv < 1.0)
  {
    m_LimelightHasTarget = false;
    m_LimelightDriveCmd = 0;
    m_LimelightTurnCmd = 0;
  }
  else
  {
    m_LimelightHasTarget = true;

    //Averaging tx to mitigate glitch
    txavg = (0.2 * tx) + (0.8 * txavg);
    cout << "txavg " << txavg << endl;
    cout << "tx " << tx << endl;
    // Proportional steering
    float steering_adjust = 0.0;
    if (txavg > 2.0 || txavg < -2.0) //this is correct !do not touch the numbers!
    {
      limelightIntegral = 0;
    }
    else if (txavg < 0.3 && txavg > -0.3) // we made this a tenth smaller was 0.25 respectively
    {
      limelightIntegral = 0;
    }
    else
    {
      limelightIntegral = limelightIntegral + txavg;
    }

    if (txavg > .3) //used to be .2
    {
      m_LimelightTurnCmd = txavg * STEER_K + /*limelightIntegral * ki*/ +min_command;
    }
    else if (txavg < -.3) //we changed the negative here //used to be .2
    {
      m_LimelightTurnCmd = txavg * STEER_K + /*limelightIntegral * ki*/ -min_command;
    }
    cout << "Limelight CMD" << m_LimelightTurnCmd << endl;
    m_LimelightTurnCmd = clamp(m_LimelightTurnCmd, -MAX_STEER, MAX_STEER);
    cout << m_LimelightTurnCmd << endl;
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