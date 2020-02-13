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
WPI_TalonFX flyWheelL{5};
WPI_TalonFX flyWheelR{6};
CANSparkMax Neo1{7, CANSparkMax::MotorType::kBrushless};
CANSparkMax Neo2{8, CANSparkMax::MotorType::kBrushless};
WPI_TalonSRX conveyorMotor{10};
WPI_VictorSPX vMotor1{11};
WPI_VictorSPX vMotor2{12};
CANSparkMax indexMotor{13, CANSparkMax::MotorType::kBrushed};
WPI_TalonFX climbL{14};
WPI_TalonFX climbR{15};

//      Solenoids     //
Solenoid brake{0};
DoubleSolenoid intakeSolenoid{1, 2};
DoubleSolenoid hoodSolenoid{3, 4};

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
int clicks = 21;

//      Drive Variables     //
bool nitros{false};

//      Drive Train     //
DifferentialDrive ArcadeDrive(LDriveMotor, RDriveMotor);

string _sb;
int _loops = 0;

void Robot::RobotInit()
{
  brake.Set(false);
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
  // set motor follow //
  flyWheelR.Follow(flyWheelL);
  flyWheelR.SetInverted(true);

  flyWheelL.Config_kF(kPIDLoopIdx, 0.1097, kTimeoutMs);
  flyWheelL.Config_kP(kPIDLoopIdx, 0.22, kTimeoutMs);
  flyWheelL.Config_kI(kPIDLoopIdx, 0, kTimeoutMs);
  flyWheelL.Config_kD(kPIDLoopIdx, 0, kTimeoutMs);

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
}

void Robot::AutonomousPeriodic() {}

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
  /*
  SmartDashboard::PutNumber("Set FlyWheel Velocity", 0);
  SmartDashboard::PutNumber("SetP", 0);
  SmartDashboard::PutNumber("SetI", 0);
  SmartDashboard::PutNumber("SetD", 0);
  SmartDashboard::PutNumber("SetF", 0); */
}

void ball() // This will increment the number of balls
{
  ballCount++;
  cout << "ball " << ballCount << endl;
}

void Robot::TeleopPeriodic()
{
  Update_Limelight_Tracking();

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

  btnX0 = driveController.GetRawButton(0);

  double flyWheelP = SmartDashboard::GetNumber("SetP", 0.1097);
  double flyWheelI = SmartDashboard::GetNumber("SetI", 0.22);
  double flyWheelD = SmartDashboard::GetNumber("SetD", 0);
  double flyWheelV = SmartDashboard::GetNumber("Set FlyWheel Velocity", 0);

  SmartDashboard::PutBoolean("Beam", breakBeamBottomIn.Get());
  SmartDashboard::PutNumber("Balls", ballCount);
  SmartDashboard::PutNumber("FlyWheel Velocity", flyWheelL.GetSelectedSensorVelocity(kPIDLoopIdx));

  //            Falcon PID configs              //


  //flyWheelL.Set(ControlMode::Velocity, flyWheelV);

  //three speed shooter control
  if (rightTrigger0) {
    flyWheelL.Set(ControlMode::Velocity, launcher());
  }

  if (leftTrigger1) {
    intakeMotor.Set(.75);
    vMotor1.Set(0.7);
    vMotor2.Set(0.9);
  } else if (leftBumper1) {
    intakeMotor.Set(0);
    vMotor1.Set(0.7);
    vMotor2.Set(0.9);
  } else {
    intakeMotor.Set(0);
    vMotor1.Set(0);
    vMotor2.Set(0);
  }

  if (rightBumper1) {
    climbL.Set(0.8);
    climbR.Set(0.8);
  } else if (rightTrigger1) {
    climbL.Set(-0.6);
    climbR.Set(-0.6);
  } else {
    climbL.Set(0);
    climbR.Set(0);
  }

  ArcadeDrive.ArcadeDrive(-1 * straightMath(0.15, 1, leftAxisY0), turnMath(0.4, 0.5, rightAxisX0));

  shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

  double ta = table->GetNumber("ta", 0.0);

  // Test Code // to be removed shortly
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