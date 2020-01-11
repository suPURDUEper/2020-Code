#include "Robot.h"
#include <ctre/phoenix.h>
#include <iostream>
#include <rev/CANSparkMax.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Joystick.h>
#include <frc/Solenoid.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/DigitalInput.h>
#include <frc/Talon.h>
#include <cameraserver/CameraServer.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

double clamp(double in,double minval,double maxval)
{
  if (in > maxval) return maxval;
  if (in < minval) return minval;
  return in;
}

using namespace frc;
using namespace std;
using namespace rev;

frc::Joystick Controller{0};
frc::Joystick FlightStick{1};

int CurrentLimit{50};

//      Drive Motors      //
CANSparkMax LDriveNeo{2, CANSparkMax::MotorType::kBrushless}; //first number is CAN ID of motor
CANSparkMax LDriveNeo2{1, CANSparkMax::MotorType::kBrushless};
CANSparkMax RDriveNeo{4, CANSparkMax::MotorType::kBrushless};
CANSparkMax RDriveNeo2{5, CANSparkMax::MotorType::kBrushless};

//      Intake Motor      //
WPI_TalonSRX IntakeMotor{0};

//      Joysticks     //
double LStickX{Controller.GetRawAxis(0)};
double LStickY{Controller.GetRawAxis(1)};
double RStickX{Controller.GetRawAxis(2)};
double RStickY{Controller.GetRawAxis(3)};

//      ABXY Buttons      //
bool BtnX{Controller.GetRawButton(1)};
bool BtnA{Controller.GetRawButton(2)};
bool BtnB{Controller.GetRawButton(3)};
bool BtnY{Controller.GetRawButton(4)};

//      Bumpers && Triggers     //
bool BtnLB{Controller.GetRawButton(5)};
bool BtnRB{Controller.GetRawButton(6)}; 
bool BtnLT{Controller.GetRawButton(7)};
bool BtnRT{Controller.GetRawButton(8)};
     
//      Misc. Buttons     //
bool BtnBack{Controller.GetRawButton(9)};
bool BtnStart{Controller.GetRawButton(10)};

//      Flight Stick      //
bool Trigger{FlightStick.GetRawButton(1)}; 
bool BtnSafety{FlightStick.GetRawButton(11)}; 
double SwitchEnable{FlightStick.GetRawAxis(3)};
bool HatX{FlightStick.GetRawAxis(5)};
bool HatY{FlightStick.GetRawAxis(6)};

//      Sensors     //
DigitalInput BreakBeam(0);

//      Drive Variables     //
double DriveStraight{0};
double DriveTurn{0};
float DriveMinPower{0.3};
float TurnMinPower{0.4};
float ModValue{0.5};
float ModValueTurn{0.5};
bool Nitros{false};

// Drive Train
DifferentialDrive ArcadeDrive(LDriveNeo, RDriveNeo);

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  frc::CameraServer::GetInstance()->StartAutomaticCapture(0);
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {
  LDriveNeo2.Follow(LDriveNeo);
  RDriveNeo2.Follow(RDriveNeo);

  LDriveNeo.SetSmartCurrentLimit(CurrentLimit);
  RDriveNeo.SetSmartCurrentLimit(CurrentLimit);
}

void Robot::TeleopPeriodic() {
  Update_Limelight_Tracking();
  
    if (BtnSafety) { //safety button held
    LDriveNeo.Set(0);
    RDriveNeo.Set(0);
    } else { //saftey button not held
      // Deadzone for drive
      if (RStickX > -0.05 && RStickX < 0.05){
      RStickX = 0;
      }
      if (LStickY > -0.05 && LStickY < 0.05){
      LStickY = 0;
      }

      //  Non-linear drive math
      DriveMinPower = .15;
      ModValue = 1;

      if (LStickY > 0) {
        DriveStraight = (DriveMinPower + (1 - DriveMinPower) * (ModValue * (pow(LStickY, 3) + (1 - ModValue) * LStickY)));
      } else if (LStickY < 0) {
        DriveStraight = ((-1 * DriveMinPower) + (1 - DriveMinPower) * (ModValue * (pow(LStickY, 3) + (1 - ModValue) * LStickY)));
      } else {
        DriveStraight = 0;
      }

      if (RStickX > 0) {
        DriveTurn = (TurnMinPower + (1 - TurnMinPower) * (ModValueTurn * (pow(RStickX, 3) + (1 - ModValueTurn) * RStickX)));
      }
      else if (RStickX < 0) {
        DriveTurn = ((-1 * TurnMinPower) + (1 - TurnMinPower) * (ModValueTurn * (pow(RStickX, 3) + (1 - ModValueTurn) * RStickX)));
      } else {
        DriveTurn = 0;
      }

    ArcadeDrive.ArcadeDrive(-1 * DriveStraight, DriveTurn);

    std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    double ta = table->GetNumber("ta",0.0);

    if (HatX) {
      cout << HatX << endl;
    }

    if (HatY) {
      cout << HatY << endl;
    }

    if (BtnLT){
      IntakeMotor.Set(100);
    }
  }
}

void Robot::TestPeriodic() {}

void Robot::Update_Limelight_Tracking(){
  //Proportional Steering Constant
  const double STEER_K = 0.05;

  //Proportional Drive Constant
  const double DRIVE_K = 0.26;

  //Area of tape when robot has reached the goal
  const double DESIRED_TARGET_AREA = 49.0; //changed from 13.0
  const double MAX_DRIVE = 0.25;
  const double MAX_STEER = 0.4f;

  std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  double tx = table->GetNumber("tx",0.0);
  double ty = table->GetNumber("ty",0.0);
  double ta = table->GetNumber("ta",0.0);
  double tv = table->GetNumber("tv",0.0);

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
        m_LimelightTurnCmd = tx*STEER_K;
        m_LimelightTurnCmd = clamp(m_LimelightTurnCmd,-MAX_STEER,MAX_STEER);

        // drive forward until the target area reaches our desired area
        if(ta>49.0){ //change 11.5 to new value for dual target
          m_LimelightDriveCmd = 0;
        } else {
           m_LimelightDriveCmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;
        }
        m_LimelightDriveCmd = clamp(m_LimelightDriveCmd,-MAX_DRIVE,MAX_DRIVE);
  }
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
