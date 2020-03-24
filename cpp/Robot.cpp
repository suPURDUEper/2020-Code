/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <Robot.h>
#include <frc/Joystick.h>
#include <iostream>
#include <ctre/phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/Solenoid.h>
#include <frc/DoubleSolenoid.h>
#include <frc/TimedRobot.h>
#include <cameraserver/CameraServer.h>
#include <frc/Spark.h>
#include <frc/DigitalInput.h>
#include <math.h>
#include <frc/DigitalOutput.h>
#include <frc/Talon.h>
#include <rev/CANSparkMax.h>
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

// Solenoids //
Solenoid LiftSingle{0};
DoubleSolenoid EjectDouble{7, 6};
DoubleSolenoid CargoDouble{4, 5};


// Drive motors //
CANSparkMax LDriveNeo{2, CANSparkMax::MotorType::kBrushless};
CANSparkMax LDriveNeo2{1, CANSparkMax::MotorType::kBrushless};
CANSparkMax RDriveNeo{4, CANSparkMax::MotorType::kBrushless};
CANSparkMax RDriveNeo2{5, CANSparkMax::MotorType::kBrushless};

/*
// PID Controller //
CANPIDController L_pidController = LDriveNeo.GetPIDController();
CANPIDController R_pidController = RDriveNeo.GetPIDController();
CANEncoder L_encoder = LDriveNeo.GetEncoder();
CANEncoder R_encoder = RDriveNeo.GetEncoder();

//default PID coefficients
double kP = 5e-5, kI = 1e-6, kD = 0, kIz = 0, kFF = 0.000156, kMaxOutput = 0.25, kMinOutput = -0.25;
*/

// Special Limiters //
int CurrentLimit{50};

// Lift motors //
WPI_TalonSRX LLiftTalon{2};
WPI_VictorSPX LLiftVictor{2};
WPI_TalonSRX RLiftTalon{3};
WPI_VictorSPX RLiftVictor{3};

// Cargo Motor //
Talon CargoMotor{0};
Talon HatchMotor{1};

// Restrictors //
bool CargoRestrictor{0};
bool HatchRestrictor{0};
bool RampRestrictor{0}; 

// Inputs/Outputs //
DigitalInput LimitSwitch(0);
DigitalInput HatchLimitSwitch(1);
DigitalInput HatchLimitSwitch2(2);
DigitalOutput LightRing(9);

// Controller buttons //
bool BtnX{Controller.GetRawButton(1)};
bool NBtnX{false};
bool BtnA{Controller.GetRawButton(2)}; //button for vision Tracking
bool NBtnA{false};
bool BtnB{Controller.GetRawButton(3)};
bool BtnY{Controller.GetRawButton(4)};
bool BtnLB{Controller.GetRawButton(5)}; //
bool NBtnLB{true};
bool BtnRB{Controller.GetRawButton(6)}; //starts Cargo
bool NBtnRB{false};
bool BtnLT{Controller.GetRawButton(7)}; //controls VelcroDouble
bool NBtnLT{true};                      // "N" stands for "new"
bool BtnRT{Controller.GetRawButton(8)}; //controls PivotSingle
bool NBtnRT{true};                      // "N" stands for "new"
bool BtnBack{Controller.GetRawButton(9)};
bool BtnStart{Controller.GetRawButton(10)};
bool Sucking{false};
bool Trigger{FlightStick.GetRawButton(1)};
bool BtnSide{FlightStick.GetRawButton(2)};
bool BtnHatch{FlightStick.GetRawButton(3)};
bool BtnRamp{FlightStick.GetRawButton(4)};
bool BtnExtend{FlightStick.GetRawButton(5)};
bool BtnAuto{FlightStick.GetRawButton(7)};
bool BtnSafety{FlightStick.GetRawButton(11)};
bool CargoGo{LimitSwitch.Get()};
bool HatchGo{HatchLimitSwitch.Get()};
bool HatchGo2{HatchLimitSwitch2.Get()};
double CargoSpeed{1};


// Controller axes //
double LStickX{Controller.GetRawAxis(0)};
double LStickY{Controller.GetRawAxis(1)}; //controls forward/backward movement
double RStickX{Controller.GetRawAxis(2)}; //controls left/right movement
double RStickY{Controller.GetRawAxis(3)};
double JoystickX{FlightStick.GetRawAxis(0)};
double JoyStickY{-1 * FlightStick.GetRawAxis(1)};
double SwitchEnable{FlightStick.GetRawAxis(3)};
int POV = -1;
int LiftPOV{0};

// Drive Variables //
double DriveStraight{0};
double DriveTurn{0};
float DriveMinPower{0.3};
float TurnMinPower{0.4};
float ModValue{0.5};
float ModValueTurn{0.5}; //changed from .75 to turn slower
bool Nitros{false};

// Misc Stuffs //
bool LockedIn{false};
bool Lifting{false};
int AlignmentX{0};
float AutoSpeed1 = 0.5;
float AutoTime1 = 1.35;
float AutoSpeed2 = 0.8;
float AutoTime2 = 2.85;
DifferentialDrive ArcadeDrive(LDriveNeo, RDriveNeo);
Timer m_timer;
Timer PunchTimer;
Timer AutoTimer;
Timer HatchTimer;

//double GameTimer = DriverStation::GetInstance.GetMatchTime();

void Robot::RobotInit()
{
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  frc::CameraServer::GetInstance()->StartAutomaticCapture(0);
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit()
{
  LDriveNeo2.Follow(LDriveNeo);
  RDriveNeo2.Follow(RDriveNeo);
  LiftSingle.Set(false);
  EjectDouble.Set(DoubleSolenoid::Value::kReverse);
  RLiftTalon.SetInverted(true);
  RLiftVictor.SetInverted(true);
  LLiftTalon.Follow(RLiftTalon);
  LLiftVictor.Follow(RLiftTalon);
  RLiftVictor.Follow(RLiftTalon);
  CargoDouble.Set(DoubleSolenoid::Value::kReverse);


  LDriveNeo.SetSmartCurrentLimit(CurrentLimit);
  RDriveNeo.SetSmartCurrentLimit(CurrentLimit);
  //RLiftTalon.Set(-0.5);
  //GameTimer.Reset();
}

void Robot::TeleopPeriodic()
{
  Update_Limelight_Tracking();

  BtnA = Controller.GetRawButton(2); // Button for VisionTracking
  BtnLB = Controller.GetRawButton(5); // Controls EjectDouble
  BtnRB = Controller.GetRawButton(6); // Nitros (Toggle)
  BtnLT = Controller.GetRawButton(7); // Controls Output (Cargo)
  BtnRT = Controller.GetRawButton(8); // Controls Intake (Cargo)
  LStickY = Controller.GetRawAxis(1); //controls forward/backward movement
  RStickX = Controller.GetRawAxis(2); //controls left/right movement
  // Flight Stick Enables //
  SwitchEnable = FlightStick.GetRawAxis(3); // Enable Joystick
  Trigger = FlightStick.GetRawButton(1);    // Starts Lift Macro
  BtnSide = FlightStick.GetRawButton(2);    // Retracts Lift
  BtnHatch = FlightStick.GetRawButton(3);
  BtnRamp = FlightStick.GetRawButton(4);
  BtnExtend = FlightStick.GetRawButton(5);
  BtnSafety = FlightStick.GetRawButton(11); // Safety Button
  LiftPOV = FlightStick.GetPOV(0);          // Manual Lift
  CargoGo = LimitSwitch.Get();
  HatchGo = HatchLimitSwitch.Get();
  HatchGo2 = HatchLimitSwitch2.Get();

  //POV = Controller.GetPOV(0);

  if (BtnSafety) // safety button //
  {
    //std::cout << "Safety Engaged!" << "/n";
    EjectDouble.Set(DoubleSolenoid::Value::kOff);
    LiftSingle.Set(false);
    LDriveNeo.Set(0);
    RDriveNeo.Set(0);
    RLiftTalon.Set(0);
    RLiftVictor.Set(0);
    LLiftTalon.Set(0);
    LLiftVictor.Set(0);
    ArcadeDrive.ArcadeDrive(0, 0);
    Sucking = false;
    CargoMotor.Set(0);
    Lifting = false;
    CargoDouble.Set(DoubleSolenoid::Value::kReverse);
  }
  // all commands (only executed if safety button not pressed) //
  else
  {

    if (BtnA)
    {
      cout<<"BtnA ";
      if (m_LimelightHasTarget)
      {
        cout<<"Target Acquired ";
        ArcadeDrive.ArcadeDrive(m_LimelightDriveCmd,m_LimelightTurnCmd);
        cout<<m_LimelightTurnCmd<<" "<<endl;
      }
      else
      {
        cout<<"No Target "<<endl;
        ArcadeDrive.ArcadeDrive(0.0,0.0);
      }  
    }
    else
    {
      cout<<"normal drive "<<endl;
      //set the drive

      // Deadzone for Drive //

      if (RStickX > -0.05 && RStickX < 0.05)
      {
      RStickX = 0;
      }
      if (LStickY > -0.05 && LStickY < 0.05)
      {
      LStickY = 0;
      }

      // Non-linear Drive Math //

      DriveMinPower = .15;
      ModValue = 1;

      if (LStickY > 0)
      {
        DriveStraight = (DriveMinPower + (1 - DriveMinPower) * (ModValue * (pow(LStickY, 3) + (1 - ModValue) * LStickY)));
      }
      else if (LStickY < 0)
      {
        DriveStraight = ((-1 * DriveMinPower) + (1 - DriveMinPower) * (ModValue * (pow(LStickY, 3) + (1 - ModValue) * LStickY)));
      }
      else
      {
        DriveStraight = 0;
      }
      if (RStickX > 0)
      {
        DriveTurn = (TurnMinPower + (1 - TurnMinPower) * (ModValueTurn * (pow(RStickX, 3) + (1 - ModValueTurn) * RStickX)));
      }
      else if (RStickX < 0)
      {
        DriveTurn = ((-1 * TurnMinPower) + (1 - TurnMinPower) * (ModValueTurn * (pow(RStickX, 3) + (1 - ModValueTurn) * RStickX)));
     }
      else
      {
        DriveTurn = 0;
      }
    
    

    if (BtnRB)
    {
      if (DriveStraight > 0.8)
      {
        DriveStraight = 1;
      }
      else if (DriveStraight < -0.8)
      {
        DriveStraight = -1;
      }
    }
    else if (!BtnRB)
    {
      if (DriveStraight > 0.8)
      {
        DriveStraight = 0.8;
      }
      else if (DriveStraight < -0.8)
      {
        DriveStraight = -0.8;
      }
    

    // Display Values //

    //std::cout << "LStick: " << LStickY << endl;
    //std::cout << "RStick: " << RStickX << endl;
    //std::cout << "DriveTurn: " << DriveTurn << std::endl;
    //std::cout << "RMotor: " << RDriveNeo.Get() << std::endl;
    //std::cout << "LMotor: " << LDriveNeo.Get() << std::endl;
    //std::cout << "DriveStraight: " << DriveStraight << std::endl;
    //std::cout << "CargoGo:" << CargoGo << std::endl;
    //std::cout << "LimitSwitch: " << CargoGo << std::endl;
    }
    ArcadeDrive.ArcadeDrive(-1 * DriveStraight, DriveTurn);
    }
    
    
    if(BtnRamp)
    {
      CargoSpeed = .3;
      CargoDouble.Set(DoubleSolenoid::Value::kForward);

    } else {
      CargoSpeed = 1;
      CargoDouble.Set(DoubleSolenoid::Value::kReverse);
    }
    
     std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    double ta = table->GetNumber("ta",0.0);
    if (BtnRT)
    {
      CargoMotor.Set(-CargoSpeed);
 
    }
    else if (BtnLT)
    {
      if (CargoGo)
      {
        CargoMotor.Set(1);
      }
      else
      {
        CargoMotor.Set(0);
      }
    }
    else
    {
      CargoMotor.Set(0);

    }


    // Hatch Code //
      
      // BtnHatch is Intake

      if (BtnExtend)
      {
        EjectDouble.Set(DoubleSolenoid::Value::kForward);
      } else if (!BtnExtend && !BtnHatch) {
        EjectDouble.Set(DoubleSolenoid::Value::kReverse);
      }

      if (BtnLB)
      {
        HatchMotor.Set(1);
      } 
      else if(BtnA&&(ta>49.0))
      {
        HatchTimer.Start();
        HatchTimer.Reset();
        while(HatchTimer.Get() < 0.85)
        {
          ArcadeDrive.ArcadeDrive(0.25, 0.0);
          Wait(0.01);
        }
        while(Controller.GetRawButton(2))
        {
          ArcadeDrive.ArcadeDrive(-0.2, 0.0);
          HatchMotor.Set(1);
          Wait(0.01);
        }
      }
      else if (BtnHatch) 
      {
        EjectDouble.Set(DoubleSolenoid::Value::kForward);
        if (HatchGo && HatchGo2)
        {
          HatchMotor.Set(-0.8);
        } else {
          HatchMotor.Set(0);
        }
      } else if (!BtnExtend) {
        HatchMotor.Set(0);
        EjectDouble.Set(DoubleSolenoid::Value::kReverse);
      } else {
        HatchMotor.Set(0);
      }

    if (BtnSide && !Lifting && SwitchEnable > 0.8) // Brings in the Lift
    {
      RLiftTalon.Set(-0.10);
    }
    else if (!BtnSide && !Lifting && SwitchEnable > 0.8)
    {
      RLiftTalon.Set(0.0);
    }
    //Problem Code //"fixed" by changing last else to else if //untested
    if (LiftPOV == 0 && !Lifting && SwitchEnable < -0.8)
    {
      RLiftTalon.Set(0.25);
    }
    else if (LiftPOV == 180 && !Lifting && SwitchEnable < -0.8)
    {
      RLiftTalon.Set(-0.25);
    }
    else if (!Lifting && SwitchEnable < -0.8)
    {
      RLiftTalon.Set(0);
    }

    // Lift Macro //

    //if (GameTimer > 75)
    //{
    if (Trigger && !Lifting && SwitchEnable < -0.8)
    {
      Lifting = true;
      m_timer.Start();
      m_timer.Reset();
    }
    if (Lifting)
    {
      for (int i = 1; i == 1; i++)
      {
        if (m_timer.Get() < .35)
        {
          //drive backwards at half speed and stop
          ArcadeDrive.ArcadeDrive(-0.39, 0.0);
        }
        else if (m_timer.Get() > .35 && m_timer.Get() < .85)
        {
          ArcadeDrive.ArcadeDrive(0.0, 0.0);
          if (BtnSafety)
          {
            Lifting = false;
            break;
          }
          //Actuate cylinders and begin lift
          LiftSingle.Set(true);
          RLiftTalon.Set(0.7);
        }
        else if (m_timer.Get() > .85 && m_timer.Get() < 2.0)
        {
          if (BtnSafety == 1)
          {
            Lifting = false;
            break;
          }
          //Retract Cylinders
          LiftSingle.Set(false);
          if (BtnSafety == 1)
          {
            Lifting = false;
            break;
          }
        }
        else if (m_timer.Get() > 2.0 && m_timer.Get() < 2.5)
        {
          RLiftTalon.Set(0.4);
          if (BtnSafety == 1)
          {
            Lifting = false;
            break;
          }

          //Stop Lift Motors
        }
        else if (m_timer.Get() > 2.5)
        {
          RLiftTalon.Set(0.0);
          Lifting = false;
        }
        
      }
    }
    //}
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
int main()
{
  return frc::StartRobot<Robot>();
}
#endif