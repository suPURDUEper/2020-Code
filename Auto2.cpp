#include "Robot.h"
#include "Auto.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <rev/CANSparkMax.h>
#include <ctre/Phoenix.h>



void Auto2()
{
    int timer = 0;
    rev::CANSparkMax LMotorDrive(1,rev::CANSparkMax::MotorType::kBrushless);
    rev::CANSparkMax RMotorDrive(2,rev::CANSparkMax::MotorType::kBrushless);
    frc::DifferentialDrive m_drive(LMotorDrive,RMotorDrive);
    m_drive.ArcadeDrive(-1,-1);
    if(timer>1000)
    {
        m_drive.ArcadeDrive(0,0);
    }
    

}
