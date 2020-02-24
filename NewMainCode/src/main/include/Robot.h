#pragma once
#ifndef robotHFile
#define robotHFile
#include <string>
#include <frc2/command/Command.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

  void Update_Limelight_Tracking();

 private:
  // Have it null by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  frc::SendableChooser<std::string> m_chooser;
  frc2::Command* m_autonomousCommand = nullptr;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  bool m_LimelightHasTarget;
  double m_LimelightTurnCmd;
  double m_LimelightDriveCmd;
  float limelightIntegral;
};
#endif