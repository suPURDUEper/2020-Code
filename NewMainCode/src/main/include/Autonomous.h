#ifndef AUTONOMOUS_H_
#define AUTONOMOUS_H_

#include "commonVariables.h"

#define pi 3.14

CANSparkMax LDriveMotor{1, CANSparkMax::MotorType::kBrushless};
CANSparkMax LDriveMotor2{2, CANSparkMax::MotorType::kBrushless};
CANSparkMax RDriveMotor{3, CANSparkMax::MotorType::kBrushless};
CANSparkMax RDriveMotor2{4, CANSparkMax::MotorType::kBrushless};

CANPIDController LpidController = LDriveMotor.GetPIDController();
CANEncoder L_encoder = LDriveMotor.GetEncoder();
CANPIDController RpidController = RDriveMotor.GetPIDController();
CANEncoder R_encoder = RDriveMotor.GetEncoder();

double leftDriveGoal = L_encoder.GetPosition();
double rightDriveGoal = R_encoder.GetPosition();

double driveStraight(float distance)
{
  LpidController.SetSmartMotionMaxVelocity(1200);
  RpidController.SetSmartMotionMaxVelocity(1200);
  float desiredDistance = distance * .57;
  leftDriveGoal = leftDriveGoal + desiredDistance;
  rightDriveGoal = rightDriveGoal - desiredDistance;
  return desiredDistance;
}

double turn(float degrees)
{
  LpidController.SetSmartMotionMaxVelocity(800);
  RpidController.SetSmartMotionMaxVelocity(800);
  float desiredDegrees = degrees * 0.1205;
  leftDriveGoal = leftDriveGoal + desiredDegrees;
  rightDriveGoal = rightDriveGoal + desiredDegrees;
  return desiredDegrees;
}

#endif