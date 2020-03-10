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
  LpidController.SetSmartMotionMaxVelocity(2600);
  RpidController.SetSmartMotionMaxVelocity(2600);
  LpidController.SetSmartMotionMaxAccel(2000);
  RpidController.SetSmartMotionMaxAccel(2000);
  float desiredDistance = distance * .57;
  leftDriveGoal = leftDriveGoal + desiredDistance;
  rightDriveGoal = rightDriveGoal - desiredDistance;
  return desiredDistance;
}

double driveStraightFast(float distance)
{
  LpidController.SetSmartMotionMaxVelocity(3000);
  RpidController.SetSmartMotionMaxVelocity(3000);
  LpidController.SetSmartMotionMaxAccel(2000);
  RpidController.SetSmartMotionMaxAccel(2000);
  float desiredDistance = distance * .57;
  leftDriveGoal = leftDriveGoal + desiredDistance;
  rightDriveGoal = rightDriveGoal - desiredDistance;
  return desiredDistance;
}

double turn(float degrees)
{
  LpidController.SetSmartMotionMaxVelocity(1600);
  RpidController.SetSmartMotionMaxVelocity(1600);
  LpidController.SetSmartMotionMaxAccel(1000);
  RpidController.SetSmartMotionMaxAccel(1000);
  float desiredDegrees = degrees * 0.1205;
  leftDriveGoal = leftDriveGoal + desiredDegrees;
  rightDriveGoal = rightDriveGoal + desiredDegrees;
  return desiredDegrees;
}

double leftSweep(float degrees)
{
  LpidController.SetSmartMotionMaxVelocity(2800);
  RpidController.SetSmartMotionMaxVelocity(2800);
  LpidController.SetSmartMotionMaxAccel(2000);
  RpidController.SetSmartMotionMaxAccel(2000);
  float desiredDegrees = degrees * 0.1205;
  rightDriveGoal = rightDriveGoal + desiredDegrees * 2;
  return desiredDegrees;
}

double rightSweep(float degrees)
{
  LpidController.SetSmartMotionMaxVelocity(2800);
  RpidController.SetSmartMotionMaxVelocity(2800);
  LpidController.SetSmartMotionMaxAccel(2000);
  RpidController.SetSmartMotionMaxAccel(2000);
  float desiredDegrees = degrees * 0.1205;
  leftDriveGoal = leftDriveGoal + desiredDegrees * 2;
  return desiredDegrees;
}

#endif