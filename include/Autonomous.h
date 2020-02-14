#ifndef AUTONOMOUS_H_
#define AUTONOMOUS_H_

#include "commonVariables.h"

CANSparkMax LDriveMotor{1, CANSparkMax::MotorType::kBrushless};
CANSparkMax LDriveMotor2{2, CANSparkMax::MotorType::kBrushless};
CANSparkMax RDriveMotor{3, CANSparkMax::MotorType::kBrushless};
CANSparkMax RDriveMotor2{4, CANSparkMax::MotorType::kBrushless};

CANPIDController LpidController = LDriveMotor.GetPIDController();
CANEncoder L_encoder = LDriveMotor.GetEncoder();
CANPIDController RpidController = RDriveMotor.GetPIDController();
CANEncoder R_encoder = RDriveMotor.GetEncoder();



void forwardFunction(float distance)
{
  float desiredDistance{distance * clicks};
  while (L_encoder.GetPosition() < desiredDistance && R_encoder.GetPosition() < desiredDistance)
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
  while (R_encoder.GetPosition() != desiredDistance && L_encoder.GetPosition() != desiredDistance)
  {
    LpidController.SetReference(desiredDistance, ControlType::kSmartMotion);
    RpidController.SetReference(-desiredDistance, ControlType::kSmartMotion);
  }
}

#endif