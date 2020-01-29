/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  // Initialize Value as 0 to start
  test_motor_velocity = kShuffleBoardStartingValue;

  // Display initial value to ShuffleBoard to be configurable during runtime.
  frc::SmartDashboard::PutNumber(kDiagnosticTitleMotorTargetVelocity, kShuffleBoardStartingValue);

  // Display initial values to ShuffleBoard to ensure the modules exist in the future.
  frc::SmartDashboard::PutNumber(kDiagnosticTitleMotorEncoderPosition, kShuffleBoardStartingValue);
  frc::SmartDashboard::PutNumber(kDiagnosticTitleMotorEncoderVelocity, kShuffleBoardStartingValue);

  // Define Motor Value at Port 0 using default settings (brushless).
  test_motor = new rev::CANSparkMax(kTestMotorPort, rev::CANSparkMax::MotorType::kBrushless);

  // Define Motor Encoder from Test Motor.
  test_motor_encoder = test_motor->GetEncoder();
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  // Retrieve current motor velocity from ShuffleBoard.
  test_motor_velocity = frc::SmartDashboard::GetNumber(kDiagnosticTitleMotorTargetVelocity, kShuffleBoardStartingValue);

  // Set the current motor speed.
  test_motor->Set(test_motor_velocity);

  // Motor Diagnostic Values.
  frc::SmartDashboard::PutNumber(kDiagnosticTitleMotorEncoderPosition, test_motor_encoder.GetPosition());
  frc::SmartDashboard::PutNumber(kDiagnosticTitleMotorEncoderVelocity, test_motor_encoder.GetVelocity());
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif