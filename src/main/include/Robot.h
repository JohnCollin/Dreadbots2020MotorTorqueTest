/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <rev/CANSparkMax.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

 private:
  // Motor Controller Encoder Object
  // NOTE to Self: This object cannot also be a pointer,
  // as the object that defines this pointer is destroyed
  // after Robot::RobotInit() by the C++ Garbage Collection.
  rev::CANEncoder test_motor_encoder;

  // Motor Controller Object
  rev::CANSparkMax* test_motor;

  // Starting Values at Robot::RobotInit() to ensure modules remain existent
  double const kShuffleBoardStartingValue = 0.0;

  // Port of the Test Motor.
  int const kTestMotorPort = 0;

  // Diagnostic Module Titles to ensure consistency.
  std::string const kDiagnosticTitleMotorTargetVelocity = "Motor Velocity";
  std::string const kDiagnosticTitleMotorEncoderPosition = "Motor Encoder Position";
  std::string const kDiagnosticTitleMotorEncoderVelocity = "Motor Encoder Velocity";

  // Velocity Variable to be defined by Shuffleboard and used as the SparkMax input.
  double test_motor_velocity;
};
