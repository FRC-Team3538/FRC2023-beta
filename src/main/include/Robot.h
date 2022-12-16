// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


#pragma once

#include <frc/TimedRobot.h>                     // for TimedRobot
#include <frc/smartdashboard/SmartDashboard.h>  // for SmartDashboard
#include <rev/ColorSensorV3.h>                  // for ColorSensorV3
#include <stdint.h>                             // for uint8_t
#include "Robotmap.h"                         // for Robotmap
#include "auto/AutoPrograms.h"                // for AutoPrograms
#include "frc/I2C.h"                            // for I2C, I2C::Port, I2C::...
#include "frc/Timer.h"                          // for Timer
#include "frc/geometry/Pose2d.h"                // for Pose2d
#include "frc/geometry/Rotation2d.h"            // for Rotation2d
#include "networktables/NetworkTableInstance.h"    // for NetworkTableEntry
#include "units/angle.h"                        // for operator""_deg
#include "units/length.h"                       // for operator""_in, operat...
#include "frc/DataLogManager.h"
#include "networktables/BooleanTopic.h"
#include "networktables/DoubleTopic.h"

// Keep this around, these are the units lib base units, in order in units::base_unit:
// METERS
// KILOGRAMS
// SECONDS
// RADIANS
// AMPERES
// KELVIN
// MOLE
// CANDELA
// BYTE

class Robot : public frc::TimedRobot
{
public:
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void SimulationInit() override;
  void SimulationPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

private:

  double deadband(double val, double min = 0.10, double max = 1.0);

  Robotmap IO;
  AutoPrograms autoprograms{IO};

  wpi::log::DataLog &log = frc::DataLogManager::GetLog();

  frc::Timer seedEncoderTimer;

  nt::BooleanTopic configEntry = nt::NetworkTableInstance::GetDefault().GetBooleanTopic("/Smartdashboard/Recalibrate");
  nt::BooleanSubscriber configSub = configEntry.Subscribe(false);
  nt::BooleanPublisher configPub = configEntry.Publish();

  nt::DoubleTopic zeroEntry = nt::NetworkTableInstance::GetDefault().GetDoubleTopic("/Smartdashboard/Drivetrain/SeedEncoderLastResult");
  nt::DoublePublisher zeroPub = zeroEntry.Publish();
};
