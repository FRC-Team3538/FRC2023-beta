// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/livewindow/LiveWindow.h>
#include <cmath>                                    // for abs, fabs
#include <exception>                                // for exception
#include <string>                                   // for string
#include "frc/Errors.h"                             // for RuntimeError
#include "lib/PS4Controller.h"                    // for PS4Controller
#include "lib/PneumaticHub.h"                     // for PneumaticHub
#include "units/base.h"                             // for unit_t, operator-
#include "units/math.h"                             // for abs
#include "units/time.h"
#include <iostream>
#include <math.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/ListenerExecutor.h>
#include <ntcore_c.h>

using namespace pathplanner;

void Robot::RobotInit()
{
  frc::SmartDashboard::PutNumber("auto/speed (feet per second)", 0);
  frc::SmartDashboard::PutNumber("auto/heading (degrees)", 0);
  frc::SmartDashboard::PutNumber("auto/duration (seconds)", 0);
  // Disable Live Window Stuff, we don't use it...
  frc::LiveWindow::DisableAllTelemetry();
  frc::LiveWindow::SetEnabled(false);

  // System Setup Stuff
  IO.ConfigureSystem();

  frc::SmartDashboard::PutData("Drivetrain", &IO.drivetrain);

  configPub.Set(false);

  // Logging Stuff
  frc::DataLogManager::LogNetworkTables(true);
  IO.RegisterDataEntries(log);
  // arg bool - log joystick data if true
  frc::DriverStation::StartDataLog(log, true);

  seedEncoderTimer.Start();
  IO.drivetrain.ResetYaw(frc::Rotation2d{});

  // localization_flag_entry.SetDefaultBoolean(false);
}

void Robot::RobotPeriodic()
{
  IO.drivetrain.UpdateOdometry();

  IO.UpdateSmartDash();
  autoprograms.SmartDash();

  for (nt::TimestampedBoolean val : configSub.ReadQueue()) {
    if (val.value) {
      IO.drivetrain.ConfigureSystem();
    }
  }

  // Logging Stuff
  IO.LogDataEntries(log);

  if (!IO.drivetrain.Active() && seedEncoderTimer.Get() > 5_s) {
      std::cout << "seeding encoders" << std::endl;

      zeroPub.Set(IO.drivetrain.SeedEncoders());
      seedEncoderTimer.Reset();
  }

  if (IO.mainController.GetOptionsButtonPressed())
  {
    IO.drivetrain.ResetYaw(0_deg);
  }

  if (IO.mainController.GetShareButtonPressed())
  {
    IO.drivetrain.SetFieldCentric(!IO.drivetrain.GetFieldCentric());
  }
}

void Robot::AutonomousInit()
{
  autoprograms.Init();
}

void Robot::AutonomousPeriodic()
{
  autoprograms.Run();
}

void Robot::TeleopInit()
{
  IO.drivetrain.SetFieldCentric(true);
}

void Robot::TeleopPeriodic()
{
  // DRIVE CODE
  auto forward = deadband(IO.mainController.GetLeftY(), 0.1, 1.0) * Drivetrain::kMaxSpeedLinear;
  auto strafe = deadband(IO.mainController.GetLeftX(), 0.1, 1.0) * Drivetrain::kMaxSpeedLinear;
  auto rotate = deadband(IO.mainController.GetRightX(), 0.1, 1.0) * Drivetrain::kMaxSpeedAngular;

  // std::cout << (forward / 1_mps).value() << ", " << (strafe / 1_mps).value() << ", " << (rotate / 1_rad_per_s).value() << std::endl;

  IO.drivetrain.Drive(frc::ChassisSpeeds{forward, strafe, rotate}, false);
}

void Robot::DisabledInit()
{
  IO.drivetrain.Stop();
}

void Robot::DisabledPeriodic()
{

}

void Robot::SimulationInit() {
  IO.SimInit();
}

void Robot::SimulationPeriodic()
{
  IO.SimPeriodic();
}

void Robot::TestInit()
{
  IO.pdp.ClearStickyFaults();
}

void Robot::TestPeriodic()
{

}

double Robot::deadband(double val, double min, double max)
{
  if (val > max)
  {
    return max;
  }
  else if (val < -max)
  {
    return -max;
  }
  else if (std::abs(val) < min)
  {
    return 0.0;
  }
  else
  {
    double sgn = val / std::abs(val);
    // return sgn * (std::abs(val) - min) / (max - min) * max;
    return sgn * (std::abs(val) - min) / (max - min) * max;
  }
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
