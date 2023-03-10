#include "subsystems/Drivetrain.h"
#include <frc/smartdashboard/SmartDashboard.h>

Drivetrain::Drivetrain()
{

}

void Drivetrain::UpdateTelemetry()
{

}

void Drivetrain::RegisterDataEntries(wpi::log::DataLog &log)
{

}

void Drivetrain::LogDataEntries(wpi::log::DataLog &log)
{

}

void Drivetrain::ConfigureSystem()
{
  m_imu.ZeroGyroBiasNow(50);
  ResetOdometry(GetPose());

  m_yawLockPID.EnableContinuousInput(-0.5_tr / 1_rad, 0.5_tr / 1_rad);
  m_yawLockPID.SetTolerance(3_deg / 1_rad);

  // Display Robot position on field
  frc::SmartDashboard::PutData("Field", &m_fieldDisplay);

  m_frontLeft.ConfigureSystem();
  m_frontRight.ConfigureSystem();
  m_backLeft.ConfigureSystem();
  m_backRight.ConfigureSystem();
}

void Drivetrain::Drive(pathplanner::PathPlannerTrajectory::PathPlannerState targetState)
{
  auto command = m_ppController.calculate(
      m_poseEstimator.GetEstimatedPosition(),
      targetState);

  command.omega = -1 *command.omega;

  Drive(command, false);
}

void Drivetrain::Drive(frc::ChassisSpeeds speed,
                       bool openLoop)
{

  // Heading Lock
  // constexpr auto noRotThreshold = 0.1_deg_per_s;
  // m_YawLockActive = units::math::abs(rot) < noRotThreshold;

  // const auto trans_mag = units::math::sqrt(xSpeed * xSpeed + ySpeed * ySpeed);
  
  // if (m_YawLockActive && trans_mag > 0.1_mps)
  // {
  //   auto r = m_yawLockPID.Calculate(GetYaw().Radians() / 1_rad);
  //   if (!m_yawLockPID.AtSetpoint())
  //   {
  //     // Robot will automatically maintain current yaw
  //     rot = units::radians_per_second_t{-r};
  //   } else {
  //     m_yawLockPID.Reset();
  //   }
  // }
  // else
  // {
  //   // Manual control, save the current yaw.
  //   m_yawLockPID.Reset();
  //   m_yawLockPID.SetSetpoint(GetYaw().Radians() / 1_rad);
  // }

  m_input = speed;
  
  // Transform Field Oriented command to a Robot Relative Command
  if (m_fieldRelative)
  {
    m_command = frc::ChassisSpeeds::FromFieldRelativeSpeeds(speed.vx, speed.vy, speed.omega, GetYaw());
  }
  else
  {
    m_command = speed;
  }

  m_originalCommand = m_command;

  // Calculate desired swerve states
  auto states = m_kinematics.ToSwerveModuleStates(m_command);
  // m_kinematics.DesaturateWheelSpeeds(&states, m_command, 16_fps, 16_fps, 360_deg_per_s);
  m_kinematics.DesaturateWheelSpeeds(&states, m_command, kTrueMaxSpeedLinear, kTrueMaxSpeedLinear, kTrueMaxSpeedAngular);

  m_command = m_kinematics.ToChassisSpeeds(states);

  // Set State of Each Module
  auto [fl, fr, bl, br] = states;
  m_frontLeft.SetModule(fl, openLoop);
  m_frontRight.SetModule(fr, openLoop);
  m_backLeft.SetModule(bl, openLoop);
  m_backRight.SetModule(br, openLoop);
}

void Drivetrain::Stop()
{
  m_frontLeft.Stop();
  m_frontRight.Stop();
  m_backLeft.Stop();
  m_backRight.Stop();
}

frc::Rotation2d Drivetrain::GetIMUYaw()
{
  return m_imu.GetRotation2d();
}

frc::Rotation2d Drivetrain::GetYaw()
{
  return GetPose().Rotation();
}

void Drivetrain::UpdateOdometry()
{
  auto estPose = m_poseEstimator.Update(GetIMUYaw(),{
                         m_frontLeft.GetModulePosition(),
                         m_frontRight.GetModulePosition(),
                         m_backLeft.GetModulePosition(),
                         m_backRight.GetModulePosition()});
  
  m_robotVelocity = m_kinematics.ToChassisSpeeds({m_frontLeft.GetState(),
                                                  m_frontRight.GetState(),
                                                  m_backLeft.GetState(),
                                                  m_backRight.GetState()});

  m_estimatedPose->SetPose(estPose);
}

void Drivetrain::ResetYaw(const frc::Rotation2d &yaw)
{
  m_imu.SetYaw(yaw.Radians() / 1_deg, 50);
  m_yawLockPID.Reset();
  m_yawLockPID.SetSetpoint(yaw.Radians() / 1_rad);
}

void Drivetrain::ResetOdometry(const frc::Pose2d &pose)
{
  m_poseEstimator.ResetPosition(GetIMUYaw(), {
                         m_frontLeft.GetModulePosition(),
                         m_frontRight.GetModulePosition(),
                         m_backLeft.GetModulePosition(),
                         m_backRight.GetModulePosition()}, pose);
}

frc::ChassisSpeeds Drivetrain::GetChassisSpeeds()
{
  return m_robotVelocity;
}

void Drivetrain::InitSendable(wpi::SendableBuilder &builder)
{
  builder.SetSmartDashboardType("DriveBase");
  builder.SetActuator(true);

  // Modules
  m_frontLeft.InitSendable(builder);
  m_frontRight.InitSendable(builder);
  m_backLeft.InitSendable(builder);
  m_backRight.InitSendable(builder);

  builder.AddDoubleProperty("gyro", [this] { return units::degree_t{m_imu.GetYaw()} / 1_rad; }, nullptr);
  
  // Pose
  builder.AddDoubleProperty(
      "poseEstimator/x", [this] { return m_poseEstimator.GetEstimatedPosition().X().value(); }, nullptr);
  builder.AddDoubleProperty(
      "poseEstimator/y", [this] { return m_poseEstimator.GetEstimatedPosition().Y().value(); }, nullptr);
  builder.AddDoubleProperty(
      "poseEstimator/yaw", [this] { return m_poseEstimator.GetEstimatedPosition().Rotation().Degrees().value(); }, nullptr);

  // builder.AddDoubleProperty(
  //     "odometry/x", [this] { return m_odometry.GetPose().X().value(); }, nullptr);
  // builder.AddDoubleProperty(
  //     "odometry/y", [this] { return m_odometry.GetPose().Y().value(); }, nullptr);
  // builder.AddDoubleProperty(
  //     "odometry/yaw", [this] { return m_odometry.GetPose().Rotation().Degrees().value(); }, nullptr);

  // Command
    builder.AddDoubleProperty(
      "input/x", [this] { return m_input.vx / 1_mps; }, nullptr);
  builder.AddDoubleProperty(
      "input/y", [this] { return m_input.vy / 1_mps; }, nullptr);
  builder.AddDoubleProperty(
      "input/yaw", [this] { return m_input.omega / 1_rad_per_s; }, nullptr);

  builder.AddDoubleProperty(
      "cmd/x", [this] { return m_command.vx / 1_mps; }, nullptr);
  builder.AddDoubleProperty(
      "cmd/y", [this] { return m_command.vy / 1_mps; }, nullptr);
  builder.AddDoubleProperty(
      "cmd/yaw", [this] { return m_command.omega / 1_rad_per_s; }, nullptr);

  builder.AddDoubleProperty(
      "demand/x", [this] { return m_originalCommand.vx / 1_mps; }, nullptr);
  builder.AddDoubleProperty(
      "demand/y", [this] { return m_originalCommand.vy / 1_mps; }, nullptr);
  builder.AddDoubleProperty(
      "demand/yaw", [this] { return m_originalCommand.omega / 1_rad_per_s; }, nullptr);

  // State
    builder.AddDoubleProperty(
      "state/x", [this] { return m_robotVelocity.vx / 1_mps; }, nullptr);
  builder.AddDoubleProperty(
      "state/y", [this] { return m_robotVelocity.vy / 1_mps; }, nullptr);
  builder.AddDoubleProperty(
      "state/yaw", [this] { return m_robotVelocity.omega / 1_rad_per_s; }, nullptr);

  // Operating Mode
  builder.AddBooleanProperty(
      "cmd/fieldRelative", [this] { return m_fieldRelative; }, nullptr);

  builder.AddDoubleProperty(
      "yawpid/setpoint", [this] { return m_yawLockPID.GetSetpoint(); }, nullptr); 
  builder.AddDoubleProperty(
      "yawpid/error", [this] { return m_yawLockPID.GetPositionError(); }, nullptr);
  builder.AddBooleanProperty(
      "yawpid/atSetpoint", [this] { return m_yawLockPID.AtSetpoint(); }, nullptr);
}

units::ampere_t Drivetrain::SimPeriodic(units::volt_t battery)
{

  // Simulated IMU
  m_imuSimCollection.AddHeading(GetChassisSpeeds().omega * 20_ms / 1_deg);

  return m_frontLeft.SimPeriodic(battery) +
    m_frontRight.SimPeriodic(battery) +
    m_backLeft.SimPeriodic(battery) +
    m_backRight.SimPeriodic(battery);
}

void Drivetrain::SimInit()
{
  m_frontLeft.SimInit();
  m_frontRight.SimInit();
  m_backLeft.SimInit();
  m_backRight.SimInit();
}

ErrorCode Drivetrain::SeedEncoders() {
  auto error = m_frontLeft.SeedTurnMotor();
  if (error != ErrorCode::OK) {
    return error;
  }

  error = m_frontRight.SeedTurnMotor();
  if (error != ErrorCode::OK) {
    return error;
  }

  error = m_backLeft.SeedTurnMotor();
  if (error != ErrorCode::OK) {
    return error;
  }

  return m_backRight.SeedTurnMotor();
}

bool Drivetrain::Active()
{
  return units::math::abs(m_frontLeft.AngularVelocity()) > 0.01_rad_per_s
    || units::math::abs(m_frontRight.AngularVelocity()) > 0.01_rad_per_s
    || units::math::abs(m_backLeft.AngularVelocity()) > 0.01_rad_per_s
    || units::math::abs(m_backRight.AngularVelocity()) > 0.01_rad_per_s;
}

frc::Pose2d Drivetrain::GetPose()
{
  return m_poseEstimator.GetEstimatedPosition();
}

bool Drivetrain::GetFieldCentric()
{
  return m_fieldRelative;
}

void Drivetrain::SetFieldCentric(bool fieldRelative)
{
  m_fieldRelative = fieldRelative;
}