#include "subsystems/SwerveModule.h"
#include <units/angle.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/RobotBase.h>
#include <iostream>

/**
 * Constructor for the SwerveModule class
 * 
 * @param moduleID String for module identification
 * @param driveMotorChannel CAN ID of the drive Falcon
 * @param turningMotorChannel CAN ID of the turning Falcon
 * @param turningEncoderChannel CAN ID of turning ABS encoder
 * @param config The configuration data for this specific module
 */
SwerveModule::SwerveModule(std::string moduleID, int driveMotorChannel, int turningMotorChannel, int turningEncoderChannel, SwerveModuleConfig config)
    : moduleID(moduleID),
      config(config),
      m_driveMotor(driveMotorChannel),
      m_turningMotor(turningMotorChannel),
      m_turnEncoder(turningEncoderChannel),
      m_driveFeedforward{config.driveFf.GetFeedForward()},
      m_turnFeedforward{config.turnFf.GetFeedForward()},
      angle_offset(config.angleOffset)
{}

/**
 * Returns the current state of the module
 * 
 * @return frc::SwerveModuleState The current state of the swerve module
 */
frc::SwerveModuleState SwerveModule::GetState()
{
  currentState.angle = GetAngle();
  currentState.speed = GetVelocity();
  return currentState;
}

frc::SwerveModulePosition SwerveModule::GetModulePosition() {
  return {
    GetPosition(),
    GetAngle()
  };
}


/**
 * Returns the current velocity of the module
 * 
 * @return units::meters_per_second_t The current velocity of the swerve module
 */
units::meters_per_second_t SwerveModule::GetVelocity()
{
  // Real Hardware
  return m_driveMotor.GetSelectedSensorVelocity(0) * 1_tr / 2048 * kDriveScaleFactor / 100_ms;
}

units::meter_t SwerveModule::GetPosition()
{
  // Real Hardware
  return m_driveMotor.GetSelectedSensorPosition(0) * 1_tr / 2048 * kDriveScaleFactor;
}

/**
 * Returns the current angle of the module
 * 
 * @return frc::Rotation2d The current angle of the swerve module
 */
frc::Rotation2d SwerveModule::GetAngle()
{
  // Real Hardware
  auto un_normalized = frc::Rotation2d(m_turnEncoder.GetAbsolutePosition() * 1_deg);
  return frc::Rotation2d(un_normalized.Cos(), un_normalized.Sin());
}

/**
 * Returns the current angle of the module
 * 
 * @return frc::Rotation2d The current angle of the swerve module
 */
frc::Rotation2d SwerveModule::GetMotorAngle()
{
  return frc::Rotation2d(m_turningMotor.GetSelectedSensorPosition() / 2048 / kTurnGearboxRatio * 2 * units::constants::pi * 1_rad);
}

void SwerveModule::Drive(units::meters_per_second_t target, bool openLoop)
{
  m_driveVolts = m_driveFeedforward.Calculate(target);

  auto target_in_native_units = target / kDriveScaleFactor * 100_ms * 2048 / 1_tr;

  if (openLoop) {
    m_driveMotor.SetVoltage(m_driveVolts);
  } else {
    m_driveMotor.Set(TalonFXControlMode::Velocity, target_in_native_units, DemandType::DemandType_ArbitraryFeedForward, m_driveVolts / 12_V);
  }
}

void SwerveModule::Turn(frc::Rotation2d target)
{
  auto normalized_target_angle = frc::Rotation2d(target.Cos(), target.Sin());

  auto normalized_current_angle = GetState().angle;
  auto actual_current_angle = GetMotorAngle();

  auto target_diff = normalized_target_angle - normalized_current_angle;

  auto actual_target_angle = actual_current_angle.Radians() + target_diff.Radians();

  auto target_in_native_units = actual_target_angle * kTurnGearboxRatio * 2048 / 1_tr;

  m_turningMotor.Set(TalonFXControlMode::MotionMagic, target_in_native_units);
}

/**
 * Sets the module's target state
 * 
 * @param state The module's target state
 */
void SwerveModule::SetModule(const frc::SwerveModuleState &state, bool openLoop)
{
  currentState = GetState();

  targetState = frc::SwerveModuleState::Optimize(state, currentState.angle);

  Drive(targetState.speed, openLoop);

  Turn(targetState.angle.Radians());
}

/**
 * Just Stop....
 *
 */
void SwerveModule::Stop()
{
  m_driveMotor.SetVoltage(0_V);
  m_turningMotor.SetVoltage(0_V);

  // Simulation
  m_driveSim.SetInputVoltage(0_V);
  m_turnSim.SetInputVoltage(0_V);
}

void SwerveModule::ConfigureSystem()
{
  std::cout << "A: " << m_turnPlant.A() << std::endl; std::cout << "B: " << m_turnPlant.B() << std::endl; std::cout << "C: " << m_turnPlant.C() << std::endl; std::cout << "D: " << m_turnPlant.D() << std::endl;
  if (frc::RobotBase::IsSimulation())
  {
    m_driveFeedforward.kS = 0_V;
    m_turnFeedforward.kS = 0_V;
  }

  // Drive Motor Configuration
  m_driveMotor.ConfigFactoryDefault();
  m_driveMotor.SetInverted(true); // Remember: forward-positive!
  m_driveMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  m_driveMotor.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, kDriveMotorCurrentLimit.value(), kDriveMotorCurrentLimit.value(), 0.0));
  m_driveMotor.SetSensorPhase(false);

  driveSlotConfig.kP = config.drivePID.kP;
  driveSlotConfig.kD = config.drivePID.kD;

  {
    auto allSettings = TalonFXConfiguration();
    m_driveMotor.GetAllConfigs(allSettings);

    allSettings.supplyCurrLimit = {true, kDriveMotorCurrentLimit / 1_A, kDriveMotorCurrentLimit / 1_A, 0.0};
    allSettings.slot0 = driveSlotConfig;

    auto accel_in_drive_motor_frame = config.drivePID.constraints.maxVelocity / kDriveScaleFactor;
    auto accel_in_drive_motor_units = accel_in_drive_motor_frame * 2048 / 1_tr * 100_ms * 1_s;

    allSettings.motionAcceleration = accel_in_drive_motor_units;

    auto error = m_driveMotor.ConfigAllSettings(allSettings);

    if (error != ErrorCode::OK) {
      std::cout << "err on config drive motor: " << error << std::endl;
    }
  }


  // Turning Motor Configuration
  m_turningMotor.ConfigFactoryDefault();
  m_turningMotor.SetInverted(false); // Remember: forward-positive!
  m_turningMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

  turnSlotConfig.kP = config.turnPID.kP;
  turnSlotConfig.kD = config.turnPID.kD;

  {
    auto allSettings = TalonFXConfiguration();
    m_turningMotor.GetAllConfigs(allSettings);

    allSettings.supplyCurrLimit = {true, kTurningMotorCurrentLimit / 1_A, kTurningMotorCurrentLimit / 1_A, 0.0};
    allSettings.slot0 = turnSlotConfig;

    auto vel_in_turn_motor_frame = config.turnPID.constraints.maxVelocity * kTurnGearboxRatio;
    auto vel_in_turn_motor_units = vel_in_turn_motor_frame * 2048 / 1_tr * 100_ms;

    auto accel_in_turn_motor_frame = config.turnPID.constraints.maxAcceleration * kTurnGearboxRatio;
    auto accel_in_turn_motor_units = accel_in_turn_motor_frame * 2048 / 1_tr * 100_ms * 1_s;

    allSettings.motionCruiseVelocity = vel_in_turn_motor_units;
    allSettings.motionAcceleration = accel_in_turn_motor_units;

    auto error = m_turningMotor.ConfigAllSettings(allSettings);

    if (error != ErrorCode::OK) {
      std::cout << "err on config turn motor: " << error << std::endl;
    }
  }

  // Turning Encoder Config
  ctre::phoenix::sensors::CANCoderConfiguration encoderConfig;
  m_turnEncoder.GetAllConfigs(encoderConfig);
  encoderConfig.enableOptimizations = true;
  encoderConfig.initializationStrategy = ctre::phoenix::sensors::SensorInitializationStrategy::BootToAbsolutePosition;
  encoderConfig.absoluteSensorRange = ctre::phoenix::sensors::AbsoluteSensorRange::Signed_PlusMinus180;
  if (frc::RobotBase::IsReal())
  {
    encoderConfig.magnetOffsetDegrees = angle_offset / 1_deg;
  }
  encoderConfig.sensorDirection = false;
  m_turnEncoder.ConfigAllSettings(encoderConfig);
}

void SwerveModule::RegisterDataEntries(wpi::log::DataLog &log)
{
}

void SwerveModule::LogDataEntries(wpi::log::DataLog &log)
{

}

void SwerveModule::UpdateTelemetry()
{
  
}

void SwerveModule::InitSendable(wpi::SendableBuilder &builder)
{
  builder.SetSmartDashboardType("SwerveModule");
  builder.SetActuator(true);

  // Turning Encoders
  builder.AddDoubleProperty(
      moduleID + "/abs_encoder", [this] { return GetAngle().Radians() / 1_rad; }, nullptr);
  builder.AddDoubleProperty(
      moduleID + "/turn_motor_encoder", [this] { return GetMotorAngle().Radians() / 1_rad; }, nullptr);

  builder.AddDoubleProperty(moduleID + "/raw_drive_volts", [this] { return m_driveVolts / 1_V; }, nullptr);
  builder.AddDoubleProperty(moduleID + "/sim_drive_volts", [this] { return driveMotorSim.GetMotorOutputLeadVoltage(); }, nullptr);
  builder.AddDoubleProperty(moduleID + "/raw_turn_volts", [this] { return m_turnVolts / 1_V; }, nullptr);
  builder.AddDoubleProperty(moduleID + "/sim_turn_volts", [this] { return turningMotorSim.GetMotorOutputLeadVoltage(); }, nullptr);

  // Thermal
  // builder.AddDoubleProperty(
  //     moduleID + "/Drive Temp [C]", [this] { return m_driveMotor.GetTemperature(); }, nullptr);
  // builder.AddDoubleProperty(
  //     moduleID + "/Angle Temp [C]", [this] { return m_turningMotor.GetTemperature(); }, nullptr);

  builder.AddDoubleProperty(
    moduleID + "/state/speed", [this] {return GetState().speed / 1_mps; }, nullptr);
  builder.AddDoubleProperty(
    moduleID + "/state/angle", [this] {return GetState().angle.Radians() / 1_rad; }, nullptr);

  builder.AddDoubleProperty(
    moduleID + "/goal/speed", [this] {return targetState.speed / 1_mps; }, nullptr);
  builder.AddDoubleProperty(
    moduleID + "/goal/angle", [this] {return targetState.angle.Radians() / 1_rad; }, nullptr);

  builder.AddDoubleProperty(moduleID + "/sim/speed", [this] { return m_driveSim.GetLinearVelocity() / 1_mps; }, nullptr);
  builder.AddDoubleProperty(moduleID + "/sim/turret_speed", [this] { return m_turnSim.GetAngularVelocity() / 1_rad_per_s; }, nullptr);
  builder.AddDoubleProperty(moduleID + "/sim/angle", [this] { return m_turnSim.GetAngle() / 1_rad; }, nullptr);

  builder.AddDoubleProperty(moduleID + "/drive/PID/kP", [this] {
    return driveSlotConfig.kP;
  },
  [this] (double val) {
    driveSlotConfig.kP = val;
    m_driveMotor.ConfigureSlot(driveSlotConfig, 0, 0);
  });
  builder.AddDoubleProperty(moduleID + "/drive/PID/kD", [this] {
    return driveSlotConfig.kD;
  },
  [this] (double val) {
    driveSlotConfig.kD = val;
    m_driveMotor.ConfigureSlot(driveSlotConfig, 0, 0);
  });

    builder.AddDoubleProperty(moduleID + "/turn/PID/kP", [this] {
    return turnSlotConfig.kP;
  },
  [this] (double val) {
    turnSlotConfig.kP = val;
    m_turningMotor.ConfigureSlot(turnSlotConfig, 0, 0);
  });
  builder.AddDoubleProperty(moduleID + "/turn/PID/kD", [this] {
    return turnSlotConfig.kD;
  },
  [this] (double val) {
    turnSlotConfig.kD = val;
    m_turningMotor.ConfigureSlot(turnSlotConfig, 0, 0);
  });

  builder.AddDoubleProperty(moduleID + "/turn/SensorVel", [this] { return m_turningMotor.GetSelectedSensorVelocity(0); }, nullptr);
  builder.AddDoubleProperty(moduleID + "/turn/SensorPos", [this] { return m_turningMotor.GetSelectedSensorPosition(0); }, nullptr);
  builder.AddDoubleProperty(moduleID + "/turn/MotorOutputPercent", [this] { return m_turningMotor.GetMotorOutputPercent(); }, nullptr);
  builder.AddDoubleProperty(moduleID + "/turn/MotorOutputSimVoltage", [this] { return m_turningMotor.GetSimCollection().GetMotorOutputLeadVoltage(); }, nullptr);
  builder.AddDoubleProperty(moduleID + "/turn/ClosedLoopError", [this] { return m_turningMotor.GetClosedLoopError(0); }, nullptr);
  builder.AddDoubleProperty(moduleID + "/turn/ClosedLoopTarget", [this] { return m_turningMotor.GetClosedLoopTarget(0); }, nullptr);

  builder.AddDoubleProperty(moduleID + "/drive/SensorVel", [this] { return m_driveMotor.GetSelectedSensorVelocity(0); }, nullptr);
  builder.AddDoubleProperty(moduleID + "/drive/SensorPos", [this] { return m_driveMotor.GetSelectedSensorPosition(0); }, nullptr);
  builder.AddDoubleProperty(moduleID + "/drive/MotorOutputPercent", [this] { return m_driveMotor.GetMotorOutputPercent(); }, nullptr);
  builder.AddDoubleProperty(moduleID + "/drive/MotorOutputSimVoltage", [this] { return m_driveMotor.GetSimCollection().GetMotorOutputLeadVoltage(); }, nullptr);
  builder.AddDoubleProperty(moduleID + "/drive/ClosedLoopError", [this] { return m_driveMotor.GetClosedLoopError(0); }, nullptr);
  builder.AddDoubleProperty(moduleID + "/drive/ClosedLoopTarget", [this] { return m_driveMotor.GetClosedLoopTarget(0); }, nullptr);
}

units::ampere_t SwerveModule::SimDrive(units::volt_t battery)
{
  driveMotorSim.SetBusVoltage(12); //battery / 1_V);
  // -1 because drive motor is inverted
  m_driveSim.SetInputVoltage(-1_V * driveMotorSim.GetMotorOutputLeadVoltage());

  m_driveSim.Update(20_ms);

  driveMotorSim.SetStatorCurrent(m_driveSim.GetCurrentDraw() / 1_A);

  auto rpm_at_wheel = m_driveSim.GetAngularVelocity();
  auto rpm_at_motor = rpm_at_wheel * kDriveGearboxRatio;
  auto rpm_in_native_units = rpm_at_motor / 1_tr * 2048 * 100_ms;

  driveMotorSim.SetIntegratedSensorVelocity(-rpm_in_native_units);
  driveMotorSim.AddIntegratedSensorPosition(-rpm_in_native_units * 20_ms / 100_ms);

  return m_driveSim.GetCurrentDraw();
}

units::ampere_t SwerveModule::SimTurn(units::volt_t battery)
{

  turningMotorSim.SetBusVoltage(12); //battery / 1_V);
  encoderSim.SetBusVoltage(12); //battery / 1_V);
  m_turnSim.SetInputVoltage(1_V * turningMotorSim.GetMotorOutputLeadVoltage());

  m_turnSim.Update(20_ms);

  turningMotorSim.SetStatorCurrent(m_turnSim.GetCurrentDraw() / 1_A);

  auto angular_pos = m_turnSim.GetAngle();
  auto angular_vel = m_turnSim.GetAngularVelocity();

  // native cancoder units are 4096 ticks per rotation, vel measured per 100ms
  auto angular_pos_at_cancoder = angular_pos / 1_tr * 4096;
  auto angular_vel_at_cancoder = angular_vel / 1_tr * 4096 * 100_ms;

  encoderSim.SetRawPosition(angular_pos_at_cancoder);
  encoderSim.SetVelocity(angular_vel_at_cancoder);

  // native falcon units are 2048 ticks per rotation, vel measured per 100ms
  auto angular_pos_at_motor = angular_pos / 1_tr * kTurnGearboxRatio * 2048;
  auto angular_vel_at_motor = angular_vel / 1_tr * kTurnGearboxRatio * 2048 * 100_ms;

  turningMotorSim.SetIntegratedSensorRawPosition(angular_pos_at_motor);
  turningMotorSim.SetIntegratedSensorVelocity(angular_vel_at_motor);

  return m_turnSim.GetCurrentDraw();
}

units::ampere_t SwerveModule::SimPeriodic(units::volt_t battery)
{
  return SimDrive(battery) + SimTurn(battery);
}

void SwerveModule::SimInit()
{
  
}

ErrorCode SwerveModule::SeedTurnMotor()
{
  auto position_at_cancoder = m_turnEncoder.GetAbsolutePosition() * 1_deg;
  auto position_at_motor = position_at_cancoder / 1_tr * kTurnGearboxRatio * 2048;

  auto error = m_turnEncoder.GetLastError();
  if (error != ErrorCode::OK) {
    return error;
  }

  m_turningMotor.Set(TalonFXControlMode::Disabled, 0);
  error = m_turningMotor.GetLastError();
  if (error != ErrorCode::OK) {
    return error;
  }

  return m_turningMotor.SetSelectedSensorPosition(position_at_motor);
}

units::radians_per_second_t SwerveModule::AngularVelocity()
{
  auto motor_rps = m_turningMotor.GetSelectedSensorVelocity() * 1_tr / 2048 / 100_ms;

  return motor_rps / kTurnGearboxRatio;
}