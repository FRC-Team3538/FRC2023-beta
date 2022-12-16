#pragma once

// Lower Level Robot Stuffs
#include "subsystems/SwerveModule.h"

// Utilities
#include <frc/smartdashboard/Field2d.h>
#include <frc/controller/HolonomicDriveController.h>
#include "Subsystem.h"
#include <frc/geometry/Translation2d.h>
#include <cmath>
#include <frc/smartdashboard/SendableChooser.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableHelper.h>
#include <units/angular_acceleration.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>

class Drivetrain : public Subsystem, 
                   public wpi::Sendable
{
public:
    Drivetrain();

    // Init Stuff
    void ConfigureSystem();

    // Telemetry
    void UpdateTelemetry();
    void RegisterDataEntries(wpi::log::DataLog &log);
    void LogDataEntries(wpi::log::DataLog &log);

    // Setters
    void Drive(units::meters_per_second_t xSpeed,
               units::meters_per_second_t ySpeed,
               units::radians_per_second_t rot,
               bool openLoop = true);
    void Drive(frc::Trajectory::State trajectoryState, units::radian_t yaw = 0_rad);
    void ResetYaw(units::radian_t yaw);
    void ResetOdometry(const frc::Pose2d &pose);
    void UpdateOdometry();
    void Stop();
    ErrorCode SeedEncoders();

    // Getters
    frc::Rotation2d GetYaw();
    frc::ChassisSpeeds GetChassisSpeeds();
    frc::Pose2d GetPose();

    // Telemetry / Smartdash
    void InitSendable(wpi::SendableBuilder &builder) override;

    // Simulation
    void SimInit() override;
    units::ampere_t SimPeriodic(units::volt_t volts) override;

    bool Active();

    bool GetFieldCentric();
    void SetFieldCentric(bool fieldCentric);

    // Public config values
    static constexpr units::meters_per_second_t kMaxSpeedLinear = 16_fps;
    static constexpr units::radians_per_second_t kMaxSpeedAngular = 360_deg_per_s;
    static constexpr units::meters_per_second_squared_t kMaxAccelerationLinear = 20_fps_sq;
    static constexpr units::inch_t kWheelToWheel = 20.5_in;

private:
    bool m_fieldRelative = true;

    // Configuration
    static constexpr auto dist = kWheelToWheel / 2;
    frc::Translation2d frontLeftLocation{+dist, +dist};
    frc::Translation2d frontRightLocation{+dist, -dist};
    frc::Translation2d backLeftLocation{-dist, +dist};
    frc::Translation2d backRightLocation{-dist, -dist};

    
    ctre::phoenix::sensors::WPI_Pigeon2 m_imu{30};
    ctre::phoenix::sensors::BasePigeonSimCollection m_imuSimCollection = m_imu.GetSimCollection();

    // Odomoetry
    frc::Field2d m_fieldDisplay;
    frc::FieldObject2d* m_estimatedPose = m_fieldDisplay.GetObject("SwerveDrivePoseEstimator");
    // frc::FieldObject2d* m_odometryPose = m_fieldDisplay.GetObject("SwerveDriveOdometry");

    // Control
    frc::ChassisSpeeds m_command;
    frc::ChassisSpeeds m_originalCommand;

    static constexpr auto kMaxModuleLinearAcceleration = 80.0_mps_sq;
    static constexpr auto kMaxModuleLinearJerk = 200.0_mps_sq / 1_s;

    static constexpr auto kMaxModuleAngularVelocity = 36_rad_per_s;
    static constexpr auto kMaxModuleAngularAcceleration = 1000_rad_per_s_sq;

    SwerveModuleConfig m_frontLeftConfig{
        -128.496_deg,
        {
            0.0092534,
            0.0,
            0.0,
            {
                kMaxModuleLinearAcceleration,
                kMaxModuleLinearJerk
            }
        },
        {
            0.098996,
            0.0,
            0.00055669,
            {
                kMaxModuleAngularVelocity,
                kMaxModuleAngularAcceleration
            }
        },
        {
            0.66323_V,
            2.1798_V / 1_mps,
            0.15467_V / 1_mps_sq
        },
        {
            0.72584_V,
            0.21377_V / 1_rad_per_s,
            0.0027946_V / 1_rad_per_s_sq
        }
    };

    SwerveModuleConfig m_frontRightConfig{
        78.838_deg,
        {
            0.0092534,
            0.0,
            0.0,
            {
                kMaxModuleLinearAcceleration,
                kMaxModuleLinearJerk
            }
        },
        {
            0.098996,
            0.0,
            0.00055669,
            {
                kMaxModuleAngularVelocity,
                kMaxModuleAngularAcceleration
            }
        },
        {
            0.66323_V,
            2.1798_V / 1_mps,
            0.15467_V / 1_mps_sq
        },
        {
            0.72584_V,
            0.21377_V / 1_rad_per_s,
            0.0027946_V / 1_rad_per_s_sq
        }
    };

    SwerveModuleConfig m_backLeftConfig{
        -4.219_deg,
        {
            0.0092534,
            0.0,
            0.0,
            {
                kMaxModuleLinearAcceleration,
                kMaxModuleLinearJerk
            }
        },
        {
            0.098996,
            0.0,
            0.00055669,
            {
                kMaxModuleAngularVelocity,
                kMaxModuleAngularAcceleration
            }
        },
        {
            0.66323_V,
            2.1798_V / 1_mps,
            0.15467_V / 1_mps_sq
        },
        {
            0.72584_V,
            0.21377_V / 1_rad_per_s,
            0.0027946_V / 1_rad_per_s_sq
        }
    };

    SwerveModuleConfig m_backRightConfig{
        -60.645_deg,
        {
            0.0092534,
            0.0,
            0.0,
            {
                kMaxModuleLinearAcceleration,
                kMaxModuleLinearJerk
            }
        },
        {
            0.098996,
            0.0,
            0.00055669,
            {
                kMaxModuleAngularVelocity,
                kMaxModuleAngularAcceleration
            }
        },
        {
            0.66323_V,
            2.1798_V / 1_mps,
            0.15467_V / 1_mps_sq
        },
        {
            0.72584_V,
            0.21377_V / 1_rad_per_s,
            0.0027946_V / 1_rad_per_s_sq
        }
    };

    // Heading Lock
    bool m_YawLockActive = true;
    frc::PIDController m_yawLockPID {5, 0.0, 0.0};
    frc::SendableChooser<std::string> yawLock;
    bool yawLockEnabled = true;

    // Odometry
    frc::SwerveDriveKinematics<4> m_kinematics{{
        frontLeftLocation,
        frontRightLocation,
        backLeftLocation,
        backRightLocation}};

    // frc::SwerveDriveOdometry<4> m_odometry{
    //     m_kinematics,
    //     frc::Rotation2d(),
    //     frc::Pose2d()};

    frc::ChassisSpeeds m_robotVelocity;

    // Swerve Modules
    SwerveModule m_frontLeft{"FL", 0, 1, 20, m_frontLeftConfig};
    SwerveModule m_frontRight{"FR", 2, 3, 21, m_frontRightConfig};
    SwerveModule m_backLeft{"BL", 4, 5, 22, m_backLeftConfig};
    SwerveModule m_backRight{"BR", 6, 7, 23, m_backRightConfig};

    frc::SwerveDrivePoseEstimator<4> m_poseEstimator{
        m_kinematics,
        frc::Rotation2d(),
        {
            m_frontLeft.GetModulePosition(),
            m_frontRight.GetModulePosition(),
            m_backLeft.GetModulePosition(),
            m_backRight.GetModulePosition(),
        },
        frc::Pose2d()};

    // Trajectory Following
    frc::HolonomicDriveController m_trajectoryController{
        frc2::PIDController{2.0, 0.0, 0.0},                      // X-error
        frc2::PIDController{2.0, 0.0, 0.0},                      // Y-error
        frc::ProfiledPIDController<units::radian>{2.0, 0.0, 0.0, // Rotation-error
                                                  frc::TrapezoidProfile<units::radian>::Constraints{
                                                      360_deg_per_s,
                                                      720_deg_per_s / 1_s}}};
};