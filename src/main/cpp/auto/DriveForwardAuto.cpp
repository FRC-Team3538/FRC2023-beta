#include <auto/DriveForwardAuto.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/PathPlanner.h>

#include <fmt/core.h>

DriveForwardAuto::DriveForwardAuto(Robotmap &IO):
    IO(IO)
{
    m_traj = pathplanner::PathPlanner::loadPath("Basic Auton Loop", pathplanner::PathConstraints{4_mps, 3_mps_sq});
}

DriveForwardAuto::~DriveForwardAuto()
{}

void DriveForwardAuto::Init()
{
    // fmt::print("Starting basic auton loop\n");
    IO.drivetrain.ResetOdometry(m_traj.getInitialHolonomicPose());
    IO.drivetrain.SetFieldCentric(false);

    timer.Start();
}

void DriveForwardAuto::Run()
{
    // fmt::print("{} >? {}\n", m_traj.getTotalTime(), timer.Get());
    if (m_traj.getTotalTime() < timer.Get()) {
        IO.drivetrain.Stop();
        // fmt::print("auton done!\n");
        return;
    }
    // fmt::print("basic auton loop\n");

    IO.drivetrain.Drive(m_traj.sample(timer.Get()));
}

std::string DriveForwardAuto::GetName()
{
    return "Basic Auton Loop";
}

void DriveForwardAuto::UpdateSmartDash()
{
    return;
}