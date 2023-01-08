#include <auto/AutoInterface.h>
#include <Robotmap.h>
#include <frc/Timer.h>
#include <pathplanner/lib/PathPlannerTrajectory.h>

class DriveForwardAuto: public AutoInterface {
public:
    DriveForwardAuto(Robotmap &);
    DriveForwardAuto() = delete;
    ~DriveForwardAuto();

    void Init() override;
    void Run() override;
    void UpdateSmartDash() override;

private:
    Robotmap &IO;

    frc::Timer timer;

    pathplanner::PathPlannerTrajectory m_traj;
public:
    static std::string GetName();
};