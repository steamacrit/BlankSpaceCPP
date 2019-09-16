#pragma once

#include <frc/commands/PIDSubsystem.h>
#include <frc/commands/Subsystem.h>
#include <ctre/Phoenix.h>
#include "subsystems/swerve_support/SwerveModule.h"

constexpr double DRIVE_BASE_WIDTH { 25.0 };
constexpr double DRIVE_BASE_WIDTH_HALVED { DRIVE_BASE_WIDTH / 2.0 };

constexpr double DRIVE_BASE_LENGTH { 25.0 };
constexpr double DRIVE_BASE_LENGTH_HAVLED { DRIVE_BASE_LENGTH / 2.0 };

constexpr double SLOW_SCALAR{ 0.4 };

enum class DriveMode
{
	RobotCentric,
	FieldCentric
};

enum class SpeedMode
{
	Crawl,
	Ramp,
	Full
};


class SwerveDriveSubsystem : public frc::Subsystem
{
public:
	static SwerveDriveSubsystem * GetInstance();
	
	void InitDefaultCommand() override;

    void SwerveDrive(double x, double y, double rot, double yaw, double percent_output = 1.0);
    void SetRotationPoint(double x, double y);

private:
    std::vector<std::unique_ptr<SwerveModule>> m_swerve;
    
	DriveMode m_drive_mode;
	SpeedMode m_speed_mode;

    double m_xrot_pt;
    double m_yrot_pt;
    double m_got;

private:
	SwerveDriveSubsystem();

    void Steer(double x, double y, double rot);
    void RotateAngle(double & x, double & y, double angle);
    void Normalize(double & value, const bool & slow);
    void TargetAngleAndSpeed(double x, double y, double rot, double width, double length,
        double * angle, double * speed);
};