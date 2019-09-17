#pragma once

#include <frc/commands/PIDSubsystem.h>
#include <frc/commands/Subsystem.h>
#include <ctre/Phoenix.h>
#include "subsystems/swerve_support/SwerveModule.h"

constexpr double DRIVE_BASE_WIDTH { 25.0 };
constexpr double DRIVE_BASE_WIDTH_HALVED { DRIVE_BASE_WIDTH / 2.0 };

constexpr double DRIVE_BASE_LENGTH { 25.0 };
constexpr double DRIVE_BASE_LENGTH_HAVLED { DRIVE_BASE_LENGTH / 2.0 };

enum class SwerveDriveMode
{
	RobotCentric,
	FieldCentric
};


class SwerveDriveSubsystem : public frc::Subsystem
{
public:
	SwerveDriveSubsystem();
	
	void InitDefaultCommand() override;
    
    void ShieldWall();
	void TankDrive(double left, double right);
	void CarDrive(double x, double y, double percent_output = 1.0);
    void SwerveDrive(double x, double y, double rot, double yaw, double percent_output = 1.0);
    void SetRotationPoint(double x, double y);
	
	inline const SwerveDriveMode & GetSwerveDriveMode() const { return m_drive_mode; }
	inline void SetSwerveDriveMode(SwerveDriveMode drive_mode) { m_drive_mode = drive_mode; }
	
    void ShowEncoders();
    void CalibrateEncoders();
private:
    std::vector<std::unique_ptr<SwerveModule>> m_swerve;
    
	SwerveDriveMode m_drive_mode;

    double m_xrot_pt;
    double m_yrot_pt;
    double m_got;

private:
    void 
	SwerveSteer(double x, double y, double rot, double percent_output = 1.0);
    void RotateAngle(double & x, double & y, double angle);
    void Normalize(double & value, const bool & slow);
    void TargetAngleAndSpeed(double x, double y, double rot, double width, double length,
        double * angle, double * speed);
};