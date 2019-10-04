#pragma once

#include <frc/commands/Subsystem.h>
#include <AHRS.h>

#include "subsystems/support/SwerveModule.h"


// DriveSubsystem
class DriveSubsystem : public frc::Subsystem
{
public: // PUBLIC METHODS
	DriveSubsystem();
	
	void InitDefaultCommand() override;
    
    // Use to scale the speed of the robot when more precise driving is needed.
	void SetSpeedScale(double scale);
	
    // Gets or Sets the swerve drive mode to either RobotCentric or FieldOriented
    inline const SwerveDriveMode & GetSwerveDriveMode() const { return m_drive_mode; }
	inline void SetSwerveDriveMode(SwerveDriveMode drive_mode) { m_drive_mode = drive_mode; }

    // Use to point all four swerve wheels outward at 45 degree angle making it 
    // harder for other robots to push us around.
    void ShieldWall();

    // Set all drive motor outputs to 0.0
    void StopDriveMotors();

    // Just for fun...no real application.
    // Takes into account speed scale setting.
	void TankDrive(double left, double right);

    // ...Okay just getting carried away now.
    // Takes into account speed scale setting.
	void CarDrive(double x, double y);

    // Use to drive robot in swerve. 
    // Takes into account both the swerve drive mode and speed scale settings.
    void SwerveDrive(double x, double y, double rot);
	
	// Use to display encoder values on the Dashboard
    void ShowEncoders();

private: // PRIVATE DATA
    // Pointers to the four swerve drive modules.
    std::vector<std::unique_ptr<SwerveModule>> m_swerve_modules;
    SwerveModule * m_left_fwd_module;
    SwerveModule * m_left_aft_module;
    SwerveModule * m_right_fwd_module;
    SwerveModule * m_right_aft_module;
    
    // Pointer to access navX functionality
	std::unique_ptr<AHRS> m_ahrs;
	
    // Used to scale the drive speeds for a drive modes.
	double m_speed_scale;

    // Stores the current swerve drive mode (RobotCentric or FieldOriented).
	SwerveDriveMode m_drive_mode;

private: // PRIVATE METHODS
    void InitializeSwerveModule(SwerveModulePosition position, 
        uint32_t drive_motor_id, uint32_t steer_motor_id, uint32_t steer_encoder_id);

    void NormalizeOutput(DriveValues<double> & output);
};