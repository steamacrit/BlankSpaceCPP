#pragma once

#include <frc/PIDController.h>
#include <frc/AnalogInput.h>
#include <frc/commands/Subsystem.h>

#include <AHRS.h>

#include <ctre/Phoenix.h>
#include "subsystems/support/TalonSRX_PID.h"

// Enumeration used to drive orientation
enum class SwerveDriveMode
{
	RobotCentric,
	FieldOriented
};

// Structure used to hold various data of type double for each individual swerve drive module
struct DriveValues
{
	double lf{ 0.0 };
	double la{ 0.0 };
	double rf{ 0.0 };
	double ra{ 0.0 };
};

// Convenience class for managing each swerve drive module separately
class DriveModule
{
public:
	DriveModule(uint32_t drive_motor_id, uint32_t steer_motor_id, uint32_t steer_encoder_id);
	
	void Drive(double speed, double angle);
	
	void Steer(double angle);
	void SetMotorPercentOutput(double output);
	
	inline TalonSRX * GetDriveMotor() { return m_drive_motor.get(); }
	inline TalonSRX * GetSteerMotor() { return m_steer_motor.get(); }
	inline frc::AnalogInput * GetSteerEncoder() { return m_steer_encoder.get(); }
	inline double GetSteerEncoderValue() { return m_steer_encoder->GetValue(); }

private: // PRIVATE DATA
	std::unique_ptr<TalonSRX> m_drive_motor;
	std::unique_ptr<TalonSRX_PID> m_steer_motor;
	std::unique_ptr<frc::AnalogInput> m_steer_encoder;
	std::unique_ptr<frc::PIDController> m_pid;
};


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
    std::unique_ptr<DriveModule> m_left_fwd_module;
    std::unique_ptr<DriveModule> m_left_aft_module;
    std::unique_ptr<DriveModule> m_right_fwd_module;
    std::unique_ptr<DriveModule> m_right_aft_module;
    
    // Pointer to access navX functionality
	std::unique_ptr<AHRS> m_ahrs;
	
    // Internal method for calculating the drive speeds and wheel angles for all swerve modules.
	void SpeedAndAngle(double x, double y, double rot, DriveValues * speeds, DriveValues * angles);
	
    // Used to scale the drive speeds for a drive modes.
	double m_speed_scale;

    // Stores the current swerve drive mode (RobotCentric or FieldOriented).
	SwerveDriveMode m_drive_mode;
};