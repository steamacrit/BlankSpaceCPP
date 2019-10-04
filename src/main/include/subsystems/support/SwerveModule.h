#pragma once

#include <frc/PIDController.h>
#include <frc/AnalogInput.h>
#include <ctre/Phoenix.h>

#include "subsystems/support/TalonSRX_PID.h"
#include "subsystems/support/SteerEncoder.h"
#include "subsystems/support/DriveValues.h"

#include "subsystems/support/t34_types.h"

// Convenience class for managing each swerve drive module separately
class SwerveModule
{
public:
	SwerveModule(SwerveModulePosition position, 
        uint32_t drive_motor_id, uint32_t steer_motor_id, uint32_t steer_encoder_id);
	
	void Drive(double speed, double angle);
	
	void Steer(double angle);
	void SetMotorPercentOutput(double output);
	
    inline SwerveModulePosition GetPosition() { return m_position; }
	inline TalonSRX * GetDriveMotor() { return m_drive_motor.get(); }
	inline TalonSRX * GetSteerMotor() { return m_steer_motor.get(); }
	inline frc::AnalogInput * GetSteerEncoder() { return m_steer_encoder.get(); }
	inline double GetSteerEncoderValue() { return m_steer_encoder->GetValue(); }

private: // PRIVATE DATA
    SwerveModulePosition m_position;
	std::unique_ptr<TalonSRX> m_drive_motor;
	std::unique_ptr<TalonSRX_PID> m_steer_motor;
	std::unique_ptr<frc::AnalogInput> m_steer_encoder;
	std::unique_ptr<frc::PIDController> m_steer_pid;

    bool m_invert;
};
