/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/PIDSubsystem.h>
#include <ctre/Phoenix.h>
#include "subsystems/swerve_support/SteerEncoderPIDSource.h"

enum class SwerveModulePosition
{
    LeftForward,
    RightForward,
    LeftAft,
    RightAft
};

class SwerveModule
{
public:
	SwerveModule(
        SwerveModulePosition position,
        uint32_t drive_controller_id, 
        uint32_t steer_controller_id, 
        uint32_t steer_encoder_id, 
        bool load_calibration = true);
	
    inline SwerveModulePosition GetModulePosition() { return m_module_position; }

	inline TalonSRX * GetDrive() { return m_drive.get(); }
//	inline T34_Encoder * GetDriveEncoder() { return m_drive_encoder.get(); }
	
	inline TalonSRX * GetSteer() { return m_steer.get(); }
	inline SteerEncoderPIDSource * GetSteerEncoder() { return m_steer_encoder.get(); }
    inline void SetSteer(double angle) { m_steer_encoder->SetTargetAngle(angle); }
    inline bool IsInverted() { return m_steer_encoder->IsInverted(); }
	inline double GetSteerEncoderValue() { return m_steer_encoder->GetScaled(); }
    inline double GetSteerPID() { return m_steer_encoder->PIDGet(); }

    inline void SetSteerEncoderCal(const double value) { m_steer_encoder->SetCalFactor(value); }
    inline const double GetSteerEncoderCal() const { return m_steer_encoder->GetCalFactor(); }

    inline void SetDriveSpeed(double speed) { m_drive->Set(ControlMode::PercentOutput, speed); }
	
    void LoadCalibration();
    void SaveCalibration();

private:
    SwerveModulePosition m_module_position;

	std::unique_ptr<TalonSRX> m_drive;
//	std::unique_ptr<T34_Encoder> m_drive_encoder;
	
	std::unique_ptr<TalonSRX> m_steer;
	std::unique_ptr<SteerEncoderPIDSource> m_steer_encoder;

    double m_steer_pid_p;
    double m_steer_pid_i;
    double m_steer_pid_d;
//    double m_steer_angle;
};