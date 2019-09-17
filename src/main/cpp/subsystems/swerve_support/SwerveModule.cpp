/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/Preferences.h>
#include "subsystems/swerve_support/SwerveModule.h"
#include <frc/smartdashboard/SmartDashboard.h>

SwerveModule::SwerveModule(
        SwerveModulePosition position,
        uint32_t drive_controller_id, 
        uint32_t steer_controller_id, 
        uint32_t steer_encoder_id, 
        bool load_calibration)
    : m_module_position(position)
 //   , m_steer_angle(0.0)
{
    frc::Preferences * prefs = frc::Preferences::GetInstance();
    m_steer_pid_p = prefs->GetDouble("STEER_PID_P", 0.03);
    m_steer_pid_i = prefs->GetDouble("STEER_PID_I", 0.0);
    m_steer_pid_d = prefs->GetDouble("STEER_PID_I", 0.0);
    //PIDSubsystem(m_steer_pid_p, m_steer_pid_i, m_steer_pid_d);
    //SetPIDSourceType(PIDSourceType::kDisplacement);
    

	m_drive.reset(new TalonSRX(drive_controller_id));
	
	m_steer_motor.reset(new TalonSRX(steer_controller_id));
	m_steer_encoder.reset(new SteerEncoderPIDSource(steer_encoder_id));
    if (load_calibration)
        LoadCalibration();
		
	m_pid.reset(new frc::PIDController(m_steer_pid_p, m_steer_pid_i, m_steer_pid_d, m_steer_encoder.get(), this));
	m_pid->Enable();
} 

void SwerveModule::SetSteerAngle(double angle)
{
	m_steer_encoder->SetTargetAngle(angle);
    double pid = m_pid->Get();
    frc::SmartDashboard::PutNumber("PID Output", pid);

switch (m_module_position)
{
    case SwerveModulePosition::LeftForward:
        frc::SmartDashboard::PutNumber("Left Fwd Enc", m_steer_encoder->GetScaled());
        break;
    case SwerveModulePosition::RightForward:
        frc::SmartDashboard::PutNumber("Right Fwd Enc", m_steer_encoder->GetScaled());
        break;
    case SwerveModulePosition::LeftAft:
        frc::SmartDashboard::PutNumber("Left Aft Enc", m_steer_encoder->GetScaled());
        break;
    case SwerveModulePosition::RightAft:
        frc::SmartDashboard::PutNumber("Right Aft Enc", m_steer_encoder->GetScaled());
        break;
}
    if (pid < 0.5)
    {
        m_steer_motor->Set(ControlMode::PercentOutput, 0.0);
        return;
    }
	m_steer_motor->Set(ControlMode::PercentOutput, m_pid->Get());	
}

void SwerveModule::PIDWrite(double output)
{
	m_pid_output = output;
}


// With a better swerve design, this function would never be needed. The absolute encoder should 
// be adjustable such that it can be adjusted to 0 when the wheels are oriented forward. 
// This function assumes the wheels are aligned forward, records the encoder values to a file
// for persistance. These values will be loaded at subsystem initialization and used as offsets 
// for the delta between each wheels's 0 and the respective encoder's 0.
// Being able to adjust the encoder to match the wheel's 0 would eliminate the additional calculations,
// speeding up the swerve algorithm.
void SwerveModule::Calibrate()
{
    m_steer_encoder->Calibrate();
    SaveCalibration();
}

void SwerveModule::LoadCalibration()
{
    frc::Preferences * prefs = frc::Preferences::GetInstance();
    switch (m_module_position)
    {
        case SwerveModulePosition::LeftForward:
            m_steer_encoder->SetCalFactor(prefs->GetDouble("LEFT_FWD_STEER_CAL", 0.0));
            break;
        case SwerveModulePosition::RightForward:
            m_steer_encoder->SetCalFactor(prefs->GetDouble("RIGHT_FWD_STEER_CAL", 0.0));
            break;
        case SwerveModulePosition::LeftAft:
            m_steer_encoder->SetCalFactor(prefs->GetDouble("LEFT_AFT_STEER_CAL", 0.0));
            break;
        case SwerveModulePosition::RightAft:
            m_steer_encoder->SetCalFactor(prefs->GetDouble("RIGHT_AFT_STEER_CAL", 0.0));
            break;
    }
}

void SwerveModule::SaveCalibration()
{
    frc::Preferences * prefs = frc::Preferences::GetInstance();
    switch (m_module_position)
    {
        case SwerveModulePosition::LeftForward:
            prefs->PutDouble("LEFT_FWD_STEER_CAL", m_steer_encoder->GetCalFactor());
            break;
        case SwerveModulePosition::RightForward:
            prefs->PutDouble("RIGHT_FWD_STEER_CAL", m_steer_encoder->GetCalFactor());
            break;
        case SwerveModulePosition::LeftAft:
            prefs->PutDouble("LEFT_AFT_STEER_CAL", m_steer_encoder->GetCalFactor());
            break;
        case SwerveModulePosition::RightAft:
            prefs->PutDouble("RIGHT_AFT_STEER_CAL", m_steer_encoder->GetCalFactor());
            break;
    }
}