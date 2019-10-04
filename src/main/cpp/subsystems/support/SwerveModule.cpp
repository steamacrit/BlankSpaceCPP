#include "subsystems/support/SwerveModule.h"

SwerveModule::SwerveModule(SwerveModulePosition position, 
    uint32_t drive_motor_id, uint32_t steer_motor_id, uint32_t steer_encoder_id)
{
    m_invert = false;
    m_position = position;

	m_drive_motor.reset(new TalonSRX(drive_motor_id));
	m_steer_motor.reset(new TalonSRX_PID(steer_motor_id));
	m_steer_encoder.reset(new frc::AnalogInput(steer_encoder_id));
	m_steer_pid.reset(new frc::PIDController(0.03, 0.0, 0.0, m_steer_encoder.get(), m_steer_motor.get()));	
	
	m_steer_pid->SetOutputRange(-1.0, 1.0);
	m_steer_pid->SetContinuous(true);
	m_steer_pid->Enable();
}

void SwerveModule::Drive(double speed, double angle)
{
	Steer(angle);
	SetMotorPercentOutput(speed);
}

void SwerveModule::Steer(double angle)
{
	double current = m_steer_encoder->GetValue() / 4096.0;
	angle /= 360.0;

	double rot = 0.0;
	double delta = modf(angle - current, &rot);
	if (fabs(delta) > 0.25) 
    {
		delta -= copysign(0.5, delta);
		m_invert = true;
	} 
    else 
        m_invert = false;

	m_steer_pid->SetSetpoint((current + delta) * 4096.0);
}

void SwerveModule::SetMotorPercentOutput(double output)
{
	m_drive_motor->Set(ControlMode::PercentOutput, m_invert ? -output : output);
}