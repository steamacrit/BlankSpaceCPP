#include <frc/Preferences.h>
#include "subsystems/DriveSubsystem.h"
#include "RobotMap.h"
#include <cmath>

#include <frc/smartdashboard/SmartDashboard.h>

constexpr double PI{ 3.14159265359 };


/* **************************************************************************************************
 *                                                                                                   *   
 * DriveModule class methods                                                                         *       
 *                                                                                                   *       
  ****************************************************************************************************/ 

DriveModule::DriveModule(uint32_t drive_motor_id, uint32_t steer_motor_id, uint32_t steer_encoder_id)
{
	m_drive_motor.reset(new TalonSRX(drive_motor_id));
	m_steer_motor.reset(new TalonSRX_PID(steer_motor_id));
	m_steer_encoder.reset(new frc::AnalogInput(steer_encoder_id));
	m_pid.reset(new frc::PIDController(0.03, 0.0, 0.0, m_steer_encoder.get(), m_steer_motor.get()));	
	
	m_pid->SetOutputRange(-1.0, 1.0);
	m_pid->SetContinuous(true);
	m_pid->Enable();
}

void DriveModule::Drive(double speed, double angle)
{
	m_pid->SetSetpoint(angle);
	m_drive_motor->Set(ControlMode::PercentOutput, speed);
}

void DriveModule::Steer(double angle)
{
	m_pid->SetSetpoint(angle);
}

void DriveModule::SetMotorPercentOutput(double output)
{
	m_drive_motor->Set(ControlMode::PercentOutput, output);
}



/* **************************************************************************************************
 *                                                                                                   *   
 * SwerveDriveSubsystem class methods                                                                *       
 *                                                                                                   *       
  ****************************************************************************************************/ 

DriveSubsystem::DriveSubsystem()
	: frc::Subsystem("SwerveDriveSubsystem")
	, m_drive_mode(SwerveDriveMode::FieldOriented)
	, m_speed_scale(1.0)
{
    m_left_fwd_module.reset(new DriveModule(
        RobotMap::LEFT_FWD_DRIVE_MOTOR, RobotMap::LEFT_FWD_STEER_MOTOR, RobotMap::LEFT_FWD_STEER_ENCODER));
    m_left_aft_module.reset(new DriveModule(
        RobotMap::LEFT_AFT_DRIVE_MOTOR, RobotMap::LEFT_AFT_STEER_MOTOR, RobotMap::LEFT_AFT_STEER_ENCODER));
    m_right_fwd_module.reset(new DriveModule(
        RobotMap::RIGHT_FWD_DRIVE_MOTOR, RobotMap::RIGHT_FWD_STEER_MOTOR, RobotMap::RIGHT_FWD_STEER_ENCODER));
    m_right_aft_module.reset(new DriveModule(
        RobotMap::RIGHT_AFT_DRIVE_MOTOR, RobotMap::RIGHT_AFT_STEER_MOTOR, RobotMap::RIGHT_AFT_STEER_ENCODER));
	
	m_ahrs.reset(new AHRS(SPI::Port::kMXP));
}

void DriveSubsystem::InitDefaultCommand()
{
	
}

// Store a value used with each drive method for scaling the drive motor output
void DriveSubsystem::SetSpeedScale(double scale)
{
	if (scale >= 0.0 && scale <= 1.0)
		m_speed_scale = scale;
}

// Use to point all four swerve wheels outward at 45 degree angle making it 
// harder for other robots to push us around.
void  DriveSubsystem::ShieldWall()
{
    // For sanity, and to prevent damage, turn off all drive motors. 
    // Wheels will be oriented such that no combination of drive motor settings is 
    // valid to move the robot. The robot should not, cannot be moved with wheels oriented
    // as below (unless a bigger, badder robot does actually push us).
    StopDriveMotors();

    m_left_fwd_module->Steer(-0.25);
    m_left_aft_module->Steer( 0.25);
    m_right_fwd_module->Steer( 0.25);
    m_right_aft_module->Steer(-0.25);
}

// Set all drive motor outputs to 0.0
void DriveSubsystem::StopDriveMotors()
{
    m_left_fwd_module->SetMotorPercentOutput(0.0);
    m_left_aft_module->SetMotorPercentOutput(0.0);
    m_right_fwd_module->SetMotorPercentOutput(0.0);
    m_right_aft_module->SetMotorPercentOutput(0.0);    
}

// Just for fun...no real application.
// Lock all wheel angles to 0 degrees and drive using traditional tank drive mode.
void DriveSubsystem::TankDrive(double left, double right)
{
    m_left_fwd_module->Steer(0.0);
    m_left_aft_module->Steer( 0.0);
    m_right_fwd_module->Steer( 0.0);
    m_right_aft_module->Steer(0.0);
	
    m_left_fwd_module->SetMotorPercentOutput(left * m_speed_scale);
    m_left_aft_module->SetMotorPercentOutput(left * m_speed_scale);
    m_right_fwd_module->SetMotorPercentOutput(right * m_speed_scale);
    m_right_aft_module->SetMotorPercentOutput(right * m_speed_scale);
}

// ...Okay just getting carried away now.
// Steer using only the fron two wheels.
void DriveSubsystem::CarDrive(double speed, double angle)
{
    m_left_fwd_module->Steer(angle);
    m_right_fwd_module->Steer(angle);

    m_right_aft_module->Steer(0.0);
    m_left_aft_module->Steer(0.0);
	
    m_left_fwd_module->SetMotorPercentOutput(speed * m_speed_scale);
    m_left_aft_module->SetMotorPercentOutput(speed * m_speed_scale);
    m_right_fwd_module->SetMotorPercentOutput(speed * m_speed_scale);
    m_right_aft_module->SetMotorPercentOutput(speed * m_speed_scale);
}

// Called to commnad robot to drive in swerve mode. 
void DriveSubsystem::SwerveDrive(double x, double y, double rot)
{
	static DriveValues speeds;
	static DriveValues angles;
	
    // If we are in FieldOriented mode, add the yaw of the navX to the rotation input.
	if (m_drive_mode == SwerveDriveMode::FieldOriented)
		rot += m_ahrs->GetAngle();

    // Determine speeds and angles for all modules.
	SpeedAndAngle(x, y, rot, &speeds, &angles);
	
    // Command each module to drive with the calculated speed and angle.
	m_left_fwd_module->Drive(speeds.lf * m_speed_scale, angles.lf);
    m_left_aft_module->Drive(speeds.la * m_speed_scale, angles.la);
    m_right_fwd_module->Drive(speeds.rf * m_speed_scale, angles.rf);
    m_right_aft_module->Drive(speeds.ra * m_speed_scale, angles.ra);
}

// Called to calculate all the required drive speeds and wheel angles for the given input.
void DriveSubsystem::SpeedAndAngle(double x, double y, double rot, DriveValues * speeds, DriveValues * angles)
{
	//static double r = std::sqrt(DRIVE_BASE_WIDTH_HALVED + DRIVE_BASE_LENGTH_HALVED);
	static double r = std::sqrt(
			(RobotMap::DRIVE_BASE_WIDTH * RobotMap::DRIVE_BASE_WIDTH) + 
			(RobotMap::DRIVE_BASE_LENGTH * RobotMap::DRIVE_BASE_LENGTH));
	
	double a = x - rot * (RobotMap::DRIVE_BASE_LENGTH / r);
	double b = x + rot * (RobotMap::DRIVE_BASE_LENGTH / r);
	double c = y - rot * (RobotMap::DRIVE_BASE_WIDTH / r);
	double d = y + rot * (RobotMap::DRIVE_BASE_WIDTH / r);
	
	speeds->lf = std::sqrt((b * b) + (c * c));
	speeds->la = std::sqrt((a * a) + (c * c));
	speeds->rf = std::sqrt((b * b) + (d * d));
	speeds->ra = std::sqrt((a * a) + (d * d));
	
	angles->lf = std::atan2(b, c) / PI;
	angles->la = std::atan2(a, c) / PI;
	angles->rf = std::atan2(b, d) / PI;
	angles->ra = std::atan2(a, d) / PI;
}

// Display steering encoder values on the dashboard.
void DriveSubsystem::ShowEncoders()
{
    frc::SmartDashboard::PutNumber("Left Fwd Steer Encoder", m_left_fwd_module->GetSteerEncoderValue());
    frc::SmartDashboard::PutNumber("Right Fwd Steer Encoder", m_right_fwd_module->GetSteerEncoderValue());
    frc::SmartDashboard::PutNumber("Left Aft Steer Encoder", m_left_aft_module->GetSteerEncoderValue());
    frc::SmartDashboard::PutNumber("Right Aft Steer Encoder", m_right_aft_module->GetSteerEncoderValue());
}