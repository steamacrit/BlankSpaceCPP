#include <frc/Preferences.h>
#include "subsystems/DriveSubsystem.h"
#include "RobotMap.h"
#include <cmath>

#include <frc/smartdashboard/SmartDashboard.h>



constexpr double DRIVE_BASE_WIDTH { 25.0 };
constexpr double DRIVE_BASE_WIDTH_HALVED { DRIVE_BASE_WIDTH / 2.0 };

constexpr double DRIVE_BASE_LENGTH { 25.0 };
constexpr double DRIVE_BASE_LENGTH_HAVLED { DRIVE_BASE_LENGTH / 2.0 };

constexpr double TO_RAD { 180.0 / M_PI };
constexpr double TO_DEG { M_PI / 180.0 };


DriveSubsystem::DriveSubsystem()
	: frc::Subsystem("SwerveDriveSubsystem")
	, m_drive_mode(SwerveDriveMode::FieldOriented)
	, m_speed_scale(1.0)
{
    InitializeSwerveModule(SwerveModulePosition::LeftForward,
        RobotMap::LEFT_FWD_DRIVE_MOTOR, RobotMap::LEFT_FWD_STEER_MOTOR, RobotMap::LEFT_FWD_STEER_ENCODER);
    InitializeSwerveModule(SwerveModulePosition::LeftAft,
        RobotMap::LEFT_AFT_DRIVE_MOTOR, RobotMap::LEFT_AFT_STEER_MOTOR, RobotMap::LEFT_AFT_STEER_ENCODER);
    InitializeSwerveModule(SwerveModulePosition::RightForward,
        RobotMap::RIGHT_FWD_DRIVE_MOTOR, RobotMap::RIGHT_FWD_STEER_MOTOR, RobotMap::RIGHT_FWD_STEER_ENCODER);
    InitializeSwerveModule(SwerveModulePosition::RightAft,
        RobotMap::RIGHT_AFT_DRIVE_MOTOR, RobotMap::RIGHT_AFT_STEER_MOTOR, RobotMap::RIGHT_AFT_STEER_ENCODER);
	
	m_ahrs.reset(new AHRS(SPI::Port::kMXP));
}

void DriveSubsystem::InitializeSwerveModule(SwerveModulePosition position, 
        uint32_t drive_motor_id, uint32_t steer_motor_id, uint32_t steer_encoder_id)
{
    size_t index = m_swerve_modules.size();
    m_swerve_modules.push_back(std::make_unique<SwerveModule>(
        position, drive_motor_id, steer_motor_id, steer_encoder_id));

    SwerveModule * p_module = m_swerve_modules[index].get();

    switch (position)
    {
        case SwerveModulePosition::LeftForward:     m_left_fwd_module = p_module;   break;
        case SwerveModulePosition::LeftAft:         m_left_aft_module = p_module;   break;
        case SwerveModulePosition::RightForward:    m_right_fwd_module = p_module;  break;
        case SwerveModulePosition::RightAft:        m_right_aft_module = p_module;  break;
    }
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

    m_left_fwd_module->Steer(-45.0);
    m_left_aft_module->Steer( 45.0);
    m_right_fwd_module->Steer( 45.0);
    m_right_aft_module->Steer(-45.0);
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
    float steer = x;
    float drive = y;

	if (m_drive_mode == SwerveDriveMode::FieldOriented) 
    {
		const double yaw = m_ahrs->GetAngle() * TO_DEG;
		drive =  y * cos(yaw) + x * sin(yaw);
		steer = -y * sin(yaw) + x * cos(yaw);
	}

	const double r = sqrt(pow(DRIVE_BASE_LENGTH, 2) + pow(DRIVE_BASE_WIDTH, 2));
	double a = steer - rot * DRIVE_BASE_LENGTH / r;
	double b = steer + rot * DRIVE_BASE_LENGTH / r;
	double c = drive - rot * DRIVE_BASE_WIDTH  / r;
	double d = drive + rot * DRIVE_BASE_WIDTH  / r;

	if (d != 0 || b != 0) 
		m_left_fwd_module->Steer(atan2(b, d) * TO_RAD);

	if (a != 0 || d != 0) 
		m_left_aft_module->Steer(atan2(a, d) * TO_RAD);

	if (b != 0 || c != 0) 
		m_right_fwd_module->Steer(atan2(b, c) * TO_RAD);

	if (a != 0 || c != 0) 
		m_right_aft_module->Steer(atan2(a, c) * TO_RAD);



	DriveValues<double> output(0.0);
	output.lf = sqrt(pow(b, 2) + pow(d, 2));
	output.la = sqrt(pow(a, 2) + pow(d, 2));
	output.rf = sqrt(pow(b, 2) + pow(c, 2));
	output.ra = sqrt(pow(a, 2) + pow(c, 2));

    NormalizeOutput(output);

	m_left_fwd_module->SetMotorPercentOutput(output.lf  * m_speed_scale);
	m_left_aft_module->SetMotorPercentOutput(output.la  * m_speed_scale);
	m_right_fwd_module->SetMotorPercentOutput(output.rf * m_speed_scale);
	m_right_aft_module->SetMotorPercentOutput(output.ra * m_speed_scale);
}

void DriveSubsystem::NormalizeOutput(DriveValues<double> & output)
{
	double max_output = 0.0;
    double fabs_output;
    for (auto d : output.values)
    {
        fabs_output = fabs(d);
        if (fabs_output > max_output)
            max_output = fabs_output;
    }

	if (max_output > 1 || max_output < -1) 
    {
		output.lf /= max_output;
		output.la /= max_output;
		output.rf /= max_output;
		output.ra /= max_output;
	} 

	output.rf = -output.rf;
	output.ra = -output.ra;
}

// Display steering encoder values on the dashboard.
void DriveSubsystem::ShowEncoders()
{
    frc::SmartDashboard::PutNumber("Left Fwd Steer Encoder", m_left_fwd_module->GetSteerEncoderValue());
    frc::SmartDashboard::PutNumber("Right Fwd Steer Encoder", m_right_fwd_module->GetSteerEncoderValue());
    frc::SmartDashboard::PutNumber("Left Aft Steer Encoder", m_left_aft_module->GetSteerEncoderValue());
    frc::SmartDashboard::PutNumber("Right Aft Steer Encoder", m_right_aft_module->GetSteerEncoderValue());
}