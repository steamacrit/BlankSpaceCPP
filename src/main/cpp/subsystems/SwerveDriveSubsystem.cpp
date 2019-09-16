#include <frc/Preferences.h>
#include "subsystems/SwerveDriveSubsystem.h"
#include "RobotMap.h"
#include <cmath>

constexpr double PI{ 3.14159265359 };
constexpr double RAD1{ 180 / PI };

constexpr uint32_t LEFT_FWD_MODULE{ 0 };
constexpr uint32_t LEFT_AFT_MODULE{ 1 };
constexpr uint32_t RIGHT_FWD_MODULE{ 2 };
constexpr uint32_t RIGHT_AFT_MODULE{ 3 };

void sincosf(const double angle, double * sin_angle, double * cos_angle)
{
    *sin_angle = sinf(angle);
    *cos_angle = cosf(angle);
}

/**
 * SWERVE MODULE
 **/

 
 
/**
 * SWERVE DRIVE
 **/
std::unique_ptr<SwerveDriveSubsystem> g_swerve_drive{ nullptr };

SwerveDriveSubsystem * SwerveDriveSubsystem::GetInstance()
{
	if (g_swerve_drive == nullptr)
		g_swerve_drive.reset(new SwerveDriveSubsystem());
		
	return g_swerve_drive.get();
}

SwerveDriveSubsystem::SwerveDriveSubsystem()
	: frc::Subsystem("SwerveDriveSubsystem")
	, m_drive_mode(DriveMode::RobotCentric)
	, m_speed_mode(SpeedMode::Full)
    , m_xrot_pt(0.0)
    , m_yrot_pt(0.0)
{
	m_swerve.push_back(std::make_unique<SwerveModule>(SwerveModulePosition::LeftForward,
        RobotMap::LEFT_FWD_DRIVE_CTRL, RobotMap::LEFT_FWD_STEER_CTRL, RobotMap::LEFT_FWD_STEER_ENCODER));

	m_swerve.push_back(std::make_unique<SwerveModule>(SwerveModulePosition::LeftAft,
        RobotMap::LEFT_AFT_DRIVE_CTRL, RobotMap::LEFT_AFT_STEER_CTRL, RobotMap::LEFT_AFT_STEER_ENCODER));

	m_swerve.push_back(std::make_unique<SwerveModule>(SwerveModulePosition::RightForward,
        RobotMap::RIGHT_FWD_DRIVE_CTRL, RobotMap::RIGHT_FWD_STEER_CTRL, RobotMap::RIGHT_FWD_STEER_ENCODER));

	m_swerve.push_back(std::make_unique<SwerveModule>(SwerveModulePosition::RightAft,
        RobotMap::RIGHT_AFT_DRIVE_CTRL, RobotMap::RIGHT_AFT_STEER_CTRL, RobotMap::RIGHT_AFT_STEER_ENCODER));

    SetRotationPoint(0.0, 0.0);
}

void SwerveDriveSubsystem::InitDefaultCommand()
{

}

void SwerveDriveSubsystem::SwerveDrive(double x, double y, double rot, double yaw, double percent_output)
{
    switch (m_drive_mode)
    {
        case DriveMode::RobotCentric:
            RotateAngle(x, y, yaw);
            break;
        default:
            RotateAngle(x, y, 0.0);
    }

    Steer(x, y, rot);

}

double SimpleStaticPID(double angle)
{
    double ouptut = angle * 0.3;

}

void  SwerveDriveSubsystem::Steer(double x, double y, double rot)
{
    for (size_t i = 0; i < m_swerve.size(); i++)
    {
        double angle{ 0.0 };
        double speed{ 0.0 };
        SwerveModule * p_module = m_swerve[i].get();

        switch (p_module->GetModulePosition())
        {
            case SwerveModulePosition::LeftForward:
                TargetAngleAndSpeed(x, y, rot, DRIVE_BASE_LENGTH_HAVLED, -DRIVE_BASE_WIDTH_HALVED,
                    &angle, &speed);
                break;
            case SwerveModulePosition::RightForward:
                TargetAngleAndSpeed(x, y, rot, -DRIVE_BASE_LENGTH_HAVLED, -DRIVE_BASE_WIDTH_HALVED,
                    &angle, &speed);
                break;     
            case SwerveModulePosition::LeftAft:
                TargetAngleAndSpeed(x, y, rot, DRIVE_BASE_LENGTH_HAVLED, DRIVE_BASE_WIDTH_HALVED,
                    &angle, &speed);
                break;  
            case SwerveModulePosition::RightAft:
                TargetAngleAndSpeed(x, y, rot, -DRIVE_BASE_LENGTH_HAVLED, DRIVE_BASE_WIDTH_HALVED,
                    &angle, &speed);
                break;  
            default:
                return;
        }

        p_module->SetSteer(angle);

        if (std::fabs(x) + std::fabs(y) + std::fabs(rot) == 0)
        {
            speed = 0.0;
            p_module->GetSteer()->Set(ControleMode::PercentOutput, 0.0);
        }
        else
            p_module->GetSteer()->Set(ControleMode::PercentOutput, p_module->GetPIDController()->Get());

        if (p_module->IsInverted())
            speed = -speed;

        if (percent_output < 1.0 && percent_output > 0.0)
                p_module->SetDriveSpeed(speed * percent_output);
        else
            for (size_t i = 0; i < m_swerve.size(); i++)
               p_module->SetDriveSpeed(speed);            
    }
}

void SwerveDriveSubsystem::RotateAngle(double & x, double & y, double angle)
{
    angle = (angle * PI) / 180.0;
    double sin_angle{ 0.0 }; 
    double cos_angle{ 0.0 };
    sincosf(angle, &sin_angle, &cos_angle);

    x = (x * cos_angle) - (y * sin_angle);
    y = (y * cos_angle) + (x * sin_angle);
}

void SwerveDriveSubsystem::SetRotationPoint(double x, double y)
{
    m_xrot_pt = x;
    m_yrot_pt = y;

    double yot = std::pow(DRIVE_BASE_WIDTH_HALVED + fabs(x), 2.0);
    double xot = std::pow(DRIVE_BASE_LENGTH_HAVLED + std::fabs(y), 2.0);
    m_got = std::sqrt(yot + xot);
}

void SwerveDriveSubsystem::TargetAngleAndSpeed(double x, double y, double rot, double width, double length,
        double * angle, double * speed)
{    
    rot /= m_got;
    double n1 = -((length + m_yrot_pt) * rot) + x;
    double n2 = ((width + m_xrot_pt) * rot) + y;
    *angle = (RAD1 * std::atan2(n1, n2)) + 180;
    *speed = std::sqrt(std::pow(n1, 2.0) + std::pow(n2, 2.0));
    if (std::fabs(*speed) > 1.0)
        *speed /= std::fabs(*speed);
}

