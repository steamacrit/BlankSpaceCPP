/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include "RobotMap.h"
#include <frc/commands/Scheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/livewindow/LiveWindow.h>

ExampleSubsystem Robot::m_subsystem;
SwerveDriveSubsystem Robot::m_drive_subsystem;

void Robot::RobotInit() 
{
	m_chooser.SetDefaultOption("Default Auto", &m_defaultAuto);
	m_chooser.AddOption("My Auto", &m_myAuto);
	frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
	
	m_drive_ctrl.reset(new T34_XboxController(RobotMap::DRIVER_CTRL_PORT));
	m_ahrs.reset(new AHRS(SPI::Port::kMXP));
	//frc::LiveWindow::GetInstance()->AddSensor("IMU", "Gyro", m_ahrs);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() { frc::Scheduler::GetInstance()->Run(); }

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString code to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional commands to the
 * chooser code above (like the commented example) or additional comparisons to
 * the if-else structure below with additional strings & commands.
 */
void Robot::AutonomousInit() {
  // std::string autoSelected = frc::SmartDashboard::GetString(
  //     "Auto Selector", "Default");
  // if (autoSelected == "My Auto") {
  //   m_autonomousCommand = &m_myAuto;
  // } else {
  //   m_autonomousCommand = &m_defaultAuto;
  // }

  m_autonomousCommand = m_chooser.GetSelected();

  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Start();
  }
}

void Robot::AutonomousPeriodic() { frc::Scheduler::GetInstance()->Run(); }

void Robot::TeleopInit() {
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
  }
	
	
}

void Robot::TeleopPeriodic() 
{ 
	frc::Scheduler::GetInstance()->Run(); 
	
    m_drive_subsystem.ShowEncoders();
    
    if (m_drive_ctrl->GetLeftBumperPressed())
        m_drive_subsystem.CalibrateEncoders();
	if (m_drive_ctrl->GetBackButtonPressed())
	{
		m_ahrs->ZeroYaw();
	}
	if (m_drive_ctrl->GetStartButtonPressed())
	{
		if (m_drive_subsystem.GetSwerveDriveMode() == SwerveDriveMode::RobotCentric)
		{
			m_drive_subsystem.SetSwerveDriveMode(SwerveDriveMode::FieldCentric);
			frc::SmartDashboard::PutString("Swerve Mode", "Field Centric");
		}
		else
		{
			m_drive_subsystem.SetSwerveDriveMode(SwerveDriveMode::RobotCentric);
			frc::SmartDashboard::PutString("Swerve Mode", "Robot Centric");
		}
	}
	if (m_drive_ctrl->GetAButtonPressed())
	{
		m_drive_mode = DriveMode::Tank;
		frc::SmartDashboard::PutString("Drive Mode", "Tank Drive");
	}
	if (m_drive_ctrl->GetBButtonPressed())
	{
		m_drive_mode = DriveMode::Swerve;
		frc::SmartDashboard::PutString("Drive Mode", "Swerve Drive");
	}
	if (m_drive_ctrl->GetYButtonPressed())
	{
		m_drive_mode = DriveMode::Car;
		frc::SmartDashboard::PutString("Drive Mode", "Car Drive");
	}
	if (m_drive_ctrl->GetXButtonPressed())
	{
		m_drive_subsystem.ShieldWall();
		frc::SmartDashboard::PutString("Drive Mode", "Shield Wall");
		return;
	}
	
    double lx = m_drive_ctrl->GetLeftStickXDB();
    double ly = m_drive_ctrl->GetLeftStickYDB();
    double ry = m_drive_ctrl->GetRightStickYDB();
    double tr = m_drive_ctrl->GetTriggersCoercedDB();
	switch (m_drive_mode)
	{
		case DriveMode::Tank:
			m_drive_subsystem.TankDrive(ly, ry);
			break;
		case DriveMode::Car:
			m_drive_subsystem.CarDrive(lx, ly);
			break;
		case DriveMode::Swerve:
        {
            double yaw = m_ahrs->GetAngle();
            frc::SmartDashboard::PutNumber("CPP Yaw", yaw);
			m_drive_subsystem.SwerveDrive(lx, ly, tr, yaw);
        }
			break;
		default:
			return;
	}
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
