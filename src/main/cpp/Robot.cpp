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
DriveSubsystem Robot::m_drive_subsystem;

void Robot::RobotInit() 
{
	m_chooser.SetDefaultOption("Default Auto", &m_defaultAuto);
	m_chooser.AddOption("My Auto", &m_myAuto);
	frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
	
	m_drive_ctrl.reset(new T34_XboxController(RobotMap::DRIVER_CTRL_PORT));

    m_drive_mode = DriveMode::SwerveFieldOriented;
    m_drive_subsystem.SetSwerveDriveMode(SwerveDriveMode::FieldOriented);
    frc::SmartDashboard::PutString("Drive Mode", "FieldOriented");
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
	
    // While left bumper is pressed the drive speed will be scaled by 1.7.
    // While the right bumper is pressed the drive speed will be scailed by 1.3.
    // When the pressed bumper is released, the drive speed will no longer be scaled (1.0).
    if (m_drive_ctrl->GetLeftBumperPressed())
        m_drive_subsystem.SetSpeedScale(1.7);
    else if (m_drive_ctrl->GetRightBumperPressed())
        m_drive_subsystem.SetSpeedScale(1.3);
    else
        m_drive_subsystem.SetSpeedScale(1.0);

    // Pressing the "back" button cycles through the four drive modes
    //  1. SwerveFieldOriented (default)
    //  2. SwerveRobotCentric
    //  3. Tank
    //  4. Car
    if (m_drive_ctrl->GetBackButtonReleased())
	{
        switch (m_drive_mode)
        {
            case DriveMode::SwerveFieldOriented:
                m_drive_mode = DriveMode::SwerveRobotCentric;
                m_drive_subsystem.SetSwerveDriveMode(SwerveDriveMode::RobotCentric);
                frc::SmartDashboard::PutString("Drive Mode", "RobotCentric");
                break;
            case DriveMode::SwerveRobotCentric:
                m_drive_mode = DriveMode::Tank;
                frc::SmartDashboard::PutString("Drive Mode", "Tank");
                break;
            case DriveMode::Tank:
                m_drive_mode = DriveMode::Car;
                frc::SmartDashboard::PutString("Drive Mode", "Car");
                break;
            case DriveMode::Car:
                m_drive_mode = DriveMode::SwerveFieldOriented;
                m_drive_subsystem.SetSwerveDriveMode(SwerveDriveMode::FieldOriented);
                frc::SmartDashboard::PutString("Drive Mode", "FieldOriented");
                break;
            default:
                m_drive_mode = DriveMode::SwerveFieldOriented;
                m_drive_subsystem.SetSwerveDriveMode(SwerveDriveMode::FieldOriented);            
                frc::SmartDashboard::PutString("Drive Mode", "FieldOriented");
        }
    }

    // If the X Button is pressed, all drive motors outputs will be set to 0.0 and the 
    // four wheels will be oriented outward at a 45.0 degree angle. This is primary for defense 
    // and should make it harder for other robots to push use off target or out of the way 
    // when we are attempting to block them.
	if (m_drive_ctrl->GetXButtonPressed())
	{
		m_drive_subsystem.ShieldWall();
		frc::SmartDashboard::PutString("Drive Mode", "Shield Wall");
		return;
	}
	
    double right_stick_x = m_drive_ctrl->GetRightStickXDB();
	switch (m_drive_mode)
	{
        case DriveMode::SwerveFieldOriented:
        case DriveMode::SwerveRobotCentric:
            // Note that that in swerve mode, either the triggers (left & right) or the 
            // right stick (x-axis) can be used to cause the robot to rotate.
            // The right stick has priority meaning that if there is an input detected
            // from the right stick then the triggers are ignored and the right stick is used.
            m_drive_subsystem.SwerveDrive(m_drive_ctrl->GetLeftStickXDB(), m_drive_ctrl->GetLeftStickYDB(),
                right_stick_x == 0.0 ? right_stick_x : m_drive_ctrl->GetTriggersCoercedDB());
            break;
		case DriveMode::Tank:
			m_drive_subsystem.TankDrive(m_drive_ctrl->GetLeftStickYDB(), m_drive_ctrl->GetRightStickYDB());
			break;
		case DriveMode::Car:
			m_drive_subsystem.CarDrive(m_drive_ctrl->GetLeftStickYDB(), m_drive_ctrl->GetLeftStickXDB());
			break;
		default:
			m_drive_subsystem.StopDriveMotors();
	}
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
