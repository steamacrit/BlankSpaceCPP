/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/XboxController.h>

// Enumeration for specifying the target axis in a generic function
enum class AxisType
{
    Trigger,
    XAxis,
    YAxis,
    ZAxis
};

// Subclass of the frc::XboxController adding extra functionality
class T34_XboxController : public frc::XboxController
{
public: // PUBLIC METHODS
    T34_XboxController(int32_t port);

	void SetAllAxisDeadband(double value);
    void SetAxisDeadband(frc::GenericHID::JoystickHand hand, AxisType axis, double value);

    double GetXDB(frc::GenericHID::JoystickHand hand) const;
    double GetYDB(frc::GenericHID::JoystickHand hand) const;
    double GetTriggerDB(frc::GenericHID::JoystickHand hand) const;
	double GetTriggersCoercedDB() const;
	
	inline double GetLeftStickXDB() const   { return GetXDB(frc::GenericHID::JoystickHand::kLeftHand); }
	inline double GetLeftStickYDB() const   { return GetYDB(frc::GenericHID::JoystickHand::kLeftHand); }
	inline double GetRightStickXDB() const  { return GetXDB(frc::GenericHID::JoystickHand::kRightHand); }
	inline double GetRightStickYDB() const  { return GetYDB(frc::GenericHID::JoystickHand::kRightHand); }
	inline double GetLeftTriggerDB() const  { return GetTriggerDB(frc::GenericHID::JoystickHand::kLeftHand); }
	inline double GetRightTriggerDB() const { return GetTriggerDB(frc::GenericHID::JoystickHand::kRightHand); }
	
	inline bool GetLeftStickPressed()       { return GetStickButtonPressed(frc::GenericHID::JoystickHand::kLeftHand); }
	inline bool GetLeftStickReleased()      { return GetStickButtonReleased(frc::GenericHID::JoystickHand::kLeftHand); }
	inline bool GetRightStickPressed()      { return GetStickButtonPressed(frc::GenericHID::JoystickHand::kRightHand); }
	inline bool GetRightStickReleased()     { return GetStickButtonReleased(frc::GenericHID::JoystickHand::kRightHand); }
	inline bool GetLeftBumperPressed()      { return GetBumperPressed(frc::GenericHID::JoystickHand::kLeftHand); }
	inline bool GetLeftBumperReleased()     { return GetBumperReleased(frc::GenericHID::JoystickHand::kLeftHand); }
	inline bool GetRightBumperPressed()     { return GetBumperPressed(frc::GenericHID::JoystickHand::kRightHand); }
	inline bool GetRightBumperReleased()    { return GetBumperReleased(frc::GenericHID::JoystickHand::kRightHand); }
	
private: //PRIVATE DATA
    double m_left_x_db;
    double m_left_y_db;
    double m_right_x_db;
    double m_right_y_db;
    double m_left_trigger_db;
    double m_right_trigger_db;
};
