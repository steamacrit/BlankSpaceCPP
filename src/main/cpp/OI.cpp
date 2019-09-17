/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "OI.h"

#include <frc/WPILib.h>
#include "RobotMap.h"

std::unique_ptr<OI> g_oi_instance{ nullptr };

OI * OI::GetInstance()
{
	if (g_oi_instance == nullptr)
		g_oi_instance.reset(new OI());
		
	return g_oi_instance.get();
}

OI::OI() 
{
	DRIVER_CTRL.reset(new T34_XboxController(RobotMap::DRIVER_CTRL_PORT));
	DRIVER_CTRL->SetAllAxisDeadband(0.06);
	
	OPERATOR_CTRL.reset(new T34_XboxController(RobotMap::OPERATOR_CTRL_PORT));
	OPERATOR_CTRL->SetAllAxisDeadband(0.06);
}
