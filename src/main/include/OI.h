/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "utils/T34_XboxController.h"

class OI 
{
public:
	static OI * GetInstance();
	
	std::shared_ptr<T34_XboxController> DRIVER_CTRL;
	std::shared_ptr<T34_XboxController> OPERATOR_CTRL;
	
private:
	OI();
};
