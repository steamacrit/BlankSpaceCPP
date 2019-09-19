/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/PIDController.h>
#include <ctre/Phoenix.h>

class TalonSRX_PID : public TalonSRX, public frc::PIDOutput
{
public:
    TalonSRX_PID(int talon_id)
        : TalonSRX(talon_id)
        , BaseMotorController(talon_id)
    {

    }

    void PIDWrite(double output) override { Set(ControlMode::PercentOutput, output); }
};
