/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/AnalogInput.h>

class SteerEncoderPIDSource : public frc::AnalogInput
{
public:
    SteerEncoderPIDSource(uint32_t steer_encoder_id);

    double PIDGet() override;
    double GetScaled();
    
    inline void SetTargetAngle(double angle) { m_target_angle = angle; }

    inline bool IsInverted() { return m_inverted; }

    void SetCalFactor(double factor);
    inline const double GetCalFactor() const { return m_cal_factor; }

private:
    bool m_inverted;
    double m_target_angle;
    bool m_use_cal_factor;
    double m_cal_factor;
};
