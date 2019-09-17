/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/swerve_support/SteerEncoderPIDSource.h"
#include <frc/smartdashboard/SmartDashboard.h>

constexpr double SCALE_FACTOR = 360.0 / 4096.0;

SteerEncoderPIDSource::SteerEncoderPIDSource(uint32_t steer_encoder_id) 
    : frc::AnalogInput(steer_encoder_id)
    , m_use_cal_factor(false)
    , m_cal_factor(0.0)
    , m_inverted(false)
    , m_target_angle(0.0)
{
    
}

double SteerEncoderPIDSource::PIDGet()
{
    double target_al = ((double)GetValue()) * SCALE_FACTOR;

    if (m_use_cal_factor)
    {
        target_al += m_cal_factor;
    }

    // ANGLE LOOP
    if (target_al > 180.0)
        target_al -= 360.0;
    else if (target_al < -180.0)
        target_al += 360.0;

    // ANGLE DISTANCE
    double dist_180 = std::fmod(((target_al + 180) - m_target_angle) + 180.0, 360.0) - 180.0;
    double dist = std::fmod((target_al - m_target_angle) + 180.0, 360.0) - 180.0;
    double pv = dist;

    if (std::fabs(dist) > 90.0)
    {
        pv = dist_180;    
        m_inverted = false;
    }
    else
        m_inverted = true;

    frc::SmartDashboard::PutNumber("PV", pv);
    return pv;
}

double SteerEncoderPIDSource::GetScaled()
{
    double output = ((double)GetValue()) * SCALE_FACTOR;

    if (m_use_cal_factor)
    {
        output += m_cal_factor;
    }  

    return output;  
}

void SteerEncoderPIDSource::SetCalFactor(double factor)
{
    if (factor == 0.0)
        m_use_cal_factor = false;
    else    
        m_use_cal_factor = true;

    m_cal_factor = factor;
}

void SteerEncoderPIDSource::Calibrate()
{
    SetCalFactor(GetValue() * SCALE_FACTOR);
}