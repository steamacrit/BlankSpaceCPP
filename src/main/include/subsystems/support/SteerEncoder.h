#pragma once

#include <frc/AnalogInput.h>


class SteerEncoder : public frc::AnalogInput
{
public:
    SteerEncoder(uint32_t encoder_id);

    double PIDGet() override { return GetValue(); }
};