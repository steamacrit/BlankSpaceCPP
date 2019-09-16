#pragma once

#include <frc/AnalogInput.h>

class T34_Encoder : public frc::AnalogInput
{
public:
	T34_Encoder(int32_t channel);

    double GetScaled();

    double PIDGet() override { return m_pv; }
    inline void SetPV(const double & pv) { m_pv = pv; }

private:
    double m_pv;
};