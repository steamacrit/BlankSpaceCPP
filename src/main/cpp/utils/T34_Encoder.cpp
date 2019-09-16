#include "utils/T34_Encoder.h"

T34_Encoder::T34_Encoder(int32_t channel)
	: frc::AnalogInput(channel)
    , m_pv(0.0)
{
	
}

double T34_Encoder::GetScaled()
{
    static const double scale = 360 / 4096;

    return ((double)GetValue()) * scale;
}

