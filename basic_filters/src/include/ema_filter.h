#pragma once

class EMAfilter
{
    float m_smoothingFactor;
    float m_previousValue;
    
    public:
    EMAfilter();
    EMAfilter(float smoothingFactor);

    float getSmoothingFactor();
    void setSmoothingFactor(float newValue);
    void resetFilter(float initialValue);

    float filterValue(float currentValue);
};