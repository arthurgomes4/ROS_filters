#include "ema_filter.h"

EMAfilter::EMAfilter(){
    m_smoothingFactor = 0.5;
    m_previousValue = 0;
}
EMAfilter::EMAfilter(float smoothingFactor){
    m_smoothingFactor = smoothingFactor;
    m_previousValue = 0;
}

inline float EMAfilter::getSmoothingFactor(){
    return m_smoothingFactor;}

inline void EMAfilter::setSmoothingFactor(float newValue){
    m_smoothingFactor = newValue;}

inline void EMAfilter::resetFilter(float initialValue = 0){
    m_previousValue = initialValue;}

float EMAfilter::filterValue(float currentValue){
    float filteredValue = currentValue*m_smoothingFactor + m_previousValue*(1-m_smoothingFactor);
    m_previousValue = filteredValue;
    return filteredValue;
}