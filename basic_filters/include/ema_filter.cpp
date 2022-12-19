#include "ema_filter.h"

EMAfilter::EMAfilter(){
    m_smoothingFactor = 0.5;
    m_previousValue = 0;
}
EMAfilter::EMAfilter(float smoothingFactor){
    m_smoothingFactor = smoothingFactor;
    m_previousValue = 0;
}

float EMAfilter::getSmoothingFactor(){
    return m_smoothingFactor;}

void EMAfilter::setSmoothingFactor(float newValue){
    m_smoothingFactor = newValue;}

void EMAfilter::resetFilter(float initialValue = 0){
    m_previousValue = initialValue;
    INIT = true;}

float EMAfilter::filterValue(float currentValue){
    if(!INIT){
        resetFilter(currentValue);
    }
    float filteredValue = currentValue*m_smoothingFactor + m_previousValue*(1-m_smoothingFactor);
    m_previousValue = filteredValue;
    return filteredValue;
}