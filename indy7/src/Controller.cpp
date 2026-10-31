/*****************************************************************************
*	Name: Controller.cpp
*	Author: Raimarius Delgado
*	Affiliation: RAIMLAB @ Myongji University
*	Description: Implementation of base controller class
*	Copyright: RAIMLAB (2025)
*****************************************************************************/

#include "Controller.h"
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstring>

CController::CController(eControllerType aeType, unsigned int auDOF)
    : m_eType(aeType)
    , m_uDOF(auDOF)
    , m_bEnabled(FALSE)
    , m_bInitialized(FALSE)
    , m_eStatus(eControllerStatusIdle)
    , m_strName("Controller")
    , m_dSampleTime(0.001)  // Default 1ms sample time
    , m_bOutputLimitsSet(FALSE)
    , m_bDebugMode(FALSE)
{
    // Initialize gain vector
    m_vGains.resize(auDOF, 1.0);
    
    // Initialize output limits
    m_vOutputLowerLimits.resize(auDOF, -1000.0);  // Default -1000 Nm
    m_vOutputUpperLimits.resize(auDOF, 1000.0);   // Default +1000 Nm
    
    // Initialize statistics
    m_stats.Reset();
    
    DBG_LOG_INFO("(%s) Base controller created: Type=%d, DOF=%u", 
                 "CController", static_cast<int>(aeType), auDOF);
}

CController::~CController()
{
    DBG_LOG_INFO("(%s) Base controller destroyed: %s", "CController", m_strName.c_str());
}

void CController::SetOutputLimits(const std::vector<double>& avLowerLimits, 
                                 const std::vector<double>& avUpperLimits)
{
    if (avLowerLimits.size() != m_uDOF || avUpperLimits.size() != m_uDOF) {
        SetError("Invalid output limits size");
        return;
    }
    
    m_vOutputLowerLimits = avLowerLimits;
    m_vOutputUpperLimits = avUpperLimits;
    m_bOutputLimitsSet = TRUE;
    
    DBG_LOG_INFO("(%s) Output limits set for %u DOF", "CController", m_uDOF);
}

void CController::GetOutputLimits(std::vector<double>& avLowerLimits, 
                                 std::vector<double>& avUpperLimits) const
{
    avLowerLimits = m_vOutputLowerLimits;
    avUpperLimits = m_vOutputUpperLimits;
}

void CController::SaturateOutput(std::vector<double>& avOutput) const
{
    if (!m_bOutputLimitsSet || avOutput.size() != m_uDOF) {
        return;
    }
    
    for (unsigned int i = 0; i < m_uDOF; ++i) {
        avOutput[i] = Clamp(avOutput[i], m_vOutputLowerLimits[i], m_vOutputUpperLimits[i]);
    }
}

void CController::SetError(const std::string& astrError)
{
    m_strLastError = astrError;
    m_eStatus = eControllerStatusError;
    m_stats.ulErrorCount++;
    
    DBG_LOG_ERROR("(%s) Controller error: %s", "CController", astrError.c_str());
}

void CController::UpdateStats(double adUpdateTime)
{
    m_stats.ulUpdateCount++;
    m_stats.dLastUpdateTime = adUpdateTime;
    
    // Update average time
    if (m_stats.ulUpdateCount == 1) {
        m_stats.dAverageUpdateTime = adUpdateTime;
        m_stats.dMaxUpdateTime = adUpdateTime;
        m_stats.dMinUpdateTime = adUpdateTime;
    } else {
        // Running average
        m_stats.dAverageUpdateTime = (m_stats.dAverageUpdateTime * (m_stats.ulUpdateCount - 1) + adUpdateTime) / m_stats.ulUpdateCount;
        m_stats.dMaxUpdateTime = std::max(m_stats.dMaxUpdateTime, adUpdateTime);
        m_stats.dMinUpdateTime = std::min(m_stats.dMinUpdateTime, adUpdateTime);
    }
}

BOOL CController::ValidateInputs(const std::vector<double>& avCurrentPos, 
                                const std::vector<double>& avCurrentVel,
                                const std::vector<double>& avCurrentTor,
                                const std::vector<double>& avOutputTorque) const
{
    // Check sizes
    if (avCurrentPos.size() != m_uDOF) {
        return FALSE;
    }
    
    if (avCurrentVel.size() != m_uDOF) {
        return FALSE;
    }
    
    if (avCurrentTor.size() != m_uDOF) {
        return FALSE;
    }
    
    if (avOutputTorque.size() != m_uDOF) {
        return FALSE;
    }
    
    // Check for NaN or infinite values
    for (unsigned int i = 0; i < m_uDOF; ++i) {
        if (std::isnan(avCurrentPos[i]) || std::isinf(avCurrentPos[i])) {
            return FALSE;
        }
        
        if (std::isnan(avCurrentVel[i]) || std::isinf(avCurrentVel[i])) {
            return FALSE;
        }
        
        if (std::isnan(avCurrentTor[i]) || std::isinf(avCurrentTor[i])) {
            return FALSE;
        }
    }
    
    return TRUE;
}

double CController::Clamp(double adValue, double adMin, double adMax) const
{
    if (adValue < adMin) return adMin;
    if (adValue > adMax) return adMax;
    return adValue;
}

void CController::LogDebug(const char* apcFormat, ...) const
{
    if (!m_bDebugMode) return;
    
    va_list args;
    va_start(args, apcFormat);
    
    char buffer[1024];
    vsnprintf(buffer, sizeof(buffer), apcFormat, args);
    
    DBG_LOG_INFO("(%s) %s", m_strName.c_str(), buffer);
    
    va_end(args);
}