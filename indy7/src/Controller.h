/*****************************************************************************
*	Name: Controller.h
*	Author: Raimarius Delgado
*	Affiliation: RAIMLAB @ Myongji University
*	Description: Base controller class for robot control systems
*	Copyright: RAIMLAB (2025)
*****************************************************************************/

#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include <vector>
#include <string>
#include "base/Common.h"

// Controller types enumeration
enum eControllerType {
    eControllerPID = 0,
    eControllerAdaptive,
    eControllerRobust,
    eControllerMPC,
    eControllerNeural,
    eControllerFuzzy,
    eControllerSliding,
    eControllerBackstepping,
    eControllerImpedance,
    eControllerAdmittance,
    eControllerHybrid,
    eControllerGravityComp,
    eControllerFullDynamics,
    eControllerComputedTorque,
    eControllerMax
};

// Controller status enumeration
enum eControllerStatus {
    eControllerStatusIdle = 0,
    eControllerStatusRunning,
    eControllerStatusError,
    eControllerStatusStopped,
    eControllerStatusInitializing,
    eControllerStatusMax
};

class CController
{
public:
    CController(eControllerType aeType, unsigned int auDOF);
    virtual ~CController();

    // Pure virtual methods that must be implemented by derived classes
    virtual BOOL Init() = 0;
    virtual BOOL Update(const std::vector<double>& avCurrentPos, 
                       const std::vector<double>& avCurrentVel,
                       const std::vector<double>& avCurrentTor,
                       std::vector<double>& avOutputTorque) = 0;
    virtual BOOL Reset() = 0;
    virtual void SetGains(const std::vector<double>& avGains) = 0;

    // Common controller methods
    void Enable(BOOL abEnable) { m_bEnabled = abEnable; }
    BOOL IsEnabled() const { return m_bEnabled; }
    
    void SetName(const TSTRING& astrName) { m_strName = astrName; }
    TSTRING GetName() const { return m_strName; }
    
    eControllerType GetType() const { return m_eType; }
    unsigned int GetDOF() const { return m_uDOF; }
    
    eControllerStatus GetStatus() const { return m_eStatus; }
    void SetStatus(eControllerStatus aeStatus) { m_eStatus = aeStatus; }
    
    BOOL IsInitialized() const { return m_bInitialized; }
    
    // Gain access methods
    std::vector<double> GetGains() const { return m_vGains; }
    
    // Sample time methods
    void SetSampleTime(double adSampleTime) { m_dSampleTime = adSampleTime; }
    double GetSampleTime() const { return m_dSampleTime; }
    
    // Limits methods
    void SetOutputLimits(const std::vector<double>& avLowerLimits, 
                        const std::vector<double>& avUpperLimits);
    void GetOutputLimits(std::vector<double>& avLowerLimits, 
                        std::vector<double>& avUpperLimits) const;
    
    // Saturation and clamping
    void SaturateOutput(std::vector<double>& avOutput) const;
    
    // Error handling
    TSTRING GetLastError() const { return m_strLastError; }
    void ClearError() { m_strLastError.clear(); }
    
    // Statistics
    struct ControllerStats {
        unsigned long ulUpdateCount;
        unsigned long ulErrorCount;
        double dLastUpdateTime;
        double dAverageUpdateTime;
        double dMaxUpdateTime;
        double dMinUpdateTime;
        
        void Reset() {
            ulUpdateCount = 0;
            ulErrorCount = 0;
            dLastUpdateTime = 0.0;
            dAverageUpdateTime = 0.0;
            dMaxUpdateTime = 0.0;
            dMinUpdateTime = 999999.0;
        }
    };
    
    ControllerStats GetStats() const { return m_stats; }
    void ResetStats() { m_stats.Reset(); }
    
    // Debug and logging support
    void SetDebugMode(BOOL abDebug) { m_bDebugMode = abDebug; }
    BOOL IsDebugMode() const { return m_bDebugMode; }

protected:
    // Protected members accessible by derived classes
    eControllerType m_eType;
    unsigned int m_uDOF;
    BOOL m_bEnabled;
    BOOL m_bInitialized;
    eControllerStatus m_eStatus;
    TSTRING m_strName;
    TSTRING m_strLastError;
    
    // Control parameters
    std::vector<double> m_vGains;
    double m_dSampleTime;
    
    // Output limits
    std::vector<double> m_vOutputLowerLimits;
    std::vector<double> m_vOutputUpperLimits;
    BOOL m_bOutputLimitsSet;
    
    // Statistics
    ControllerStats m_stats;
    
    // Debug mode
    BOOL m_bDebugMode;
    
    // Utility methods for derived classes
    void SetError(const TSTRING& astrError);
    void UpdateStats(double adUpdateTime);
    BOOL ValidateInputs(const std::vector<double>& avCurrentPos, 
                       const std::vector<double>& avCurrentVel,
                       const std::vector<double>& avCurrentTor,
                       const std::vector<double>& avOutputTorque) const;
    
    // Helper methods
    double Clamp(double adValue, double adMin, double adMax) const;
    void LogDebug(const char* apcFormat, ...) const;
};

#endif // __CONTROLLER_H__