/*****************************************************************************
*	Name: Config.cpp
*	Description: The configuration file of the controller. See Config.h.
*****************************************************************************/
#include "Config.h"

#include <cctype>
#include <cerrno>
#include <climits>
#include <cstdlib>
#include <fstream>
#include <strings.h>
#include <sys/stat.h>

stConfigSystem::stConfigSystem()
{
	strName = "ROBOT";
	bSim = FALSE;
	strURDFPath = "";
	bEnableControllerAtStartup = FALSE;
	nRobotAxis = 0;
	nFfGate = 1;
	strSimMJCFPath = "";
	nSimStateDelay = 2;
	nSimTorqueDelay = 2;
	bSimFriction = FALSE;
	nSimFaultJoint = 0;
	nSimFaultWhat = 0;
	nSimFaultPersist = 0;
	nSimEstopReports = 0;
	nViewPort = 0;
	dViewRateHz = 50.0;
	vJtFastDq = {0.3, 0.3, 0.3, 0.0, 0.0, 0.0};
	vJtSlowDq = {0.9, 0.6, 0.9, 0.9, 0.9, 0.0};
	dJtFastT = 1.0;
	dJtSlowT = 3.0;
	dJtDwell = 1.0;
	strJtLogDir = "jt_logs";
	nTsJoint = 4;   dTsTau = 1.0;   dTsOn = 0.2;  dTsOff = 0.3;  nTsRep = 20;
	nBaJoint = 4;   dBaRate = 2.0;  dBaMax = 15.0; dBaQd = 0.02; nBaRep = 5;
	nSineJoint = 4; dSineAmp = 0.006; dSineFreq = 4.0; nSinePeriods = 32; nSineFf = 1; dSineV0 = 0.2;
	dHlDelta = 0.006;
	dHlLambda = 0.01;
	dHlStepMax = 0.2;
	dHlTol = 1e-4;
	nHlIterCycle = 10;
	nHlIterGoal = 100;
	dHlVelMax = 0.6;
	dHlAccMax = 2.0;
	dHlJerkMax = 10.0;
	dHlMargin = 0.26;
	dHlLead = 0.10;
	vHlReadyQ = {0.0, 0.0, -1.5707963267948966, 0.0, -1.5707963267948966, 0.0};
	vHlTestGoal = {0.2121320343559643, 0.0, -0.2121320343559643};
	vHlTestJump = {0.0, 0.15, 0.0};
	dHlTestTrigger = 0.15;
	dHlTestNoise = 0.001;
	dHlTestNoiseHz = 15.0;
	dHlTestNoiseT = 10.0;
	dHlTestRefT = 60.0;
	nHlRandGoals = 50;
	nHlRandSeed = 1;
	vHlRandBox = {0.30, 0.94, -0.37, 0.63, 0.25, 0.50};
	dHlRandRMax = 0.92;
	nHlRandJumps = 50;
	vHlRandJumpLen = {0.10, 0.30};
	vHlRandJumpAt = {0.2, 0.8};
	nHlStepBaseline = 0;
	strHlNoiseFile = "";
	nServoOnTimeoutMs = 2000;
	strCalibDir = "calib_data";
	nAppBridge = 0;
	vAppGoalBox = {0.30, 0.94, -0.37, 0.63, 0.10, 0.50};
	dAppGoalRMax = 0.92;
	dAppGoalFloor = 0.25;
	dAppHover = 0.05;
	dAppKappa = 0.4;
	dAppGapS = 1.0;
	dAppLossS = 1.0;
	dAppDurationS = 0.0;
}

namespace {

struct Entry
{
	TSTRING	strKey, strValue;
	int		nLine;
	bool	bUsed;
};
struct Section
{
	TSTRING				strName;
	int					nLine;
	std::vector<Entry>	vEntries;
};

TSTRING Trim(const TSTRING& astr)
{
	size_t uB = 0, uE = astr.size();
	while (uB < uE && isspace((unsigned char)astr[uB]))
		uB++;
	while (uE > uB && isspace((unsigned char)astr[uE - 1]))
		uE--;
	return astr.substr(uB, uE - uB);
}

TSTRING Upper(const TSTRING& astr)
{
	TSTRING str = astr;
	for (char& c : str)
		c = (char)toupper((unsigned char)c);
	return str;
}

/* the file, read once */
class CCfgFile
{
public:
	BOOL Read(const TSTRING& astrPath)
	{
		std::ifstream cIn(astrPath.c_str());
		if (!cIn)
			return FALSE;
		TSTRING strLine;
		int nLine = 0;
		while (std::getline(cIn, strLine))
		{
			nLine++;
			const size_t uComment = strLine.find_first_of(";#");
			if (uComment != TSTRING::npos)
				strLine.erase(uComment);
			strLine = Trim(strLine);
			if (strLine.empty())
				continue;
			if (strLine[0] == '[')
			{
				const size_t uEnd = strLine.find(']');
				Section stS;
				stS.strName = Upper(Trim(strLine.substr(1, uEnd == TSTRING::npos ? TSTRING::npos : uEnd - 1)));
				stS.nLine = nLine;
				m_vSections.push_back(stS);
				continue;
			}
			const size_t uEq = strLine.find('=');
			if (uEq == TSTRING::npos || m_vSections.empty())
			{
				fprintf(stderr, "[cfg] line %d: \"%s\" is not KEY=VALUE in a section - ignored\n", nLine,
				        strLine.c_str());
				continue;
			}
			Entry stE;
			stE.strKey = Upper(Trim(strLine.substr(0, uEq)));
			stE.strValue = Trim(strLine.substr(uEq + 1));
			stE.nLine = nLine;
			stE.bUsed = false;
			std::vector<Entry>& vE = m_vSections.back().vEntries;
			bool bTwice = false;
			for (const Entry& e : vE)
				if (e.strKey == stE.strKey)
					bTwice = true;
			if (bTwice)
				fprintf(stderr, "[cfg] [%s] %s given twice (line %d): the first one is used\n",
				        m_vSections.back().strName.c_str(), stE.strKey.c_str(), nLine);
			else
				vE.push_back(stE);
		}
		return TRUE;
	}

	/* the value of a key, NULL if it is not given */
	const TSTRING* Get(const char* apcSection, const char* apcKey)
	{
		const TSTRING strSec = Upper(apcSection), strKey = Upper(apcKey);
		for (Section& s : m_vSections)
		{
			if (s.strName != strSec)
				continue;
			for (Entry& e : s.vEntries)
				if (e.strKey == strKey)
				{
					e.bUsed = true;
					return &e.strValue;
				}
			return NULL;
		}
		return NULL;
	}
	BOOL HasSection(const char* apcSection) const
	{
		const TSTRING strSec = Upper(apcSection);
		for (const Section& s : m_vSections)
			if (s.strName == strSec)
				return TRUE;
		return FALSE;
	}

	/* typed reads: a key that is not given, or is empty, keeps the value */
	void Int(const char* apcSec, const char* apcKey, INT32& anOut)
	{
		const TSTRING* p = Get(apcSec, apcKey);
		if (p != NULL && !p->empty())
			anOut = (INT32)atoi(p->c_str());
	}
	void Bool(const char* apcSec, const char* apcKey, BOOL& abOut)
	{
		INT32 n = abOut ? 1 : 0;
		Int(apcSec, apcKey, n);
		abOut = (n != 0) ? TRUE : FALSE;
	}
	void Double(const char* apcSec, const char* apcKey, double& adOut, double adScale = 1.0)
	{
		const TSTRING* p = Get(apcSec, apcKey);
		if (p != NULL && !p->empty())
			adOut = atof(p->c_str()) * adScale;
	}
	void Hex(const char* apcSec, const char* apcKey, UINT32& auOut)
	{
		const TSTRING* p = Get(apcSec, apcKey);
		if (p == NULL || p->empty())
			return;
		errno = 0;
		char* pcEnd = NULL;
		const unsigned long uVal = strtoul(p->c_str(), &pcEnd, 16);
		auOut = (pcEnd != p->c_str() && errno == 0) ? (UINT32)uVal : (UINT32)atoi(p->c_str());
	}
	void String(const char* apcSec, const char* apcKey, TSTRING& astrOut)
	{
		const TSTRING* p = Get(apcSec, apcKey);
		if (p != NULL)
			astrOut = *p;
	}
	/* a list of numbers: the first values of avOut are replaced by those in
	 * the file, up to its size or the first entry that is not a number */
	void List(const char* apcSec, const char* apcKey, std::vector<double>& avOut)
	{
		const TSTRING* p = Get(apcSec, apcKey);
		if (p == NULL || p->empty())
			return;
		const char* pcCur = p->c_str();
		for (size_t i = 0; i < avOut.size(); i++)
		{
			char* pcEnd = NULL;
			const double dVal = strtod(pcCur, &pcEnd);
			if (pcEnd == pcCur)
				break;
			avOut[i] = dVal;
			pcCur = pcEnd;
		}
	}
	/* a list of anN numbers: the values in the file, the rest 0 */
	void ListN(const char* apcSec, const char* apcKey, std::vector<double>& avOut, int anN)
	{
		const TSTRING* p = Get(apcSec, apcKey);
		if (p == NULL || p->empty())
			return;
		avOut.assign(anN > 0 ? (size_t)anN : 0, 0.0);
		List(apcSec, apcKey, avOut);
	}

	/* the keys that no read asked for */
	int ReportUnused() const
	{
		int nUnused = 0;
		for (const Section& s : m_vSections)
			for (const Entry& e : s.vEntries)
				if (!e.bUsed)
				{
					fprintf(stderr, "[cfg] [%s] %s (line %d): unknown key - ignored\n", s.strName.c_str(),
					        e.strKey.c_str(), e.nLine);
					nUnused++;
				}
		return nUnused;
	}
	/* sections that no read asked for */
	void ReportSections(const std::vector<TSTRING>& avKnown) const
	{
		for (const Section& s : m_vSections)
		{
			bool bKnown = false;
			for (const TSTRING& str : avKnown)
				if (s.strName == str)
					bKnown = true;
			if (!bKnown)
				fprintf(stderr, "[cfg] [%s] (line %d): unknown section - ignored\n", s.strName.c_str(), s.nLine);
		}
	}

private:
	std::vector<Section> m_vSections;
};

BOOL FileExists(const TSTRING& astrPath)
{
	struct stat stInfo;
	return (0 == stat(astrPath.c_str(), &stInfo) && S_ISREG(stInfo.st_mode)) ? TRUE : FALSE;
}

TSTRING DirOf(const TSTRING& astrPath)
{
	const size_t uSlash = astrPath.rfind('/');
	return (uSlash == TSTRING::npos) ? TSTRING(".") : astrPath.substr(0, uSlash == 0 ? 1 : uSlash);
}

} // namespace

CConfig::CConfig()
{
}

const char*
CConfig::SyncModeName(int anMode)
{
	return anMode == eSyncSm ? "SM" : anMode == eSyncDc ? "DC" : "FREE_RUN";
}

TSTRING
CConfig::Find(const TSTRING& astrGiven, const char* apcArgv0)
{
	if (!astrGiven.empty())
		return astrGiven;
	if (FileExists("INDY7.cfg"))
		return "INDY7.cfg";
	/* the program in <dir>/bin: <dir>/INDY7.cfg */
	char acExe[PATH_MAX];
	const ssize_t nLen = readlink("/proc/self/exe", acExe, sizeof(acExe) - 1);
	TSTRING strExe = (nLen > 0) ? TSTRING(acExe, (size_t)nLen) : TSTRING(apcArgv0 != NULL ? apcArgv0 : "");
	if (!strExe.empty())
	{
		const TSTRING strCand = DirOf(DirOf(strExe)) + "/INDY7.cfg";
		if (FileExists(strCand))
			return strCand;
	}
	return "INDY7.cfg";
}

TSTRING
CConfig::Resolve(const TSTRING& astrPath) const
{
	if (astrPath.empty() || astrPath[0] == '/')
		return astrPath;
	return DirOf(m_strPath) + "/" + astrPath;
}

BOOL
CConfig::Load(const TSTRING& astrPath, BOOL abForceSim)
{
	m_strPath = astrPath;
	if (FALSE == Parse())
		return FALSE;
	if (abForceSim)
		m_stSystem.bSim = TRUE;
	return TRUE;
}

BOOL
CConfig::Parse()
{
	CCfgFile cF;
	if (FALSE == cF.Read(m_strPath))
	{
		DBG_LOG_ERROR("(CConfig) cannot read %s", m_strPath.c_str());
		return FALSE;
	}
	ST_CONFIG_SYSTEM& s = m_stSystem;
	const char* S = "SYSTEM";

	/* [SYSTEM] */
	cF.String(S, "NAME", s.strName);
	cF.Bool(S, "SIMULATION_MODE", s.bSim);
	cF.String(S, "URDF_PATH", s.strURDFPath);
	cF.Int(S, "ROBOT_DOF", s.nRobotAxis);
	const int nDof = s.nRobotAxis > 0 ? s.nRobotAxis : 0;
	s.vKp.assign((size_t)nDof, 0.0);
	s.vKd.assign((size_t)nDof, 0.0);
	s.vKf.assign((size_t)nDof, 0.0);
	s.vRotorB.assign((size_t)nDof, 0.0);
	for (int i = 0; i < nDof; i++)
	{
		char acKey[32];
		snprintf(acKey, sizeof(acKey), "KP_%d", i);
		cF.Double(S, acKey, s.vKp[i]);
		snprintf(acKey, sizeof(acKey), "KD_%d", i);
		cF.Double(S, acKey, s.vKd[i]);
		snprintf(acKey, sizeof(acKey), "KF_%d", i);
		cF.Double(S, acKey, s.vKf[i]);
		snprintf(acKey, sizeof(acKey), "ROTOR_INERTIA_%d", i);
		cF.Double(S, acKey, s.vRotorB[i]);
	}
	s.vSimArmature = s.vRotorB;
	cF.ListN(S, "SIM_ARMATURE", s.vSimArmature, nDof);
	cF.Int(S, "FRICTION_FF_GATE", s.nFfGate);
	cF.Bool(S, "ENABLE_CONTROLLER_AT_STARTUP", s.bEnableControllerAtStartup);
	cF.Int(S, "SERVO_ON_TIMEOUT_MS", s.nServoOnTimeoutMs);

	cF.String(S, "SIM_MJCF_PATH", s.strSimMJCFPath);
	cF.Int(S, "SIM_STATE_DELAY_STEPS", s.nSimStateDelay);
	cF.Int(S, "SIM_TORQUE_DELAY_STEPS", s.nSimTorqueDelay);
	cF.Bool(S, "SIM_FRICTION_ON", s.bSimFriction);
	s.vSimTauS.assign((size_t)nDof, 0.0);
	s.vSimInitialQ.assign((size_t)nDof, 0.0);
	cF.ListN(S, "SIM_TAU_S", s.vSimTauS, nDof);
	cF.ListN(S, "SIM_INITIAL_Q", s.vSimInitialQ, nDof);
	cF.Int(S, "SIM_FAULT_JOINT", s.nSimFaultJoint);
	cF.Int(S, "SIM_FAULT_PERSIST", s.nSimFaultPersist);
	{
		TSTRING str = "torque";
		cF.String(S, "SIM_FAULT_WHAT", str);
		s.nSimFaultWhat = (str.empty() || strcasecmp(str.c_str(), "torque") == 0) ? 0 :
		                  (strcasecmp(str.c_str(), "state") == 0) ? 1 :
		                  (strcasecmp(str.c_str(), "dropout") == 0) ? 2 :
		                  (strcasecmp(str.c_str(), "estop") == 0) ? 3 : -1;
		str = "sod";
		cF.String(S, "SIM_ESTOP_REPORTS", str);
		s.nSimEstopReports = (str.empty() || strcasecmp(str.c_str(), "sod") == 0) ? 0 :
		                     (strcasecmp(str.c_str(), "ready") == 0) ? 1 :
		                     (strcasecmp(str.c_str(), "fault") == 0) ? 2 : -1;
	}
	cF.Int(S, "VIEW_PORT", s.nViewPort);
	cF.Double(S, "VIEW_RATE_HZ", s.dViewRateHz);

	s.vJtFastDq.resize((size_t)nDof, 0.0);
	s.vJtSlowDq.resize((size_t)nDof, 0.0);
	cF.ListN(S, "JOINT_TEST_FAST_DQ", s.vJtFastDq, nDof);
	cF.ListN(S, "JOINT_TEST_SLOW_DQ", s.vJtSlowDq, nDof);
	cF.Double(S, "JOINT_TEST_FAST_T", s.dJtFastT);
	cF.Double(S, "JOINT_TEST_SLOW_T", s.dJtSlowT);
	cF.Double(S, "JOINT_TEST_DWELL", s.dJtDwell);
	cF.String(S, "JOINT_TEST_LOG_DIR", s.strJtLogDir);
	cF.Int(S, "TORQUE_STEP_JOINT", s.nTsJoint);
	cF.Double(S, "TORQUE_STEP_NM", s.dTsTau);
	cF.Double(S, "TORQUE_STEP_ON_S", s.dTsOn);
	cF.Double(S, "TORQUE_STEP_OFF_S", s.dTsOff);
	cF.Int(S, "TORQUE_STEP_REPEAT", s.nTsRep);
	cF.Int(S, "BREAKAWAY_JOINT", s.nBaJoint);
	cF.Double(S, "BREAKAWAY_RATE_NMS", s.dBaRate);
	cF.Double(S, "BREAKAWAY_MAX_NM", s.dBaMax);
	cF.Double(S, "BREAKAWAY_QD", s.dBaQd);
	cF.Int(S, "BREAKAWAY_REPEAT", s.nBaRep);
	cF.Int(S, "SINE_TEST_JOINT", s.nSineJoint);
	cF.Double(S, "SINE_TEST_AMP_RAD", s.dSineAmp);
	cF.Double(S, "SINE_TEST_FREQ_HZ", s.dSineFreq);
	cF.Int(S, "SINE_TEST_PERIODS", s.nSinePeriods);
	cF.Double(S, "SINE_TEST_SWEEP_RADS", s.dSineV0);
	cF.Int(S, "SINE_TEST_FF", s.nSineFf);

	cF.Double(S, "HL_GOAL_DELTA_MM", s.dHlDelta, 1e-3);
	cF.Double(S, "HL_IK_LAMBDA", s.dHlLambda);
	cF.Double(S, "HL_IK_STEP_MAX", s.dHlStepMax);
	cF.Double(S, "HL_IK_TOL_MM", s.dHlTol, 1e-3);
	cF.Int(S, "HL_IK_ITER_CYCLE", s.nHlIterCycle);
	cF.Int(S, "HL_IK_ITER_GOAL", s.nHlIterGoal);
	cF.Double(S, "HL_VEL_MAX", s.dHlVelMax);
	cF.Double(S, "HL_ACC_MAX", s.dHlAccMax);
	cF.Double(S, "HL_JERK_MAX", s.dHlJerkMax);
	cF.Double(S, "HL_LIMIT_MARGIN", s.dHlMargin);
	cF.Double(S, "HL_LEAD_MAX", s.dHlLead);
	cF.List(S, "HL_READY_Q", s.vHlReadyQ);
	cF.List(S, "HL_TEST_GOAL", s.vHlTestGoal);
	cF.List(S, "HL_TEST_JUMP", s.vHlTestJump);
	cF.Double(S, "HL_TEST_TRIGGER_M", s.dHlTestTrigger);
	cF.Double(S, "HL_TEST_NOISE_MM", s.dHlTestNoise, 1e-3);
	cF.Double(S, "HL_TEST_NOISE_HZ", s.dHlTestNoiseHz);
	cF.Double(S, "HL_TEST_NOISE_S", s.dHlTestNoiseT);
	cF.Double(S, "HL_TEST_REF_S", s.dHlTestRefT);
	cF.String(S, "HL_TEST_NOISE_FILE", s.strHlNoiseFile);
	cF.Int(S, "HL_RAND_GOALS", s.nHlRandGoals);
	cF.Int(S, "HL_RAND_SEED", s.nHlRandSeed);
	cF.List(S, "HL_RAND_BOX_M", s.vHlRandBox);
	cF.Double(S, "HL_RAND_RMAX_M", s.dHlRandRMax);
	cF.Int(S, "HL_RAND_JUMPS", s.nHlRandJumps);
	cF.List(S, "HL_RAND_JUMP_LEN_M", s.vHlRandJumpLen);
	cF.List(S, "HL_RAND_JUMP_AT", s.vHlRandJumpAt);
	cF.Int(S, "HL_STEP_BASELINE", s.nHlStepBaseline);

	cF.String(S, "CALIB_DIR", s.strCalibDir);
	cF.Int(S, "APP_BRIDGE", s.nAppBridge);
	cF.List(S, "APP_GOAL_BOX_M", s.vAppGoalBox);
	cF.Double(S, "APP_GOAL_RMAX_M", s.dAppGoalRMax);
	cF.Double(S, "APP_GOAL_FLOOR_M", s.dAppGoalFloor);
	cF.Double(S, "APP_HOVER_M", s.dAppHover);
	cF.Double(S, "APP_EMA_KAPPA", s.dAppKappa);
	cF.Double(S, "APP_GAP_S", s.dAppGapS);
	cF.Double(S, "APP_LOSS_S", s.dAppLossS);
	cF.Double(S, "APP_DURATION_S", s.dAppDurationS);

	/* the paths of the file, relative to its directory */
	s.strURDFPath = Resolve(s.strURDFPath);
	s.strSimMJCFPath = Resolve(s.strSimMJCFPath);
	s.strJtLogDir = Resolve(s.strJtLogDir);
	s.strCalibDir = Resolve(s.strCalibDir);
	s.strHlNoiseFile = Resolve(s.strHlNoiseFile);

	/* [RT_TASKS], [TASKn] */
	std::vector<TSTRING> vKnown = {"SYSTEM", "RT_TASKS", "ECAT_MASTER"};
	INT32 nTasks = 0;
	cF.Int("RT_TASKS", "NO_OF_TASKS", nTasks);
	m_vTasks.clear();
	for (int n = 0; n < nTasks; n++)
	{
		char acSec[16];
		snprintf(acSec, sizeof(acSec), "TASK%d", n);
		vKnown.push_back(acSec);
		ST_CONFIG_TASK stT;
		stT.strName = "NULL";
		cF.Bool(acSec, "ENABLED", stT.bEnabled);
		cF.String(acSec, "NAME", stT.strName);
		cF.Int(acSec, "PRIORITY", stT.nPriority);
		cF.Int(acSec, "TASK_PERIOD", stT.nPeriod);
		cF.Int(acSec, "START_DELAY", stT.nStartDelay);
		cF.Int(acSec, "CPU", stT.nCpu);
		m_vTasks.push_back(stT);
	}

	/* [ECAT_MASTER], [AXISn] */
	ST_CONFIG_ECAT_MASTER& m = m_stMaster;
	cF.Int("ECAT_MASTER", "SLAVENUMBER", m.nNoOfSlaves);
	cF.Int("ECAT_MASTER", "CYCLE_TIME", m.nCycleTime);
	cF.Int("ECAT_MASTER", "SEND_DEADLINE", m.nSendDeadline);
	cF.Int("ECAT_MASTER", "DC_SYNC0_SHIFT", m.nDcSync0Shift);
	{
		TSTRING str = "FREE_RUN";
		cF.String("ECAT_MASTER", "SYNC_MODE", str);
		str = Upper(str);
		if (str == "FREE_RUN" || str.empty())
			m.nSyncMode = eSyncFreeRun;
		else if (str == "SM")
			m.nSyncMode = eSyncSm;
		else if (str == "DC")
			m.nSyncMode = eSyncDc;
		else
		{
			DBG_LOG_ERROR("(CConfig) [ECAT_MASTER] SYNC_MODE=%s: FREE_RUN, SM or DC", str.c_str());
			return FALSE;
		}
	}
	m_vSlaves.clear();
	for (int n = 0; n < m.nNoOfSlaves; n++)
	{
		char acSec[16];
		snprintf(acSec, sizeof(acSec), "AXIS%d", n);
		vKnown.push_back(acSec);
		if (!cF.HasSection(acSec))
		{
			DBG_LOG_ERROR("(CConfig) SLAVENUMBER=%d, but no section [%s]", (int)m.nNoOfSlaves, acSec);
			return FALSE;
		}
		ST_CONFIG_ECAT_SLAVE stSl;
		cF.String(acSec, "NAME", stSl.strName);
		cF.Hex(acSec, "VENDOR_ID", stSl.uVendorID);
		cF.Hex(acSec, "PRODUCT_CODE", stSl.uProductCode);
		cF.Int(acSec, "SLAVE_TYPE", stSl.nSlaveType);
		if (stSl.nSlaveType == 0)
		{
			cF.Bool(acSec, "MOVE_CCW", stSl.bCCW);
			cF.Int(acSec, "DRIVE_MODE", stSl.nDriveMode);
			cF.Double(acSec, "ENCODER_RESOLUTION", stSl.dEncResolution);
			cF.Double(acSec, "GEAR_RATIO", stSl.dGearRatio);
			cF.Double(acSec, "TRANSMISSION_RATIO", stSl.dTransRatio);
			cF.Double(acSec, "TORQUE_CONSTANT", stSl.dTorqueConstant);
			cF.Double(acSec, "CURRENT_RATIO", stSl.dCurrentRatio);
			cF.Int(acSec, "HOME_POSITION_OFFSET", stSl.nHomePositionOffset);
			cF.Double(acSec, "TORQUE_LIMIT_U", stSl.dTorLimitU);
		}
		m_vSlaves.push_back(stSl);
	}

	cF.ReportSections(vKnown);
	cF.ReportUnused();
	return TRUE;
}
