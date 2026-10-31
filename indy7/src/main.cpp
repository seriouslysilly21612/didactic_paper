/*****************************************************************************
*	Name: main.cpp
*	Description: The controller of the Indy7 (paper, Fig. 1).
*
*	    indy7_ctrl [-f FILE] [-s | -b] [-t DIR]
*
*	  -f FILE  the configuration; without it ./INDY7.cfg, then INDY7.cfg in
*	           the directory above the program (bin/..)
*	  -s       simulation: MuJoCo in place of the bus (paper, Sec. IV, V)
*	  -b       the bus of the robot in OP with its drives kept off, beside the
*	           simulated arm (the host-timing test, paper, Sec. III-C)
*	  -t DIR   the timing of every period into DIR (TimingLog.h,
*	           tools/timing_summary.py)
*
*	The keys of the operator are listed at the start and with '?'
*	(Indy7Ctrl.cpp, PrintHelp). 'q', SIGINT or SIGTERM end the controller:
*	the drives are switched off while the exchange goes on for 1 s, then the
*	master is released.
*****************************************************************************/
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <sys/mman.h>

#include "Indy7Ctrl.h"

namespace {

CRobotIndy7* volatile g_pRobot = NULL;

// a signal asks the tasks to stop; the EtherCAT task then switches the
// drives off, and main() ends the application as after 'q'
void SignalHandler(int)
{
	CRobotIndy7* const pRobot = g_pRobot;
	if (pRobot != NULL)
		pRobot->StopTasks();
}

void Usage(const char* apcProg)
{
	fprintf(stderr, "usage: %s [-f FILE] [-s | -b] [-t DIR]\n"
	        "  -f FILE  configuration (default: ./INDY7.cfg, then <program dir>/../INDY7.cfg)\n"
	        "  -s       simulation: MuJoCo in place of the bus\n"
	        "  -b       the bus of the robot, drives off, beside the simulated arm (host-timing test)\n"
	        "  -t DIR   timing of every period into DIR\n", apcProg);
}

} // namespace

int main(int argc, char** argv)
{
	// the terminal mode, restored at the end (the keyboard task changes it)
	struct termios stTermios;
	const bool bTermios = (0 == tcgetattr(STDIN_FILENO, &stTermios));

	TSTRING strCfg, strTimingDir;
	BOOL bSim = FALSE, bHybrid = FALSE;
	int nOpt;
	while ((nOpt = getopt(argc, argv, "f:sbt:h")) != -1)
	{
		switch (nOpt)
		{
			case 'f': strCfg = optarg; break;
			case 's': bSim = TRUE; break;
			case 'b': bHybrid = TRUE; bSim = TRUE; break;   // the arm is simulated as with -s
			case 't': strTimingDir = optarg; break;
			default:
				Usage(argv[0]);
				return 1;
		}
	}

	strCfg = CConfig::Find(strCfg, argv[0]);
	CConfig cConfig;
	if (FALSE == cConfig.Load(strCfg, bSim))
	{
		Usage(argv[0]);
		return 1;
	}
	DBG_LOG_INFO("configuration %s: %s", cConfig.Path().c_str(),
	             bHybrid ? "-b, the bus of the robot (drives off) beside the simulated arm" :
	             cConfig.System().bSim ? "simulation, MuJoCo in place of the bus" : "the robot");

	// SIGINT and SIGTERM stop the tasks (the real-time tasks block both)
	struct sigaction stSa;
	memset(&stSa, 0, sizeof(stSa));
	stSa.sa_handler = SignalHandler;
	sigemptyset(&stSa.sa_mask);
	sigaction(SIGINT, &stSa, NULL);
	sigaction(SIGTERM, &stSa, NULL);

	// every page locked, before any task is created: without the lock the
	// real-time tasks can take page faults
	if (0 != mlockall(MCL_CURRENT | MCL_FUTURE))
	{
		DBG_LOG_ERROR("mlockall failed: %s (the memlock limit must be unlimited, see run.sh)", strerror(errno));
		return 1;
	}

	CRobotIndy7* pRobot = new CRobotIndy7(&cConfig);
	pRobot->SetHybridBus(bHybrid);
	if (!strTimingDir.empty())
		pRobot->SetTimingLogDir(strTimingDir);
	g_pRobot = pRobot;

	int nRet = 0;
	if (FALSE == pRobot->Init())
	{
		DBG_LOG_ERROR("the controller cannot start");
		pRobot->StopTasks();
		nRet = 1;
	}
	else if (FALSE == pRobot->CheckSyncMode())
	{
		pRobot->StopTasks();
		nRet = 1;
	}

	while (!pRobot->CheckStopTask())
		usleep(1000);
	// after a stop the EtherCAT task switches the drives off for 1 s while
	// the exchange goes on, releases the master, and ends; the Trajectory
	// and the Dynamics task end with it, and the summaries are printed
	if (pRobot->AreTasksStarted())
	{
		for (int nWait = 0; nWait < 5000 && FALSE == pRobot->IsControlTaskDone(); nWait++)
			usleep(1000);
		if (FALSE == pRobot->IsControlTaskDone())
			DBG_LOG_WARN("the real-time tasks have not ended within 5 s");
	}
	pRobot->StopServices();
	DBG_LOG_INFO("the controller has ended");

	// the keyboard task may still wait for a key and hold the lock of stdin,
	// which exit() would wait for: restore the terminal, flush, and end the
	// process without the teardown of stdio
	if (bTermios)
		tcsetattr(STDIN_FILENO, TCSANOW, &stTermios);
	fflush(stdout);
	fflush(stderr);
	_exit(nRet);
}
