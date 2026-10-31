/*****************************************************************************
*	Name: ViewStream.h
*	Description: Live joint angles for an external viewer (tools/sim_view.py).
*
*	The EtherCAT task calls Publish() once per period: an atomic slot
*	(AtomicSlot.h), no system call, no allocation. A non-real-time thread
*	serves one TCP client and sends it the latest angles at a fixed rate as
*	text:
*	    # g2t-view 1 dof=6 rate=50
*	    <t [s]> <q0> ... <q5>      [rad], one line per sample
*	so that the viewer only displays: the control and the simulation stay on
*	the real-time core of the controller.
*****************************************************************************/
#ifndef G2T_VIEW_STREAM_H
#define G2T_VIEW_STREAM_H

#include <atomic>
#include <thread>

#include "Common.h"
#include "AtomicSlot.h"

class CViewStream
{
public:
	enum { MAX_DOF = 8 };

	CViewStream();
	~CViewStream();

	/* not real-time: the server thread; anPort 0: off */
	BOOL	Start		(int anPort, double adRateHz, int anDof);
	void	Stop		(	);
	BOOL	IsRunning	(	) const { return m_th.joinable() ? TRUE : FALSE; }

	/* the EtherCAT task: the newest joint angles [rad] */
	void	Publish		(const double* adQ);

private:
	struct View { double adQ[MAX_DOF]; };
	void	Run			(	);

	CAtomicSlot<View>		m_cSlot;
	std::thread				m_th;
	std::atomic<bool>		m_bStop;
	int						m_nPort;
	int						m_nDof;			// set before the thread and the tasks start
	double					m_dRateHz;
};

#endif // G2T_VIEW_STREAM_H
