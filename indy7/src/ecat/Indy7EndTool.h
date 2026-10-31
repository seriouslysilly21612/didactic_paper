/*****************************************************************************
*	Name: Indy7EndTool.h
*	Description: The end-tool module of the Indy7 (the last slave of the
*	bus): its LED shows the state of the controller. The force-torque
*	sensor of the module stays off. EtherCAT task only.
*****************************************************************************/
#ifndef G2T_INDY7_END_TOOL_H
#define G2T_INDY7_END_TOOL_H

#include "EcatMaster.h"

class CIndy7EndTool : public CEcatSlave
{
public:
	enum eLed { NO_COLOR = 0, BLINK_RED, STEADY_RED, BLINK_GREEN, STEADY_GREEN, BLINK_BLUE, STEADY_BLUE,
	            BLINK_YELLOW, STEADY_YELLOW };

	CIndy7EndTool(const ST_CONFIG_ECAT_SLAVE& astCfg);

	virtual BOOL	Configure		(ec_master_t* apMaster, ec_domain_t* apIn, ec_domain_t* apOut);
	virtual void	ReadFromSlave	(	);
	virtual void	WriteToSlave	(	);

	void	LedRed		(BOOL abBlink) { m_ucLedMode = abBlink ? BLINK_RED : STEADY_RED; }
	void	LedGreen	(BOOL abBlink) { m_ucLedMode = abBlink ? BLINK_GREEN : STEADY_GREEN; }
	void	LedBlue		(BOOL abBlink) { m_ucLedMode = abBlink ? BLINK_BLUE : STEADY_BLUE; }
	void	LedYellow	(BOOL abBlink) { m_ucLedMode = abBlink ? BLINK_YELLOW : STEADY_YELLOW; }
	void	LedOff		(	) { m_ucLedMode = NO_COLOR; }

private:
	enum { FT_STOP_DATA_OUTPUT = 0x0000000C };
	enum { OUT_ILED = 0, OUT_IGRIPPER, OUT_FTCONFIG, OUT_LEDMODE, OUT_LEDG, OUT_LEDR, OUT_LEDB, OUT_N };
	enum { IN_ISTATUS = 0, IN_IBUTTON, IN_FX, IN_FY, IN_FZ, IN_TX, IN_TY, IN_TZ, IN_OVERLOAD, IN_ERROR, IN_N };

	int		m_anOffOut[OUT_N];
	int		m_anOffIn[IN_N];
	UINT8	m_ucLedMode;
	UINT8	m_ucStatus, m_ucButton, m_ucOverload, m_ucError;
	INT16	m_asFt[6];

	ec_pdo_entry_info_t	m_astEntries[17];
	ec_pdo_info_t		m_astPdos[2];
	ec_sync_info_t		m_astSyncs[5];
};

#endif // G2T_INDY7_END_TOOL_H
