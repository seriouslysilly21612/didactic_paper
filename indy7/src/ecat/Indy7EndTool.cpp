/*****************************************************************************
*	Name: Indy7EndTool.cpp
*	Description: The end-tool module of the Indy7. See Indy7EndTool.h.
*****************************************************************************/
#include "Indy7EndTool.h"

CIndy7EndTool::CIndy7EndTool(const ST_CONFIG_ECAT_SLAVE& astCfg)
	: m_ucLedMode(NO_COLOR), m_ucStatus(0), m_ucButton(0), m_ucOverload(0), m_ucError(0)
{
	m_strSlaveName = astCfg.strName;
	m_uVendor = astCfg.uVendorID;
	m_uProduct = astCfg.uProductCode;
	for (int i = 0; i < OUT_N; i++)
		m_anOffOut[i] = -1;
	for (int i = 0; i < IN_N; i++)
		m_anOffIn[i] = -1;
	memset(m_asFt, 0, sizeof(m_asFt));

	const ec_pdo_entry_info_t astE[17] = {
		/* outputs of the master, RxPDO 0x1600 */
		{0x7000, 0x01, 8},		// ILed
		{0x7000, 0x02, 8},		// IGripper
		{0x7000, 0x03, 32},		// FTConfigParam
		{0x7000, 0x04, 8},		// LEDMode
		{0x7000, 0x05, 8},		// LEDG
		{0x7000, 0x06, 8},		// LEDR
		{0x7000, 0x07, 8},		// LEDB
		/* inputs of the master, TxPDO 0x1A00 */
		{0x6000, 0x01, 8},		// IStatus
		{0x6000, 0x02, 8},		// IButton
		{0x6000, 0x03, 16},		// FTRawFx
		{0x6000, 0x04, 16},		// FTRawFy
		{0x6000, 0x05, 16},		// FTRawFz
		{0x6000, 0x06, 16},		// FTRawTx
		{0x6000, 0x07, 16},		// FTRawTy
		{0x6000, 0x08, 16},		// FTRawTz
		{0x6000, 0x09, 8},		// FTOverloadStatus
		{0x6000, 0x0A, 8},		// FTErrorFlag
	};
	memcpy(m_astEntries, astE, sizeof(m_astEntries));
	m_astPdos[0].index = 0x1600;
	m_astPdos[0].n_entries = 7;
	m_astPdos[0].entries = m_astEntries + 0;
	m_astPdos[1].index = 0x1A00;
	m_astPdos[1].n_entries = 10;
	m_astPdos[1].entries = m_astEntries + 7;
	const ec_sync_info_t astS[5] = {
		{0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
		{1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
		{2, EC_DIR_OUTPUT, 1, m_astPdos + 0, EC_WD_ENABLE},
		{3, EC_DIR_INPUT, 1, m_astPdos + 1, EC_WD_DISABLE},
		{0xff, EC_DIR_INVALID, 0, NULL, EC_WD_DEFAULT},
	};
	memcpy(m_astSyncs, astS, sizeof(m_astSyncs));
}

BOOL
CIndy7EndTool::Configure(ec_master_t* apMaster, ec_domain_t* apIn, ec_domain_t* apOut)
{
	if (FALSE == ConfigureSlave(apMaster, m_astSyncs))
		return FALSE;
	BOOL bOk = TRUE;
	for (int i = 0; i < OUT_N; i++)
		if ((m_anOffOut[i] = RegisterEntry(apOut, m_astEntries[i].index, m_astEntries[i].subindex)) < 0)
			bOk = FALSE;
	for (int i = 0; i < IN_N; i++)
		if ((m_anOffIn[i] = RegisterEntry(apIn, m_astEntries[OUT_N + i].index, m_astEntries[OUT_N + i].subindex)) < 0)
			bOk = FALSE;
	return bOk;
}

void
CIndy7EndTool::ReadFromSlave()
{
	if (FALSE == IsOnline())
		return;
	m_ucStatus = EC_READ_U8(m_pIn + m_anOffIn[IN_ISTATUS]);
	m_ucButton = EC_READ_U8(m_pIn + m_anOffIn[IN_IBUTTON]);
	for (int k = 0; k < 6; k++)
		m_asFt[k] = EC_READ_S16(m_pIn + m_anOffIn[IN_FX + k]);
	m_ucOverload = EC_READ_U8(m_pIn + m_anOffIn[IN_OVERLOAD]);
	const UINT8 ucError = EC_READ_U8(m_pIn + m_anOffIn[IN_ERROR]);
	if (ucError != 0 && ucError != m_ucError)
		DBG_LOG_WARN("(CIndy7EndTool) force-torque sensor error 0x%02x", (unsigned)ucError);
	m_ucError = ucError;
}

void
CIndy7EndTool::WriteToSlave()
{
	if (FALSE == IsOnline())
		return;
	EC_WRITE_U8(m_pOut + m_anOffOut[OUT_ILED], 0);
	EC_WRITE_U8(m_pOut + m_anOffOut[OUT_IGRIPPER], 0);
	EC_WRITE_S32(m_pOut + m_anOffOut[OUT_FTCONFIG], (INT32)FT_STOP_DATA_OUTPUT);	// the sensor stays off
	EC_WRITE_U8(m_pOut + m_anOffOut[OUT_LEDMODE], m_ucLedMode);
	EC_WRITE_U8(m_pOut + m_anOffOut[OUT_LEDG], 0);
	EC_WRITE_U8(m_pOut + m_anOffOut[OUT_LEDR], 0);
	EC_WRITE_U8(m_pOut + m_anOffOut[OUT_LEDB], 0);
}
