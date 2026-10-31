/*****************************************************************************
*	Name: ROS2PickBridgeNoRos.cpp
*	Description: CROS2PickBridge for a build without ROS 2 (make ROS=0):
*	Init() fails, so the controller runs without the bridge, as when the
*	bridge cannot start ('p' and 'y' 8 are refused); the [RT] methods only
*	let it link.
*****************************************************************************/
#ifndef G2T_NO_ROS
#error "ROS2PickBridgeNoRos.cpp is built only with ROS=0 (-DG2T_NO_ROS)"
#endif

#include "ROS2PickBridge.h"

#include <cstdio>

CROS2PickBridge::CROS2PickBridge()
{
}

CROS2PickBridge::~CROS2PickBridge()
{
	DeInit();
}

BOOL
CROS2PickBridge::Init(const Config&, CGoalSlot*)
{
	printf("[bridge] built without ROS 2 (make ROS=0): no bridge, the camera application ('p', 'y' 8) is "
	       "unavailable\n");
	m_bInit = FALSE;
	return FALSE;
}

BOOL
CROS2PickBridge::DeInit()
{
	m_bInit = FALSE;
	return TRUE;
}

BOOL CROS2PickBridge::FeedDigit(char)                             { return FALSE; }
void CROS2PickBridge::PushJointState(const double*, const double*, int) { }
