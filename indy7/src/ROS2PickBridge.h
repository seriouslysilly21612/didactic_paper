/*****************************************************************************
*	Name: ROS2PickBridge.h
*	Description: The ROS 2 goal bridge (paper, Sec. II and V-C, Fig. 1): the
*	only ROS 2 node inside the control process, between the camera pipeline
*	(ros2/) and the real-time tasks. Two threads, both on the non-isolated
*	cores (they inherit the affinity of main(), which excludes the isolated
*	core; only the real-time tasks are pinned to it):
*	  executor  /pick_target_base (my_interfaces/PickTarget3D, base frame)
*	            -> goal checks and smoothing (GoalFilter.h) -> the goal slot
*	            of the camera application, which the high-level controller
*	            reads in TRACK; /detections -> the objects in view (menu)
*	  worker    /joint_states (sensor_msgs/JointState, 20 Hz) from the joint
*	            state that the EtherCAT task publishes every period; the
*	            dialogue with the operator: the object menu ('p', then a digit
*	            sets desired_class of /pick_logic_node, and the arm follows the
*	            object) and the reports (target found, target lost after
*	            APP_LOSS_S without an accepted sample, rejected samples)
*	The slot carries the smoothed object position; the high-level controller
*	adds the hover height (CTrajectoryRT::SetGoalOffset). The real-time tasks
*	call only the methods marked [RT]: atomics and atomic slots, no lock, no
*	allocation, no system call.
*
*	make ROS=0 builds the controller without ROS 2: ROS2PickBridgeNoRos.cpp,
*	whose Init() fails, so the controller runs without the bridge ('p' and
*	'y' 8 are refused). The bridge loads the type support of its messages at
*	run time: source /opt/ros/jazzy/setup.bash and ros2/install/setup.bash
*	before run.sh.
*****************************************************************************/
#ifndef G2T_ROS2_PICK_BRIDGE_H
#define G2T_ROS2_PICK_BRIDGE_H

#include "base/Common.h"
#include "base/AtomicSlot.h"
#include "GoalSlot.h"
#include "GoalFilter.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#ifndef G2T_NO_ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"
#include "my_interfaces/msg/pick_target3_d.hpp"
#include "my_interfaces/msg/detection_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#endif

class CROS2PickBridge
{
public:
	struct Config
	{
		CGoalFilter::Params stFilter;   // goal checks and smoothing
		double dLossS;                  // report a lost target after this time without an accepted sample [s]
	};
	enum { JS_MAX = 8 };

	CROS2PickBridge();
	~CROS2PickBridge();

	/* non-real-time: before the application uses the slot, and at its end */
	BOOL Init(const Config& astCfg, CGoalSlot* apSlot);
	BOOL DeInit();
	BOOL IsInit() const { return m_bInit; }

	/* ---- [RT] wait-free, for the real-time tasks ---- */
	// 'p' (Dynamics task): the worker prints the objects in view and opens the menu
	void RequestMenu() { m_bMenuReq.store(true, std::memory_order_release); }
	// a digit while the menu is open (Dynamics task): TRUE when the menu took it
	BOOL FeedDigit(char acDigit);
	// TRUE once after an object was selected in the menu (Trajectory task):
	// the arm is to follow it
	BOOL TakeFollowRequest() { return m_bFollowReq.exchange(false, std::memory_order_acq_rel) ? TRUE : FALSE; }
	// the joint state of this period, for /joint_states (EtherCAT task, the
	// only writer of the slot)
	void PushJointState(const double* adQ, const double* adQd, int anDof);

	/* counters (any thread) */
	long Samples() const  { return m_nSamples.load(std::memory_order_relaxed); }
	long Written() const  { return m_nWritten.load(std::memory_order_relaxed); }

private:
	void SpinLoop();
	void WorkerLoop();
	void TickJointStates();
	void TickReport();
	void ShowMenu();
	void HandleDigit(int anSel);
	BOOL SetDesiredClass(const std::string& astrClass);
	static double NowS();
#ifndef G2T_NO_ROS
	void OnTarget(const my_interfaces::msg::PickTarget3D::SharedPtr apMsg);
	void OnDetections(const my_interfaces::msg::DetectionArray::SharedPtr apMsg);

	std::shared_ptr<rclcpp::Node> m_pNode;
	std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> m_pExec;
	rclcpp::Subscription<my_interfaces::msg::PickTarget3D>::SharedPtr   m_pSubTarget;
	rclcpp::Subscription<my_interfaces::msg::DetectionArray>::SharedPtr m_pSubDetections;
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr         m_pPubJoints;
	std::shared_ptr<rclcpp::AsyncParametersClient> m_pParamClient;
#endif

	Config      m_stCfg{};
	CGoalSlot*  m_pSlot{NULL};
	BOOL        m_bInit{FALSE};
	BOOL        m_bOwnContext{FALSE};   // this bridge called rclcpp::init, so it shuts it down
	std::thread m_thSpin, m_thWorker;
	std::atomic<bool> m_bStop{false};

	/* executor thread only */
	CGoalFilter m_cFilter;

	/* executor -> worker */
	std::atomic<long>   m_nSamples{0}, m_nWritten{0}, m_nRestarts{0};
	std::atomic<long>   m_anRejected[CGoalFilter::eVerdicts]{};
	std::atomic<int>    m_nLastReject{-1};
	std::atomic<double> m_dLastAcceptS{-1.0};
	std::mutex          m_Mtx;              // the two members below and m_mapSeen (non-real-time threads only)
	double              m_adLastGoal[3]{};  // last smoothed object position written into the slot
	std::string         m_strLastClass;
	std::map<std::string, std::pair<int, double>> m_mapSeen;   // class -> (detections, last time [s])

	/* worker thread only */
	bool        m_bHaveTarget{false};
	long        m_anRejShown[CGoalFilter::eVerdicts]{};
	double      m_dRejectShownS{0.0};
	std::vector<std::string> m_vMenu;        // menu entry -> class ("" = any class)
	std::string m_strSelected;               // desired_class set by the menu ("" = any)

	/* real-time tasks <-> worker */
	std::atomic<bool> m_bMenuReq{false};
	std::atomic<bool> m_bMenuOpen{false};
	std::atomic<int>  m_nDigit{-1};
	std::atomic<bool> m_bFollowReq{false};
	struct JointState { int nDof; double adQ[JS_MAX], adQd[JS_MAX]; };
	CAtomicSlot<JointState> m_jointSlot;     // EtherCAT task -> worker

	static constexpr double MENU_WINDOW_S  = 1.0;   // classes seen in the last second
	static constexpr int    MENU_MIN_COUNT = 3;     // ... in at least three images
	static constexpr double REJECT_PRINT_S = 2.0;   // at most one rejection report per 2 s
};

#endif // G2T_ROS2_PICK_BRIDGE_H
