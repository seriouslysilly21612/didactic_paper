##############################################################################
#	Makefile of the controller of the Indy7 (paper, Fig. 1)
#
#	make                  bin/indy7_ctrl
#	make MUJOCO=0|1       without / with the simulation (-s, -b); default 1
#	                      if $(MUJOCO_DIR) has the C library of MuJoCo
#	make ROS=0|1          without / with the bridge of the camera
#	                      application; default 1 if ROS 2 ($(ROS_DIR)) and
#	                      the messages of ../ros2 (colcon build) are there
#	make MALLOC_COUNT=1   the test build that counts the heap allocations of
#	                      the real-time tasks (make clean first)
#	make tools            the offline checks: bin/model_check (MUJOCO=1),
#	                      bin/hl_check, bin/goal_filter_check, bin/slot_check
#	make clean
#
#	The dependencies (paper, Table 1): the IgH EtherCAT master 1.6
#	(IGH_DIR), RBDL with its URDF reader (RBDL_DIR), Ruckig (RUCKIG_DIR),
#	Eigen 3, MuJoCo (MUJOCO_DIR), ROS 2 Jazzy (ROS_DIR). RT-POSIX is built
#	from ../third_party/rt-posix.
##############################################################################

ROOT_DIR    := ..
SRC_DIR     := src
OBJ_DIR     := obj
BIN_DIR     := bin
RTPOSIX_DIR := $(ROOT_DIR)/third_party/rt-posix
TOOLS_DIR   := $(ROOT_DIR)/tools

IGH_DIR     ?= /opt/etherlab
RBDL_DIR    ?= /usr/local
RUCKIG_DIR  ?= /usr/local
EIGEN_DIR   ?= /usr/include/eigen3
MUJOCO_DIR  ?= /opt/mujoco
ROS_DIR     ?= /opt/ros/jazzy
MY_IFACE_DIR ?= $(abspath $(ROOT_DIR)/ros2/install/my_interfaces)

CXX ?= g++
CC  ?= gcc
CXXFLAGS := -O2 -std=gnu++17 -Wall -fexceptions -D_GNU_SOURCE
CFLAGS   := -O2 -Wall -D_GNU_SOURCE
INCLUDES := -I$(SRC_DIR) -I$(RTPOSIX_DIR)/include -I$(IGH_DIR)/include \
            -isystem $(EIGEN_DIR) -isystem $(RBDL_DIR)/include -isystem $(RUCKIG_DIR)/include
LDLIBS   := -L$(IGH_DIR)/lib -lethercat -Wl,-rpath,$(IGH_DIR)/lib \
            -L$(RBDL_DIR)/lib -lrbdl -lrbdl_urdfreader -Wl,-rpath,$(RBDL_DIR)/lib \
            -L$(RUCKIG_DIR)/lib -lruckig -Wl,-rpath,$(RUCKIG_DIR)/lib \
            -lm -lrt -lpthread

# the malloc-count test build (base/MallocCount.h)
MALLOC_COUNT ?= 0
ifeq ($(MALLOC_COUNT),1)
CXXFLAGS += -DG2T_MALLOC_COUNT
endif

# MuJoCo, the simulated arm (sim/SimArm.h)
MUJOCO ?= $(if $(wildcard $(MUJOCO_DIR)/include/mujoco/mujoco.h),1,0)
ifeq ($(MUJOCO),1)
CXXFLAGS += -DG2T_WITH_MUJOCO
INCLUDES += -isystem $(MUJOCO_DIR)/include
LDLIBS   += -L$(MUJOCO_DIR)/lib -lmujoco -Wl,-rpath,$(MUJOCO_DIR)/lib
endif

# ROS 2 Jazzy, the bridge of the camera application (ROS2PickBridge.h). Run
# a build with ROS=1 after sourcing /opt/ros/jazzy/setup.bash and
# ros2/install/setup.bash: the type support of the messages is loaded at run
# time from LD_LIBRARY_PATH.
ROS ?= $(if $(and $(wildcard $(ROS_DIR)/include/rclcpp/rclcpp/rclcpp.hpp),$(wildcard $(MY_IFACE_DIR)/include/my_interfaces/my_interfaces/msg/pick_target3_d.hpp)),1,0)
ifeq ($(ROS),1)
# -I for every ROS package in $(ROS_DIR)/include, i.e. every directory <pkg>
# that holds a directory <pkg>; this leaves out vendored libraries whose
# headers would hide those of the C library
ROS_INC_DIRS := $(foreach d,$(wildcard $(ROS_DIR)/include/*),$(if $(wildcard $(d)/$(notdir $(d))/.),$(d)))
INCLUDES += -isystem $(ROS_DIR)/include $(addprefix -isystem ,$(ROS_INC_DIRS)) -isystem $(MY_IFACE_DIR)/include/my_interfaces
ROS_CORE_LIBS = rclcpp rcl rcutils rmw rmw_implementation rcpputils rcl_yaml_param_parser rcl_logging_interface \
                tracetools libstatistics_collector ament_index_cpp rosidl_runtime_c rosidl_typesupport_c \
                rosidl_typesupport_cpp rosidl_dynamic_typesupport
ROS_MSG_PKGS = sensor_msgs std_msgs builtin_interfaces rcl_interfaces statistics_msgs rosgraph_msgs \
               service_msgs type_description_interfaces
ROS_TS = rosidl_generator_c rosidl_typesupport_c rosidl_typesupport_cpp rosidl_typesupport_fastrtps_c \
         rosidl_typesupport_fastrtps_cpp rosidl_typesupport_introspection_c rosidl_typesupport_introspection_cpp
LDLIBS += $(wildcard $(foreach t,$(ROS_TS),$(MY_IFACE_DIR)/lib/libmy_interfaces__$(t).so)) \
          $(wildcard $(foreach p,$(ROS_MSG_PKGS),$(foreach t,$(ROS_TS),$(ROS_DIR)/lib/lib$(p)__$(t).so))) \
          $(wildcard $(foreach l,$(ROS_CORE_LIBS),$(ROS_DIR)/lib/lib$(l).so)) \
          -Wl,-rpath,$(MY_IFACE_DIR)/lib -Wl,-rpath,$(ROS_DIR)/lib
BRIDGE_SOURCE := ROS2PickBridge.cpp
else
CXXFLAGS += -DG2T_NO_ROS
BRIDGE_SOURCE := ROS2PickBridgeNoRos.cpp
endif
$(info [indy7] MUJOCO=$(MUJOCO) ROS=$(ROS) MALLOC_COUNT=$(MALLOC_COUNT))

SOURCES := base/Config.cpp base/RtTask.cpp base/ViewStream.cpp base/MallocCount.cpp \
           ecat/EcatMaster.cpp ecat/Indy7Joint.cpp ecat/Indy7EndTool.cpp \
           sim/SimArm.cpp sim/SimJoint.cpp \
           Controller.cpp FullDynControllerRT.cpp HighLevelRT.cpp TrajectoryRT.cpp \
           JointTestLog.cpp HighLevelLog.cpp TimingLog.cpp CalibCapture.cpp $(BRIDGE_SOURCE) \
           Indy7Ctrl.cpp TaskEtherCAT.cpp TaskTrajectory.cpp TaskDynamics.cpp main.cpp
RTPOSIX_SOURCES := posix_rt.c commons.c

OBJECTS := $(addprefix $(OBJ_DIR)/,$(SOURCES:.cpp=.o)) \
           $(addprefix $(OBJ_DIR)/rt-posix/,$(RTPOSIX_SOURCES:.c=.o))

all: $(BIN_DIR)/indy7_ctrl

$(BIN_DIR)/indy7_ctrl: $(OBJECTS)
	@mkdir -p $(BIN_DIR)
	$(CXX) -o $@ $(OBJECTS) $(LDLIBS)

# the objects depend on the options: a stamp per combination
STAMP := $(OBJ_DIR)/.build_$(MUJOCO)$(ROS)$(MALLOC_COUNT)
$(STAMP):
	@mkdir -p $(OBJ_DIR); rm -f $(OBJ_DIR)/.build_*; touch $@

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp $(STAMP)
	@mkdir -p $(dir $@)
	$(CXX) -MD -MP $(CXXFLAGS) $(INCLUDES) -c -o $@ $<

$(OBJ_DIR)/rt-posix/%.o: $(RTPOSIX_DIR)/src/%.c $(STAMP)
	@mkdir -p $(dir $@)
	$(CC) -MD -MP $(CFLAGS) -I$(RTPOSIX_DIR)/include -c -o $@ $<

# the offline checks link the same objects as the controller
TOOL_OBJS := $(OBJ_DIR)/FullDynControllerRT.o $(OBJ_DIR)/Controller.o $(OBJ_DIR)/HighLevelRT.o \
             $(OBJ_DIR)/base/MallocCount.o $(OBJ_DIR)/rt-posix/posix_rt.o $(OBJ_DIR)/rt-posix/commons.o

tools: $(BIN_DIR)/hl_check $(BIN_DIR)/goal_filter_check $(BIN_DIR)/slot_check \
       $(if $(filter 1,$(MUJOCO)),$(BIN_DIR)/model_check)

# the RBDL model of the controller against the MuJoCo model (models/indy7_mujoco.xml)
$(BIN_DIR)/model_check: $(TOOLS_DIR)/model_check.cpp $(TOOL_OBJS)
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $< $(TOOL_OBJS) $(LDLIBS)

# the high-level controller on an ideal arm: the reference contract in every period
$(BIN_DIR)/hl_check: $(TOOLS_DIR)/hl_check.cpp $(TOOL_OBJS)
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $< $(TOOL_OBJS) $(LDLIBS)

# the goal checks and the smoothing of the bridge (GoalFilter.h), without ROS 2
$(BIN_DIR)/goal_filter_check: $(TOOLS_DIR)/goal_filter_check.cpp $(SRC_DIR)/GoalFilter.h
	@mkdir -p $(BIN_DIR)
	$(CXX) -O2 -std=gnu++17 -Wall -I$(SRC_DIR) -o $@ $<

# the concurrent-reader test of the atomic slot (paper, Sec. II-A)
$(BIN_DIR)/slot_check: $(TOOLS_DIR)/slot_check.cpp $(SRC_DIR)/base/AtomicSlot.h
	@mkdir -p $(BIN_DIR)
	$(CXX) -O2 -std=gnu++17 -Wall -I$(SRC_DIR) -o $@ $< -lpthread

clean:
	rm -rf $(OBJ_DIR) $(BIN_DIR)

.PHONY: all tools clean
-include $(shell find $(OBJ_DIR) -name '*.d' 2>/dev/null)
