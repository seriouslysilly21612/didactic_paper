##############################################################################
#	Makefile of the humanoid in simulation (paper, Sec. VI)
#
#	make                  bin/humanoid_sim
#	make clean
#
#	It reuses the high-level controller and the real-time task of the arm
#	(../indy7/src: HighLevelRT, base/RtTask, CallTiming.h) and RT-POSIX
#	(../third_party/rt-posix). The dependencies (paper, Table 1): RBDL with
#	its URDF reader (RBDL_DIR), Ruckig (RUCKIG_DIR), Eigen 3, MuJoCo
#	(MUJOCO_DIR).
#	The models (models/README.md):
#	    python3 tools/g1_models.py <g1_29dof.urdf> models
#	    python3 ../tools/urdf2mjcf.py models/g1_fixed.urdf models/g1_mujoco.xml --verify
##############################################################################

ROOT_DIR    := ..
INDY7_SRC   := $(ROOT_DIR)/indy7/src
RTPOSIX_DIR := $(ROOT_DIR)/third_party/rt-posix
OBJ_DIR     := obj
BIN_DIR     := bin

RBDL_DIR    ?= /usr/local
RUCKIG_DIR  ?= /usr/local
EIGEN_DIR   ?= /usr/include/eigen3
MUJOCO_DIR  ?= /opt/mujoco

CXX ?= g++
CC  ?= gcc
CXXFLAGS := -O2 -std=gnu++17 -Wall -fexceptions -D_GNU_SOURCE
CFLAGS   := -O2 -Wall -D_GNU_SOURCE
INCLUDES := -I$(INDY7_SRC) -I$(RTPOSIX_DIR)/include -isystem $(EIGEN_DIR) -isystem $(RBDL_DIR)/include \
            -isystem $(RUCKIG_DIR)/include -isystem $(MUJOCO_DIR)/include
LDLIBS   := -L$(RBDL_DIR)/lib -lrbdl -lrbdl_urdfreader -Wl,-rpath,$(RBDL_DIR)/lib \
            -L$(RUCKIG_DIR)/lib -lruckig -Wl,-rpath,$(RUCKIG_DIR)/lib \
            -L$(MUJOCO_DIR)/lib -lmujoco -Wl,-rpath,$(MUJOCO_DIR)/lib \
            -lm -lrt -lpthread

OBJECTS := $(OBJ_DIR)/HumanoidSim.o $(OBJ_DIR)/HighLevelRT.o $(OBJ_DIR)/RtTask.o \
           $(OBJ_DIR)/posix_rt.o $(OBJ_DIR)/commons.o

all: $(BIN_DIR)/humanoid_sim

$(BIN_DIR)/humanoid_sim: $(OBJECTS)
	@mkdir -p $(BIN_DIR)
	$(CXX) -o $@ $(OBJECTS) $(LDLIBS)

$(OBJ_DIR)/HumanoidSim.o: src/HumanoidSim.cpp
	@mkdir -p $(OBJ_DIR)
	$(CXX) -MD -MP $(CXXFLAGS) $(INCLUDES) -c -o $@ $<

$(OBJ_DIR)/HighLevelRT.o: $(INDY7_SRC)/HighLevelRT.cpp
	@mkdir -p $(OBJ_DIR)
	$(CXX) -MD -MP $(CXXFLAGS) $(INCLUDES) -c -o $@ $<

$(OBJ_DIR)/RtTask.o: $(INDY7_SRC)/base/RtTask.cpp
	@mkdir -p $(OBJ_DIR)
	$(CXX) -MD -MP $(CXXFLAGS) $(INCLUDES) -c -o $@ $<

$(OBJ_DIR)/%.o: $(RTPOSIX_DIR)/src/%.c
	@mkdir -p $(OBJ_DIR)
	$(CC) -MD -MP $(CFLAGS) -I$(RTPOSIX_DIR)/include -c -o $@ $<

clean:
	rm -rf $(OBJ_DIR) $(BIN_DIR)

.PHONY: all clean
-include $(wildcard $(OBJ_DIR)/*.d)
