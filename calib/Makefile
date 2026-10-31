##############################################################################
#	Makefile of calib/save_camera_params (optional; the calibration of the
#	paper, Sec. V-C, uses the Python programs of this directory)
#
#	make VISP_DIR=<the build directory of ViSP> [RS2_DIR=/usr/local]
#	The source modules of ViSP are taken from $(VISP_SRC) (default:
#	$(VISP_DIR)/../modules); with an installed ViSP, set
#	VISP_INCLUDE=<prefix>/include and VISP_LIB=<prefix>/lib instead.
##############################################################################
VISP_DIR     ?= $(HOME)/visp/build
VISP_SRC     ?= $(VISP_DIR)/../modules
VISP_LIB     ?= $(VISP_DIR)/lib
VISP_INCLUDE ?= -I$(VISP_DIR)/include -I$(VISP_SRC)/core/include -I$(VISP_SRC)/sensor/include \
                -I$(VISP_SRC)/io/include
RS2_DIR      ?= /usr/local

CXXFLAGS := -O2 -std=c++17 $(VISP_INCLUDE) -I$(RS2_DIR)/include -isystem /usr/include/eigen3 \
            -isystem /usr/include/opencv4
LDLIBS   := -L$(VISP_LIB) -lvisp_core -lvisp_sensor -lvisp_io -Wl,-rpath,$(VISP_LIB) \
            -L$(RS2_DIR)/lib -lrealsense2

all: save_camera_params

save_camera_params: save_camera_params.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDLIBS)

clean:
	rm -f save_camera_params

.PHONY: all clean
