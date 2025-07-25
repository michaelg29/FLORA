
ifndef GUROBI_HOME
$(error You must set GUROBI_HOME to match your installation of Gurobi to use this Makefile. Also make sure that LD_LIBRARY_PATH points to the library files with the Gurobi installation.)
endif

CC = g++
CFLAGS = -Iflora/include/
CFLAGS += -Iinclude/
CFLAGS += -I$(GUROBI_HOME)/include
CFLAGS += -std=c++11
CFLAGS += -L$(GUROBI_HOME)/lib
#CFLAGS += -ggdb -g3

LDFLAGS = -lgurobi_g++4.2 -lgurobi_c++ -lgurobi91 -lm

all:
	@echo "Please run the make file using the following format"
	@echo ""
	@echo "make flora FPGA=type_of_FPGA"
	@echo "for type of FPGA please use VC707, VCU118 or VCU128"
	@echo " "
	@echo "For example make flora FPGA=VC707"


SOURCES_SHARED = src/csv_data_manipulator.cpp include/fpga.h flora/src/main.cpp

ifeq ($(FPGA),PYNQ)
SOURCES_SHARED += include/pynq.h src/pynq.cpp include/pynq_fine_grained.h src/pynq_fine_grained.cpp
CFLAGS += -DFPGA_PYNQ
else ifeq ($(FPGA),ZYNQ)
SOURCES_SHARED += include/zynq.h src/zynq.cpp include/zynq_fine_grained.h src/zynq_fine_grained.cpp
CFLAGS += -DFPGA_ZYNQ
else ifeq ($(FPGA),VC707)
SOURCES_SHARED += include/vc707.h src/vc707.cpp include/vc707_fine_grained.h src/vc707_fine_grained.cpp
CFLAGS += -DFPGA_VC707
else ifeq ($(FPGA),VCU118)
SOURCES_SHARED += include/vcu118.h src/vcu118.cpp include/vcu118_fine_grained.h src/vcu118_fine_grained.cpp
CFLAGS += -DFPGA_VCU118
else ifeq ($(FPGA),VCU128)
SOURCES_SHARED += include/vcu128.h src/vcu128.cpp include/vcu128_fine_grained.h src/vcu128_fine_grained.cpp
CFLAGS += -DFPGA_VCU128
else
SOURCES_SHARED += include/zynq.h src/zynq.cpp include/zynq_fine_grained.h src/zynq_fine_grained.cpp
CFLAGS += -DFPGA_ZYNQ
endif

CFLAGS += -DFLORA_CORRECTED
CFLAGS += -DFLORA_AUTOGEN_CONSTR

ifeq ($(FPGA),VC707)
flora: SOURCES_MILP = src/milp_model_vc707.cpp
else ifeq ($(FPGA),VCU118)
flora: SOURCES_MILP = src/milp_model_vcu118.cpp
else ifeq ($(FPGA),VCU128)
flora: SOURCES_MILP = src/milp_model_vcu128.cpp
endif

flora: SOURCES += flora/src/flora.cpp
flora: BIN = flora
flora: CFLAGS += -DRUN_FLORA
flora: build

build:
	mkdir -p bin
	$(CC) -o bin/$(BIN) $(CFLAGS) $(SOURCES_SHARED) $(SOURCES_MILP) $(SOURCES) $(LDFLAGS)

.PHONY: clean
clean:
	rm -f bin/*run*
