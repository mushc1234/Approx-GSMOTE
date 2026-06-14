# Unified Makefile -- ann_wrapper RTL and sigwin engine testbenches
#
# Target quick-reference:
#   make                   build + run all testbenches (wrapper + engine)
#   make engine_all        build + run all engine testbenches
#   make wrapper_all       build + run wrapper integration and result_store
#   make <module_name>     build + run a specific module
#   make wave_<name>       run a test, open VCD in gtkwave
#   make lint              lint-only check of the top-level wrapper
#   make clean             wipe build artefacts

VERILATOR := verilator
# Combined VFLAGS: includes trace from both, and lint suppressions from wrapper
VFLAGS    := --cc --exe --build -j 1 --trace -Wno-DECLFILENAME -Wno-PINCONNECTEMPTY

# Default parameter overrides
PARAMS := -GW=8 -GD=8 -GN_MAX_LOG2=12
GOLDEN ?= ./golden
ENG_GOLDEN ?= $(GOLDEN)

# ----------------------------------------------------------------------
# File Lists
# ----------------------------------------------------------------------

WRAPPER_RTL := \
    rtl_wrapper/ann_wrapper.sv \
    rtl_wrapper/axi_lite_regs.sv \
    rtl_wrapper/result_store.sv \
    rtl_wrapper/result_bram.sv \
    rtl_wrapper/clear_fsm.sv \
    rtl_wrapper/harvest_fsm.sv \
    rtl_wrapper/axi_master_read.sv \
    rtl_wrapper/record_splitter.sv \
    rtl_wrapper/feeder_fsm.sv \
    rtl_wrapper/run_fsm.sv
ENGINE_RTL := \
    rtl_engine/window_engine.sv \
    rtl_engine/window_slot.sv \
    rtl_engine/distance_unit.sv \
    rtl_engine/slot_scheduler.sv \
    rtl_engine/arrival_aggregator.sv

# ----------------------------------------------------------------------
# Phony Targets
# ----------------------------------------------------------------------

.PHONY: all engine_all wrapper_all clean help lint \
        window_slot distance_unit slot_scheduler window_engine arrival_aggregator \
        result_store wrapper \
        wave_window_slot wave_distance_unit wave_slot_scheduler wave_window_engine

# Default target builds everything
all: engine_all wrapper_all

engine_all: window_slot distance_unit slot_scheduler window_engine arrival_aggregator
wrapper_all: result_store wrapper

# ----------------------------------------------------------------------
# Wrapper & Integration Build Rules
# ----------------------------------------------------------------------

obj_dir_result_store/Vresult_store: \
        rtl_wrapper/result_store.sv rtl_wrapper/result_bram.sv rtl_wrapper/clear_fsm.sv rtl_wrapper/harvest_fsm.sv \
        tb/tb_result_store.cpp
	$(VERILATOR) $(VFLAGS) --top-module result_store \
	    -GN_MAX_LOG2=6 \
	    --Mdir obj_dir_result_store \
	    rtl_wrapper/result_store.sv rtl_wrapper/result_bram.sv rtl_wrapper/clear_fsm.sv rtl_wrapper/harvest_fsm.sv \
	    tb/tb_result_store.cpp

result_store: obj_dir_result_store/Vresult_store
	./obj_dir_result_store/Vresult_store

obj_dir_wrapper/Vann_wrapper: $(WRAPPER_RTL) $(ENGINE_RTL) tb/tb_ann_wrapper.cpp
	$(VERILATOR) $(VFLAGS) --top-module ann_wrapper \
	    $(PARAMS) \
	    --Mdir obj_dir_wrapper \
	    $(WRAPPER_RTL) $(ENGINE_RTL) \
	    tb/tb_ann_wrapper.cpp

wrapper: obj_dir_wrapper/Vann_wrapper
	./obj_dir_wrapper/Vann_wrapper

# ----------------------------------------------------------------------
# Engine Components Build Rules
# ----------------------------------------------------------------------

obj_dir_window_slot/Vwindow_slot: \
		rtl_engine/window_slot.sv rtl_engine/distance_unit.sv \
		tb/tb_window_slot.cpp
	$(VERILATOR) $(VFLAGS) --top-module window_slot \
		--Mdir obj_dir_window_slot \
		rtl_engine/window_slot.sv rtl_engine/distance_unit.sv \
		tb/tb_window_slot.cpp

window_slot: obj_dir_window_slot/Vwindow_slot
	./obj_dir_window_slot/Vwindow_slot

obj_dir_distance_unit/Vdistance_unit: rtl_engine/distance_unit.sv tb/tb_distance_unit.cpp
	$(VERILATOR) $(VFLAGS) --top-module distance_unit \
		--Mdir obj_dir_distance_unit \
		rtl_engine/distance_unit.sv tb/tb_distance_unit.cpp

distance_unit: obj_dir_distance_unit/Vdistance_unit
	./obj_dir_distance_unit/Vdistance_unit --golden $(GOLDEN)

obj_dir_slot_scheduler/Vslot_scheduler: rtl_engine/slot_scheduler.sv tb/tb_slot_scheduler.cpp
	$(VERILATOR) $(VFLAGS) --top-module slot_scheduler \
		-GW=4 -GLATENCY=5 -GD=2 \
		--Mdir obj_dir_slot_scheduler \
		rtl_engine/slot_scheduler.sv tb/tb_slot_scheduler.cpp

slot_scheduler: obj_dir_slot_scheduler/Vslot_scheduler
	./obj_dir_slot_scheduler/Vslot_scheduler
 
obj_dir_window_engine/Vwindow_engine: rtl_engine/window_engine.sv rtl_engine/window_slot.sv \
		rtl_engine/distance_unit.sv rtl_engine/slot_scheduler.sv tb/tb_window_engine.cpp
	$(VERILATOR) $(VFLAGS) --top-module window_engine \
		-GW=4 -GD=8 \
		--Mdir obj_dir_window_engine \
		rtl_engine/window_engine.sv rtl_engine/window_slot.sv \
		rtl_engine/distance_unit.sv rtl_engine/slot_scheduler.sv \
		rtl_engine/arrival_aggregator.sv \
		tb/tb_window_engine.cpp

window_engine: obj_dir_window_engine/Vwindow_engine
	./obj_dir_window_engine/Vwindow_engine --golden $(ENG_GOLDEN)

obj_dir_arrival_aggregator/Varrival_aggregator: \
		rtl_engine/arrival_aggregator.sv tb/tb_arrival_aggregator.cpp
	$(VERILATOR) $(VFLAGS) --top-module arrival_aggregator \
		-GW=4 -GACC_WIDTH=40 -GTAG_WIDTH=32 \
		--Mdir obj_dir_arrival_aggregator \
		rtl_engine/arrival_aggregator.sv tb/tb_arrival_aggregator.cpp

arrival_aggregator: obj_dir_arrival_aggregator/Varrival_aggregator
	./obj_dir_arrival_aggregator/Varrival_aggregator

# ----------------------------------------------------------------------
# Waveform viewing
# ----------------------------------------------------------------------

wave_window_slot: window_slot
	@if command -v gtkwave > /dev/null 2>&1; then \
	    gtkwave waves.vcd & \
	else \
	    echo "(gtkwave not installed; waveform at waves.vcd)"; \
	fi

wave_distance_unit: distance_unit
	@if command -v gtkwave > /dev/null 2>&1; then \
	    gtkwave waves_distance_unit.vcd & \
	else \
	    echo "(gtkwave not installed; waveform at waves_distance_unit.vcd)"; \
	fi

# Add additional wave_ targets here if needed for scheduler/engine

# ----------------------------------------------------------------------
# Linting & Utility
# ----------------------------------------------------------------------

lint:
	$(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME -Wno-PINCONNECTEMPTY \
	    --top-module ann_wrapper $(PARAMS) \
	    $(WRAPPER_RTL) $(ENGINE_RTL)

clean:
	rm -rf obj_dir_* waves*.vcd

help:
	@echo "Targets:"
	@echo "  make                  build + run all testbenches (wrapper + engine)"
	@echo "  make engine_all       build + run all engine testbenches"
	@echo "  make wrapper_all      build + run integration test (full wrapper) & result_store"
	@echo "  make <module>         build + run specific test (e.g., make window_slot)"
	@echo "  make lint             lint-only check of wrapper + engine"
	@echo "  make wave_<name>      run a test, open VCD in gtkwave (e.g., make wave_distance_unit)"
	@echo "  make clean            remove all build artefacts"
	@echo ""
	@echo "Options:"
	@echo "  GOLDEN=path/to/golden point distance_unit at a different golden dir (default: ./golden)"