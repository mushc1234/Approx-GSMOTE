# Makefile -- Verilator builds for sigwin engine testbenches
#
#   make                           build + run all testbenches
#   make window_slot               build + run only tb_window_slot
#   make distance_unit             build + run only tb_distance_unit
#   make wave_<name>               run, then open the VCD in gtkwave
#   make clean                     wipe build artefacts
#
#   make distance_unit GOLDEN=path/to/golden    # override golden dir
#
# Requires verilator (>= 4.200) on PATH. Optional: gtkwave for waves.

VERILATOR := verilator
VFLAGS    := --cc --exe --build -j 0 --trace

GOLDEN ?= ./golden

.PHONY: all clean help \
        window_slot distance_unit \
        wave_window_slot wave_distance_unit

all: window_slot distance_unit

# ----------------------------------------------------------------------
# Per-testbench build rules. Each gets its own obj_dir so they don't
# stomp on each other.
# ----------------------------------------------------------------------

obj_dir_window_slot/Vwindow_slot: \
		rtl/window_slot.sv rtl/distance_unit.sv \
		tb/tb_window_slot.cpp
	$(VERILATOR) $(VFLAGS) --top-module window_slot \
		--Mdir obj_dir_window_slot \
		rtl/window_slot.sv rtl/distance_unit.sv \
		tb/tb_window_slot.cpp


obj_dir_distance_unit/Vdistance_unit: rtl/distance_unit.sv tb/tb_distance_unit.cpp
	$(VERILATOR) $(VFLAGS) --top-module distance_unit \
		--Mdir obj_dir_distance_unit \
		rtl/distance_unit.sv tb/tb_distance_unit.cpp

window_slot: obj_dir_window_slot/Vwindow_slot
	./obj_dir_window_slot/Vwindow_slot

distance_unit: obj_dir_distance_unit/Vdistance_unit
	./obj_dir_distance_unit/Vdistance_unit --golden $(GOLDEN)

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

# ----------------------------------------------------------------------
# Utility
# ----------------------------------------------------------------------

clean:
	rm -rf obj_dir_* waves*.vcd

help:
	@echo "Targets:"
	@echo "  make                   build + run all testbenches"
	@echo "  make window_slot       just the window slot test"
	@echo "  make distance_unit     just the distance unit test, golden vectors generated seperately"
	@echo "  make wave_<name>       run a test, open VCD in gtkwave"
	@echo "  make clean             remove all build artefacts"
	@echo ""
	@echo "  GOLDEN=path/to/golden  point distance_unit at a different"
	@echo "                          golden directory (default: ./golden)"
