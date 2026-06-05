# Makefile -- Verilator build for window_slot testbench
#
#   make           build and run the testbench
#   make wave      same, then open GTKWave on the produced VCD
#   make clean     wipe build artefacts
#
# Requires verilator (>= 4.200 recommended) on PATH. Optional: gtkwave.

VERILATOR := verilator
VFLAGS    := --cc --exe --build -j 0 --trace

TOP   := window_slot
RTL   := rtl/window_slot.sv
TB    := tb/tb_window_slot.cpp
BIN   := obj_dir/V$(TOP)

.PHONY: all run wave clean help

all: run

# Verilator builds with one invocation (-build runs make for us)
$(BIN): $(RTL) $(TB)
	$(VERILATOR) $(VFLAGS) --top-module $(TOP) $(RTL) $(TB)

run: $(BIN)
	./$(BIN)

wave: run
	@if command -v gtkwave > /dev/null 2>&1; then \
	    gtkwave waves.vcd & \
	else \
	    echo "(gtkwave not installed; waveform saved to waves.vcd)"; \
	fi

clean:
	rm -rf obj_dir waves.vcd

help:
	@echo "Targets:"
	@echo "  make           build and run the testbench"
	@echo "  make wave      same, plus open GTKWave"
	@echo "  make clean     remove obj_dir/ and waves.vcd"
