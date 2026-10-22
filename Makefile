#===============================================================================
# Makefile for UART Controller Project
#
# Targets:
#   sim               - Run iverilog simulation (all configs)
#   fast-sim          - Run fast subset (115200 only)
#   veri-sim          - Run Verilator C++ testbench
#   waves             - View waveforms with GTKWave
#   clean             - Remove build artifacts
#   lint              - Run linting (requires Verilator)
#   help              - Show available targets
#
# Requirements:
#   - Icarus Verilog (iverilog) for RTL simulation
#   - GTKWave for waveform viewing (optional)
#   - Verilator for C++ testbench (optional)
#===============================================================================

# Directories
RTL_DIR     := rtl
TB_DIR      := tb
BUILD_DIR   := build
VERI_DIR    := verilator
VERI_OBJ    := obj_dir

# RTL files in COMPILATION ORDER (package FIRST, then dependents)
PKG_FILE    := $(RTL_DIR)/uart_pkg.sv
BAUD_FILE   := $(RTL_DIR)/uart_baud_gen.sv
TX_FILE     := $(RTL_DIR)/uart_tx.sv
RX_FILE     := $(RTL_DIR)/uart_rx.sv
TOP_FILE    := $(RTL_DIR)/uart_top.sv
RTL_ORDERED := $(PKG_FILE) $(BAUD_FILE) $(TX_FILE) $(RX_FILE) $(TOP_FILE)

# All RTL files for Verilator (includes Verilator wrapper)
VERI_RTL    := $(RTL_ORDERED) $(RTL_DIR)/uart_top_verilator.sv

# Compiler flags
VLOG_FLAGS  := -g2012 -Wall -Winfloop
VLOG_FLAGS  += -I $(RTL_DIR) -I $(TB_DIR)

# Simulator command
VLOG        := iverilog
VVP         := vvp

# Waveform viewer
WAVE_VIEWER := gtkwave

# Colors
RED     := \033[0;31m
GREEN   := \033[0;32m
YELLOW  := \033[0;33m
CYAN    := \033[0;36m
RESET   := \033[0m

#===============================================================================
# Targets
#===============================================================================

.PHONY: all sim fast-sim veri-sim waves clean lint help

# Create build directory
$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

# Pattern rule: compile a specific testbench (includes uart_fcov if present)
$(BUILD_DIR)/tb_uart.vvp: $(BUILD_DIR) $(RTL_ORDERED) $(TB_DIR)/tb_uart.sv $(TB_DIR)/uart_fcov.sv
	@echo "$(CYAN)Compiling tb_uart...$(RESET)"
	$(VLOG) $(VLOG_FLAGS) -o $@ $(RTL_ORDERED) $(TB_DIR)/uart_fcov.sv $(TB_DIR)/tb_uart.sv
	@echo "$(GREEN)Compilation successful: $@$(RESET)"

$(BUILD_DIR)/%.vvp: $(BUILD_DIR) $(RTL_ORDERED) $(TB_DIR)/%.sv
	@echo "$(CYAN)Compiling $*...$(RESET)"
	$(VLOG) $(VLOG_FLAGS) -o $@ $(RTL_ORDERED) $(TB_DIR)/$*.sv
	@echo "$(GREEN)Compilation successful: $@$(RESET)"

#===============================================================================
# Full simulation (all baud rates, all configs)
#===============================================================================
sim: $(BUILD_DIR)/tb_uart.vvp
	@echo "$(CYAN)Running full UART simulation...$(RESET)"
	$(VVP) $<
	@echo "$(GREEN)Simulation complete.$(RESET)"

#===============================================================================
# Parity error detection test (direct RX drive)
#===============================================================================
parity-sim: $(BUILD_DIR)/tb_uart_parity.vvp
	@echo "$(CYAN)Running parity error test (direct RX)...$(RESET)"
	$(VVP) $<
	@echo "$(GREEN)Parity test complete.$(RESET)"

#===============================================================================
# Loopback parity error injection test (full uart_top path)
#===============================================================================
loopback-parity-sim: $(BUILD_DIR)/tb_uart_parity_loopback.vvp
	@echo "$(CYAN)Running loopback parity error test...$(RESET)"
	$(VVP) $<
	@echo "$(GREEN)Loopback parity test complete.$(RESET)"

#===============================================================================
# Break detection test
#===============================================================================
break-sim: $(BUILD_DIR)/tb_uart_break.vvp
	@echo "$(CYAN)Running break detection test...$(RESET)"
	$(VVP) $<
	@echo "$(GREEN)Break detection test complete.$(RESET)"

#===============================================================================
# Interrupt test
#===============================================================================
intr-sim: $(BUILD_DIR)/tb_uart_intr.vvp
	@echo "$(CYAN)Running interrupt test...$(RESET)"
	$(VVP) $<
	@echo "$(GREEN)Interrupt test complete.$(RESET)"

#===============================================================================
# Fast simulation (115200 baud only)
#===============================================================================
fast-sim: $(BUILD_DIR)/tb_uart_fast.vvp
	@echo "$(CYAN)Running fast UART simulation...$(RESET)"
	$(VVP) $<
	@echo "$(GREEN)Fast simulation complete.$(RESET)"

#===============================================================================
# Verilator C++ simulation
#===============================================================================
veri-sim:
	@# Clean stale obj_dir to avoid confusion after RTL/testbench edits
	rm -rf $(VERI_OBJ)
	@echo "$(CYAN)Building Verilator simulation...$(RESET)"
	# --timing omitted intentionally: RTL uses only standard posedge/negedge
	# always_ff patterns that Verilator handles correctly without it.
	verilator --cc --exe --build -j 4 --trace \
		--top-module uart_top_verilator \
		-Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-SYNCASYNCNET \
		-Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-fatal \
		-CFLAGS "-std=c++17 -O0 -g" \
		$(VERI_RTL) \
		$(VERI_DIR)/testbench.cpp 2>&1 | tail -20
	@echo "$(GREEN)Verilator build complete. Running simulation...$(RESET)"
	./$(VERI_OBJ)/Vuart_top_verilator
	@echo "$(GREEN)Verilator simulation complete.$(RESET)"

veri-clean:
	@echo "$(CYAN)Cleaning Verilator artifacts...$(RESET)"
	rm -rf $(VERI_OBJ)
	rm -f $(VERI_DIR)/*.vcd

#===============================================================================
# Run all testbenches (except tb_uart full sim which is slow)
#===============================================================================
quick-check: fast-sim parity-sim loopback-parity-sim break-sim lint
	@echo "$(GREEN)All quick checks passed!$(RESET)"

#===============================================================================
# View waveforms with GTKWave
#===============================================================================
waves: tb_uart.vcd
	@if command -v $(WAVE_VIEWER) > /dev/null 2>&1; then \
		$(WAVE_VIEWER) $<; \
	else \
		echo "$(YELLOW)GTKWave not found. Install it with:$(RESET)"; \
		echo "  sudo apt-get install gtkwave"; \
		echo "  brew install gtkwave"; \
	fi

parity-waves: tb_uart_parity.vcd
	@if command -v $(WAVE_VIEWER) > /dev/null 2>&1; then \
		$(WAVE_VIEWER) $<; \
	else \
		echo "$(YELLOW)GTKWave not found. Install it with:$(RESET)"; \
		echo "  sudo apt-get install gtkwave"; \
		echo "  brew install gtkwave"; \
	fi

loopback-parity-waves: tb_uart_parity_loopback.vcd
	@if command -v $(WAVE_VIEWER) > /dev/null 2>&1; then \
		$(WAVE_VIEWER) $<; \
	else \
		echo "$(YELLOW)GTKWave not found. Install it with:$(RESET)"; \
		echo "  sudo apt-get install gtkwave"; \
		echo "  brew install gtkwave"; \
	fi

#===============================================================================
# View Verilator waveforms
#===============================================================================
veri-waves: tb_uart_verilator.vcd
	@if command -v $(WAVE_VIEWER) > /dev/null 2>&1; then \
		$(WAVE_VIEWER) $<; \
	else \
		echo "$(YELLOW)GTKWave not found.$(RESET)"; \
	fi

#===============================================================================
# Lint (requires Verilator)
#===============================================================================
lint:
	@if command -v verilator > /dev/null 2>&1; then \
		echo "$(CYAN)Running Verilator lint...$(RESET)"; \
		verilator --lint-only -Wall -I$(RTL_DIR) $(RTL_ORDERED) 2>&1; \
	else \
		echo "$(YELLOW)Verilator not found. Install it for lint support.$(RESET)"; \
	fi

#===============================================================================
# Clean build artifacts
#===============================================================================
clean:
	@echo "$(CYAN)Cleaning build artifacts...$(RESET)"
	rm -rf $(BUILD_DIR)
	rm -f *.vcd
	rm -f *.lxt *.lxt2 *.vpd
	rm -rf $(VERI_OBJ)
	rm -f $(VERI_DIR)/*.vcd
	@echo "$(GREEN)Clean complete.$(RESET)"

#===============================================================================
# Coverage simulation (includes functional coverage monitor)
#===============================================================================
coverage-sim: $(BUILD_DIR)/tb_uart.vvp
	@echo "$(CYAN)Running UART simulation with functional coverage...$(RESET)"
	$(VVP) $<
	@echo "$(GREEN)Coverage simulation complete. See FCOV summary above.$(RESET)"

#===============================================================================
# Help
#===============================================================================
help:
	@echo "UART Controller Project - Makefile"
	@echo ""
	@echo "Usage: make <target>"
	@echo ""
	@echo "Targets:"
	@echo "  sim               Run iverilog simulation (all configs)"
	@echo "  coverage-sim      Run simulation with functional coverage"
	@echo "  parity-sim        Run parity error test (direct RX)"
	@echo "  loopback-parity-sim  Run loopback parity error test (full path)"
	@echo "  break-sim         Run break detection test"
	@echo "  quick-check       Run fast tests + lint (non-blocking check)"
	@echo "  intr-sim          Run interrupt test"
	@echo "  veri-sim          Run Verilator C++ testbench"
	@echo "  waves             View iverilog waveforms (requires GTKWave)"
	@echo "  parity-waves      View direct-RX parity test waveform"
	@echo "  loopback-parity-waves  View loopback parity test waveform"
	@echo "  veri-waves        View Verilator waveforms (requires GTKWave)"
	@echo "  lint              Run lint check (requires Verilator)"
	@echo "  clean             Remove build artifacts"
	@echo "  help              Show this help"
