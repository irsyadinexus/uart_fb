//==============================================================================
// testbench.cpp — Verilator C++ Testbench for UART Controller
//
// Drives the UART controller with loopback (TX connected to RX) and validates
// all configuration combinations: data bits (5-8), parity (none/even/odd),
// stop bits (1/1.5/2), and baud rate divisors.
//
// Tests three data patterns per configuration:
//   - All 0s (0x00)
//   - All 1s (0xFF masked to data bits)
//   - Random data (LFSR-based)
//
// Build (from project root):
//   verilator --cc --exe --build -j 4 --timing --trace \
//     --top-module uart_top_verilator \
//     -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-SYNCASYNCNET \
//     -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-fatal \
//     -CFLAGS "-std=c++17 -O0 -g" \
//     rtl/uart_pkg.sv rtl/uart_baud_gen.sv rtl/uart_tx.sv \
//     rtl/uart_rx.sv rtl/uart_top.sv rtl/uart_top_verilator.sv \
//     verilator/testbench.cpp
//
// Run: ./obj_dir/Vuart_top_verilator
//==============================================================================

#include <verilated.h>
#include <verilated_vcd_c.h>
#include "Vuart_top_verilator.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

//==============================================================================
// Configuration
//==============================================================================

constexpr int CLK_FREQ     = 50'000'000;
constexpr int CLK_PERIOD_NS = 20;    // 50 MHz
constexpr int MAX_SIM_NS   = 50'000'000;  // 50 ms timeout (~5M clock cycles)

//==============================================================================
// LFSR for pseudo-random test data
//==============================================================================
class LFSR {
  uint32_t state;
public:
  LFSR(uint32_t seed = 0xABCDEF01) : state(seed) {}
  uint8_t next8() {
    uint8_t val = state & 0xFF;
    for (int i = 0; i < 8; i++) {
      uint32_t bit = (state ^ (state >> 2) ^ (state >> 3) ^ (state >> 5)) & 1;
      state = (state >> 1) | (bit << 31);
    }
    return val;
  }
};

//==============================================================================
// Test Configuration
//==============================================================================
struct UartConfig {
  const char*  name;
  uint16_t     baud_div;
  uint8_t      data_bits;    // 0=5, 1=6, 2=7, 3=8
  uint8_t      parity;       // 0=none, 1=even, 2=odd
  uint8_t      stop_bits;    // 0=1, 1=1.5, 2=2
};

// Test all 36 frame configurations at 115200 baud
const UartConfig kConfigs[] = {
  // name               baud_div     data  parity  stop
  {"115200_5N1",  434,  0, 0, 0},
  {"115200_5N1.5",434,  0, 0, 1},
  {"115200_5N2",  434,  0, 0, 2},
  {"115200_5E1",  434,  0, 1, 0},
  {"115200_5E1.5",434,  0, 1, 1},
  {"115200_5E2",  434,  0, 1, 2},
  {"115200_5O1",  434,  0, 2, 0},
  {"115200_5O1.5",434,  0, 2, 1},
  {"115200_5O2",  434,  0, 2, 2},
  {"115200_6N1",  434,  1, 0, 0},
  {"115200_6N1.5",434,  1, 0, 1},
  {"115200_6N2",  434,  1, 0, 2},
  {"115200_6E1",  434,  1, 1, 0},
  {"115200_6E1.5",434,  1, 1, 1},
  {"115200_6E2",  434,  1, 1, 2},
  {"115200_6O1",  434,  1, 2, 0},
  {"115200_6O1.5",434,  1, 2, 1},
  {"115200_6O2",  434,  1, 2, 2},
  {"115200_7N1",  434,  2, 0, 0},
  {"115200_7N1.5",434,  2, 0, 1},
  {"115200_7N2",  434,  2, 0, 2},
  {"115200_7E1",  434,  2, 1, 0},
  {"115200_7E1.5",434,  2, 1, 1},
  {"115200_7E2",  434,  2, 1, 2},
  {"115200_7O1",  434,  2, 2, 0},
  {"115200_7O1.5",434,  2, 2, 1},
  {"115200_7O2",  434,  2, 2, 2},
  {"115200_8N1",  434,  3, 0, 0},
  {"115200_8N1.5",434,  3, 0, 1},
  {"115200_8N2",  434,  3, 0, 2},
  {"115200_8E1",  434,  3, 1, 0},
  {"115200_8E1.5",434,  3, 1, 1},
  {"115200_8E2",  434,  3, 1, 2},
  {"115200_8O1",  434,  3, 2, 0},
  {"115200_8O1.5",434,  3, 2, 1},
  {"115200_8O2",  434,  3, 2, 2},
  // Spot-check at 57600 and 9600
  {"57600_8N1",   868,  3, 0, 0},
  {"57600_8E1",   868,  3, 1, 0},
  {"57600_8O2",   868,  3, 2, 2},
  {"9600_8N1",   5208, 3, 0, 0},
  {"9600_8E1",   5208, 3, 1, 0},
  {"9600_8O2",   5208, 3, 2, 2},
};

constexpr int NUM_CONFIGS = sizeof(kConfigs) / sizeof(kConfigs[0]);

constexpr int DATA_PATTERNS = 3;  // all-0, all-1, random
constexpr int BYTES_PER_PATTERN = 8;

//==============================================================================
// Forward declarations
//==============================================================================

struct SimState;
void sim_tick(SimState& s);

//==============================================================================
// Simulation State
//==============================================================================
struct SimState {
  Vuart_top_verilator* top;
  VerilatedVcdC*       trace;
  uint64_t             tick_count;
  uint64_t             sim_time_ps;

  // Test tracking
  int                  total_tests;
  int                  passed_tests;
  int                  failed_tests;
};

//==============================================================================
// Helper Functions
//==============================================================================

void sim_init(SimState& s) {
  s.top         = new Vuart_top_verilator;
  s.tick_count  = 0;
  s.sim_time_ps = 0;
  s.total_tests = 0;
  s.passed_tests = 0;
  s.failed_tests = 0;

  // Enable waveform tracing
  Verilated::traceEverOn(true);
  s.trace = new VerilatedVcdC;
  s.top->trace(s.trace, 99);
  s.trace->open("tb_uart_verilator.vcd");

  // Initialize all inputs
  s.top->clk            = 0;
  s.top->rst_n          = 0;
  s.top->cfg_baud_div   = 434;    // default 115200
  s.top->cfg_data_bits  = 3;      // 8 data bits
  s.top->cfg_parity     = 0;      // none
  s.top->cfg_stop_bits  = 0;      // 1 stop bit
  s.top->tx_data        = 0;
  s.top->tx_valid       = 0;
  s.top->rx_ready       = 1;
  s.top->clr_errors     = 0;
  s.top->break_detect   = 0;  // output, but Verilator requires initialization
  s.top->irq            = 0;
  s.top->int_en_rx      = 0;
  s.top->int_en_tx      = 0;
  s.top->int_en_err     = 0;

  // Reset
  for (int i = 0; i < 20; i++) sim_tick(s);
  s.top->rst_n = 1;
  for (int i = 0; i < 20; i++) sim_tick(s);
}

void sim_tick(SimState& s) {
  // Loopback is handled in uart_top_verilator.sv via continuous assign
  // (wire rxd_loop = txd). The C++ rxd drive is unused in loopback mode.
  s.top->clk = 0;
  s.top->eval();
  if (s.trace) s.trace->dump(s.sim_time_ps);
  s.sim_time_ps += CLK_PERIOD_NS * 500;

  s.top->clk = 1;
  s.top->eval();
  if (s.trace) s.trace->dump(s.sim_time_ps);
  s.sim_time_ps += CLK_PERIOD_NS * 500;

  s.tick_count++;
}

void sim_wait_ticks(SimState& s, uint64_t ticks) {
  for (uint64_t i = 0; i < ticks; i++) sim_tick(s);
}

void sim_wait_ns(SimState& s, uint64_t ns) {
  uint64_t ticks_needed = (ns + CLK_PERIOD_NS - 1) / CLK_PERIOD_NS;
  sim_wait_ticks(s, ticks_needed);
}

void configure_uart(SimState& s, const UartConfig& cfg) {
  // Pulse reset to clear any stale state (FSM, counters, flags)
  // before changing configuration. This ensures clean state for
  // each test configuration.
  s.top->rst_n = 0;
  sim_wait_ticks(s, 10);
  s.top->rst_n = 1;

  s.top->cfg_baud_div  = cfg.baud_div;
  s.top->cfg_data_bits = cfg.data_bits;
  s.top->cfg_parity    = cfg.parity;
  s.top->cfg_stop_bits = cfg.stop_bits;
  sim_wait_ticks(s, 10);  // Let configuration settle

  // Ensure rx_ready is high for the first byte
  s.top->rx_ready = 1;
}

//==============================================================================
// Send a byte and wait for it to be received (loopback)
// Returns true on success
//==============================================================================
bool send_and_check(SimState& s, uint8_t tx_val, uint8_t expected_rx_val) {
  // Wait until UART is ready to transmit
  int timeout = 1000000;
  while (!s.top->tx_ready && timeout > 0) {
    sim_tick(s);
    timeout--;
  }
  if (timeout <= 0) {
    printf("  TIMEOUT waiting for tx_ready\n");
    return false;
  }
  // Send byte
  s.top->tx_data  = tx_val;
  s.top->tx_valid = 1;
  sim_tick(s);
  s.top->tx_valid = 0;

  // Ensure rx_ready is high so the RX can present received data
  // and also so the valid/ready handshake can clear rx_valid after
  // each reception. Keeping rx_ready high continuously avoids
  // stale rx_valid leaking into the next byte's check.
  s.top->rx_ready = 1;

  // Wait for rx_valid
  timeout = 1000000;
  while (timeout > 0) {
    sim_tick(s);
    timeout--;

    // Check for received byte
    if (s.top->rx_valid) {
      uint8_t rx_val = s.top->rx_data;

      // RX data is right-aligned — directly comparable.
      bool match = (rx_val == expected_rx_val);

      // Drain rx_valid via handshake.
      for (int i = 0; i < 10; i++) sim_tick(s);

      return match;
    }
  }
  printf("  TIMEOUT waiting for rx_valid (tx_val=0x%02x)\n", tx_val);
  return false;
}

//==============================================================================
// Test a configuration with all-0, all-1, and random data patterns
//==============================================================================
bool test_config(SimState& s, const UartConfig& cfg) {
  printf("\n  ── Testing %s ──\n", cfg.name);

  configure_uart(s, cfg);

  int num_data_bits = 5 + (cfg.data_bits > 3 ? 3 : cfg.data_bits);
  uint8_t max_val = (1 << num_data_bits) - 1;
  bool all_pass = true;

  // --- All-0s pattern ---
  printf("    All-0s: ");
  bool zero_pass = true;
  for (int i = 0; i < BYTES_PER_PATTERN; i++) {
    if (!send_and_check(s, 0x00, 0x00)) {
      zero_pass = false;
      printf("FAIL[%d] ", i);
    }
  }
  printf("%s\n", zero_pass ? "PASS" : "FAIL");
  all_pass &= zero_pass;

  // --- All-1s pattern ---
  printf("    All-1s: ");
  bool one_pass = true;
  for (int i = 0; i < BYTES_PER_PATTERN; i++) {
    uint8_t tx_val = (num_data_bits < 8) ? max_val : 0xFF;
    if (!send_and_check(s, tx_val, tx_val)) {
      one_pass = false;
      printf("FAIL[%d] ", i);
    }
  }
  printf("%s\n", one_pass ? "PASS" : "FAIL");
  all_pass &= one_pass;

  // --- Random pattern ---
  LFSR lfsr;
  printf("    Random: ");
  bool rand_pass = true;
  for (int i = 0; i < BYTES_PER_PATTERN; i++) {
    uint8_t tx_val = lfsr.next8() & max_val;
    if (!send_and_check(s, tx_val, tx_val)) {
      rand_pass = false;
      printf("FAIL[%d] ", i);
    }
  }
  printf("%s\n", rand_pass ? "PASS" : "FAIL");
  all_pass &= rand_pass;

  printf("  ──> %s: %s\n", cfg.name, all_pass ? "PASS" : "FAIL");
  return all_pass;
}

//==============================================================================
// Debug: Run a 1.5-stop-bit test with tick-level signal tracing
//==============================================================================
void debug_test_8n1_5(SimState& s) {
  printf("\n╔══ DEBUG: 115200_8N1.5 (All-0s, 4 bytes) ══╗\n");
  static int dbg_byte_count = 0;

  // Configure 8N1.5 (data=3 for 8-bit, parity=0 for None, stop=1 for 1.5)
  UartConfig cfg = {"115200_8N1.5", 434, 3, 0, 1};
  configure_uart(s, cfg);

  // Send 4 bytes of 0x00 with per-byte tracing
  for (int byte_idx = 0; byte_idx < 4; byte_idx++) {
    printf("\n=== Sending byte %d (0x00) ===\n", byte_idx);

  // Wait for tx_ready
  int timeout = 100000;
  while (!s.top->tx_ready && timeout > 0) {
    sim_tick(s);
    timeout--;
  }
  printf("[TICK %8lld] After tx_ready wait: tx_ready=%d\n",
         (long long)s.tick_count, s.top->tx_ready);

  // Latch byte
  s.top->tx_data  = 0x00;
  s.top->tx_valid = 1;
  printf("[TICK %8lld] TX_VALID=1 latch byte 0x00\n", (long long)s.tick_count);
  sim_tick(s);
  s.top->tx_valid = 0;

  // Track frame timing
  const int MAX_WAIT = 10000;
  bool got_rx_valid = false;
  uint64_t start_tick = s.tick_count;
  uint64_t last_print_tick = 0;

  s.top->rx_ready = 1;

  for (int i = 0; i < MAX_WAIT; i++) {
    sim_tick(s);

    // Print tick-level signals at key points
    bool tx_change = (i == 0);  // Print first tick
    if (i > 0 && i % 500 == 0) tx_change = true;  // Print every 500 ticks

    // Print every tick when either signal changes
    static int last_txd = -1;
    static int last_rx_busy = -1;
    int cur_txd = s.top->txd;
    int cur_rx_busy = s.top->rx_busy;
    int cur_rx_valid = s.top->rx_valid;

    if (cur_txd != last_txd || cur_rx_busy != last_rx_busy || cur_rx_valid) {
      printf("[TICK %8lld] txd=%d tx_busy=%d tx_ready=%d rx_busy=%d rx_valid=%d rx_data=0x%02x err_fe=%d err_pe=%d\n",
             (long long)s.tick_count,
             cur_txd, s.top->tx_busy, s.top->tx_ready,
             cur_rx_busy, cur_rx_valid, s.top->rx_data,
             s.top->framing_error, s.top->parity_error);
      last_txd = cur_txd;
      last_rx_busy = cur_rx_busy;
    }

    if (s.top->rx_valid) {
      got_rx_valid = true;
      printf("\n>>> [TICK %8lld] RECEIVED: rx_data=0x%02x (took %lld ticks)\n",
             (long long)s.tick_count, s.top->rx_data,
             (long long)(s.tick_count - start_tick));
      break;
    }
  }

  if (!got_rx_valid) {
    printf("\n!!! TIMEOUT after %d ticks - RX never asserted rx_valid\n", MAX_WAIT);
    printf("    Final state: txd=%d tx_busy=%d tx_ready=%d rx_busy=%d rx_valid=%d\n",
           s.top->txd, s.top->tx_busy, s.top->tx_ready,
           s.top->rx_busy, s.top->rx_valid);
  }

    // Delay between bytes to show state
    for (int d = 0; d < 50; d++) sim_tick(s);
  }

  printf("\n╔══ END DEBUG ══╗\n");
}

//==============================================================================
// Main
//==============================================================================
int main(int argc, char** argv) {
  // Check for debug mode
  bool debug_mode = false;
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "+DEBUG") == 0) {
      debug_mode = true;
    }
  }

  Verilated::commandArgs(argc, argv);  // Pass through (for $test$plusargs)

  SimState sim;
  sim_init(sim);

  printf("╔═════════════════════════════════════════╗\n");
  printf("║  UART Controller — Verilator Testbench  ║\n");
  printf("╚═════════════════════════════════════════╝\n");

  if (debug_mode) {
    printf("║  MODE: DEBUG (+DEBUG detected)          ║\n");
    printf("╚═════════════════════════════════════════╝\n");
    printf("Running single 8N1.5 test with RTL $display() tracing...\n");
    printf("+DEBUG passed - RTL $test$plusargs will fire\n\n");
    debug_test_8n1_5(sim);
  } else {
    printf("Clock: %d MHz\n", CLK_FREQ / 1000000);
    printf("Configurations: %d\n", NUM_CONFIGS);
    printf("Bytes per config: %d + %d + %d = %d\n",
           BYTES_PER_PATTERN, BYTES_PER_PATTERN, BYTES_PER_PATTERN,
           BYTES_PER_PATTERN * 3);
    printf("\n");

    sim.total_tests = NUM_CONFIGS;

    // Loop through all configurations
    for (int i = 0; i < NUM_CONFIGS; i++) {
      bool passed = test_config(sim, kConfigs[i]);
      if (passed) {
        sim.passed_tests++;
      } else {
        sim.failed_tests++;
      }
    }

    // Summary
    printf("\n");
    printf("╔═════════════════════════════════════════╗\n");
    printf("║  RESULTS                                ║\n");
    printf("╠═════════════════════════════════════════╣\n");
    printf("║  Total configurations: %3d             ║\n", sim.total_tests);
    printf("║  Passed:               %3d             ║\n", sim.passed_tests);
    printf("║  Failed:               %3d             ║\n", sim.failed_tests);
    printf("║  Total bytes TX/RX:    %3d             ║\n",
           sim.total_tests * BYTES_PER_PATTERN * DATA_PATTERNS * 2);
    printf("║  Simulation ticks:     %8lld       ║\n", (long long)sim.tick_count);
    printf("╚═════════════════════════════════════════╝\n");
    printf("Waveform: tb_uart_verilator.vcd\n");
  }

  // Cleanup
  if (sim.trace) {
    sim.trace->close();
    delete sim.trace;
  }
  delete sim.top;

  return (sim.failed_tests > 0) ? 1 : 0;
}
