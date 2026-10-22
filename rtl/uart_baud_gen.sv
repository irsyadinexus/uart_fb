//==============================================================================
// uart_baud_gen.sv
// UART Baud Rate Generator
//
// Generates two tick signals from the system clock:
//   - baud_tick:     One-cycle pulse at the configured baud rate
//   - baud_x16_tick: One-cycle pulse at 16x the baud rate (for RX oversampling)
//
// The divisor is calculated as:
//   baud_div_16 = CLK_FREQ / (BAUD_RATE * 16)
//   baud_div_1  = CLK_FREQ / BAUD_RATE
//
// cfg_baud_div should be set to CLK_FREQ / BAUD_RATE by the user.
//==============================================================================

`timescale 1ns / 1ps

/* verilator lint_off IMPORTSTAR */
import uart_pkg::*;
/* verilator lint_on IMPORTSTAR */

module uart_baud_gen (
  // ---------------------------------------------------------------------------
  // System Interface
  // ---------------------------------------------------------------------------
  input  logic       clk,
  input  logic       rst_n,

  // ---------------------------------------------------------------------------
  // Configuration
  // ---------------------------------------------------------------------------
  input  logic [15:0] cfg_baud_div,   // Baud rate divisor = CLK_FREQ / BAUD_RATE

  // ---------------------------------------------------------------------------
  // Outputs
  // ---------------------------------------------------------------------------
  output logic       baud_tick,       // One-cycle pulse at baud rate
  output logic       baud_x16_tick    // One-cycle pulse at 16x baud rate
);

  // ---------------------------------------------------------------------------
  // 16x Oversample Clock Divider
  // ---------------------------------------------------------------------------
  // Divides the system clock by (cfg_baud_div / 16) to produce a tick at
  // 16x the baud rate. The x16 tick is used as the time base for the RX
  // oversampling state machine.
  //
  // baud_div_16 = cfg_baud_div / OVERSAMPLE_RATIO
  // We use separate counters for the x16 and x1 dividers.
  //
  // The x16 counter cycles every baud_div_16 clock cycles, producing a tick
  // each time it wraps.
  // ---------------------------------------------------------------------------

  // Compute the x16 divider value: ceil(cfg_baud_div / 16)
  // Using integer arithmetic: (cfg_baud_div + 15) / 16
  logic [11:0] baud_div_16;
  logic [11:0] x16_cnt;
  // verilator lint_off UNUSEDSIGNAL
  logic        x16_cnt_zero;
  // verilator lint_on UNUSEDSIGNAL

  // Calculate baud_div_16 = ceil(baud_div / 16)
  // This runs every cycle, but only changes when cfg_baud_div changes.
  // Since cfg_baud_div is semi-static (changes between transactions), this
  // combinational path is acceptable.
  always_comb begin
    baud_div_16 = 12'((32'(cfg_baud_div[15:0]) + OVERSAMPLE_RATIO - 1) / OVERSAMPLE_RATIO);
    // Clamp to minimum value of 1
    if (baud_div_16 == 12'd0)
      baud_div_16 = 12'd1;
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      x16_cnt      <= '0;
      baud_x16_tick <= 1'b0;
    end else begin
      baud_x16_tick <= 1'b0;  // Default: no tick

      if (x16_cnt >= baud_div_16 - 1) begin
        x16_cnt      <= '0;
        baud_x16_tick <= 1'b1;  // Tick every baud_div_16 clock cycles
      end else begin
        x16_cnt <= x16_cnt + 1;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Baud Rate (1x) Clock Divider
  // ---------------------------------------------------------------------------
  // Counts 16 x16 ticks to produce one baud tick. This ensures the baud_tick
  // is synchronized with the x16 tick for consistent timing.
  // ---------------------------------------------------------------------------

  logic [3:0] bit_tick_cnt;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      bit_tick_cnt <= '0;
      baud_tick    <= 1'b0;
    end else begin
      baud_tick <= 1'b0;  // Default: no tick

      if (baud_x16_tick) begin
        if (32'(bit_tick_cnt) >= OVERSAMPLE_RATIO - 1) begin
          bit_tick_cnt <= '0;
          baud_tick    <= 1'b1;  // One baud tick per 16 x16 ticks
        end else begin
          bit_tick_cnt <= bit_tick_cnt + 1;
        end
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Debug (runtime-enabled with +DEBUG via $test$plusargs)
  // ---------------------------------------------------------------------------
  // Debug: compile with +define+VERILATOR_DEBUG and run with +DEBUG

endmodule : uart_baud_gen
