//==============================================================================
// uart_top_verilator.sv — Verilator-compatible UART Controller
//
// This module is functionally identical to uart_top but uses only logic ports
// (no inout) for Verilator compatibility. The UART serial interface (txd, rxd)
// is already simple logic-level, so no special adaptation is needed — this
// wrapper exists to provide a consistent top-level naming convention and
// to allow extending with Verilator-specific debug signals in the future.
//
// Usage (Verilator builds):
//   uart_top_verilator #(...) u_uart (...);
//==============================================================================

`timescale 1ns / 1ps

/* verilator lint_off IMPORTSTAR */
import uart_pkg::*;
/* verilator lint_on IMPORTSTAR */

module uart_top_verilator #(
  parameter int  CLK_FREQ = 50_000_000,
  // Loopback: 1 = connect TX output to RX input internally.
  // Set to 0 for external RX injection (e.g., for error-injection tests).
  parameter bit  LOOPBACK = 1
) (
  // System Interface
  input  logic       clk,
  input  logic       rst_n,

  // Configuration Interface
  input  logic [15:0] cfg_baud_div,
  input  logic [2:0]  cfg_data_bits,
  input  logic [1:0]  cfg_parity,
  input  logic [1:0]  cfg_stop_bits,

  // Transmit Interface
  input  logic [7:0]  tx_data,
  input  logic        tx_valid,
  output logic        tx_ready,
  output logic        tx_busy,

  // Receive Interface
  output logic [7:0]  rx_data,
  output logic        rx_valid,
  input  logic        rx_ready,
  output logic        rx_busy,

  // Error Status
  output logic        framing_error,
  output logic        parity_error,
  output logic        overrun_error,
  output logic        break_detect,

  // Interrupt
  output logic        irq,
  input  logic        int_en_rx,
  input  logic        int_en_tx,
  input  logic        int_en_err,

  // Error Clear (W1C)
  input  logic        clr_errors,

  // UART Serial Interface
  output logic        txd,
  /* verilator lint_off UNUSEDSIGNAL */
  input  logic        rxd
  /* verilator lint_on UNUSEDSIGNAL */
);

  // -------------------------------------------------------------------------
  // Internal loopback: connect TX output to RX input combinatorially
  // -------------------------------------------------------------------------
  // A C++ after-eval loopback (rxd = txd) has a 1-cycle delay that shifts
  // the RX sample timing by 1 clock cycle, causing data corruption on
  // multi-byte transmissions (first bit of byte 1 sampled as the stop bit
  // of byte 0). An internal continuous assign has zero delay.
  // -------------------------------------------------------------------------
  wire rxd_loop;
  generate
    if (LOOPBACK) begin : g_loopback
      assign rxd_loop = txd;
    end else begin : g_external
      assign rxd_loop = rxd;
    end
  endgenerate

  // ---------------------------------------------------------------------------
  // Instantiate the standard UART top module
  // ---------------------------------------------------------------------------
  uart_top #(
    .CLK_FREQ (CLK_FREQ)
  ) u_uart_top (
    .clk             (clk),
    .rst_n           (rst_n),
    .cfg_baud_div    (cfg_baud_div),
    .cfg_data_bits   (cfg_data_bits),
    .cfg_parity      (cfg_parity),
    .cfg_stop_bits   (cfg_stop_bits),
    .tx_data         (tx_data),
    .tx_valid        (tx_valid),
    .tx_ready        (tx_ready),
    .tx_busy         (tx_busy),
    .rx_data         (rx_data),
    .rx_valid        (rx_valid),
    .rx_ready        (rx_ready),
    .rx_busy         (rx_busy),
    .framing_error   (framing_error),
    .parity_error    (parity_error),
    .overrun_error   (overrun_error),
    .break_detect    (break_detect),
    .irq             (irq),
    .int_en_rx       (int_en_rx),
    .int_en_tx       (int_en_tx),
    .int_en_err      (int_en_err),
    .clr_errors      (clr_errors),
    .txd             (txd),
    .rxd             (rxd_loop)
  );

endmodule : uart_top_verilator
