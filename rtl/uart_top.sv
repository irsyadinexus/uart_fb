//==============================================================================
// uart_top.sv
// Top-Level UART Controller for BMC
//
// Features:
//   - Fully synthesizable UART for BMC (Baseboard Management Controller)
//   - Runtime-configurable frame format (via control signals):
//       - Baud rate:   1200 to 3M+ (cfg_baud_div = CLK_FREQ / BAUD_RATE)
//       - Data bits:   5, 6, 7, 8
//       - Parity:      None, Even, Odd
//       - Stop bits:   1, 1.5, 2
//   - Full-duplex operation
//   - Status outputs: framing error, parity error, overrun error
//
// Architecture:
//   - uart_baud_gen:  Generates baud_tick (1x) and baud_x16_tick (16x)
//   - uart_tx:        Transmitter FSM with configurable frame format
//   - uart_rx:        Receiver FSM with 16x oversampling
//
// Host Interface (for BMC firmware):
//   - Configuration via dedicated control signals (like memory-mapped regs)
//   - TX: Data register + valid/ready handshake
//   - RX: Data register + valid/ready handshake
//   - Status: framing_error, parity_error, overrun_error
//==============================================================================

`timescale 1ns / 1ps

/* verilator lint_off IMPORTSTAR */
import uart_pkg::*;
/* verilator lint_on IMPORTSTAR */

module uart_top #(
  /* verilator lint_off UNUSEDPARAM */
  // System clock frequency in Hz (for simulation timing reference)
  parameter int CLK_FREQ = 50_000_000
  /* verilator lint_on UNUSEDPARAM */
) (
  // ---------------------------------------------------------------------------
  // System Interface
  // ---------------------------------------------------------------------------
  input  logic       clk,
  input  logic       rst_n,

  // ---------------------------------------------------------------------------
  // Configuration Interface (runtime programmable)
  // ---------------------------------------------------------------------------
  // Baud rate divisor: should be set to CLK_FREQ / BAUD_RATE
  // Examples for 50 MHz:
  //   9600    → 50_000_000 / 9600    = 5208 (0x1458)
  //   19200   → 50_000_000 / 19200   = 2604 (0x0A2C)
  //   38400   → 50_000_000 / 38400   = 1302 (0x0516)
  //   57600   → 50_000_000 / 57600   = 868  (0x0364)
  //   115200  → 50_000_000 / 115200  = 434  (0x01B2)
  //   230400  → 50_000_000 / 230400  = 217  (0x00D9)
  //   460800  → 50_000_000 / 460800  = 108  (0x006C)
  //   921600  → 50_000_000 / 921600  = 54   (0x0036)
  //   3M      → 50_000_000 / 3000000 = 16   (0x0010)
  input  logic [15:0] cfg_baud_div,    // Baud rate divisor

  // Data bits:    000=5, 001=6, 010=7, 011=8 (>3 clamped to 8)
  input  logic [2:0]  cfg_data_bits,

  // Parity:       00=None, 01=Even, 10=Odd, 11=None
  input  logic [1:0]  cfg_parity,

  // Stop bits:    00=1, 01=1.5, 10=2, 11=1
  input  logic [1:0]  cfg_stop_bits,

  // ---------------------------------------------------------------------------
  // Transmit Interface
  // ---------------------------------------------------------------------------
  // Write data to the TX holding register. tx_ready will assert when the
  // transmitter is ready to accept a new byte. Hold tx_valid until tx_ready
  // is asserted to complete the handshake.
  input  logic [7:0]  tx_data,
  input  logic        tx_valid,
  output logic        tx_ready,
  output logic        tx_busy,         // Transmitter active

  // ---------------------------------------------------------------------------
  // Receive Interface
  // ---------------------------------------------------------------------------
  // Read data from the RX holding register. rx_valid asserts when a new byte
  // has been received. Hold rx_ready high or assert it when you want to
  // consume the data. The receiver will hold the next byte until rx_ready.
  output logic [7:0]  rx_data,
  output logic        rx_valid,
  input  logic        rx_ready,
  output logic        rx_busy,         // Receiver active

  // ---------------------------------------------------------------------------
  // Error Status (sticky, W1C via clr_errors)
  // Once asserted, error flags remain high until clr_errors is pulsed.
  // This allows software to poll the status reliably.
  // ---------------------------------------------------------------------------
  output logic        framing_error,   // Stop bit not detected
  output logic        parity_error,    // Parity mismatch
  output logic        overrun_error,   // Previous RX data not read
  output logic        break_detect,    // RX line low > full frame duration

  // ---------------------------------------------------------------------------
  // Interrupt
  // ---------------------------------------------------------------------------
  // irq is asserted when any enabled interrupt event occurs.
  // int_en_* signals act as enable-bits (like a memory-mapped int_en register):
  //   int_en_rx  = 1 → irq on rx_valid (data available)
  //   int_en_tx  = 1 → irq on tx_ready (TX ready to accept new byte)
  //   int_en_err = 1 → irq on any error flag (framing/parity/overrun/break)
  // ---------------------------------------------------------------------------
  output logic        irq,             // Interrupt request (OR of enabled events)
  input  logic        int_en_rx,       // Enable interrupt on RX data available
  input  logic        int_en_tx,       // Enable interrupt on TX ready
  input  logic        int_en_err,      // Enable interrupt on any error flag

  // ---------------------------------------------------------------------------
  // Error Clear (W1C - Write-1-to-Clear)
  // Pulse high for one cycle to clear sticky error flags.
  // ---------------------------------------------------------------------------
  input  logic        clr_errors,

  // ---------------------------------------------------------------------------
  // UART Serial Interface
  // ---------------------------------------------------------------------------
  output logic        txd,             // Serial transmit data
  input  logic        rxd              // Serial receive data
);

  // ---------------------------------------------------------------------------
  // Internal Signals
  // ---------------------------------------------------------------------------
  /* verilator lint_off UNUSEDSIGNAL */
  logic baud_tick;
  /* verilator lint_on UNUSEDSIGNAL */
  logic baud_x16_tick;

  // ---------------------------------------------------------------------------
  // Baud Rate Generator
  // ---------------------------------------------------------------------------
  uart_baud_gen u_baud_gen (
    .clk            (clk),
    .rst_n          (rst_n),
    .cfg_baud_div   (cfg_baud_div),
    .baud_tick      (baud_tick),
    .baud_x16_tick  (baud_x16_tick)
  );

  // ---------------------------------------------------------------------------
  // UART Transmitter
  // ---------------------------------------------------------------------------
  // Timing uses baud_x16_tick only (internal bit_timer provides deterministic
  // bit-period measurement).  The baud_tick signal from the baud gen is not
  // used by the TX; it remains available for other potential consumers.
  uart_tx u_tx (
    .clk            (clk),
    .rst_n          (rst_n),
    .cfg_data_bits  (cfg_data_bits),
    .cfg_parity     (cfg_parity),
    .cfg_stop_bits  (cfg_stop_bits),
    .baud_x16_tick  (baud_x16_tick),
    .tx_data        (tx_data),
    .tx_valid       (tx_valid),
    .tx_ready       (tx_ready),
    .tx_busy        (tx_busy),
    .txd            (txd)
  );

  // ---------------------------------------------------------------------------
  // UART Receiver
  // ---------------------------------------------------------------------------
  uart_rx u_rx (
    .clk            (clk),
    .rst_n          (rst_n),
    .cfg_data_bits  (cfg_data_bits),
    .cfg_parity     (cfg_parity),
    .cfg_stop_bits  (cfg_stop_bits),
    .baud_x16_tick  (baud_x16_tick),
    .rx_data        (rx_data),
    .rx_valid       (rx_valid),
    .rx_ready       (rx_ready),
    .rx_busy        (rx_busy),
    .framing_error  (framing_error),
    .parity_error   (parity_error),
    .overrun_error  (overrun_error),
    .break_detect   (break_detect),
    .clr_errors     (clr_errors),
    .rxd            (rxd)
  );

  // ---------------------------------------------------------------------------
  // Interrupt Generation
  // ---------------------------------------------------------------------------
  // Combinatorial OR of enabled interrupt sources:
  //   RX:   Data available (rx_valid)
  //   TX:   Transmitter ready for new data (tx_ready)
  //   ERR:  Any error flag asserted (framing, parity, overrun, break)
  // ---------------------------------------------------------------------------
  assign irq = (int_en_rx  && rx_valid)
             || (int_en_tx  && tx_ready)
             || (int_en_err && (framing_error || parity_error || overrun_error || break_detect));

endmodule : uart_top
