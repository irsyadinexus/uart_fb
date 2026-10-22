//==============================================================================
// uart_pkg.sv
// UART Controller Package - Parameters, typedefs, and constants
//
// This package defines the parameters and types used by the UART transmitter
// and receiver modules for a fully synthesizable BMC UART controller.
//==============================================================================

`timescale 1ns / 1ps

package uart_pkg;

  // ---------------------------------------------------------------------------
  // UART Frame Configuration Encodings
  // ---------------------------------------------------------------------------

  // Parity type encoding
  typedef enum logic [1:0] {
    PARITY_NONE = 2'b00,
    PARITY_EVEN = 2'b01,
    PARITY_ODD  = 2'b10
  } parity_t;

  // Stop bit configuration encoding
  // Value maps to number of stop bit periods:
  //   00 = 1 stop bit
  //   01 = 1.5 stop bits
  //   10 = 2 stop bits
  //   11 = reserved (treated as 1)
  typedef enum logic [1:0] {
    STOP_1   = 2'b00,
    STOP_1_5 = 2'b01,
    STOP_2   = 2'b10
  } stop_bits_t;

  // Data bits encoding (offset by 5)
  //   000 = 5 data bits
  //   001 = 6 data bits
  //   010 = 7 data bits
  //   011 = 8 data bits
  //   >3  = clamped to 8
  typedef enum logic [2:0] {
    DATA_BITS_5 = 3'b000,
    DATA_BITS_6 = 3'b001,
    DATA_BITS_7 = 3'b010,
    DATA_BITS_8 = 3'b011
  } data_bits_t;

  // ---------------------------------------------------------------------------
  // TX FSM States
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {
    TX_IDLE      = 3'b000,
    TX_START     = 3'b001,
    TX_DATA      = 3'b010,
    TX_PARITY    = 3'b011,
    TX_STOP      = 3'b100,
    TX_STOP_HALF = 3'b101   // Trailing half stop bit for 1.5-stop framing
  } tx_state_t;

  // ---------------------------------------------------------------------------
  // RX FSM States
  // ---------------------------------------------------------------------------
  typedef enum logic [3:0] {
    RX_IDLE        = 4'b0000,
    RX_START_SYNC  = 4'b0001,  // Wait for falling edge, glitch filter
    RX_START_WAIT  = 4'b0010,  // Wait half-bit to center of start bit
    RX_DATA        = 4'b0011,  // Sample data bits
    RX_PARITY      = 4'b0100,  // Sample parity bit
    RX_STOP        = 4'b0101,  // Sample first stop bit
    RX_DONE        = 4'b0110   // Byte complete, output data
  } rx_state_t;

  // ---------------------------------------------------------------------------
  // UART Protocol Constants
  // ---------------------------------------------------------------------------

  // Number of oversampling clock ticks per bit period (16x)
  parameter int OVERSAMPLE_RATIO = 16;

  // Half-bit period in oversample ticks (for center alignment)
  parameter int HALF_BIT_TICKS   = OVERSAMPLE_RATIO / 2;

  // Number of bits to delay for metastability synchronizer
  parameter int SYNC_STAGES = 2;

endpackage : uart_pkg
