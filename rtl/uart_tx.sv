//==============================================================================
// uart_tx.sv
// UART Transmitter
//
// Features:
//   - Configurable data bits (5-8)
//   - Configurable parity (none, even, odd)
//   - Configurable stop bits (1, 1.5, 2)
//   - Double-buffered data interface with handshake
//   - LSB-first serialization
//   - Idle line high (standard UART)
//
// FSM states:
//   TX_IDLE   → TX_START → TX_DATA → TX_PARITY → TX_STOP → TX_STOP_HALF → TX_IDLE
//            (or skip)    (or skip)   (or skip)              (if half_stop)
//
// Timing: ALL bit periods are driven by a deterministic baud_x16_tick counter
// (bit_timer). Every full-bit state lasts exactly 16 baud_x16_ticks,
// eliminating the variable-duration issue inherent to the old baud_tick-based
// approach where TX_START duration depended on baud_tick alignment.
//==============================================================================

`timescale 1ns / 1ps

/* verilator lint_off IMPORTSTAR */
import uart_pkg::*;
/* verilator lint_on IMPORTSTAR */

module uart_tx (
  // ---------------------------------------------------------------------------
  // System Interface
  // ---------------------------------------------------------------------------
  input  logic       clk,
  input  logic       rst_n,

  // ---------------------------------------------------------------------------
  // Configuration
  // ---------------------------------------------------------------------------
  input  logic [2:0] cfg_data_bits,   // 0=5, 1=6, 2=7, 3=8 (offset by 5)
  input  logic [1:0] cfg_parity,      // 00=none, 01=even, 10=odd
  input  logic [1:0] cfg_stop_bits,   // 00=1, 01=1.5, 10=2

  // ---------------------------------------------------------------------------
  // Timing
  // ---------------------------------------------------------------------------
  input  logic       baud_x16_tick,   // 16x oversample tick (time base for all states)

  // ---------------------------------------------------------------------------
  // Host Interface
  // ---------------------------------------------------------------------------
  input  logic [7:0] tx_data,         // Data to transmit
  input  logic       tx_valid,        // Data valid strobe
  output logic       tx_ready,        // Ready to accept new data
  output logic       tx_busy,         // Transmission in progress

  // ---------------------------------------------------------------------------
  // Serial Output
  // ---------------------------------------------------------------------------
  output logic       txd              // Serial data output (idle = high)
);

  // ---------------------------------------------------------------------------
  // Internal Signals
  // ---------------------------------------------------------------------------
  tx_state_t state, next_state;

  // Shared bit-period timer: counts 0-15 on baud_x16_tick. Resets to 0
  // on each state transition, providing exactly 16 x16_ticks per state.
  logic [3:0] bit_timer;
  wire        bit_done;

  // Shift register for data to be transmitted
  logic [7:0] shift_reg;
  logic [3:0] bit_cnt;       // Bit counter (0 to max 7)
  logic [4:0] stop_cnt;      // Stop bit counter (for 1.5 and 2 stop bits)
  logic [3:0] half_stop_cnt; // Half-bit counter (for 1.5-stop timing)
  logic       parity_bit;     // Computed parity bit
  logic       last_data_bit;  // Last data bit being transmitted

  // Latch for the byte being transmitted (holds value during transmission)
  logic [7:0] tx_byte;
  logic       byte_latched;

  // Derived configuration
  logic [3:0] num_data_bits;  // Actual number of data bits (5-8)
  logic [1:0] num_stop_bits;  // Actual number of stop bit periods (1 or 2)
  logic       half_stop;      // Additional half stop bit (for 1.5)
  logic       has_parity;     // Parity enabled

  always_comb begin
    // Data bits: 5 + cfg_data_bits (clamped to max 8)
    num_data_bits = (cfg_data_bits > 3) ? 4'd8 : {1'b0, cfg_data_bits} + 4'd5;

    // Stop bits: 1.5 maps to 1 + half, 2 maps to 2
    case (cfg_stop_bits)
      STOP_1:   begin num_stop_bits = 2'd1; half_stop = 1'b0; end
      STOP_1_5: begin num_stop_bits = 2'd1; half_stop = 1'b1; end
      STOP_2:   begin num_stop_bits = 2'd2; half_stop = 1'b0; end
      default:  begin num_stop_bits = 2'd1; half_stop = 1'b0; end
    endcase

    // Parity enabled
    has_parity = (cfg_parity != PARITY_NONE);
  end

  // ---------------------------------------------------------------------------
  // Bit-Period Timer
  // ---------------------------------------------------------------------------
  // Runs on baud_x16_tick in all active states. Resets to 0 on state
  // transitions (detected by state != next_state), ensuring each state
  // gets a fresh 16-tick count.  TX_STOP_HALF uses its own half_stop_cnt
  // counter for the half-bit period (8 ticks).
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      bit_timer <= '0;
    end else if (baud_x16_tick) begin
      // Reset on any state transition (state about to change)
      if (state != next_state)
        bit_timer <= '0;
      else if (32'(bit_timer) >= OVERSAMPLE_RATIO - 1)
        bit_timer <= '0;
      else
        bit_timer <= bit_timer + 1;
    end
  end

  // bit_done: one full bit period (16 x16_ticks) has elapsed.
  // Used by FSM, shift register, and counters.
  assign bit_done = baud_x16_tick && (32'(bit_timer) >= OVERSAMPLE_RATIO - 1);

  // ---------------------------------------------------------------------------
  // FSM State Register
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      state <= TX_IDLE;
    else
      state <= next_state;
  end

  // ---------------------------------------------------------------------------
  // Byte Latch
  // ---------------------------------------------------------------------------
  // IMPORTANT: byte_latched is set when tx_valid is sampled in IDLE,
  // and cleared ONLY when the FSM acknowledges it (at end of START bit
  // or when transitioning out of IDLE). DO NOT clear it on !tx_valid in
  // IDLE — that would race the FSM transition and lose the byte!
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tx_byte      <= '0;
      byte_latched <= 1'b0;
    end else if (state == TX_IDLE && tx_valid) begin
      tx_byte      <= tx_data;
      byte_latched <= 1'b1;
    end else if (state == TX_START && bit_done) begin
      // Data has been consumed by the FSM
      byte_latched <= 1'b0;
    end else if (state != TX_IDLE) begin
      // Not in IDLE: clear stale latch (redundant with above, safe)
      byte_latched <= 1'b0;
    end
  end

  // ---------------------------------------------------------------------------
  // Shift Register
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      shift_reg <= '0;
    end else if (state == TX_START && bit_done) begin
      // Load data into shift register at end of start bit
      shift_reg <= tx_byte;
    end else if (state == TX_DATA && bit_done) begin
      // Shift right (LSB first)
      shift_reg <= {1'b0, shift_reg[7:1]};
    end
  end

  // ---------------------------------------------------------------------------
  // Bit Counter
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      bit_cnt <= '0;
    end else if (state == TX_START && bit_done) begin
      bit_cnt <= '0;
    end else if (state == TX_DATA && bit_done) begin
      if (bit_cnt >= num_data_bits - 1)
        bit_cnt <= '0;
      else
        bit_cnt <= bit_cnt + 1;
    end
  end

  assign last_data_bit = (state == TX_DATA) && (bit_cnt >= num_data_bits - 1);

  // ---------------------------------------------------------------------------
  // Stop Bit Counter
  // ---------------------------------------------------------------------------
  // IMPORTANT: stop_cnt must ONLY be reset when NOT in TX_STOP. An `else`
  // branch (no condition) would reset stop_cnt on every cycle where bit_done
  // is low — but bit_done is low for most cycles. This would prevent stop_cnt
  // from ever reaching >= num_stop_bits - 1 for 2-stop bits, causing the TX
  // to hang in TX_STOP forever.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      stop_cnt <= '0;
    end else if (state == TX_STOP && bit_done) begin
      if (stop_cnt >= {3'd0, num_stop_bits})  // Stop counter includes full bit count
        stop_cnt <= '0;
      else
        stop_cnt <= stop_cnt + 1;
    end else if (state != TX_STOP) begin
      stop_cnt <= '0;
    end
    // else: stay in TX_STOP, retain stop_cnt across cycles
  end

  // stop_data_done: all stop bits have been transmitted.
  // For num_stop_bits=1: transition on first bit_done (stop_cnt >= 0 = always true)
  // For num_stop_bits=2: transition on second bit_done (stop_cnt >= 1)
  // stop_cnt is 0 on entry to STOP (reset by the else branch), so we need
  // the comparison to be stop_cnt >= num_stop_bits - 1.
  logic stop_data_done;
  always_comb begin
    stop_data_done = (stop_cnt >= {3'd0, num_stop_bits - 1'd1});
  end

  // ---------------------------------------------------------------------------
  // Parity Bit Computation
  // ---------------------------------------------------------------------------
  // Even parity: XOR of all data bits (number of 1s is even)
  // Odd parity:  XNOR of all data bits (number of 1s is odd)
  // Uses a loop to avoid Iverilog's restriction on variable-width part selects.
  always_comb begin
    parity_bit = 1'b0;
    if (cfg_parity == PARITY_EVEN || cfg_parity == PARITY_ODD) begin
      for (int pi = 0; pi < 8; pi++) begin
        if (pi < num_data_bits)
          parity_bit = parity_bit ^ tx_byte[pi];
      end
      if (cfg_parity == PARITY_ODD)
        parity_bit = ~parity_bit;
    end
  end

  // ---------------------------------------------------------------------------
  // FSM Next State Logic
  // ---------------------------------------------------------------------------
  always_comb begin
    next_state = state;

    case (state)

      TX_IDLE: begin
        if (byte_latched)
          next_state = TX_START;
      end

      TX_START: begin
        // Drive start bit (low) for exactly 16 baud_x16_ticks
        if (bit_done)
          next_state = TX_DATA;
      end

      TX_DATA: begin
        // Transmit data bits LSB first, 16 x16_ticks each
        if (bit_done && last_data_bit) begin
          if (has_parity)
            next_state = TX_PARITY;
          else
            next_state = TX_STOP;
        end
      end

      TX_PARITY: begin
        // Transmit parity bit for 16 x16_ticks
        if (bit_done)
          next_state = TX_STOP;
      end

      TX_STOP: begin
        // Transmit stop bit(s), 16 x16_ticks each
        if (bit_done && stop_data_done) begin
          if (half_stop)
            next_state = TX_STOP_HALF;  // 1.5 stop: extra half-bit
          else
            next_state = TX_IDLE;
        end
      end

      TX_STOP_HALF: begin
        // Trailing half stop bit (for 1.5-stop framing).
        // Uses half_stop_cnt to measure exactly HALF_BIT_TICKS (8) x16 ticks.
        // The counter was pre-loaded when leaving TX_STOP and decremented each
        // x16 tick; when 0 is reached, the next x16 tick exits the state.
        if (baud_x16_tick && (half_stop_cnt == 0)) begin
          next_state = TX_IDLE;
        end
      end

      default: next_state = TX_IDLE;
    endcase
  end

  // ---------------------------------------------------------------------------
  // Half-Stop Bit Counter (for 1.5-stop framing)
  // ---------------------------------------------------------------------------
  // Pre-loaded when the FSM leaves TX_STOP for 1.5 stop bits, then
  // decremented on each baud_x16_tick while in TX_STOP_HALF.  The FSM
  // exits TX_STOP_HALF when half_stop_cnt reaches 0 and a baud_x16_tick
  // fires, providing exactly HALF_BIT_TICKS (8) x16 ticks of the
  // trailing half-stop.
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      half_stop_cnt <= '0;
    end else if (state == TX_STOP && bit_done && stop_data_done && half_stop) begin
      half_stop_cnt <= 4'(HALF_BIT_TICKS - 1);  // Pre-load: 7
    end else if (state == TX_STOP_HALF && baud_x16_tick && (half_stop_cnt > 0)) begin
      half_stop_cnt <= half_stop_cnt - 1;
    end else if (state != TX_STOP_HALF) begin
      half_stop_cnt <= '0;
    end
  end

  // ---------------------------------------------------------------------------
  // Serial Output Generation
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      txd <= 1'b1;  // Idle high
    end else begin
      case (state)
        TX_IDLE: begin
          txd <= 1'b1;  // Idle high
        end

        TX_START: begin
          txd <= 1'b0;  // Start bit (low)
        end

        TX_DATA: begin
          // LSB first: shift_reg[0] is the current bit
          txd <= shift_reg[0];
        end

        TX_PARITY: begin
          txd <= parity_bit;
        end

        TX_STOP: begin
          txd <= 1'b1;  // Stop bit (high)
        end

        TX_STOP_HALF: begin
          txd <= 1'b1;  // Half-stop bit (high)
        end

        default: txd <= 1'b1;
      endcase
    end
  end

  // ---------------------------------------------------------------------------
  // Status Outputs
  // ---------------------------------------------------------------------------
  assign tx_busy  = (state != TX_IDLE);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      tx_ready <= 1'b0;
    else
      tx_ready <= (state == TX_IDLE && !byte_latched);
  end

endmodule : uart_tx
