//==============================================================================
// uart_rx.sv
// UART Receiver
//
// Features:
//   - 16x oversampling with center-aligned bit sampling
//   - Configurable data bits (5-8)
//   - Configurable parity (none, even, odd)
//   - Configurable stop bits (1, 1.5, 2)
//   - 4-cycle glitch filter on start bit detection
//   - Framing error detection
//   - Parity error detection
//   - Overrun error detection
//   - 2-stage metastability synchronizer on RXD
//
// Sampling strategy:
//   1. Wait for falling edge on RXD → apply 4-cycle glitch filter
//   2. Wait 8 x16 ticks to reach the center of the start bit
//   3. Verify start bit is still low
//   4. For data/parity/stop bits: wait 16 x16 ticks, then sample
//   5. For 1.5 stop bits: treated the same as 1 stop bit (only the first
//      stop bit is sampled; the extra half-bit is idle-high line that the
//      receiver naturally ignores by returning to RX_IDLE).
//==============================================================================

`timescale 1ns / 1ps

/* verilator lint_off IMPORTSTAR */
import uart_pkg::*;
/* verilator lint_on IMPORTSTAR */

module uart_rx (
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
  input  logic       baud_x16_tick,   // 16x oversample tick

  // ---------------------------------------------------------------------------
  // Host Interface
  // ---------------------------------------------------------------------------
  output logic [7:0] rx_data,         // Received data byte
  output logic       rx_valid,        // New byte received (pulse)
  input  logic       rx_ready,        // Host consumed data
  output logic       rx_busy,         // Reception in progress

  // ---------------------------------------------------------------------------
  // Status
  // ---------------------------------------------------------------------------
  output logic       framing_error,   // Stop bit not 1
  output logic       parity_error,    // Parity mismatch
  output logic       overrun_error,   // Previous data not read
  output logic       break_detect,    // RX line low > full frame duration

  // ---------------------------------------------------------------------------
  // Error Clear (W1C - Write-1-to-Clear)
  // ---------------------------------------------------------------------------
  // Pulse clr_errors high for one cycle to clear all sticky error flags.
  // Software typically drives this from a register write decode.
  input  logic       clr_errors,      // W1C strobe: clear all error flags

  // ---------------------------------------------------------------------------
  // Serial Input
  // ---------------------------------------------------------------------------
  input  logic       rxd              // Serial data input
);

  // ===========================================================================
  // RXD Synchronization
  // ===========================================================================
  // IMPORTANT: Use `wire` for rxd_sampled to create a continuous assignment!
  // `logic rxd_sampled = rxd_sync[...]` is only an INITIAL value, not a
  // continuous assignment — rxd_sampled would never track changes!
  // ===========================================================================
  logic [SYNC_STAGES-1:0] rxd_sync;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) rxd_sync <= '1;
    else        rxd_sync <= {rxd_sync[0], rxd};
  end
  wire rxd_sampled = rxd_sync[SYNC_STAGES-1];

  // ===========================================================================
  // Break Detection Timer
  // ===========================================================================
  // Counts continuous x16 ticks while rxd_sampled is low. When the count
  // exceeds BREAK_THRESHOLD_TICKS (~12.5 bit periods), a UART break
  // condition is flagged. The timer resets immediately when the line goes
  // high (end of break). break_detect is sticky W1C like other error flags.
  //
  // Threshold analysis:
  //   Max continuous low in a valid frame: 8E1 with 0x00 data
  //     = start(16) + 8 data(128) + even parity(16) = 160 x16 ticks
  //   Break threshold: 200 ticks (~12.5 bit periods)
  //   Margin: 40 ticks (2.5 bit periods) over the worst-case valid frame
  // ===========================================================================
  localparam int BREAK_THRESHOLD_TICKS = 200;

  logic [7:0] break_tmr;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      break_tmr <= '0;
    end else if (baud_x16_tick) begin
      if (rxd_sampled) begin
        break_tmr <= '0;  // Line high → reset timer
      end else if (32'(break_tmr) < BREAK_THRESHOLD_TICKS) begin
        break_tmr <= break_tmr + 1;
      end
      // else: saturate at threshold (prevents wraparound)
    end
  end

  // ===========================================================================
  // Edge Detection
  // ===========================================================================
  /* verilator lint_off UNUSEDSIGNAL */
  logic rxd_prev;
  /* verilator lint_on UNUSEDSIGNAL */
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) rxd_prev <= 1'b1;
    else        rxd_prev <= rxd_sampled;
  end

  // ===========================================================================
  // Derived Configuration
  // ===========================================================================
  logic [3:0] num_data_bits;
  logic       has_parity;
  logic [1:0] num_stop_bits;

  always_comb begin
    num_data_bits = (cfg_data_bits > 3) ? 4'd8 : {1'b0, cfg_data_bits} + 4'd5;
    has_parity    = (cfg_parity != PARITY_NONE);
    case (cfg_stop_bits)
      STOP_1:   num_stop_bits = 2'd1;
      STOP_1_5: num_stop_bits = 2'd1;
      STOP_2:   num_stop_bits = 2'd2;
      default:  num_stop_bits = 2'd1;
    endcase
  end

  // ===========================================================================
  // Glitch Filter: 4 consecutive samples, at least 3 must be low
  // ===========================================================================
  logic [3:0] glitch_shift;
  logic       glitch_pass;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)                     glitch_shift <= '1;
    else if (state == RX_IDLE)      glitch_shift <= {glitch_shift[2:0], rxd_sampled};
    else                            glitch_shift <= '1;
  end
  assign glitch_pass = (glitch_shift[0] + glitch_shift[1] +
                        glitch_shift[2] + glitch_shift[3]) <= 1;

  // ===========================================================================
  // FSM State & Signals
  // ===========================================================================
  rx_state_t state, next_state;

  logic [7:0] shift_reg;
  logic [3:0] bit_cnt;
  logic [4:0] sample_cnt;
  logic       sample_center;
  logic       recv_parity;
  logic       expected_parity;
  logic [1:0] stop_bit_cnt;  // Which stop bit we're sampling (0-based)

  // ===========================================================================
  // Sample Counter
  // ===========================================================================
  // The counter is loaded ONCE per frame when a start bit is detected
  // (start_detected = rising edge of glitch_pass) to the half-bit wait
  // value (HALF_BIT_TICKS = 8).  From there, each sample_center event
  // re-arms the counter to OVERSAMPLE_RATIO - 1 (15), providing exactly
  // 16 ticks between all bit-center samples.
  //
  // NOTE: The TX now uses deterministic baud_x16_tick counters for ALL
  // states (every bit period is exactly 16 x16_ticks).  There is no longer
  // any variable TX_START duration, so the first data bit can be sampled
  // at the proper UART interval (16 ticks from start-bit center, putting
  // the sample at the center of the first data bit).  No special 8-tick
  // first-data workaround is needed.
  //
  // IMPORTANT: Only start_detected loads the counter.  FSM transitions
  // (DATA→STOP, STOP→DONE, etc.) do NOT load it — they rely on the
  // re-arm from sample_center that already fired on the same cycle.
  // This prevents the cumulative drift bug where an extra tick per
  // transition would shift the sampling window, eventually causing the
  // receiver to sample the stop bit (idle-high = 1) as the last data
  // bit — producing rx_data[7]=1 (e.g. 0x80 instead of 0x00).
  // ===========================================================================

  // ===========================================================================
  // FSM Transition Detection (for sample counter initialization)
  // ===========================================================================
  // Only the RX_START_WAIT entry needs a counter load (for the half-bit wait
  // to the start-bit center).  All other transitions (DATA→STOP, STOP→DONE,
  // DONE→IDLE, etc.) are driven by sample_center, which ALREADY re-arms the
  // counter to `sample_reload - 1` on the same cycle.  Loading the counter
  // again on these transitions would add 1 extra tick per transition,
  // causing cumulative drift that shifts the sampling by one full bit
  // period after 2-3 frames — producing rx_data[7]=1 (e.g. 0x80 vs 0x00)
  // as the stop bit is sampled instead of the last data bit.
  // ===========================================================================
  logic        last_glitch_pass;
  logic        start_detected;

  // Detect the start bit: rising edge of glitch_pass (stays high for
  // many cycles while line is low, so edge detection ensures one-shot).
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) last_glitch_pass <= 1'b0;
    else        last_glitch_pass <= glitch_pass;
  end
  assign start_detected = glitch_pass && !last_glitch_pass;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sample_cnt       <= '0;
      sample_center    <= 1'b0;
    end else begin
      sample_center <= 1'b0;

      // Start bit detected: load the half-bit wait counter
      if (start_detected) begin
        sample_cnt <= 5'(HALF_BIT_TICKS);
      end else if (baud_x16_tick) begin
        if (sample_cnt == 0) begin
          sample_center <= 1'b1;
          // Re-arm: load with sample_reload - 1 so the counter counts
          // (N-1) down to 0, taking N ticks total between sample_centers.
          // Always re-arm on sample_center to prevent continuous
          // sample_center firing while stuck at 0 in any state.
          if (state == RX_IDLE     || state == RX_DATA    ||
              state == RX_PARITY   || state == RX_START_WAIT ||
              state == RX_STOP)
            sample_cnt <= 5'(OVERSAMPLE_RATIO - 1);
        end else begin
          sample_cnt <= sample_cnt - 1;
        end
      end
    end
  end

  // ===========================================================================
  // State Register
  // ===========================================================================
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) state <= RX_IDLE;
    else        state <= next_state;
  end

  // ===========================================================================
  // Next State Logic (single clean block)
  //
  // NOTE: sample_center is the ONLY qualifier needed. It is asserted on the
  // cycle AFTER baud_x16_tick fires (1-cycle pipeline delay), so checking
  // baud_x16_tick && sample_center would NEVER be true — the two signals
  // are never high simultaneously by design!
  // ===========================================================================
  always_comb begin
    next_state = state;
    case (state)

      RX_IDLE: begin
        if (glitch_pass) next_state = RX_START_WAIT;
      end

      RX_START_WAIT: begin
        // The glitch filter has already validated the start (3 of 4 samples low).
        // Proceed unconditionally to DATA. The start bit may be short
        // (if TX_START is misaligned with baud_tick), but the half-bit wait
        // still provides the correct timing base for subsequent bit samples.
        if (sample_center) begin
          next_state = RX_DATA;
        end
      end

      RX_DATA: begin
        if (sample_center) begin
          if (bit_cnt >= num_data_bits - 1) begin
            if (has_parity)
              next_state = RX_PARITY;
            else
              next_state = RX_STOP;
          end
        end
      end

      RX_PARITY: begin
        if (sample_center) next_state = RX_STOP;
      end

      RX_STOP: begin
        if (sample_center) begin
          if (num_stop_bits == 2 && stop_bit_cnt < 1)
            ;  // Stay for second stop bit sample
          else
            next_state = RX_DONE;  // 1 or 1.5 stop: single sample sufficient
        end
      end

      RX_DONE: begin
        next_state = RX_IDLE;
      end

      default: next_state = RX_IDLE;
    endcase
  end

  // ===========================================================================
  // Bit Counter
  // ===========================================================================
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      bit_cnt <= '0;
    end else if (state == RX_DATA && sample_center) begin
      bit_cnt <= (bit_cnt >= num_data_bits - 1) ? '0 : bit_cnt + 1;
    end else if (state != RX_DATA) begin
      bit_cnt <= '0;
    end
  end

  // ===========================================================================
  // Stop Bit Counter
  // ===========================================================================
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      stop_bit_cnt <= '0;
    end else if (state == RX_STOP && sample_center) begin
      if (num_stop_bits == 2 && stop_bit_cnt < 1)
        stop_bit_cnt <= stop_bit_cnt + 1;
      else
        stop_bit_cnt <= '0;
    end else if (state != RX_STOP) begin
      stop_bit_cnt <= '0;
    end
  end

  // ===========================================================================
  // Shift Register (LSB first → right-aligned on output)
  // ===========================================================================
  // LSB-first bits shift in from the MSB side, pushing right. This places
  // the first-received bit (d0, LSB) at shift_reg[0] and the last-received
  // bit (d7, MSB) at shift_reg[7] — correct byte order.  For <8 data bits,
  // the value is left-aligned in the register (positions [7:(8-N)]), so the
  // output stage right-shifts to right-align the data for the consumer.
  // ===========================================================================
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      shift_reg <= '0;
    end else if (state == RX_DATA && sample_center) begin
      // LSB first: each new bit goes into MSB, shifting previous bits right
      shift_reg <= {rxd_sampled, shift_reg[7:1]};
    end
  end

  // ===========================================================================
  // Parity Capture
  // ===========================================================================
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      recv_parity <= 1'b0;
    end else if (state == RX_PARITY && sample_center) begin
      recv_parity <= rxd_sampled;
    end
  end

  // Expected parity from received data
  // Reads shift_reg[7-p] because the {rxd_sampled, shift_reg[7:1]} shift
  // places data bits at positions [7:(8-num_data_bits)], not at
  // positions [num_data_bits-1:0].  Using a loop avoids Iverilog's
  // restriction on variable-width part selects.
  always_comb begin
    expected_parity = 1'b0;
    if (cfg_parity == PARITY_EVEN || cfg_parity == PARITY_ODD) begin
      for (int p = 0; p < 8; p++) begin
        if (p < num_data_bits)
          expected_parity = expected_parity ^ shift_reg[7 - p];
      end
      if (cfg_parity == PARITY_ODD)
        expected_parity = ~expected_parity;
    end
  end

  // ===========================================================================
  // Data Output & Status
  // ===========================================================================
  // rx_valid is held high until rx_ready is asserted (standard valid/ready
  // handshake). This ensures the testbench cannot miss the rx_valid pulse.
  // ===========================================================================
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rx_data       <= '0;
      rx_valid      <= 1'b0;
      framing_error <= 1'b0;
      parity_error  <= 1'b0;
      overrun_error <= 1'b0;
      break_detect   <= 1'b0;
    end else begin
      // Error flags are sticky (W1C): once set they stay high until
      // explicitly cleared by pulsing clr_errors high for one cycle.
      // This ensures software/consumers can poll them reliably without
      // race conditions from frame timing.
      if (clr_errors) begin
        framing_error <= 1'b0;
        parity_error  <= 1'b0;
        overrun_error <= 1'b0;
        break_detect  <= 1'b0;
      end

      // Break detection: RX line held low for >= BREAK_THRESHOLD_TICKS
      // x16 ticks (~12.5 bit periods). The condition fires on each x16
      // tick once the threshold is reached or exceeded, but break_detect
      // is sticky so it only matters once.
      if (baud_x16_tick && (32'(break_tmr) >= BREAK_THRESHOLD_TICKS - 1) && !rxd_sampled)
        break_detect <= 1'b1;

      if (state == RX_DONE) begin
        // Right-align data for consumer (shift out stale lower bits)
        rx_data <= shift_reg >> (8 - num_data_bits);
        rx_valid <= 1'b1;
        if (has_parity && (recv_parity != expected_parity)) parity_error <= 1'b1;
      end else if (rx_ready && rx_valid) begin
        // Clear valid when host acknowledges
        rx_valid <= 1'b0;
      end

      // Overrun: new data arrives while previous not read (sticky)
      // Clears on start_detected (next frame) along with other error flags.
      if (state == RX_DONE && rx_valid && !rx_ready)
        overrun_error <= 1'b1;

      // Framing error on first stop bit being low
      if (state == RX_STOP && sample_center &&
          stop_bit_cnt == 0 && !rxd_sampled)
        framing_error <= 1'b1;
    end
  end

  // ===========================================================================
  // Status Outputs
  // ===========================================================================
  assign rx_busy = (state != RX_IDLE) && (state != RX_DONE);

  // ===========================================================================
  // Debug (enabled with +DEBUG via $test$plusargs)
  // ===========================================================================
  `ifndef SYNTHESIS
    always @(posedge clk) begin
      if ($test$plusargs("DEBUG")) begin
        if (state != next_state || sample_center)
          $display("[%0d] RX: st=%d->%d scnt=%d sc=%b gp=%b shr=0x%02x bc=%d rxd=%b rxv=%b pe=%b brk=%b btmr=%d",
            $time, state, next_state, sample_cnt, sample_center, glitch_pass,
            shift_reg, bit_cnt, rxd_sampled, rx_valid, parity_error,
            break_detect, break_tmr);
      end
    end
  `endif

endmodule : uart_rx
