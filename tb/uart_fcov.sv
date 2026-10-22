//==============================================================================
// uart_fcov.sv
// Functional Coverage Monitor for UART Controller
//
// Tracks coverage of configuration space, error events, and data transfers
// using simple counters (Iverilog-compatible). Reports summary at end of sim.
//==============================================================================

`timescale 1ns / 1ps

import uart_pkg::*;

module uart_fcov (
  // System
  input logic       clk,
  input logic       rst_n,

  // Configuration
  input logic [2:0]  cfg_data_bits,
  input logic [1:0]  cfg_parity,
  input logic [1:0]  cfg_stop_bits,
  input logic [15:0] cfg_baud_div,

  // TX Interface
  input logic [7:0]  tx_data,
  input logic        tx_valid,
  input logic        tx_ready,
  input logic        tx_busy,

  // RX Interface
  input logic [7:0]  rx_data,
  input logic        rx_valid,
  input logic        rx_ready,
  input logic        rx_busy,

  // Error Status
  input logic        framing_error,
  input logic        parity_error,
  input logic        overrun_error,
  input logic        break_detect,

  // Interrupt
  input logic        irq,
  input logic        int_en_rx,
  input logic        int_en_tx,
  input logic        int_en_err
);

  // ===========================================================================
  // Configuration Space Coverage (counter-based)
  // ===========================================================================
  int cfg_data_cnt[4];        // [0]=5bit, [1]=6bit, [2]=7bit, [3]=8bit
  int cfg_parity_cnt[3];      // [0]=none, [1]=even, [2]=odd
  int cfg_stop_cnt[3];        // [0]=1, [1]=1.5, [2]=2
  int cfg_baud_cnt[9];        // known baud divisors for 50 MHz

  // ---------------------------------------------------------------------------
  // Per-Combination (Cross) Coverage
  // ---------------------------------------------------------------------------
  // Tracks which (data_bits × parity × stop_bits) combinations have been
  // exercised. Uses flat 1D encoding for Iverilog compatibility:
  //   index = data_bits*9 + parity*3 + stop_bits
  // This gives 4*3*3 = 36 bins covering all frame-format combinations.
  //
  // Also tracks (baud × data_bits) cross coverage:
  //   index = baud_idx*4 + data_bits
  // This gives 9*4 = 36 bins covering all tested baud/data combinations.
  // ---------------------------------------------------------------------------
  int cfg_cross_dps[36];            // data×parity×stop cross coverage
  int cfg_cross_baud_data[36];      // baud×data cross coverage
  int cfg_cross_baud_parity[27];    // baud×parity cross coverage (9*3=27)
  int cfg_cross_baud_stop[27];      // baud×stop cross coverage (9*3=27)

  // Compute flat cross index from configuration values
  function int cross_idx(input logic [2:0] db, input logic [1:0] pa, input logic [1:0] st);
    return (db <= 3 ? db : 3) * 9 + (pa <= 2 ? pa : 0) * 3 + (st <= 2 ? st : 0);
  endfunction

  // Map baud divisor to index (0-8), returns 0 for unrecognized divisors
  function int baud_idx(input logic [15:0] div);
    case (div)
      5208:  return 0;  // 9600
      2604:  return 1;  // 19200
      1302:  return 2;  // 38400
      868:   return 3;  // 57600
      434:   return 4;  // 115200
      217:   return 5;  // 230400
      108:   return 6;  // 460800
      54:    return 7;  // 921600
      16:    return 8;  // 3M
      default: return 0;
    endcase
  endfunction

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int i = 0; i < 4; i++) cfg_data_cnt[i] <= 0;
      for (int i = 0; i < 3; i++) cfg_parity_cnt[i] <= 0;
      for (int i = 0; i < 3; i++) cfg_stop_cnt[i] <= 0;
      for (int i = 0; i < 9; i++) cfg_baud_cnt[i] <= 0;
      for (int i = 0; i < 36; i++) begin
        cfg_cross_dps[i] <= 0;
        cfg_cross_baud_data[i] <= 0;
      end
      for (int i = 0; i < 27; i++) begin
        cfg_cross_baud_parity[i] <= 0;
        cfg_cross_baud_stop[i] <= 0;
      end
    end else if (tx_valid && tx_ready) begin
      // Track configuration at each TX transaction
      if (cfg_data_bits <= 3) cfg_data_cnt[cfg_data_bits] <= cfg_data_cnt[cfg_data_bits] + 1;
      if (cfg_parity <= 2)    cfg_parity_cnt[cfg_parity] <= cfg_parity_cnt[cfg_parity] + 1;
      if (cfg_stop_bits <= 2) cfg_stop_cnt[cfg_stop_bits] <= cfg_stop_cnt[cfg_stop_bits] + 1;

      // Track data×parity×stop cross coverage (compute index once)
      begin
        int ci = cross_idx(cfg_data_bits, cfg_parity, cfg_stop_bits);
        cfg_cross_dps[ci] <= cfg_cross_dps[ci] + 1;
      end

      // Track baud×data cross coverage (compute index once)
      begin
        int bi  = baud_idx(cfg_baud_div);
        int di  = (cfg_data_bits <= 3) ? cfg_data_bits : 3;
        int bdi = bi * 4 + di;
        cfg_cross_baud_data[bdi] <= cfg_cross_baud_data[bdi] + 1;
      end

      // Track baud×parity cross coverage
      begin
        int bi  = baud_idx(cfg_baud_div);
        int pi  = (cfg_parity <= 2) ? cfg_parity : 0;
        int bpi = bi * 3 + pi;
        cfg_cross_baud_parity[bpi] <= cfg_cross_baud_parity[bpi] + 1;
      end

      // Track baud×stop cross coverage
      begin
        int bi  = baud_idx(cfg_baud_div);
        int si  = (cfg_stop_bits <= 2) ? cfg_stop_bits : 0;
        int bsi = bi * 3 + si;
        cfg_cross_baud_stop[bsi] <= cfg_cross_baud_stop[bsi] + 1;
      end

      // Baud rate tracking
      case (cfg_baud_div)
        5208:  cfg_baud_cnt[0] <= cfg_baud_cnt[0] + 1;  // 9600
        2604:  cfg_baud_cnt[1] <= cfg_baud_cnt[1] + 1;  // 19200
        1302:  cfg_baud_cnt[2] <= cfg_baud_cnt[2] + 1;  // 38400
        868:   cfg_baud_cnt[3] <= cfg_baud_cnt[3] + 1;  // 57600
        434:   cfg_baud_cnt[4] <= cfg_baud_cnt[4] + 1;  // 115200
        217:   cfg_baud_cnt[5] <= cfg_baud_cnt[5] + 1;  // 230400
        108:   cfg_baud_cnt[6] <= cfg_baud_cnt[6] + 1;  // 460800
        54:    cfg_baud_cnt[7] <= cfg_baud_cnt[7] + 1;  // 921600
        16:    cfg_baud_cnt[8] <= cfg_baud_cnt[8] + 1;  // 3M
        default: ;  // Don't track unrecognized divisors
      endcase
    end
  end

  // ===========================================================================
  // Busy Tracking (for coverage visualization)
  // ===========================================================================
  int tx_busy_cycles;
  int rx_busy_cycles;
  int idle_cycles;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tx_busy_cycles <= 0;
      rx_busy_cycles <= 0;
      idle_cycles <= 0;
    end else begin
      if (tx_busy) tx_busy_cycles <= tx_busy_cycles + 1;
      if (rx_busy) rx_busy_cycles <= rx_busy_cycles + 1;
      if (!tx_busy && !rx_busy) idle_cycles <= idle_cycles + 1;
    end
  end

  // ===========================================================================
  // Error Event Tracking
  // ===========================================================================
  int framing_err_cnt;
  int parity_err_cnt;
  int overrun_err_cnt;
  int break_cnt;

  logic framing_error_d1, parity_error_d1, overrun_error_d1, break_detect_d1;
  logic irq_d1;
  int    irq_cnt;
  int    int_en_cfg_cnt;  // Track how many configurations used interrupts

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      framing_error_d1 <= 1'b0;
      parity_error_d1  <= 1'b0;
      overrun_error_d1 <= 1'b0;
      break_detect_d1  <= 1'b0;
      irq_d1           <= 1'b0;
      framing_err_cnt  <= 0;
      parity_err_cnt   <= 0;
      overrun_err_cnt  <= 0;
      break_cnt        <= 0;
      irq_cnt          <= 0;
      int_en_cfg_cnt   <= 0;
    end else begin
      framing_error_d1 <= framing_error;
      parity_error_d1  <= parity_error;
      overrun_error_d1 <= overrun_error;
      break_detect_d1  <= break_detect;
      irq_d1           <= irq;

      if ( framing_error && !framing_error_d1) framing_err_cnt <= framing_err_cnt + 1;
      if ( parity_error  && !parity_error_d1)  parity_err_cnt  <= parity_err_cnt  + 1;
      if ( overrun_error && !overrun_error_d1) overrun_err_cnt <= overrun_err_cnt + 1;
      if ( break_detect  && !break_detect_d1)  break_cnt       <= break_cnt + 1;
      if ( irq && !irq_d1)                     irq_cnt         <= irq_cnt + 1;
    end
  end

  // Track int_en usage (sampled on TX transactions)
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      int_en_cfg_cnt <= 0;
    end else if (tx_valid && tx_ready) begin
      if (int_en_rx || int_en_tx || int_en_err)
        int_en_cfg_cnt <= int_en_cfg_cnt + 1;
    end
  end

  // ===========================================================================
  // Transfer Tracking
  // ===========================================================================
  int tx_transfer_cnt;
  int rx_transfer_cnt;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tx_transfer_cnt <= 0;
      rx_transfer_cnt <= 0;
    end else begin
      if (tx_valid && tx_ready) tx_transfer_cnt <= tx_transfer_cnt + 1;
      if (rx_valid && rx_ready) rx_transfer_cnt <= rx_transfer_cnt + 1;
    end
  end

  // ===========================================================================
  // Coverage Summary (displayed at end of simulation or on request)
  // ===========================================================================
  task print_summary();
    int cfg_data_covered, cfg_parity_covered, cfg_stop_covered, cfg_baud_covered;

    $display("");
    $display("============================================================");
    $display("  Functional Coverage Summary");
    $display("============================================================");

    // Data bits coverage
    cfg_data_covered = 0;
    for (int i = 0; i < 4; i++) if (cfg_data_cnt[i] > 0) cfg_data_covered++;
    $display("  Data bits (5/6/7/8):     %0d/4 configurations exercised",
             cfg_data_covered);
    for (int i = 0; i < 4; i++)
      if (cfg_data_cnt[i] > 0)
        $display("    %d-bit: %0d transfers", i + 5, cfg_data_cnt[i]);

    // Parity coverage
    cfg_parity_covered = 0;
    for (int i = 0; i < 3; i++) if (cfg_parity_cnt[i] > 0) cfg_parity_covered++;
    $display("  Parity (none/even/odd):  %0d/3 modes exercised",
             cfg_parity_covered);
    for (int i = 0; i < 3; i++) begin
      string pname;
      if (i == 0) pname = "None";
      else if (i == 1) pname = "Even";
      else pname = "Odd";
      if (cfg_parity_cnt[i] > 0)
        $display("    %s: %0d transfers", pname, cfg_parity_cnt[i]);
    end

    // Stop bits coverage
    cfg_stop_covered = 0;
    for (int i = 0; i < 3; i++) if (cfg_stop_cnt[i] > 0) cfg_stop_covered++;
    $display("  Stop bits (1/1.5/2):     %0d/3 modes exercised",
             cfg_stop_covered);

    // Baud rate coverage
    cfg_baud_covered = 0;
    for (int i = 0; i < 9; i++) if (cfg_baud_cnt[i] > 0) cfg_baud_covered++;
    $display("  Baud rates:              %0d/9 divisors exercised",
             cfg_baud_covered);

    // -----------------------------------------------------------------------
    // Per-Combination (Cross) Coverage: data × parity × stop
    // -----------------------------------------------------------------------
    begin
      int cross_covered, cross_total;
      cross_total = 36;
      cross_covered = 0;
      for (int i = 0; i < cross_total; i++) begin
        if (cfg_cross_dps[i] > 0) cross_covered++;
      end
      $display("  Cross (data×parity×stop):  %0d/%d combinations exercised (%0d%%)",
               cross_covered, cross_total, (cross_covered * 100) / cross_total);

      if (cross_covered < cross_total) begin
        $display("    Missing combinations:");
        for (int d = 0; d < 4; d++) begin
          for (int p = 0; p < 3; p++) begin
            for (int s = 0; s < 3; s++) begin
              int idx = d * 9 + p * 3 + s;
              if (cfg_cross_dps[idx] == 0) begin
                string pname;
                if (p == 0) pname = "N";
                else if (p == 1) pname = "E";
                else pname = "O";
                $display("      %d-bit %s %s", d + 5, pname,
                         (s == 0) ? "1" : (s == 1) ? "1.5" : "2");
              end
            end
          end
        end
      end else begin
        $display("    All 36 (data×parity×stop) combos exercised");
      end
    end

    // -----------------------------------------------------------------------
    // Per-Combination (Cross) Coverage: baud × data
    // -----------------------------------------------------------------------
    begin
      int bd_covered, bd_total;
      string baud_names[9];
      baud_names[0] = "9600"; baud_names[1] = "19200"; baud_names[2] = "38400";
      baud_names[3] = "57600"; baud_names[4] = "115200"; baud_names[5] = "230400";
      baud_names[6] = "460800"; baud_names[7] = "921600"; baud_names[8] = "3M";

      bd_total = 36;
      bd_covered = 0;
      for (int i = 0; i < bd_total; i++) begin
        if (cfg_cross_baud_data[i] > 0) bd_covered++;
      end
      $display("  Cross (baud×data):         %0d/%d combinations exercised (%0d%%)",
               bd_covered, bd_total, (bd_covered * 100) / bd_total);

      if (bd_covered < bd_total) begin
        $display("    Missing combinations:");
        for (int b = 0; b < 9; b++) begin
          for (int d = 0; d < 4; d++) begin
            if (cfg_cross_baud_data[b * 4 + d] == 0 && cfg_baud_cnt[b] > 0) begin
              // Only report missing combos for baud rates that are actually used
              $display("      %s %d-bit", baud_names[b], d + 5);
            end
          end
        end
      end else begin
        $display("    All 36 (baud×data) combos exercised");
      end
    end

    // -----------------------------------------------------------------------
    // Per-Combination (Cross) Coverage: baud × parity
    // -----------------------------------------------------------------------
    begin
      int bp_covered;
      string baud_names[9];
      baud_names[0] = "9600"; baud_names[1] = "19200"; baud_names[2] = "38400";
      baud_names[3] = "57600"; baud_names[4] = "115200"; baud_names[5] = "230400";
      baud_names[6] = "460800"; baud_names[7] = "921600"; baud_names[8] = "3M";

      bp_covered = 0;
      for (int i = 0; i < 27; i++) begin
        if (cfg_cross_baud_parity[i] > 0) bp_covered++;
      end
      $display("  Cross (baud×parity):       %0d/27 combinations exercised (%0d%%)",
               bp_covered, (bp_covered * 100) / 27);

      if (bp_covered < 27) begin
        $display("    Missing combinations:");
        for (int b = 0; b < 9; b++) begin
          for (int p = 0; p < 3; p++) begin
            if (cfg_cross_baud_parity[b * 3 + p] == 0 && cfg_baud_cnt[b] > 0) begin
              string pname;
              if (p == 0) pname = "None";
              else if (p == 1) pname = "Even";
              else pname = "Odd";
              $display("      %s %s", baud_names[b], pname);
            end
          end
        end
      end else begin
        $display("    All 27 (baud×parity) combos exercised");
      end
    end

    // -----------------------------------------------------------------------
    // Per-Combination (Cross) Coverage: baud × stop
    // -----------------------------------------------------------------------
    begin
      int bs_covered;
      string baud_names[9];
      baud_names[0] = "9600"; baud_names[1] = "19200"; baud_names[2] = "38400";
      baud_names[3] = "57600"; baud_names[4] = "115200"; baud_names[5] = "230400";
      baud_names[6] = "460800"; baud_names[7] = "921600"; baud_names[8] = "3M";

      bs_covered = 0;
      for (int i = 0; i < 27; i++) begin
        if (cfg_cross_baud_stop[i] > 0) bs_covered++;
      end
      $display("  Cross (baud×stop):         %0d/27 combinations exercised (%0d%%)",
               bs_covered, (bs_covered * 100) / 27);

      if (bs_covered < 27) begin
        $display("    Missing combinations:");
        for (int b = 0; b < 9; b++) begin
          for (int s = 0; s < 3; s++) begin
            if (cfg_cross_baud_stop[b * 3 + s] == 0 && cfg_baud_cnt[b] > 0) begin
              $display("      %s %s", baud_names[b],
                       (s == 0) ? "1 stop" : (s == 1) ? "1.5 stop" : "2 stop");
            end
          end
        end
      end else begin
        $display("    All 27 (baud×stop) combos exercised");
      end
    end

    // Error events
    $display("  Error events:");
    $display("    Framing errors:  %0d", framing_err_cnt);
    $display("    Parity errors:   %0d", parity_err_cnt);
    $display("    Overrun errors:  %0d", overrun_err_cnt);
    $display("    Break detects:   %0d", break_cnt);

    // Interrupt events
    $display("  Interrupt events:         IRQ=%0d", irq_cnt);
    $display("  IntEn transactions:       %0d", int_en_cfg_cnt);

    // Transfers
    $display("  Data transfers:          TX=%0d, RX=%0d",
             tx_transfer_cnt, rx_transfer_cnt);

    // Busy cycles
    $display("  Busy cycles:             TX=%0d, RX=%0d, Idle=%0d",
             tx_busy_cycles, rx_busy_cycles, idle_cycles);

    $display("============================================================");
  endtask

  // print_summary is called from tb_uart.sv before $fatal (fail case)
  // or before $finish (pass case). The following final block provides
  // a simpler fallback for other testbenches that instantiate uart_fcov
  // but don't call print_summary() explicitly.
  final begin
    int cross_covered, bd_covered, bp_covered, bs_covered;
    cross_covered = 0; bd_covered = 0; bp_covered = 0; bs_covered = 0;
    for (int i = 0; i < 36; i++) begin
      if (cfg_cross_dps[i] > 0) cross_covered++;
      if (cfg_cross_baud_data[i] > 0) bd_covered++;
    end
    for (int i = 0; i < 27; i++) begin
      if (cfg_cross_baud_parity[i] > 0) bp_covered++;
      if (cfg_cross_baud_stop[i] > 0) bs_covered++;
    end
    $display("");
    $display("=== FCOV: data=%0d/4 par=%0d/3 stop=%0d/3 baud=%0d/9 " +
             "dps=%0d/36 bd=%0d/36 bp=%0d/27 bs=%0d/27 " +
             "errs=%0d/%0d/%0d/%0d irq=%0d tx=%0d rx=%0d ===",
             (cfg_data_cnt[0]>0)+(cfg_data_cnt[1]>0)+(cfg_data_cnt[2]>0)+(cfg_data_cnt[3]>0),
             (cfg_parity_cnt[0]>0)+(cfg_parity_cnt[1]>0)+(cfg_parity_cnt[2]>0),
             (cfg_stop_cnt[0]>0)+(cfg_stop_cnt[1]>0)+(cfg_stop_cnt[2]>0),
             (cfg_baud_cnt[0]>0)+(cfg_baud_cnt[1]>0)+(cfg_baud_cnt[2]>0)+(cfg_baud_cnt[3]>0)+
             (cfg_baud_cnt[4]>0)+(cfg_baud_cnt[5]>0)+(cfg_baud_cnt[6]>0)+(cfg_baud_cnt[7]>0)+(cfg_baud_cnt[8]>0),
             cross_covered, bd_covered, bp_covered, bs_covered,
             framing_err_cnt, parity_err_cnt, overrun_err_cnt, break_cnt, irq_cnt,
             tx_transfer_cnt, rx_transfer_cnt);
  end

endmodule : uart_fcov
