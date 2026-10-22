//==============================================================================
// tb_uart_diag.sv - Diagnostic test with detailed TX/baud tracing
//==============================================================================

`timescale 1ns / 1ps
import uart_pkg::*;

module tb_uart_diag();

  parameter CLK_PERIOD = 20;
  parameter CLK_FREQ   = 50_000_000;

  logic clk, rst_n;
  logic [15:0] cfg_baud_div;
  logic [2:0] cfg_data_bits;
  logic [1:0] cfg_parity, cfg_stop_bits;
  logic [7:0] tx_data, rx_data;
  logic tx_valid, tx_ready, tx_busy, rx_valid, rx_ready, rx_busy;
  logic framing_error, parity_error, overrun_error, txd, rxd;
  logic clr_errors;  // W1C: pulse high to clear sticky error flags

  uart_top #(.CLK_FREQ(CLK_FREQ)) u_uart (.*);
  assign rxd = txd;
  assign clr_errors = 1'b0;

  initial begin
    $dumpfile("tb_uart_diag.vcd");
    $dumpvars(0, tb_uart_diag);
  end

  initial clk = 0;
  always #(CLK_PERIOD/2) clk = ~clk;

  task wait_clks(input int n);
    repeat (n) @(posedge clk);
  endtask

  initial begin
    integer timeout;
    integer clk_cnt;

    $display("=== UART DIAGNOSTIC (5 data bits, trace mode) ===");

    rst_n = 0; tx_valid = 0; rx_ready = 1;
    cfg_baud_div = CLK_FREQ/115200; cfg_data_bits = 0;
    cfg_parity = 0; cfg_stop_bits = 0;
    wait_clks(10); rst_n = 1; wait_clks(10);
    $display("[%0t] Ready", $time);

    // Send 0x00
    $display("[%0t] Sending 0x00...", $time);
    tx_data = 8'h00; tx_valid = 1; @(posedge clk); tx_valid = 0;

    // Poll for rx_valid, print TX state periodically
    timeout = 2000000;
    clk_cnt = 0;
    while (!rx_valid && timeout > 0) begin
      @(posedge clk);
      timeout = timeout - 1;
      clk_cnt = clk_cnt + 1;
      // Print TX state every 10000 cycles
      if ((clk_cnt % 10000) == 0) begin
        $display("[%0t] clk_cnt=%0d: state_tx=%d stop_cnt=%d baud_tick=%b baud_x16=%b",
          $time, clk_cnt, u_uart.u_tx.state, u_uart.u_tx.stop_cnt,
          u_uart.u_baud_gen.baud_tick, u_uart.u_baud_gen.baud_x16_tick);
      end
    end

    if (rx_valid) begin
      $display("  PASS: rx_data=0x%02x at clk_cnt=%0d", rx_data, clk_cnt);
      $display("  Final tx state=%d stop_cnt=%d", u_uart.u_tx.state, u_uart.u_tx.stop_cnt);
      $display("  stop_data_done=%b", u_uart.u_tx.stop_data_done);
    end else begin
      $display("  FAIL: TIMEOUT");
    end

    #10000;
    $finish;
  end

endmodule
