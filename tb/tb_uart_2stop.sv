//==============================================================================
// tb_uart_2stop.sv - Isolate 2-stop bit test with separate reset
//==============================================================================

`timescale 1ns / 1ps
import uart_pkg::*;

module tb_uart_2stop();

  parameter CLK_PERIOD = 20;
  parameter CLK_FREQ   = 50_000_000;

  logic clk, rst_n;
  logic [15:0] cfg_baud_div;
  logic [2:0] cfg_data_bits;
  logic [1:0] cfg_parity, cfg_stop_bits;
  logic [7:0] tx_data, rx_data;
  logic tx_valid, tx_ready, tx_busy, rx_valid, rx_ready, rx_busy;
  logic framing_error, parity_error, overrun_error, break_detect, txd, rxd;
  logic irq;
  logic int_en_rx, int_en_tx, int_en_err;
  logic clr_errors;  // W1C: pulse high to clear sticky error flags

  uart_top #(.CLK_FREQ(CLK_FREQ)) u_uart (.*);
  assign rxd = txd;
  assign clr_errors = 1'b0;

  initial begin
    $dumpfile("tb_uart_2stop.vcd");
    $dumpvars(0, tb_uart_2stop);
  end

  initial clk = 0;
  always #(CLK_PERIOD/2) clk = ~clk;

  task wait_clks(input int n);
    repeat (n) @(posedge clk);
  endtask

  initial begin
    integer timeout;

    $display("=== UART 2-STOP ISOLATED TEST ===");

    // Fresh reset with 2-stop config
    rst_n = 0; tx_valid = 0; rx_ready = 1;
    int_en_rx = 0; int_en_tx = 0; int_en_err = 0;
    cfg_baud_div = CLK_FREQ/115200;
    cfg_data_bits = 3;  // 8 data bits
    cfg_parity = 0;     // none
    cfg_stop_bits = 2;  // 2 STOP BITS
    wait_clks(10);
    rst_n = 1;
    wait_clks(10);
    $display("[%0t] Ready: tx_ready=%b, half_stop=%b, num_stop=%d",
      $time, tx_ready, u_uart.u_tx.half_stop, u_uart.u_tx.num_stop_bits);
    $display("[%0t] state=%d, half_stop_cnt=%d", $time, u_uart.u_tx.state, u_uart.u_tx.half_stop_cnt);

    // Send 0x55
    $display("[%0t] Sending 0x55...", $time);
    tx_data = 8'h55; tx_valid = 1; @(posedge clk); tx_valid = 0;

    timeout = 200000;
    while (!rx_valid && timeout > 0) begin @(posedge clk); timeout = timeout - 1; end

    if (rx_valid) begin
      $display("  PASS: rx_data=0x%02x at time=%0t", rx_data, $time);
    end else begin
      $display("  FAIL: TIMEOUT");
    end
    $display("  Final state=%d tx_ready=%b half_stop_cnt=%d stop_data_done=%b stop_cnt=%d",
      u_uart.u_tx.state, tx_ready, u_uart.u_tx.half_stop_cnt,
      u_uart.u_tx.stop_data_done, u_uart.u_tx.stop_cnt);

    // Send another byte (tests if TX returns to ready)
    tx_data = 8'hAA; tx_valid = 1; @(posedge clk); tx_valid = 0;
    $display("[%0t] Second byte sent, tx_ready=%b", $time, tx_ready);

    timeout = 200000;
    while (!rx_valid && timeout > 0) begin @(posedge clk); timeout = timeout - 1; end

    if (rx_valid) begin
      $display("  PASS: rx_data=0x%02x (second byte)", rx_data);
    end else begin
      $display("  FAIL: TIMEOUT second byte");
    end

    $display("=== 2-STOP TEST COMPLETE ===");
    #10000;
    $finish;
  end

endmodule
