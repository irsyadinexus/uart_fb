//==============================================================================
// tb_uart_min.sv - Diagnostic UART test
// Tests multiple byte patterns at 115200 8N1 
//==============================================================================

`timescale 1ns / 1ps
import uart_pkg::*;

module tb_uart_min();

  parameter CLK_PERIOD = 20;
  parameter CLK_FREQ   = 50_000_000;

  logic clk, rst_n;
  logic [15:0] cfg_baud_div;
  logic [2:0] cfg_data_bits;
  logic [1:0] cfg_parity, cfg_stop_bits;
  logic [7:0] tx_data, rx_data;
  logic tx_valid, tx_ready, tx_busy;
  logic rx_valid, rx_ready, rx_busy;
  logic framing_error, parity_error, overrun_error;
  logic txd, rxd;
  logic clr_errors;  // W1C: pulse high to clear sticky error flags

  uart_top #(.CLK_FREQ(CLK_FREQ)) u_uart (.*);
  assign rxd = txd;
  assign clr_errors = 1'b0;

  initial begin
    $dumpfile("tb_uart_min.vcd");
    $dumpvars(0, tb_uart_min);
  end

  initial clk = 0;
  always #(CLK_PERIOD/2) clk = ~clk;

  // Test patterns
  logic [7:0] patterns [0:7];
  initial begin
    patterns[0] = 8'h00; patterns[1] = 8'hFF;
    patterns[2] = 8'h55; patterns[3] = 8'hAA;
    patterns[4] = 8'hA5; patterns[5] = 8'h5A;
    patterns[6] = 8'hF0; patterns[7] = 8'h0F;
  end

  initial begin
    integer timeout;
    integer p;
    int pass, fail;

    $display("=== UART DIAGNOSTIC TEST ===");

    rst_n = 0; tx_valid = 0; rx_ready = 1;
    cfg_baud_div = CLK_FREQ/115200;
    cfg_data_bits = 3; cfg_parity = 0; cfg_stop_bits = 0;
    repeat (10) @(posedge clk);
    rst_n = 1;
    repeat (10) @(posedge clk);
    $display("[%0t] Ready, tx_ready=%b", $time, tx_ready);

    pass = 0; fail = 0;

    for (p = 0; p < 8; p++) begin
      // Send byte
      tx_data = patterns[p];
      tx_valid = 1;
      @(posedge clk);
      tx_valid = 0;

      timeout = 2000000;
      while (!rx_valid && timeout > 0) begin @(posedge clk); timeout = timeout - 1; end

      if (rx_valid) begin
        if (rx_data == patterns[p]) begin
          $display("  PASS: sent 0x%02x, got 0x%02x", patterns[p], rx_data);
          pass = pass + 1;
        end else begin
          $display("  FAIL: sent 0x%02x, got 0x%02x (bin %b)", patterns[p], rx_data, rx_data);
          fail = fail + 1;
        end
        rx_ready = 1;
        @(posedge clk);
        rx_ready = 0;
      end else begin
        $display("  FAIL: TIMEOUT for 0x%02x", patterns[p]);
        fail = fail + 1;
      end

      // Wait between bytes
      repeat (100) @(posedge clk);
    end

    $display("=== DIAGNOSTIC RESULT: %d pass, %d fail ===", pass, fail);
    #10000;
    $finish;
  end

endmodule
