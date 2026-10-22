//==============================================================================
// tb_uart_debug.sv
// Minimal UART Testbench for Debug
// Tests a single byte at 115200 8N1
//==============================================================================

`timescale 1ns / 1ps

import uart_pkg::*;

module tb_uart_debug();

  parameter CLK_PERIOD = 20;
  parameter CLK_FREQ   = 50_000_000;

  logic        clk, rst_n;
  logic [15:0] cfg_baud_div;
  logic [2:0]  cfg_data_bits;
  logic [1:0]  cfg_parity, cfg_stop_bits;
  logic [7:0]  tx_data, rx_data;
  logic        tx_valid, tx_ready, tx_busy;
  logic        rx_valid, rx_ready, rx_busy;
  logic        framing_error, parity_error, overrun_error;
  logic        txd, rxd;
  logic        clr_errors;  // W1C: pulse high to clear sticky error flags

  uart_top #(.CLK_FREQ(CLK_FREQ)) u_uart (
    .clk, .rst_n,
    .cfg_baud_div, .cfg_data_bits, .cfg_parity, .cfg_stop_bits,
    .tx_data, .tx_valid, .tx_ready, .tx_busy,
    .rx_data, .rx_valid, .rx_ready, .rx_busy,
    .framing_error, .parity_error, .overrun_error,
    .clr_errors, .txd, .rxd
  );

  assign rxd = txd;
  assign clr_errors = 1'b0;

  initial begin
    clk = 1'b0;
    forever #(CLK_PERIOD / 2) clk = ~clk;
  end

  initial begin
    $dumpfile("tb_uart_debug.vcd");
    $dumpvars(0, tb_uart_debug);
    $display("Starting debug simulation...");

    // Reset
    rst_n = 0;
    tx_data = 0; tx_valid = 0; rx_ready = 1;
    cfg_baud_div = CLK_FREQ / 115200; // 434
    cfg_data_bits = 3; // 8 bits
    cfg_parity = 0;    // none
    cfg_stop_bits = 0; // 1 stop
    repeat (10) @(posedge clk);
    rst_n = 1;
    repeat (10) @(posedge clk);

    $display("Reset done. tx_ready=%b", tx_ready);

    // Wait for tx_ready
    repeat (1000) @(posedge clk);
    $display("After wait. tx_ready=%b, state_tx=%d", tx_ready, u_uart.u_tx.state);

    // Send byte 0x55
    tx_data = 8'h55;
    tx_valid = 1;
    @(posedge clk);
    tx_valid = 0;
    $display("Byte sent: 0x55. tx_busy=%b rx_busy=%b", tx_busy, rx_busy);

    // Wait for reception
    repeat (100000) @(posedge clk);

    if (rx_valid)
      $display("RECEIVED: 0x%02x (expected 0x55)", rx_data);
    else begin
      $display("NOT RECEIVED. tx_busy=%b rx_busy=%b framing_err=%b parity_err=%b",
               tx_busy, rx_busy, framing_error, parity_error);
      $display("TX state=%d, RX state=%d", u_uart.u_tx.state, u_uart.u_rx.state);
    end

    #10000;
    $finish;
  end

  // Monitor RX FSM state transitions
  always @(u_uart.u_rx.state) begin
    $display("[%0t] RX_STATE=%d sample_cnt=%0d sample_center=%b baud_x16=%b rxd=%b rxd_s=%b glitch=%b",
             $time, u_uart.u_rx.state, u_uart.u_rx.sample_cnt,
             u_uart.u_rx.sample_center, u_uart.u_baud_gen.baud_x16_tick,
             u_uart.u_rx.rxd_sampled, rxd,
             u_uart.u_rx.glitch_pass);
  end

endmodule
