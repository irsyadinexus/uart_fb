//==============================================================================
// tb_uart_parity.sv — Parity Error Detection Test
// Drives uart_rx directly with UART frames containing correct and deliberately
// wrong parity bits to verify the parity_error output.
//==============================================================================

`timescale 1ns / 1ps
import uart_pkg::*;

module tb_uart_parity();

  parameter BIT_PERIOD = 434;

  logic clk, rst_n;
  logic [2:0] cfg_data_bits;
  logic [1:0] cfg_parity, cfg_stop_bits;
  logic [7:0] rx_data;
  logic rx_valid, rx_ready, rx_busy;
  logic framing_error, parity_error, overrun_error, break_detect, rxd;
  logic baud_x16_tick;
  logic [11:0] x16_cnt;
  logic clr_errors;

  // Clear sticky error flags (W1C: pulse high for one cycle)
  task clear_errors();
    clr_errors = 1'b1;
    @(posedge clk);
    clr_errors = 1'b0;
  endtask

  // Wait until rx_valid is asserted (rx_ready low holds it until we read)
  task wait_for_rx(output int tout, output logic ok);
    tout = 50000; ok = 0;
    while (!rx_valid && tout > 0) begin @(posedge clk); tout = tout - 1; end
    if (tout > 0) ok = 1;
  endtask

  // Acknowledge received data (pulse rx_ready to clear rx_valid)
  task ack_rx();
    rx_ready = 1'b1;
    @(posedge clk);
    rx_ready = 1'b0;
  endtask

  // x16 tick gen (28 cycles per tick, matching 115200 baud at 50MHz)
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin x16_cnt <= '0; baud_x16_tick <= 1'b0; end
    else begin
      baud_x16_tick <= 1'b0;
      if (x16_cnt >= 27) begin x16_cnt <= '0; baud_x16_tick <= 1'b1; end
      else x16_cnt <= x16_cnt + 1;
    end
  end

  uart_rx u_rx (
    .clk, .rst_n,
    .cfg_data_bits, .cfg_parity, .cfg_stop_bits,
    .baud_x16_tick,
    .rx_data, .rx_valid, .rx_ready, .rx_busy,
    .framing_error, .parity_error, .overrun_error, .break_detect,
    .clr_errors,
    .rxd
  );

  initial clk = 1'b0;
  always #10 clk = ~clk;

  initial begin
    $dumpfile("tb_uart_parity.vcd");
    $dumpvars(0, tb_uart_parity);
  end

  task wait_clks(input int n);
    repeat (n) @(posedge clk);
  endtask

  // Send one bit for BIT_PERIOD clock cycles
  task send_one(input logic bit_val);
    rxd = bit_val;
    repeat (BIT_PERIOD) @(posedge clk);
  endtask

  // Send a UART frame: start + data_bits + parity_bit + stop_bit
  task send_frame(input logic [7:0] data, input logic parity, input int bits);
    integer i;
    send_one(1'b0);                     // start bit
    for (i = 0; i < bits; i = i + 1) begin
      rxd = data[i];
      repeat (BIT_PERIOD) @(posedge clk);
    end
    send_one(parity);                   // parity bit
    send_one(1'b1);                     // stop bit
  endtask

  initial begin
    integer tp, tf, timeout;
    reg waited;

    tp = 0; tf = 0;
    $display("=== UART Parity Error Detection Test ===\n");

    rst_n = 1'b0; rxd = 1'b1; rx_ready = 1'b0; clr_errors = 1'b0;
    cfg_data_bits = 3'd3; cfg_parity = PARITY_EVEN; cfg_stop_bits = STOP_1;
    wait_clks(5); rst_n = 1'b1; wait_clks(10);

    // TEST 1: 0x55 EVEN correct(0) -> pe=0
    clear_errors();
    cfg_data_bits = 3'd3; cfg_parity = PARITY_EVEN; wait_clks(5);
    send_frame(8'h55, 1'b0, 8);
    wait_for_rx(timeout, waited);
    if (waited && !parity_error) begin $display("  TEST 1 PASS"); tp=tp+1; end
    else begin $display("  TEST 1 FAIL pe=%b", parity_error); tf=tf+1; end
    ack_rx();

    // TEST 2: 0x55 ODD correct(1) -> pe=0
    clear_errors();
    cfg_parity = PARITY_ODD; wait_clks(5);
    send_frame(8'h55, 1'b1, 8);
    wait_for_rx(timeout, waited);
    if (waited && !parity_error) begin $display("  TEST 2 PASS"); tp=tp+1; end
    else begin $display("  TEST 2 FAIL pe=%b", parity_error); tf=tf+1; end
    ack_rx();

    // TEST 3: 0x55 EVEN WRONG(1) -> pe=1
    clear_errors();
    cfg_parity = PARITY_EVEN; wait_clks(5);
    send_frame(8'h55, 1'b1, 8);
    wait_for_rx(timeout, waited);
    if (waited && parity_error) begin $display("  TEST 3 PASS"); tp=tp+1; end
    else begin $display("  TEST 3 FAIL pe=%b", parity_error); tf=tf+1; end
    ack_rx();

    // TEST 4: 0x55 ODD WRONG(0) -> pe=1
    clear_errors();
    cfg_parity = PARITY_ODD; wait_clks(5);
    send_frame(8'h55, 1'b0, 8);
    wait_for_rx(timeout, waited);
    if (waited && parity_error) begin $display("  TEST 4 PASS"); tp=tp+1; end
    else begin $display("  TEST 4 FAIL pe=%b", parity_error); tf=tf+1; end
    ack_rx();

    // TEST 5: 0xAA EVEN correct(0) -> pe=0
    clear_errors();
    cfg_parity = PARITY_EVEN; wait_clks(5);
    send_frame(8'hAA, 1'b0, 8);
    wait_for_rx(timeout, waited);
    if (waited && !parity_error) begin $display("  TEST 5 PASS"); tp=tp+1; end
    else begin $display("  TEST 5 FAIL pe=%b", parity_error); tf=tf+1; end
    ack_rx();

    // TEST 6: 0xAA ODD correct(1) -> pe=0
    clear_errors();
    cfg_parity = PARITY_ODD; wait_clks(5);
    send_frame(8'hAA, 1'b1, 8);
    wait_for_rx(timeout, waited);
    if (waited && !parity_error) begin $display("  TEST 6 PASS"); tp=tp+1; end
    else begin $display("  TEST 6 FAIL pe=%b", parity_error); tf=tf+1; end
    ack_rx();

    // TEST 7: 0xAA EVEN WRONG(1) -> pe=1
    clear_errors();
    cfg_parity = PARITY_EVEN; wait_clks(5);
    send_frame(8'hAA, 1'b1, 8);
    wait_for_rx(timeout, waited);
    if (waited && parity_error) begin $display("  TEST 7 PASS"); tp=tp+1; end
    else begin $display("  TEST 7 FAIL pe=%b", parity_error); tf=tf+1; end
    ack_rx();

    // TEST 8: 0xAA ODD WRONG(0) -> pe=1
    clear_errors();
    cfg_parity = PARITY_ODD; wait_clks(5);
    send_frame(8'hAA, 1'b0, 8);
    wait_for_rx(timeout, waited);
    if (waited && parity_error) begin $display("  TEST 8 PASS"); tp=tp+1; end
    else begin $display("  TEST 8 FAIL pe=%b", parity_error); tf=tf+1; end
    ack_rx();

    // TEST 9: 0x01 EVEN correct(1) -> pe=0
    clear_errors();
    cfg_parity = PARITY_EVEN; wait_clks(5);
    send_frame(8'h01, 1'b1, 8);
    wait_for_rx(timeout, waited);
    if (waited && !parity_error) begin $display("  TEST 9 PASS"); tp=tp+1; end
    else begin $display("  TEST 9 FAIL pe=%b", parity_error); tf=tf+1; end
    ack_rx();

    // TEST 10: 0x01 EVEN WRONG(0) -> pe=1
    clear_errors();
    cfg_parity = PARITY_EVEN; wait_clks(5);
    send_frame(8'h01, 1'b0, 8);
    wait_for_rx(timeout, waited);
    if (waited && parity_error) begin $display("  TEST 10 PASS"); tp=tp+1; end
    else begin $display("  TEST 10 FAIL pe=%b", parity_error); tf=tf+1; end
    ack_rx();

    // TEST 11: 0x01 ODD correct(0) -> pe=0
    clear_errors();
    cfg_parity = PARITY_ODD; wait_clks(5);
    send_frame(8'h01, 1'b0, 8);
    wait_for_rx(timeout, waited);
    if (waited && !parity_error) begin $display("  TEST 11 PASS"); tp=tp+1; end
    else begin $display("  TEST 11 FAIL pe=%b", parity_error); tf=tf+1; end
    ack_rx();

    // TEST 12: 0x01 ODD WRONG(1) -> pe=1
    clear_errors();
    cfg_parity = PARITY_ODD; wait_clks(5);
    send_frame(8'h01, 1'b1, 8);
    wait_for_rx(timeout, waited);
    if (waited && parity_error) begin $display("  TEST 12 PASS"); tp=tp+1; end
    else begin $display("  TEST 12 FAIL pe=%b", parity_error); tf=tf+1; end
    ack_rx();

    $display("");
    $display("=== RESULTS: %0d pass, %0d fail (12 tests) ===", tp, tf);
    $display("Waveform: tb_uart_parity.vcd");
    #5000; $finish;
  end

  initial begin
    #200_000_000; // 200ms timeout
    $display("TIMEOUT");
    $finish;
  end

endmodule
