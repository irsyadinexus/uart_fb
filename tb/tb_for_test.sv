module tb_for_test();
  logic clk, rxd;
  logic [7:0] data;
  initial clk = 0;
  always #10 clk = ~clk;

  task send_bit;
    repeat (10) @(posedge clk);
  endtask

  initial begin
    integer i;
    data = 8'h55;
    i = 0;
    while (i < 5) begin
      rxd = data[i]; send_bit;
      i = i + 1;
    end
    #10 $finish;
  end
endmodule
