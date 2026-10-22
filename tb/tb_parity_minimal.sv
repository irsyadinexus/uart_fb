module tb_parity_minimal();
  logic clk;
  initial clk = 0;
  always #10 clk = ~clk;

  task send_bit;
    repeat (434) @(posedge clk);
  endtask

  initial begin
    integer i;
    rxd = 1'b0; send_bit;
    for (i=0;i<8;i=i+1) begin
      rxd = 8'h55[i]; send_bit;
    end
    #10 $finish;
  end

  logic rxd;
endmodule
