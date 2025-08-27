module tb;
  logic clk,rst,read_i,write_i;
  logic rd_valid_o;
  logic [31:0] rd_data_o;
  
  
  day20 dut5(clk,rst,read_i,write_i,rd_valid_o,rd_data_o);
  
  //generate clock signal
  initial
    begin
      clk=0;
    end
  
  always #5 clk=~clk;
  
  initial
    begin
      rst=1;
      #30;
      rst=0;
      for(int i=0;i<16;i++)
        begin
          read_i=$urandom_range(0,10)%2;
          write_i=$urandom_range(0,15)%2;
          @(posedge clk); //wait for another pos edge clk before generating new values
        end
      @(posedge clk);
    end
  
  initial
    begin
      $dumpfile("test.vcd");
      $dumpvars(0);
      #200;
      $finish;
    end
endmodule
      
