// Code your design here






//top module (interfacing all the read/write system connecting all the systems
module day20 (clk,rst,read_i,write_i,rd_valid_o,rd_data_o);
  input wire clk,rst,read_i,write_i;
  output reg rd_valid_o;
  output reg [31:0] rd_data_o;
  
  
  //define the reg present in the input and output ports of other modules
  
  //
  
//Fixed priority arbiter
  reg r_g,w_g;
  
  //instantiate the module
  day14 #(.NUM_PORTS(2)) dut1({read_i,write_i},{r_g,w_g});  //{read_i,write_i} = input
  
  
  //for fifo
  logic push_i;
  logic [1:0] push_data_i;
  logic pop_i;
  logic [1:0] pop_dat_o;
  logic full;
  logic empty;
  
  //for apb_master
  logic psel_o;
  logic penable_o;
  logic [31:0] paddr_o;
  logic pwrite_o;
  logic [31:0] pwdata_o;
  logic pready_i;
  logic [31:0] prdata_i;
  //define the push signal
  assign push_i=|{r_g,w_g}; //even if it is read or write request(for apb), push into the fifo
  
  assign pop_i=!empty & ~(psel_o & penable_o); //apbm will read from fifo only when not empty and when not in enable state(sel=1 and enable=1)
  
  //define push data
  assign push_data_i={r_g,w_g}; //output of the priority encoder (decides which operation should the apbm perform
  
  
  //Fifo instantiation
  day19 #(.DEPTH(16),.DATA_W(2)) dut2(clk,rst,push_i,push_data_i,pop_i,pop_data_o,full,empty);
  
  
  /*module day16 (
  input       wire        clk,
  input       wire        reset,

  input       wire[1:0]   cmd_i,

  output      wire        psel_o,
  output      wire        penable_o,
  output      wire[31:0]  paddr_o,
  output      wire        pwrite_o,
  output      wire[31:0]  pwdata_o,
  input       wire        pready_i,
  input       wire[31:0]  prdata_i
);*/

  
  //apbm instantiation
  day16 dut3(clk,rst,pop_data_o,psel_o,penable_o,paddr_o[9:0],pwrite_o,pwdata_o,pready_i,prdata_i);
  
  
  //declare reg for slave
  /*module day18 (
  input         wire        clk,
  input         wire        reset,

  input         wire        psel_i,
  input         wire        penable_i,
  input         wire[9:0]   paddr_i,
  input         wire        pwrite_i,
  input         wire[31:0]  pwdata_i,
  output        wire[31:0]  prdata_o,
  output        wire        pready_o
);*/
  
  //declare slave reg
  logic psel_i,penable_i;
  logic [9:0] paddr_i;
  logic pwrite_i;
  logic [31:0] pwdata_i;
  logic [31:0] prdata_o;
  logic pready_o;
  
  day18 dut4(clk,rst,psel_i,penable_i,paddr_i,pwrite_i,pwdata_i,prdata_o,pready_o);
  
  
endmodule
  
module day14 #(
  parameter NUM_PORTS = 4
)(
    input       wire[NUM_PORTS-1:0] req_i,
    output      wire[NUM_PORTS-1:0] gnt_o   // One-hot grant signal
);
  // Port[0] has highest priority
  assign gnt_o[0] = req_i[0];

  genvar i;
  for (i=1; i<NUM_PORTS; i=i+1) begin
    assign gnt_o[i] = req_i[i] & ~(|gnt_o[i-1:0]);
  end

endmodule


//fifo
module day19 #(
  parameter DEPTH   = 4,
  parameter DATA_W  = 1
)(
  input         wire              clk,
  input         wire              reset,

  input         wire              push_i,
  input         wire[DATA_W-1:0]  push_data_i,

  input         wire              pop_i,
  output        wire[DATA_W-1:0]  pop_data_o,

  output        wire              full_o,
  output        wire              empty_o
);

  typedef enum logic[1:0] {ST_PUSH = 2'b01,
                           ST_POP  = 2'b10,
                           ST_BOTH = 2'b11} fifo_state_t;

  parameter PTR_W = $clog2(DEPTH);

  logic [PTR_W:0] nxt_rd_ptr;
  logic [PTR_W:0] rd_ptr_q;
  logic [PTR_W:0] nxt_wr_ptr;
  logic [PTR_W:0] wr_ptr_q;

  logic [DATA_W-1:0] fifo_pop_data;

  assign pop_data_o = fifo_pop_data;

  // Fifo storage
  logic [DEPTH-1:0] [DATA_W-1:0] fifo_mem;

  // Flops for pointer
  always_ff @(posedge clk or posedge reset)
    if (reset) begin
      rd_ptr_q <= {PTR_W+1{1'b0}};
      wr_ptr_q <= {PTR_W+1{1'b0}};
    end else begin
      rd_ptr_q <= nxt_rd_ptr;
      wr_ptr_q <= nxt_wr_ptr;
    end

  // Fifo state based on push/pop
  always_comb begin
    nxt_rd_ptr = rd_ptr_q;
    nxt_wr_ptr = wr_ptr_q;
    fifo_pop_data = fifo_mem[rd_ptr_q[PTR_W-1:0]];
    case ({pop_i, push_i})
      ST_PUSH: begin
        // Increment the write pointer
        nxt_wr_ptr = wr_ptr_q + {{PTR_W{1'b0}}, 1'b1};
      end
      ST_POP: begin
        // Increment the read pointer
        nxt_rd_ptr = rd_ptr_q + {{PTR_W{1'b0}}, 1'b1};
        // Drive the pop data
        fifo_pop_data = fifo_mem[rd_ptr_q[PTR_W-1:0]];
      end
      ST_BOTH: begin
        nxt_wr_ptr = wr_ptr_q + {{PTR_W{1'b0}}, 1'b1};
        nxt_rd_ptr = rd_ptr_q + {{PTR_W{1'b0}}, 1'b1};
      end
    endcase
  end

  // Flops for fifo storage
  always_ff @(posedge clk)
    if (push_i)
      fifo_mem[wr_ptr_q[PTR_W-1:0]] <= push_data_i;

  // Full when wrap bits are not equal
  assign full_o = (rd_ptr_q[PTR_W] != wr_ptr_q[PTR_W]) &
                  (rd_ptr_q[PTR_W-1:0] == wr_ptr_q[PTR_W-1:0]);

  assign empty_o = (rd_ptr_q[PTR_W:0] == wr_ptr_q[PTR_W:0]);

endmodule




//APBM
module day16 (
  input       wire        clk,
  input       wire        reset,

  input       wire[1:0]   cmd_i,

  output      wire        psel_o,
  output      wire        penable_o,
  output      wire[31:0]  paddr_o,
  output      wire        pwrite_o,
  output      wire[31:0]  pwdata_o,
  input       wire        pready_i,
  input       wire[31:0]  prdata_i
);

  // Enum for the APB state
  typedef enum logic[1:0] {ST_IDLE = 2'b00, ST_SETUP = 2'b01, ST_ACCESS = 2'b10} apb_state_t;

  apb_state_t nxt_state;
  apb_state_t state_q;

  logic[31:0] rdata_q;

  always_ff @(posedge clk or posedge reset)
    if (reset)
      state_q <= ST_IDLE;
    else
      state_q <= nxt_state;

  always_comb begin
    nxt_state = state_q;
    case (state_q)
      ST_IDLE   : if (|cmd_i) nxt_state = ST_SETUP; else nxt_state = ST_IDLE;
      ST_SETUP  : nxt_state = ST_ACCESS;
      ST_ACCESS : begin
        if (pready_i) nxt_state = ST_IDLE;
      end
      default   : nxt_state = state_q;
    endcase
  end

  assign psel_o     = (state_q == ST_SETUP) | (state_q == ST_ACCESS);
  assign penable_o  = (state_q == ST_ACCESS);
  assign pwrite_o   = cmd_i[1];
  assign paddr_o    = 32'hDEAD_CAFE;
  assign pwdata_o   = rdata_q + 32'h1;

  // Capture the read data to store it for the next write
  always_ff @(posedge clk or posedge reset)
    if (reset)
      rdata_q <= 32'h0;
    else if (penable_o && pready_i)
      rdata_q <= prdata_i;

endmodule


//APBS
module day18 (
  input         wire        clk,
  input         wire        reset,

  input         wire        psel_i,
  input         wire        penable_i,
  input         wire[9:0]   paddr_i,
  input         wire        pwrite_i,
  input         wire[31:0]  pwdata_i,
  output        wire[31:0]  prdata_o,
  output        wire        pready_o
);

  // Valid APB request
  logic apb_req;

  assign apb_req = psel_i & penable_i;

  // Instantiate the memory interface
  day17 DAY17 (
    .clk            (clk),
    .reset          (reset),
    .req_i          (apb_req),
    .req_rnw_i      (~pwrite_i),
    .req_addr_i     (paddr_i),
    .req_wdata_i    (pwdata_i),
    .req_ready_o    (pready_o),
    .req_rdata_o    (prdata_o)
  );

endmodule

//memory interface for apb slave
module day17 (
  input       wire        clk,
  input       wire        reset,

  input       wire        req_i,
  input       wire        req_rnw_i,    // 1 - read, 0 - write
  input       wire[3:0]   req_addr_i,
  input       wire[31:0]  req_wdata_i,
  output      wire        req_ready_o,
  output      wire[31:0]  req_rdata_o
);

  // Memory array
`ifdef FORMAL
  logic [31:0] mem[15:0];
`else
  logic [15:0][31:0] mem;
`endif

  logic mem_rd;
  logic mem_wr;

  logic req_rising_edge;

  logic [3:0] lfsr_val;
  logic [3:0] count;

  assign mem_rd = req_i &  req_rnw_i;
  assign mem_wr = req_i & ~req_rnw_i;

  // Detect a rising edge on the req_i
  day3 DAY3 (
    .clk            (clk),
    .reset          (reset),
    .a_i            (req_i),
    .rising_edge_o  (req_rising_edge),
    .falling_edge_o (/* Not needed */)
  );

  // Load a counter with random value on the rising edge
  logic[3:0] count_ff;
  logic[3:0] nxt_count;

  always_ff @(posedge clk or posedge reset)
    if (reset)
      count_ff <= 4'h0;
    else
      count_ff <= nxt_count;

  assign nxt_count = req_rising_edge ? lfsr_val:
                                       count_ff + 4'h1;

  assign count = count_ff;

  // Generate a random load value
  day7 DAY7 (
    .clk            (clk),
    .reset          (reset),
    .lfsr_o         (lfsr_val)
  );

  // Write into the mem when the counter is 0
  always_ff @(posedge clk)
    if (mem_wr & ~|count)
      mem[req_addr_i] <= req_wdata_i;

  // Read directly
  assign req_rdata_o = mem[req_addr_i] & {32{mem_rd}};

  // Assert ready only when counter is at 0
  // This will add random delays on when memory gives the ready
  assign req_ready_o = ~|count;

endmodule


module day3 (
  input     wire    clk,
  input     wire    reset,

  input     wire    a_i,

  output    wire    rising_edge_o,
  output    wire    falling_edge_o
);

  logic a_ff;

  always_ff @(posedge clk or posedge reset)
    if (reset)
      a_ff <= 1'b0;
    else
      a_ff <= a_i;

  // Rising edge when delayed signal is 0 but current is 1
  assign rising_edge_o = ~a_ff & a_i;

  // Falling edge when delayed signal is 1 but current is 0
  assign falling_edge_o = a_ff & ~a_i;

endmodule


module day7 (
  input     wire      clk,
  input     wire      reset,

  output    wire[3:0] lfsr_o
);

  logic [3:0] lfsr_ff;
  logic [3:0] nxt_lfsr;

  always_ff @(posedge clk or posedge reset)
    if (reset)
      lfsr_ff <= 4'hE;
    else
      lfsr_ff <= nxt_lfsr;

  assign nxt_lfsr = {lfsr_ff[2:0], lfsr_ff[1] ^ lfsr_ff[3]};

  assign lfsr_o = lfsr_ff;

endmodule

