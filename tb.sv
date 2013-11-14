// test bit stream encoder
module bitStreamEncoder_tb;
  logic        clk, rst;
  logic        pkt_avail;
  logic [7:0]  pid_in;
  logic [6:0]  addr_in;
  logic [3:0]  endp_in;
  logic [63:0] data_in;
  logic        stall;
  logic        bit_out, start, last;

  logic [95:0]  result;   // largest is SYNC + PID + DATA0 + CRC16

  bitStreamEncoder dut(.*);

  initial begin
    clk = 1;
    forever #1 clk = ~clk;
  end

  // used for keeping track of the bit stream
  // note that result is always 1 clock cycle late!
  always_ff @(posedge clk, posedge rst) begin
    if (rst)
      result <= 0;
    else if (~stall) begin
      result <= result << 1;
      result[0] <= bit_out;
    end
  end

  initial begin
    rst <= 1; @(posedge clk);
    rst <= 0; @(posedge clk);
    // test sending OUT to endpoint 4, data = CAFEBABEDEADBEEF
    $monitor($time,, "stall = %b, start = %b, last = %b, result = %h",
                      stall, start, last, result);
    stall <= 0;
    pid_in <= 8'b1110_0001; addr_in <= 5; endp_in <= 4;
    pkt_avail <= 1;
    $display("SENDING OUT to endpoint 4");
    @(posedge clk);
    pkt_avail <= 0;
    repeat (33) @(posedge clk);
    // test stall
    rst <= 1; @(posedge clk);
    rst <= 0; @(posedge clk);
    pkt_avail <= 1;
    $display("SENDING OUT to endpoint 4 with stall");
    @(posedge clk);
    pkt_avail <= 0;
    repeat (24) @(posedge clk);
    stall <= 1; repeat (5) @(posedge clk);
    stall <= 0; repeat (9) @(posedge clk);
    // test sending DATA
    $display("SENDING DATA = CAFEBABEDEADBEEF");
    pid_in <= 8'b1100_0011; data_in <= 64'hCAFEBABEDEADBEEF;
    pkt_avail <= 1;
    @(posedge clk);
    pkt_avail <= 0;
    repeat (97) @(posedge clk);
    // test sending ACK
    rst <= 1; @(posedge clk);
    rst <= 0; @(posedge clk);
    $display("SENDING ACK");
    pid_in <= 8'b1101_0010;
    pkt_avail <= 1;
    @(posedge clk);
    pkt_avail <= 0;
    repeat (17) @(posedge clk);
    $finish;
  end

endmodule

// test bse + bitstuffing + nrzi
module big_tb;
  logic clk, rst;
  logic pkt_avail;
  logic [7:0]  pid_in;
  logic [6:0]  addr_in;
  logic [3:0]  endp_in;
  logic [63:0] data_in;
  logic        stall;
  logic        start, last;
  logic        raw_bit_stream, stuffed_bit_stream, stream_out;

  logic [109:0]  result;   // largest is SYNC + PID + DATA0 + CRC16

  bitStreamEncoder dut0(.*, .bit_out(raw_bit_stream));
  bitStuffer dut1(.*, .bit_in(raw_bit_stream), .bit_out(stuffed_bit_stream));
  nrzi dut2(.*, .bit_stream(stuffed_bit_stream));

  initial begin
    clk = 1;
    forever #1 clk = ~clk;
  end

  // used for keeping track of the bit stream
  // note that result is always 1 clock cycle late!
  always_ff @(posedge clk, posedge rst) begin
    if (rst)
      result <= 0;
    else begin
      result <= result << 1;
      result[0] <= stream_out;
    end
  end

  initial begin
    rst <= 1; @(posedge clk);
    pid_in <= 0; addr_in <= 0; endp_in <= 0; data_in <= 0; pkt_avail <= 0;
    rst <= 0; @(posedge clk);
    // test sending OUT to endpoint 4, data = CAFEBABEDEADBEEF
    $monitor($time,, "stall = %b, start = %b, last = %b, result = %h",
                      stall, start, last, result);
    pid_in <= 8'b1110_0001; addr_in <= 5; endp_in <= 4;
    pkt_avail <= 1;
    $display("SENDING OUT to endpoint 4");
    @(posedge clk);
    pkt_avail <= 0;
    repeat (40) @(posedge clk);
    // test sending DATA
    $display("SENDING DATA = CAFEBABEDEADBEEF");
    pid_in <= 8'b1100_0011; data_in <= 64'hCAFEBABEDEADBEEF;
    pkt_avail <= 1;
    @(posedge clk);
    pkt_avail <= 0;
    repeat (110) @(posedge clk);
    // test sending ACK
    $display("SENDING ACK");
    pid_in <= 8'b1101_0010;
    pkt_avail <= 1;
    @(posedge clk);
    pkt_avail <= 0;
    repeat (17) @(posedge clk);
    $finish;
  end

endmodule

// test bit stuffing
module bitStuffer_tb;
  logic clk, rst;
  logic pkt_avail;
  logic start;
  logic last;
  logic bit_in;
  logic bit_out;
  logic stall;

  bitStuffer dut(.*);

  initial begin
    clk = 0;
    forever #5 clk = ~clk;
  end

  initial begin
    $monitor($time,, "bit_in = %b, bit_out = %b, stall = %b",
                      bit_in, bit_out, stall);
    rst <= 1; @(posedge clk);
    rst <= 0;
    start <= 0; last <= 0;
    // test some sequence: 011001111111111
    // should get:         0110011111101111
    bit_in <= 0;  start <= 1; @(posedge clk);
    bit_in <= 1;  start <= 0; @(posedge clk);
    bit_in <= 1; @(posedge clk);
    bit_in <= 0; @(posedge clk);
    bit_in <= 0; @(posedge clk);
    repeat (6) begin bit_in <= 1; @(posedge clk); end
    @(posedge clk);
    repeat (3) begin bit_in <= 1; @(posedge clk); end
    bit_in <= 1;  last <= 1; @(posedge clk);
    last <= 0; @(posedge clk);
    $finish;
  end

endmodule

module nrzi_tb;
  logic bit_stream, pkt_avail;
  logic clk, rst;
  logic stream_out;

  initial begin
    clk = 1;
    forever #5 clk = ~clk;
  end

  nrzi n1(.*);

  initial begin
    $monitor($time,, "bit_stream = %b, pkt_avail = %b, stream_out = %b, prev_bit = %b, state %b",
              bit_stream, pkt_avail, stream_out, n1.prev_bit, n1.nrzi_state);
    rst <= 1'b1;
    @(posedge clk);
    rst <= 1'b0;
    pkt_avail <= 1'b1;
    bit_stream <= 1'b1;
    @(posedge clk);
    bit_stream <= 1'b0;
    pkt_avail <= 1'b0;
    @(posedge clk);
    @(posedge clk);
    @(posedge clk);
    @(posedge clk);
    @(posedge clk);
    @(posedge clk);
    bit_stream <= 1'b1;
    @(posedge clk);
    @(posedge clk);
    @(posedge clk);
    pkt_avail <= 1'b1;
    @(posedge clk);
    pkt_avail <= 1'b0;
    @(posedge clk);
    @(posedge clk);
    @(posedge clk);
    @(posedge clk);
    @(posedge clk);
    @(posedge clk);
    bit_stream <= 1'b0;
    @(posedge clk);
    @(posedge clk);
    @(posedge clk);
    $finish;
  end
endmodule: nrzi_tb
