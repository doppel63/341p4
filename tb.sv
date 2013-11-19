// test bit stream encoder
module bitStreamEncoder_tb;
  logic         clk, rst_L;
  bit           pkt_avail;
  pkt_t         pkt_out;
  bit           send_stall;
  bit           bit_out, send_start, send_last;

  logic [95:0]  result;   // largest is SYNC + PID + DATA0 + CRC16

  bitStreamEncoder dut(.*);
  clock c1(.*);

  // used for keeping track of the bit stream
  // note that result is always 1 clock cycle late!
  always_ff @(posedge clk, negedge rst_L) begin
    if (~rst_L)
      result <= 0;
    else if (~send_stall) begin
      result <= result << 1;
      result[0] <= bit_out;
    end
  end

  initial begin
    rst_L <= 0; @(posedge clk);
    rst_L <= 1; @(posedge clk);
    // test sending OUT to endpoint 4
    $monitor($time,, "stall = %b, start = %b, last = %b, result = %h",
                      send_stall, send_start, send_last, result);
    send_stall <= 0;
    pkt_out.pid <= 8'b1110_0001; pkt_out.addr <= 5; pkt_out.endp <= 4;
    pkt_avail <= 1;
    $display("SENDING OUT to endpoint 4");
    @(posedge clk);
    pkt_avail <= 0;
    repeat (33) @(posedge clk);
    // test stall
    pkt_avail <= 1;
    $display("SENDING OUT to endpoint 4 with stall");
    @(posedge clk);
    pkt_avail <= 0;
    repeat (24) @(posedge clk);
    send_stall <= 1; repeat (5) @(posedge clk);
    send_stall <= 0; repeat (9) @(posedge clk);
    // test sending DATA, data = CAFEBABEDEADBEEF
    $display("SENDING DATA = CAFEBABEDEADBEEF");
    pkt_out.pid <= 8'b1100_0011; pkt_out.data <= 64'hCAFEBABEDEADBEEF;
    pkt_avail <= 1;
    @(posedge clk);
    pkt_avail <= 0;
    repeat (97) @(posedge clk);
    // test sending ACK
    $display("SENDING ACK");
    pkt_out.pid <= 8'b1101_0010;
    pkt_avail <= 1;
    @(posedge clk);
    pkt_avail <= 0;
    repeat (17) @(posedge clk);
    $finish;
  end

endmodule

// test bit stream decoder
module bitStreamDecoder_tb;
  logic clk, rst_L;
  // encoder
  bit   pkt_avail;
  pkt_t pkt_out;
  bit   send_stall;
  bit   bit_out, send_start, send_last;
  // decoder
  bit   bit_in, rcv_stall;
  bit   bit_stuff_ok, EOP_ok;
  bit   rcv_start, rcv_last;
  bit   ack;
  pkt_t pkt_in;
  bit   pkt_rcvd, pkt_ok;

  bitStreamEncoder dut0(.*);
  bitStreamDecoder dut1(.*);
  clock c1(.*);
  assign bit_in = bit_out;

  initial begin
    rst_L = 0; @(posedge clk);
    rst_L <= 1; @(posedge clk);
    // test sending OUT to endpoint 4
    $monitor($time,, "stall = %b, start = %b, last = %b, result = %h",
                      send_stall, send_start, send_last, pkt_in);
    send_stall <= 0; rcv_stall <= 0;
    bit_stuff_ok <= 1; EOP_ok <= 1; ack <= 0;
    pkt_out.pid <= 8'b1110_0001; pkt_out.addr <= 5; pkt_out.endp <= 4;
    pkt_avail <= 1;
    $display("SENDING OUT to endpoint 4");
    @(posedge clk);
    pkt_avail <= 0;
    repeat (33) @(posedge clk);
    ack <= 1; @(posedge clk);
    ack <= 0;
    // test stall
    pkt_avail <= 1;
    $display("SENDING OUT to endpoint 4 with stall");
    @(posedge clk);
    pkt_avail <= 0;
    repeat (24) @(posedge clk);
    send_stall <= 1; rcv_stall <= 1; repeat (5) @(posedge clk);
    send_stall <= 0; rcv_stall <= 0;repeat (9) @(posedge clk);
    ack <= 1; @(posedge clk);
    ack <= 0;
    // test sending DATA, data = CAFEBABEDEADBEEF
    $display("SENDING DATA = CAFEBABEDEADBEEF");
    pkt_out.pid <= 8'b1100_0011; pkt_out.data <= 64'hCAFEBABEDEADBEEF;
    pkt_avail <= 1;
    @(posedge clk);
    pkt_avail <= 0;
    repeat (97) @(posedge clk);
    ack <= 1; @(posedge clk);
    ack <= 0;
    // test sending ACK
    $display("SENDING ACK");
    pkt_out.pid <= 8'b1101_0010;
    pkt_avail <= 1;
    @(posedge clk);
    pkt_avail <= 0;
    repeat (17) @(posedge clk);
    ack <= 1; @(posedge clk);
    ack <= 0;

    $finish;
  end

endmodule


// test bse + bitstuffing + nrzi
module big_tb;
  logic clk, rst_L;
  bit   pkt_avail, pkt_sent;
  pkt_t pkt_out;
  bit   send_stall;
  bit   send_start, send_last;
  bit   send_raw_bit_stream, send_stuffed_bit_stream, stream_out;
  usbWires wires();

  logic [109:0]  result;   // largest is SYNC + PID + DATA0 + CRC16

  bitStreamEncoder dut0(.*, .bit_out(send_raw_bit_stream));
  bitStuffer dut1(.*, .bit_in(send_raw_bit_stream),
                      .bit_out(send_stuffed_bit_stream));
  nrzi dut2(.*, .bit_stream(send_stuffed_bit_stream));
  dpdm d1(.*);
  clock c1(.*);

  // used for keeping track of the bit stream
  // note that result is always 1 clock cycle late!
  always_ff @(posedge clk, negedge rst_L) begin
    if (~rst_L)
      result <= 0;
    else begin
      result <= result << 1;
      result[0] <= stream_out;
    end
  end

  initial begin
    rst_L = 0; @(posedge clk);
    pkt_out.pid <= 0; pkt_out.addr <= 0; pkt_out.endp <= 0; pkt_out.data <= 0;
    pkt_avail <= 0;
    rst_L <= 1; @(posedge clk);
    // test sending OUT to endpoint 4, data = CAFEBABEDEADBEEF
    $monitor($time,, "stall = %b, start = %b, last = %b, result = %h",
                      send_stall, send_start, send_last, result);
    pkt_out.pid <= 8'b1110_0001; pkt_out.addr <= 5; pkt_out.endp <= 4;
    pkt_avail <= 1;
    $display("SENDING OUT to endpoint 4");
    @(posedge clk);
    pkt_avail <= 0;
    repeat (40) @(posedge clk);
    // test sending DATA
    $display("SENDING DATA = CAFEBABEDEADBEEF");
    pkt_out.pid <= 8'b1100_0011; pkt_out.data <= 64'hCAFEBABEDEADBEEF;
    pkt_avail <= 1;
    @(posedge clk);
    pkt_avail <= 0;
    repeat (110) @(posedge clk);
    // test sending ACK
    $display("SENDING ACK");
    pkt_out.pid <= 8'b1101_0010;
    pkt_avail <= 1;
    @(posedge clk);
    pkt_avail <= 0;
    repeat (17) @(posedge clk);
    $finish;
  end

endmodule

// test receiving stuff
module big2_tb;
  logic clk, rst_L;
  // sender
  bit   pkt_avail, pkt_sent;
  pkt_t pkt_out;
  bit   send_stall;
  bit   send_start, send_last;
  bit   send_raw_bit_stream, send_stuffed_bit_stream, stream_out;
  bit   invalid_input;
  usbWires wires();
  // receiver
  bit   ack, pkt_rcvd, pkt_ok;
  pkt_t pkt_in;
  bit   rcv_stall;
  bit   rcv_start, rcv_last;
  bit   rcv_raw_bit_stream, rcv_stuffed_bit_stream, stream_in;
  bit   EOP_ok, bit_stuff_ok, sending;


  // sender
  bitStreamEncoder dut0(.*, .bit_out(send_raw_bit_stream));
  bitStuffer dut1(.*, .bit_in(send_raw_bit_stream),
                      .bit_out(send_stuffed_bit_stream));
  nrzi dut2(.*, .bit_stream(send_stuffed_bit_stream));
  dpdm d1(.*, .stream_in(), .EOP_ok(), .sending(), .rcv_last(), .ack(),
          .invalid_input());
  clock c1(.*);

  // receiver
  bitStreamDecoder dut3(.*, .bit_in(rcv_raw_bit_stream));
  bitUnstuffer dut4(.*, .bit_in(rcv_stuffed_bit_stream),
                        .bit_out(rcv_raw_bit_stream));
  nrzi_dec dut5(.*, .bit_stream(rcv_stuffed_bit_stream));
  dpdm d2(.*, .stream_out(), .pkt_avail(), .send_last(), .pkt_sent());

  initial begin
    rst_L = 0; @(posedge clk);
    pkt_out.pid <= 0; pkt_out.addr <= 0; pkt_out.endp <= 0; pkt_out.data <= 0;
    pkt_avail <= 0;
    rst_L <= 1; @(posedge clk);
    // test sending OUT to endpoint 4
    $display("SENDING OUT to endpoint 4");
    pkt_out.pid <= 8'b1110_0001; pkt_out.addr <= 5; pkt_out.endp <= 4;
    pkt_avail <= 1;
    @(posedge clk);
    pkt_avail <= 0;
    wait(pkt_rcvd);
    ack <= 1; @(posedge clk); ack <= 0;
    assert(pkt_in.pid == 8'b1110_0001);
    assert(pkt_in.addr == 5);
    assert(pkt_in.endp == 4);
    assert(pkt_rcvd);
    assert(pkt_ok);
    $display("passed OUT test");
    // test sending DATA, data = CAFEBABEDEADBEEF
    $display("SENDING DATA = CAFEBABEDEADBEEF");
    pkt_out.pid <= 8'b1100_0011; pkt_out.data <= 64'hCAFEBABEDEADBEEF;
    pkt_avail <= 1;
    @(posedge clk);
    pkt_avail <= 0;
    wait(pkt_rcvd);
    ack <= 1; @(posedge clk); ack <= 0;
    assert(pkt_in.pid == 8'b1100_0011);
    assert(pkt_in.data == 64'hCAFEBABEDEADBEEF);
    assert(pkt_rcvd);
    assert(pkt_ok);
    $display("passed DATA test");
    // test sending ACK
    $display("SENDING ACK");
    pkt_out.pid <= 8'b1101_0010;
    pkt_avail <= 1;
    @(posedge clk);
    pkt_avail <= 0;
    wait(pkt_rcvd);
    ack <= 1; @(posedge clk); ack <= 0;
    assert(pkt_in.pid == 8'b1101_0010);
    assert(pkt_rcvd);
    assert(pkt_ok);
    $display("passed ACK test");
    
    // check errors
    // test bad bit stuffing (got a 1 instead of 0)
    $display("SENDING DATA = CAFEBABEDEADBEEF with bad bit stuff");
    pkt_out.pid <= 8'b1100_0011; pkt_out.data <= 64'hCAFEBABEDEADBEEF;
    pkt_avail <= 1;
    @(posedge clk);
    pkt_avail <= 0;
    wait(dut1.cnt == 5 && dut1.bit_in)
      @(posedge clk);
      force dut1.bit_out = 1;
    wait(pkt_rcvd);
    ack <= 1; @(posedge clk); ack <= 0;
    assert(pkt_in.pid == 8'b1100_0011);
    assert(pkt_rcvd);
    assert(~pkt_ok);
    $display("passed bad bit stuffing test");
    release dut1.bit_out;
    // test badly formed EOP
    $display("SENDING ACK with bad EOP");
    pkt_out.pid <= 8'b1101_0010;
    pkt_avail <= 1;
    @(posedge clk);
    pkt_avail <= 0;
    @(posedge clk);
    wait(~d1.en_dp & ~d1.en_dm)
      @(posedge clk);
      force d1.en_dp = 1;
    wait(pkt_rcvd);
    ack <= 1; @(posedge clk); ack <= 0;
    assert(pkt_in.pid == 8'b1101_0010);
    assert(pkt_rcvd);
    assert(~pkt_ok);
    $display("passed bad EOP test");
    release d1.en_dp;
    // test bad PID
    $display("SENDING BAD PID");
    pkt_out.pid <= 8'b1101_0011;
    pkt_avail <= 1;
    @(posedge clk);
    pkt_avail <= 0;
    wait(pkt_rcvd);
    ack <= 1; @(posedge clk); ack <= 0;
    assert(pkt_rcvd);
    assert(~pkt_ok);
    $display("passed bad PID test");
    // test bad crc
    $display("SENDING OUT to endpoint 4 with corrupted bits");
    pkt_out.pid <= 8'b1110_0001; pkt_out.addr <= 5; pkt_out.endp <= 4;
    pkt_avail <= 1;
    @(posedge clk);
    pkt_avail <= 0;
    wait(dut3.pid_cnt == 7)
      repeat (2) @(posedge clk);
      force dut3.bit_in = 0;
      @(posedge clk);
      release dut3.bit_in;
    wait(pkt_rcvd);
    ack <= 1; @(posedge clk); ack <= 0;
    assert(pkt_in.pid == 8'b1110_0001);
    assert(pkt_in.addr == 4);
    assert(pkt_in.endp == 4);
    assert(pkt_rcvd);
    assert(~pkt_ok);
    $display("passed bad CRC test");
    // test invalid inputs on the dpdm wires
    $display("SENDING OUT to endpoint 4 with bad DPDM wires");
    pkt_out.pid <= 8'b1110_0001; pkt_out.addr <= 5; pkt_out.endp <= 4;
    pkt_avail <= 1;
    @(posedge clk);
    pkt_avail <= 0;
    wait(dut3.pid_cnt == 7)
      repeat (2) @(posedge clk);
      force wires.DP = 1;
      force wires.DM = 1;
      @(posedge clk);
      release wires.DP;
      release wires.DM;
    wait(pkt_rcvd);
    ack <= 1; @(posedge clk); ack <= 0;
    assert(pkt_in.pid == 8'b1110_0001);
    assert(pkt_rcvd);
    assert(~pkt_ok);
    $display("passed bad DPDM test");

    
    @(posedge clk);
    $finish;
  end

endmodule


// test bit stuffing
module bitStuffer_tb;
  logic clk, rst_L;
  logic pkt_avail;
  logic send_start;
  logic send_last;
  logic bit_in;
  logic bit_out;
  logic send_stall;

  bitStuffer dut(.*);
  clock c1(.*);

  initial begin
    $monitor($time,, "bit_in = %b, bit_out = %b, stall = %b",
                      bit_in, bit_out, send_stall);
    rst_L <= 0; @(posedge clk);
    rst_L <= 1;
    send_start <= 0; send_last <= 0;
    // test some sequence: 011001111111111
    // should get:         0110011111101111
    bit_in <= 0;  send_start <= 1; @(posedge clk);
    bit_in <= 1;  send_start <= 0; @(posedge clk);
    bit_in <= 1; @(posedge clk);
    bit_in <= 0; @(posedge clk);
    bit_in <= 0; @(posedge clk);
    repeat (6) begin bit_in <= 1; @(posedge clk); end
    @(posedge clk);
    repeat (3) begin bit_in <= 1; @(posedge clk); end
    bit_in <= 1;  send_last <= 1; @(posedge clk);
    send_last <= 0; @(posedge clk);
    $finish;
  end

endmodule

// test bit unstuffing
module bitUnstuffer_tb;
  logic clk, rst_L;
  logic ack;
  logic rcv_start;
  logic rcv_last;
  logic bit_in;
  logic bit_out;
  logic rcv_stall;
  logic bit_stuff_ok;

  bitUnstuffer dut(.*);
  clock c1(.*);

  initial begin
    $monitor($time,, "bit_in = %b, bit_out = %b, stall = %b, ok = %b",
                      bit_in, bit_out, rcv_stall, bit_stuff_ok);
    rst_L = 0; @(posedge clk);
    rst_L <= 1;
    rcv_start <= 0; rcv_last <= 0; ack <= 0;
    // test some sequence: 01100111111_0_111
    // should get:         01100111111_111
    $display("SENDING 01100111111_0_111");
    bit_in <= 0;  rcv_start <= 1; @(posedge clk);
    bit_in <= 1;  rcv_start <= 0; @(posedge clk);
    bit_in <= 1; @(posedge clk);
    bit_in <= 0; @(posedge clk);
    bit_in <= 0; @(posedge clk);
    repeat (6) begin bit_in <= 1; @(posedge clk); end
    bit_in <= 0; @(posedge clk);
    repeat (3) begin bit_in <= 1; @(posedge clk); end
    bit_in <= 1;  rcv_last <= 1; @(posedge clk);
    rcv_last <= 0; @(posedge clk);
    ack <= 1; @(posedge clk);
    ack <= 0;
    // test some sequence: 01100111111_1_111
    // should get:         01100111111_111 but error
    $display("SENDING 01100111111_1_111");
    bit_in <= 0;  rcv_start <= 1; @(posedge clk);
    bit_in <= 1;  rcv_start <= 0; @(posedge clk);
    bit_in <= 1; @(posedge clk);
    bit_in <= 0; @(posedge clk);
    bit_in <= 0; @(posedge clk);
    repeat (6) begin bit_in <= 1; @(posedge clk); end
    bit_in <= 1; @(posedge clk);
    repeat (3) begin bit_in <= 1; @(posedge clk); end
    bit_in <= 1;  rcv_last <= 1; @(posedge clk);
    rcv_last <= 0; @(posedge clk);
    ack <= 1; @(posedge clk);
    ack <= 0;

    $finish;
  end

endmodule


module nrzi_tb;
  logic bit_stream, pkt_avail, send_last;
  logic clk, rst_L;
  logic stream_out;

  nrzi n1(.*);
  clock c1(.*);

  initial begin
    $monitor($time,, "bit_stream = %b, pkt_avail = %b, stream_out = %b, prev_bit = %b",
              bit_stream, pkt_avail, stream_out, n1.prev_bit);
    rst_L <= 1'b0;
    @(posedge clk);
    rst_L <= 1'b1;
    pkt_avail <= 1'b1;
    bit_stream <= 1'b1;
    @(posedge clk);
    bit_stream <= 1'b0;
    pkt_avail <= 1'b0;
    repeat (6) @(posedge clk);
    bit_stream <= 1'b1;
    repeat (3) @(posedge clk);
    pkt_avail <= 1'b1;
    @(posedge clk);
    pkt_avail <= 1'b0;
    repeat (6) @(posedge clk);
    bit_stream <= 1'b0;
    repeat (3) @(posedge clk);
    $finish;
  end
endmodule: nrzi_tb

module nrzi_dec_tb;
  logic clk, rst_L;
  bit   stream_in, sending, ack;
  bit   bit_stream;

  nrzi_dec dut(.*);
  clock c1(.*);

  initial begin
    $monitor($time,, "stream_in = %b, sending = %b, bit_stream = %b",
                      stream_in, sending, bit_stream);
    stream_in <= 0; sending <= 0; ack <= 0;
    rst_L = 0;  @(posedge clk);
    rst_L <= 1;
    // send 0101_0100 (SYNC)
    $display("SENDING 0101_0100 (SYNC)");
    repeat (3) begin
      stream_in <= 0; @(posedge clk);
      stream_in <= 1; @(posedge clk);
    end
    stream_in <= 0;
    repeat (2) @(posedge clk);
    // wait a while, then acknowledge
    repeat (5) @(posedge clk);
    ack <= 1; @(posedge clk);
    // send 1010_1011 (com of SYNC)
    $display("SENDING 1010_1011 (~SYNC)");
    repeat (3) begin
      stream_in <= 1; @(posedge clk);
      stream_in <= 0; @(posedge clk);
    end
    stream_in <= 1;
    repeat (2) @(posedge clk);
    // wait a while, then acknowledge
    repeat (5) @(posedge clk);
    ack <= 1; @(posedge clk);
    // send 0101_0100 (SYNC) but while sending
    $display("SENDING 0101_0100 (SYNC) with sending");
    sending <= 1;
    repeat (3) begin
      stream_in <= 0; @(posedge clk);
      stream_in <= 1; @(posedge clk);
    end
    stream_in <= 0;
    repeat (2) @(posedge clk);
    sending <= 0;

    @(posedge clk);
    $finish;
  end

endmodule: nrzi_dec_tb

module dpdm_tb;
  logic stream_out, pkt_avail, send_last;
  logic clk, rst_L;
  usbWires wires();

  dpdm dpdm1(.*);
  clock c1(.*);

  initial begin
    $monitor($time,, "s_out = %b, pkt_avail = %b, en = %b, last = %b, \
rst_L = %b, dm = %b, dp = %b, state = %s", stream_out, pkt_avail, dpdm1.en, 
      send_last, rst_L, wires.DM, wires.DP, dpdm1.DPDM_state);
    @(posedge clk);
    rst_L <= 1'b0;
    @(posedge clk);
    rst_L <= 1'b1;
    pkt_avail <= 1'b1;
    send_last <= 0;
    @(posedge clk);
    pkt_avail <= 0;
    repeat (5) begin
      stream_out <= 0;
      @(posedge clk);
      stream_out <= 1;
      @(posedge clk);
    end
    @(posedge clk);
    send_last <= 1;
    repeat (6) @(posedge clk);
    $finish;
  end
endmodule

module clock(
  output logic clk);

  initial begin 
    clk = 1;
    forever #1 clk = ~clk;
  end
endmodule: clock


