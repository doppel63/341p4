// main testbench
module test(
  output  logic clk, rst_L);

  logic [15:0]  flash_addr;
  logic [63:0]  flash_data;
  logic [63:0]  receivedMsg;
  logic         success;

  clock c1(.*);

  initial begin
    rst_L = 0;  @(posedge clk);
    rst_L <= 1; @(posedge clk);
    // check normal operation
    /*
    // write
    $display($time,, "Writing 64'h1234_5678_90AB_CDEF to addr 16'hABCD");
    flash_addr = 16'hABCD;  flash_data = 64'h1234_5678_90AB_CDEF;
    host.writeData(flash_addr, flash_data, success);
    if (success) $display("successful write!");
    else $display($time,, "unsuccessful write");
    */
    // try reading from uninitialized memory
    $display($time,, "Reading from addr 16'hABCD");
    flash_addr = 16'hABCD; flash_data = 0;
    host.readData(flash_addr, receivedMsg, success);
    if (success)
      if (receivedMsg == flash_data) $display("successful read!");
      else $display("successful read, but got %x instead of %x",
                    receivedMsg, flash_data);
    else $display($time,, "unsuccessful read");
    $display($time,, "Reading from addr 16'h0001");
    flash_addr = 16'h0001; flash_data = 0;
    host.readData(flash_addr, receivedMsg, success);
    if (success)
      if (receivedMsg == flash_data) $display("successful read!");
      else $display("successful read, but got %x instead of %x",
                    receivedMsg, flash_data);
    else $display($time,, "unsuccessful read");

    // write
    $display($time,, "Writing 64'hCAFE_BABE_DEAD_BEEF to addr 16'hABCD");
    flash_addr = 16'hABCD;  flash_data = 64'hCAFE_BABE_DEAD_BEEF;
    host.writeData(flash_addr, flash_data, success);
    if (success) $display("successful write!");
    else $display($time,, "unsuccessful write");
    // read
    $display($time,, "Reading from addr 16'hABCD");
    host.readData(flash_addr, receivedMsg, success);
    if (success)
      if (receivedMsg == 64'hCAFE_BABE_DEAD_BEEF) $display("successful read!");
      else $display("successful read, but got %x instead of %x",
                    receivedMsg, 64'hCAFE_BABE_DEAD_BEEF);
    else $display($time,, "unsuccessful read");
    $display($time,, "Reading from addr 16'h0001");
    flash_addr = 16'h0001; flash_data = 0;
    host.readData(flash_addr, receivedMsg, success);
    if (success)
      if (receivedMsg == flash_data) $display("successful read!");
      else $display("successful read, but got %x instead of %x",
                    receivedMsg, flash_data);
    else $display($time,, "unsuccessful read");

    $display($time,, "Writing 64'h1234_5678_90AB_CDEF to addr 16'h0001");
    flash_addr = 16'h0001;  flash_data = 64'h1234_5678_90AB_CDEF;
    host.writeData(flash_addr, flash_data, success);
    if (success) $display("successful write!");
    else $display($time,, "unsuccessful write");
    $display($time,, "Reading from addr 16'hABCD");
    flash_addr = 16'hABCD; flash_data = 64'hCAFE_BABE_DEAD_BEEF;
    host.readData(flash_addr, receivedMsg, success);
    if (success)
      if (receivedMsg == flash_data) $display("successful read!");
      else $display("successful read, but got %x instead of %x",
                    receivedMsg, flash_data);
    else $display($time,, "unsuccessful read");
    $display($time,, "Reading from addr 16'h0001");
    flash_addr = 16'h0001; flash_data = 64'h1234_5678_90AB_CDEF;
    host.readData(flash_addr, receivedMsg, success);
    if (success)
      if (receivedMsg == 64'h1234_5678_90AB_CDEF) $display("successful read!");
      else $display("successful read, but got %x instead of %x",
                    receivedMsg, 64'h1234_5678_90AB_CDEF);
    else $display($time,, "unsuccessful read");


    rst_L = 0;  @(posedge clk);
    rst_L <= 1; @(posedge clk);
    host.start <= 1; host.read <= 1; host.p_mempage <= 16'hABCD;
    @(posedge clk);
    host.start <= 0;
    #384 force wires.DP = 0;
    force wires.DM = 0;
    wait(host.done)
    @(posedge clk);
    release wires.DP;
    release wires.DM;
    assert(~host.trans_OK);
    $display("tested total time out");
    // write
    $display($time,, "Writing 64'hCAFE_BABE_DEAD_BEEF to addr 16'hABCD");
    flash_addr = 16'hABCD;  flash_data = 64'hCAFE_BABE_DEAD_BEEF;
    host.writeData(flash_addr, flash_data, success);
    if (success) $display("successful write!");
    else $display($time,, "unsuccessful write");
    // read
    $display($time,, "Reading from addr 16'hABCD");
    host.readData(flash_addr, receivedMsg, success);
    if (success)
      if (receivedMsg == 64'hCAFE_BABE_DEAD_BEEF) $display("successful read!");
      else $display("successful read, but got %x instead of %x",
                    receivedMsg, 64'hCAFE_BABE_DEAD_BEEF);
    else $display($time,, "unsuccessful read");



    @(posedge clk);
    $finish;

  end

/*
  initial begin
    #2000 $display("TIME OUT FROM TB");
    $finish;
  end
*/
endmodule: test

// test bit stream encoder
module bitStreamEncoder_tb;
  logic         clk, rst_L;
  bit           pkt_avail;
  pkt_t         pkt_out;
  bit           send_stall;
  bit           bit_out, send_start, send_last;
  bit           invalid_input;

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
    pkt_out.pid <= PID_OUT; pkt_out.addr <= 5; pkt_out.endp <= 4;
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
    pkt_out.pid <= PID_DATA; pkt_out.data <= 64'hCAFEBABEDEADBEEF;
    pkt_avail <= 1;
    @(posedge clk);
    pkt_avail <= 0;
    repeat (97) @(posedge clk);
    // test sending ACK
    $display("SENDING ACK");
    pkt_out.pid <= PID_ACK;
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
    pkt_out.pid <= PID_OUT;
    pkt_out.addr <= 0; pkt_out.endp <= 0; pkt_out.data <= 0;
    pkt_avail <= 0;
    rst_L <= 1; @(posedge clk);
    // test sending OUT to endpoint 4
    $display("SENDING OUT to endpoint 4");
    pkt_out.pid <= PID_OUT; pkt_out.addr <= 5; pkt_out.endp <= 4;
    pkt_avail <= 1;
    @(posedge clk);
    pkt_avail <= 0;
    wait(pkt_rcvd);
    ack <= 1; @(posedge clk); ack <= 0;
    assert(pkt_in.pid == PID_OUT);
    assert(pkt_in.addr == 5);
    assert(pkt_in.endp == 4);
    assert(pkt_rcvd);
    assert(pkt_ok);
    $display("passed OUT test");
    // test sending DATA, data = CAFEBABEDEADBEEF
    $display("SENDING DATA = CAFEBABEDEADBEEF");
    pkt_out.pid <= PID_DATA; pkt_out.data <= 64'hCAFEBABEDEADBEEF;
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
    pkt_out.pid <= PID_ACK;
    pkt_avail <= 1;
    @(posedge clk);
    pkt_avail <= 0;
    wait(pkt_rcvd);
    ack <= 1; @(posedge clk); ack <= 0;
    assert(pkt_in.pid == PID_ACK);
    assert(pkt_rcvd);
    assert(pkt_ok);
    $display("passed ACK test");
    
    // check errors
    // test bad bit stuffing (got a 1 instead of 0)
    $display("SENDING DATA = CAFEBABEDEADBEEF with bad bit stuff");
    pkt_out.pid <= PID_DATA; pkt_out.data <= 64'hCAFEBABEDEADBEEF;
    pkt_avail <= 1;
    @(posedge clk);
    pkt_avail <= 0;
    wait(dut1.cnt == 5 && dut1.bit_in)
      @(posedge clk);
      force dut1.bit_out = 1;
    wait(pkt_rcvd);
    ack <= 1; @(posedge clk); ack <= 0;
    assert(pkt_in.pid == PID_DATA);
    assert(pkt_rcvd);
    assert(~pkt_ok);
    $display("passed bad bit stuffing test");
    release dut1.bit_out;
    // test badly formed EOP
    $display("SENDING ACK with bad EOP");
    pkt_out.pid <= PID_ACK;
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
    pkt_out.pid <= PID_ACK;
    force pkt_out.pid[0] = 1;
    pkt_avail <= 1;
    @(posedge clk);
    pkt_avail <= 0;
    wait(pkt_rcvd);
    ack <= 1; @(posedge clk); ack <= 0;
    assert(pkt_rcvd);
    assert(~pkt_ok);
    $display("passed bad PID test");
    release pkt_out.pid[0];
    // test bad crc
    $display("SENDING OUT to endpoint 4 with corrupted bits");
    pkt_out.pid <= PID_OUT; pkt_out.addr <= 5; pkt_out.endp <= 4;
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
    assert(pkt_in.pid == PID_OUT);
    assert(pkt_in.addr == 4);
    assert(pkt_in.endp == 4);
    assert(pkt_rcvd);
    assert(~pkt_ok);
    $display("passed bad CRC test");
    // test invalid inputs on the dpdm wires
    $display("SENDING OUT to endpoint 4 with bad DPDM wires");
    pkt_out.pid <= PID_OUT; pkt_out.addr <= 5; pkt_out.endp <= 4;
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
    assert(pkt_in.pid == PID_OUT);
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

module protocolFSM_tb;

  logic         transaction, pkt_ok, pkt_sent, pkt_rcvd, send_addr;
  logic         pkt_avail, protocol_avail;
  logic         clk, rst_L;
  logic         start, clear;
  pkt_t         pkt_in, pkt_out;
  logic [63:0]  data_in, data_out;

  protocolFSM pFSM(.*);
  clock ck1(.*);

  initial begin
    $monitor($time,, 
              "rst_L = %b, IN_pid = %b, IN_addr = %b, IN_endp = %b\n\
              IN_data = %x, OUT_data = %x\n\
              OUT_pid = %b, OUT_addr = %b, OUT_endp = %b\n\
              transaction = %b, pkt_ok = %b, pkt_sent = %b, pkt_rcvd = %b\n\
              send_addr = %b, pkt_avail = %b, protocol_avail = %b, state = %s\n",
              rst_L, pkt_in.pid, pkt_in.addr, pkt_in.endp,
              pkt_in.data, pkt_out.data,
              pkt_out.pid, pkt_out.addr, pkt_out.endp,
              transaction, pkt_ok, pkt_sent, pkt_rcvd,
              send_addr, pkt_avail, protocol_avail, pFSM.protocolState);
    @(posedge clk);
    rst_L <= 0;
    @(posedge clk);
    rst_L <= 1;
    transaction <= 0;
    start <= 1;
    $display("OUT TRANSACTION");
    @(posedge clk);
    start <= 0;
    @(posedge clk);
    send_addr <= 1'b1;
    pkt_sent <= 1;
    @(posedge clk);
    send_addr <= 0;
    pkt_sent <= 0;
    pkt_rcvd <= 0;
    repeat (2) @(posedge clk);
    pkt_rcvd <= 1;
    pkt_ok <= 1;
    pkt_in.data <= 64'hFA20;
    repeat (2) @(posedge clk);
    pkt_sent <= 1'b1;
    pkt_rcvd <= 0;
    pkt_ok <= 0;
    @(posedge clk);
    pkt_rcvd <= 1'b1;
    pkt_in.pid <= PID_ACK;
    @(posedge clk);
    $display("DONE WITH OUT TRANSACTION");
    @(posedge clk);
    $finish;
  end

endmodule: protocolFSM_tb

module clock(
  output logic clk);

  initial begin 
    clk = 1;
    forever #1 clk = ~clk;
  end
endmodule: clock

module FSM_tb;
  logic         transaction, protocol_avail, pStart;
  logic         clear, send_addr;
  logic [63:0]  data_in, data_out;

  // internal inputs to protocol
  logic pkt_ok, pkt_sent, pkt_rcvd, pkt_avail;
  pkt_t pkt_in;

  // internal outputs from protocol
  logic ack, protocol_OK;
  pkt_t pkt_out;

  // inputs from tb
  logic        clk, rst_L;
  logic        read, start;
  logic [15:0] mempage;
  logic [63:0] write_data;

  // outputs
  logic        done, trans_OK;
  logic [63:0] data_tb;

  protocolFSM pFSM(.*);
  rwFSM rFSM(.*);
  clock ck1(.*);

  initial begin
    $monitor($time,, "mempage = %h, data = %h, read = %b, start = %b, addr? = %b\n\
                      pState = %s, rwState = %s\n",
                      mempage, write_data, read, start, send_addr, pFSM.protocolState, rFSM.RWState);
    @(posedge clk);
    rst_L <= 0; $display("Resetting...");
    @(posedge clk);
    rst_L <= 1;
    mempage <= 16'd22;
    start <= 1;
    write_data <= 64'habcd;
    read <= 0;
    $display("OUT TRANSACTION as part of WRITE");
    @(posedge clk);
    start <= 0;
    assert(data_in && ~transaction && send_addr);
    repeat (12) @(posedge clk);
    pkt_sent <= 1;                // packet sent by sender, ready for data
    @(posedge clk);
    $display("Sent OUT address packet.");
    pkt_sent <= 0;
    pkt_rcvd <= 0;
    repeat (20) @(posedge clk);
    pkt_rcvd <= 1;                // packet received by receiver
    pkt_ok <= 1;
    pkt_in.data <= 64'hFA20;
    repeat (2) @(posedge clk);
    $display("Sent DATA address packet.");
    pkt_sent <= 1'b1;
    pkt_rcvd <= 0;
    pkt_ok <= 0;
    @(posedge clk);
    pkt_sent <= 1'b0;
    repeat (22) @(posedge clk);
    pkt_rcvd <= 1'b1;
    pkt_in.pid <= PID_ACK;
    @(posedge clk);
    $display("Received ACK address packet.");
    $display("DONE WITH OUT TRANSACTION");
    @(posedge clk);
    $display("OUT TRANSACTION as part of WRITE");
    @(posedge clk);
    start <= 0;
    repeat (12) @(posedge clk);
    pkt_sent <= 1;                // packet sent by sender, ready for data
    @(posedge clk);
    $display("Sent OUT address packet.");
    pkt_sent <= 0;
    pkt_rcvd <= 0;
    repeat (20) @(posedge clk);
    pkt_rcvd <= 1;                // packet received by receiver
    pkt_ok <= 1;
    pkt_in.data <= 64'hFA20;
    repeat (2) @(posedge clk);
    $display("Sent DATA address packet.");
    pkt_sent <= 1'b1;
    pkt_rcvd <= 0;
    pkt_ok <= 0;
    @(posedge clk);
    repeat (22) @(posedge clk);
    pkt_rcvd <= 1'b1;
    pkt_in.pid <= PID_ACK;
    @(posedge clk);
    $display("Received ACK address packet.");
    @(posedge clk);

    $finish;
  end

endmodule: FSM_tb
