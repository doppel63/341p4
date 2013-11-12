// Write your usb host here.  Do not modify the port list.
/*
module usbHost
  (input logic clk, rst_L, 
  usbWires wires);
 */
  /* Tasks needed to be finished to run testbenches */
/*
  task prelabRequest
  // sends an OUT packet with ADDR=5 and ENDP=4
  // packet should have SYNC and EOP too
  (input bit  [7:0] data);
    
  endtask: prelabRequest

  task readData
  // host sends memPage to thumb drive and then gets data back from it
  // then returns data and status to the caller
  (input  bit [15:0]  mempage, // Page to write
   output bit [63:0] data, // array of bytes to write
   output bit        success);

  endtask: readData

  task writeData
  // Host sends memPage to thumb drive and then sends data
  // then returns status to the caller
  (input  bit [15:0]  mempage, // Page to write
   input  bit [63:0] data, // array of bytes to write
   output bit        success);

  endtask: writeData



  // usbHost starts here!!
  // prelab: we have pid, addr, endp, data (set by task prelabRequest)



endmodule: usbHost
*/

// takes a packet and converts it to a bit string when sent_sync is asserted.
// asserts last when on the last bit of the bit stream.
module bitStreamEncoder(
  input   logic        clk, rst,
  input   logic        pkt_avail,
  input   logic [7:0]  pid_in,
  input   logic [6:0]  addr_in,
  input   logic [3:0]  endp_in,
  input   logic [63:0] data_in,
  input   logic        stall, sent_sync,
  output  logic        bit_out, last);

  // internal wires
  enum logic [7:0] {OUT = 8'b1110_0001, IN = 8'b0110_1001,
                    DATA0 = 8'b1100_0011,
                    ACK = 8'b1101_0010, NAK = 8'b0101_1010} pid;
  logic [6:0]   addr;
  logic [3:0]   endp;
  logic [63:0]  data;
  logic         crc5_in, crc5, crc16_in, crc16;
  // internal control points; also wires anyway
  logic         crc5_en, crc16_en;
  logic [1:0]   endp_cnt;
  logic [2:0]   pid_cnt, addr_cnt;
  logic [5:0]   data_cnt;
  logic [2:0]   crc5_cnt;
  logic [3:0]   crc16_cnt;

  // states for FSM
  enum logic [2:0] {IDLE, PID, ADDR, ENDP, CRC5, DATA, CRC16} state;

  // instantiate stuff in datapath
  // counters = registers for these
  counter #(8)  pidReg(.clk(clk), .rst(rst), .clr(), .ld(pkt_avail), .en(),
                       .up(), .val(pid_in), .cnt(pid));
  counter #(7)  addrReg(.clk(clk), .rst(rst), .clr(), .ld(pkt_avail), .en(),
                        .up(), .val(addr_in), .cnt(addr));
  counter #(4)  endpReg(.clk(clk), .rst(rst), .clr(), .ld(pkt_avail), .en(),
                        .up(), .val(endp_in), .cnt(endp));
  counter #(64) dataReg(.clk(clk), .rst(rst), .clr(), .ld(pkt_avail), .en(),
                        .up(), .val(data_in), .cnt(data));
  // crc sender modules
  crc5Sender  crc5s(.clk(clk), .rst(rst), .en(~stall && crc5_en),
                      .msg_in(crc5_in), .msg_out(crc5));
  crc16Sender crc16s(.clk(clk), .rst(rst), .en(~stall && crc16_en),
                      .msg_in(crc16_in), .msg_out(crc16));

  // combinational logic to fill out gaps. mostly muxes.
  always_comb begin
    // output mux, determined by which state we're in
    bit_out = 'd0; // could be x?
    crc5_en = 0;
    crc16_en = 0;
    case (state)
      PID:              bit_out = pid[pid_cnt];
      ADDR, ENDP, CRC5: begin
                          bit_out = crc5;
                          crc5_en = 1;
                        end
      DATA, CRC16:      begin
                          bit_out = crc16;
                          crc16_en = 1;
                        end
    endcase

    // select input to crc5 (addr or endp) and crc16 (data)
    crc5_in = (state == ADDR) ? addr[addr_cnt] : endp[endp_cnt];
    crc16_in = data[data_cnt];

    // last signal asserted when on the last bit of crc5, crc16 or pid depending
    // on packet AND not stalling for bit stuffing
    last = ~stall && ((crc5_cnt == 4) || (crc16_cnt == 15) ||
            (pid == PID && pid_cnt == 7));
  end

  // FSM: controls stuff, keeps track of state
  always_ff @(posedge clk, posedge rst) begin
    if (rst)
      state <= IDLE;
    else if (~stall) begin
      // only do stuff if no stall signal from bit stuffing
      case (state)
        IDLE:   begin
                  pid_cnt <= 0;
                  addr_cnt <= 0;
                  endp_cnt <= 0;
                  data_cnt <= 0;
                  crc5_cnt <= 0;
                  crc16_cnt <= 0;
                  state <= (sent_sync) ? PID : IDLE;
                end
        PID:    begin
                  if (pid_cnt < 7) begin
                    pid_cnt <= pid_cnt + 1;
                    state <= PID;
                  end
                  else begin
                    pid_cnt <= 0;
                    case (pid)
                      IN, OUT:  state <= ADDR;
                      DATA0:    state <= DATA;
                      default:  state <= IDLE;
                    endcase
                   end
                end
        ADDR:   begin
                  addr_cnt <= (addr_cnt < 6) ? addr_cnt + 1 : 0;
                  state <= (addr_cnt < 6) ? ADDR : ENDP;
                end
        ENDP:   begin
                  endp_cnt <= (endp_cnt < 3) ? endp_cnt + 1 : 0;
                  state <= (endp_cnt < 3) ? ENDP : CRC5;
                end
        CRC5:   begin
                  crc5_cnt <= (crc5_cnt < 4) ? crc5_cnt + 1 : 0;
                  state <= (crc5_cnt < 4) ? CRC5 : IDLE;
                end
        DATA:   begin
                  data_cnt <= (data_cnt < 63) ? data_cnt + 1 : 0;
                  state <= (data_cnt < 63) ? DATA : CRC16;
                end
        CRC16:  begin
                  crc16_cnt <= (crc16_cnt < 15) ?  crc16_cnt + 1 : 0;
                  state <= (crc16_cnt < 15) ? CRC16 : IDLE;
                end
      endcase
    end
  end

endmodule

// test bit stream encoder
module bitStreamEncoder_tb;
  logic        clk, rst;
  logic        pkt_avail;
  logic [7:0]  pid_in;
  logic [6:0]  addr_in;
  logic [3:0]  endp_in;
  logic [63:0] data_in;
  logic        stall, sent_sync;
  logic        bit_out, last;

  logic [87:0]  result;   // largest is PID + DATA0 + CRC16

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
    $monitor($time,, "stall = %b, sent_sync = %b, result = %h, last = %b",
                      stall, sent_sync, result, last);
    stall <= 0;
    pid_in <= 8'b1110_0001; addr_in <= 5; endp_in <= 4;
    sent_sync <= 1; pkt_avail <= 1;
    $display("SENDING OUT to endpoint 4");
    @(posedge clk);
    sent_sync <= 0; pkt_avail <= 0;
    repeat (25) @(posedge clk);
    // test stall
    rst <= 1; @(posedge clk);
    rst <= 0; @(posedge clk);
    sent_sync <= 1; pkt_avail <= 1;
    $display("SENDING OUT to endpoint 4 with stall");
    @(posedge clk);
    sent_sync <= 0; pkt_avail <= 0;
    repeat (16) @(posedge clk);
    stall <= 1; repeat (5) @(posedge clk);
    stall <= 0; repeat (9) @(posedge clk);
    // test sending DATA
    $display("SENDING DATA = CAFEBABEDEADBEEF");
    pid_in <= 8'b1100_0011; data_in <= 64'hCAFEBABEDEADBEEF;
    sent_sync <= 1; pkt_avail <= 1;
    @(posedge clk);
    sent_sync <= 0; pkt_avail <= 0;
    repeat (89) @(posedge clk);
    $finish;
  end

endmodule

// CRC stuff
// CRC5 calculator: rem 01100
module crc5Calc(
  input   logic       clk, rst, en, crc_clr, crc_in,
  output  logic [4:0] crc_out);

  always_ff @(posedge clk, posedge rst) begin
    if (rst)
      crc_out <= 5'b11111;
    else if (crc_clr)
      crc_out <= 5'b11111;
    else if (en) begin
      crc_out <= crc_out << 1;
      crc_out[2] <= crc_out[1] ^ (crc_out[4] ^ crc_in);
      crc_out[0] <= crc_out[4] ^ crc_in;
    end
  end

endmodule

// CRC16 calculator: rem 800d
module crc16Calc(
  input   logic         clk, rst, en, crc_clr, crc_in,
  output  logic [15:0]  crc_out);

  always_ff @(posedge clk, posedge rst) begin
    if (rst)
      crc_out <= 16'hffff;
    else if (crc_clr)
      crc_out <= 16'hffff;
    else if (en) begin
      crc_out <= crc_out << 1;
      crc_out[2] <= crc_out[1] ^ (crc_out[15] ^ crc_in);
      crc_out[15] <= crc_out[14] ^ (crc_out[15] ^ crc_in);
      crc_out[0] <= crc_out[15] ^ crc_in;
    end
  end

endmodule


// counter: default 4 bits wide
module counter
  #(parameter WIDTH = 4)
  (input  logic             clk, rst, clr, ld, en, up,
  input   logic [WIDTH-1:0] val,
  output  logic [WIDTH-1:0] cnt);

  always_ff @(posedge clk, posedge rst) begin
    if (rst)
      cnt <= 'd0;
    else if (clr)
      cnt <= 'd0;
    else if (ld)
      cnt <= val;
    else if (en)
      if (up)
        cnt <= cnt + 1;
      else
        cnt <= cnt - 1;
  end
endmodule

// shift reg: default 11 bits wide
module shiftReg
  #(parameter WIDTH = 11)
  (input  logic             clk, rst, clr, en, in,
  output  logic [WIDTH-1:0] out);

  always_ff @(posedge clk, posedge rst) begin
    if (rst)
      out <= 'd0;
    else if (clr)
      out <= 'd0;
    else if (en) begin
      out <= out << 1;
      out[0] <= in;
    end
  end
endmodule

// CRC5 sender
module crc5Sender(
  input   logic clk, rst, en, msg_in,
  output  logic msg_out);

  logic crc_in;
  logic [4:0] crc_out, com_rem;
  logic crc_clr;
  logic cnt_clr, cnt_en, cnt_up;
  logic rem_ld, rem_en, rem_up;
  logic [3:0] cnt, rem_cnt;

  logic body, firstRem, rem_bit;
  enum logic [1:0] {BODY, REM, DONE} cs, ns;

  // instantiate stuff
  crc5Calc calc(.*);
  counter msgCnt(clk, rst, cnt_clr, , en && cnt_en, cnt_up, , cnt);
  counter remCnt(clk, rst, , rem_ld, en && rem_en, rem_up, 4'd3, rem_cnt);
  // counters work as regs too!
  counter #(5) rem(clk, rst, , rem_ld, , , ~crc_out, com_rem);

  // muxes
  assign crc_in = (body) ? msg_in : rem_bit;
  assign rem_bit = (firstRem) ? ~crc_out[4] : com_rem[rem_cnt];
  assign msg_out = crc_in;

  // FSM to control state
  always_ff @(posedge clk, posedge rst) begin
    if (rst)
      cs <= BODY;
    else if (en)
      cs <= ns;
  end

  // ns/output logic
  always_comb begin
    // defaults
    body = 1;
    firstRem = 1;
    crc_clr = 0;
    cnt_clr = 0; cnt_en = 0; cnt_up = 0;
    rem_ld = 0; rem_en = 0; rem_up = 0;
    ns = cs;
    case (cs)
      BODY: begin
              cnt_en = (cnt != 11) ? 1 : 0;
              cnt_up = (cnt != 11) ? 1 : 0;
              body = (cnt != 11) ? 1 : 0;
              firstRem = (cnt != 11) ? 0 : 1;
              rem_ld = (cnt != 11) ? 0 : 1;
              ns = (cnt != 11) ? BODY : REM;
            end
      REM:  begin
              body = 0;
              firstRem = 0;
              rem_en = 1;
              rem_up = 0;
              ns = (rem_cnt != 0) ? REM : DONE;
            end
      DONE: begin
              crc_clr = 1;
              cnt_clr = 1;
              ns = BODY;
            end
    endcase
  end

endmodule

// CRC16 sender
module crc16Sender(
  input   logic clk, rst, en, msg_in,
  output  logic msg_out);

  logic crc_in;
  logic [15:0]  crc_out, com_rem;
  logic crc_clr;
  logic cnt_clr, cnt_en, cnt_up;
  logic rem_ld, rem_en, rem_up;
  logic [6:0] cnt;
  logic [3:0] rem_cnt;

  logic body, firstRem, rem_bit;
  enum logic [1:0] {BODY, REM, DONE} cs, ns;

  // instantiate stuff
  crc16Calc calc(.*);
  counter #(7) msgCnt(clk, rst, cnt_clr, , en && cnt_en, cnt_up, , cnt);
  counter remCnt(clk, rst, , rem_ld, en && rem_en, rem_up, 4'd14, rem_cnt);
  // counters work as regs too!
  counter #(16) rem(clk, rst, , rem_ld, , , ~crc_out, com_rem);

  // muxes
  assign crc_in = (body) ? msg_in : rem_bit;
  assign rem_bit = (firstRem) ? ~crc_out[15] : com_rem[rem_cnt];
  assign msg_out = crc_in;

  // FSM to control state
  always_ff @(posedge clk, posedge rst) begin
    if (rst)
      cs <= BODY;
    else if (en)
      cs <= ns;
  end

  // ns/output logic
  always_comb begin
    // defaults
    body = 1;
    firstRem = 1;
    crc_clr = 0;
    cnt_clr = 0; cnt_en = 0; cnt_up = 0;
    rem_ld = 0; rem_en = 0; rem_up = 0;
    ns = cs;
    case (cs)
      BODY: begin
              cnt_en = (cnt != 64) ? 1 : 0;
              cnt_up = (cnt != 64) ? 1 : 0;
              body = (cnt != 64) ? 1 : 0;
              firstRem = (cnt != 64) ? 0 : 1;
              rem_ld = (cnt != 64) ? 0 : 1;
              ns = (cnt != 64) ? BODY : REM;
            end
      REM:  begin
              body = 0;
              firstRem = 0;
              rem_en = 1;
              rem_up = 0;
              ns = (rem_cnt != 0) ? REM : DONE;
            end
      DONE: begin
              crc_clr = 1;
              cnt_clr = 1;
              ns = BODY;
            end
    endcase
  end

endmodule

// CRC5 receiver
module crc5Receiver(
  input   logic         clk, rst, en, msg_in,
  output  logic         done, OK,
  output  logic [10:0]  msg);

  logic [4:0] crc_out;
  logic       rcv_en;
  logic       crc_clr;
  logic       cnt_clr, cnt_en, cnt_up;
  logic [3:0] cnt;
  enum logic [1:0] {BODY, REM, DONE} cs, ns;

  // instantiate stuff
  crc5Calc calc(clk, rst, en, crc_clr, msg_in, crc_out);
  shiftReg rcvd(clk, rst, , en && rcv_en, msg_in, msg);
  counter msgCnt(clk, rst, cnt_clr, , en && cnt_en, cnt_up, , cnt);

  assign OK = crc_out == 5'b01100;

  // FSM to control state
  always_ff @(posedge clk, posedge rst) begin
    if (rst)
      cs <= BODY;
    else if (en)
      cs <= ns;
  end

  // ns/output logic
  always_comb begin
    // defaults
    cnt_clr = 0; cnt_en = 0; cnt_up = 0;
    rcv_en = 0;
    done = 0;
    ns = cs;
    case (cs)
      BODY: begin
              cnt_en = (cnt != 11) ? 1 : 0;
              cnt_up = (cnt != 11) ? 1 : 0;
              cnt_clr = (cnt != 11) ? 0 : 1;
              rcv_en = (cnt != 11) ? 1 : 0;
              ns = (cnt != 11) ? BODY : REM;
            end
      REM:  begin
              cnt_en = 1;
              cnt_up = 1;
              cnt_clr = (cnt != 3) ? 0 : 1;
              ns = (cnt != 3) ? REM : DONE;
            end
      DONE: begin
              done = 1;
              cnt_clr = 1;
              ns = BODY;
            end
    endcase
  end

  always @(posedge done)
    #1 $display("msg received! msg: %b, OK = %b", msg, OK);

endmodule

// CRC16 receiver
module crc16Receiver(
  input   logic         clk, rst, en, msg_in,
  output  logic         done, OK,
  output  logic [63:0]  msg);

  logic [15:0]  crc_out;
  logic       rcv_en;
  logic       crc_clr;
  logic       cnt_clr, cnt_en, cnt_up;
  logic [6:0] cnt;
  enum logic [1:0] {BODY, REM, DONE} cs, ns;

  // instantiate stuff
  crc16Calc calc(clk, rst, en, crc_clr, msg_in, crc_out);
  shiftReg #(64) rcvd(clk, rst, , en && rcv_en, msg_in, msg);
  counter #(7) msgCnt(clk, rst, cnt_clr, , en && cnt_en, cnt_up, , cnt);

  assign OK = crc_out == 16'h800D;

  // FSM to control state
  always_ff @(posedge clk, posedge rst) begin
    if (rst)
      cs <= BODY;
    else if (en)
      cs <= ns;
  end

  // ns/output logic
  always_comb begin
    // defaults
    cnt_clr = 0; cnt_en = 0; cnt_up = 0;
    rcv_en = 0;
    done = 0;
    ns = cs;
    case (cs)
      BODY: begin
              cnt_en = (cnt != 64) ? 1 : 0;
              cnt_up = (cnt != 64) ? 1 : 0;
              cnt_clr = (cnt != 64) ? 0 : 1;
              rcv_en = (cnt != 64) ? 1 : 0;
              ns = (cnt != 64) ? BODY : REM;
            end
      REM:  begin
              cnt_en = 1;
              cnt_up = 1;
              cnt_clr = (cnt != 14) ? 0 : 1;
              ns = (cnt != 14) ? REM : DONE;
            end
      DONE: begin
              done = 1;
              cnt_clr = 1;
              ns = BODY;
            end
    endcase
  end

  always @(posedge done)
    #1 $display("msg received! msg: %h, OK = %b", msg, OK);

endmodule

