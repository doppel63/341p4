/*
// Write your usb host here.  Do not modify the port list.
module usbHost
  (input logic clk, rst_L, 
  usbWires wires);

  assign wires.DP =  
 
  // Tasks needed to be finished to run testbenches

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
  input   logic        stall,
  output  logic        bit_out, start, last);

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
  logic [2:0]   sync_cnt;
  logic [1:0]   endp_cnt;
  logic [2:0]   pid_cnt, addr_cnt;
  logic [5:0]   data_cnt;
  logic [2:0]   crc5_cnt;
  logic [3:0]   crc16_cnt;

  // states for FSM
  enum logic [2:0] {IDLE, SYNC, PID, ADDR, ENDP, CRC5, DATA, CRC16} state;

  // instantiate stuff in datapath
  // use counters as registers for holding stuff
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
      SYNC:             bit_out = sync_cnt == 7;
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
    // tell bit stuffer to start checking for 1's on the last bit of PID
    start = pid_cnt == 7;
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
                  sync_cnt <= 0;
                  pid_cnt <= 0;
                  addr_cnt <= 0;
                  endp_cnt <= 0;
                  data_cnt <= 0;
                  crc5_cnt <= 0;
                  crc16_cnt <= 0;
                  state <= (pkt_avail) ? SYNC : IDLE;
                end
        SYNC:   begin
                  sync_cnt <= sync_cnt + 1;
                  state <= (sync_cnt == 7) ? PID : SYNC;
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
                  crc16_cnt <= (crc16_cnt < 15) ? crc16_cnt + 1 : 0;
                  state <= (crc16_cnt < 15) ? CRC16 : IDLE;
                end
      endcase
    end
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


module bitStuffer(
  input   logic clk, rst,
  input   logic start,
  input   logic last,
  input   logic bit_in,
  output  logic bit_out,
  output  logic stall);

  logic       clr;
  logic [2:0] cnt;
  enum logic [1:0] {IDLE, COUNTING, STALL} state;

  assign bit_out = (stall) ? 0 : bit_in;
  assign stall = state == STALL;
  assign clr = state == IDLE;

  // counter for counting 1's. sounds like a band name
  counter #(3) onesCnt(.clk(clk), .rst(rst), .clr(stall|clr|~bit_in), .ld(),
                       .en(bit_in), .up(bit_in), .val(), .cnt(cnt));

  // FSM
  always_ff @(posedge clk, posedge rst) begin
    if (rst)
      state <= IDLE;
    else begin
      case (state)
        IDLE:     state <= (start) ? COUNTING : IDLE;
        COUNTING: state <= (cnt == 5 && bit_in) ? STALL : COUNTING;
        STALL:    state <= (last) ? IDLE : COUNTING;
        default:  state <= state;
      endcase
    end
  end

endmodule

module nrzi(
  input     reg       bit_stream,
  input     bit       pkt_avail,
  input     bit       clk, rst,
  input     bit       last,
  output    reg       stream_out);

  reg                   prev_bit;

  /***
   * Things to remember:
   *    - output changes on 0
   *    - output stays the same on 1
   *    - first output bit is as if previous bit was a 1
   *    - all field types are sent except for the EOP
   */

  enum logic {START, RUN
              } nrzi_state, next_nrzi_state;

  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      nrzi_state <= START;
      prev_bit <= 1'b1;
    end
    else begin
      nrzi_state <= next_nrzi_state;
      prev_bit <= (bit_stream) ? prev_bit : ~prev_bit;
    end
  end

  always_comb begin
    stream_out = prev_bit;
    next_nrzi_state = nrzi_state;
    case (nrzi_state)
      START: begin
        if (~pkt_avail)
          next_nrzi_state = START;
        else if (pkt_avail) begin
          next_nrzi_state = RUN;
          //stream_out = (bit_stream) ? prev_bit : ~prev_bit;
        end
      end
      RUN: begin
        if (last) begin
          next_nrzi_state = START;
        end
        else if (~last) begin
          // output         stays the same on 1   changes on 0
          stream_out = (bit_stream) ? prev_bit : ~prev_bit;
          next_nrzi_state = RUN;
        end
      end
    endcase
  end
endmodule: nrzi
  
/*

task K(usbWires wires);
  wires.DP = 1'b0;
  wires.DM = 1'b1;
endtask

task SE0(usbWires wires);
  wires.DP = 1'b0;
  wires.DP = 1'b0;
entask

task EOP(input logic clk, usbWires wires);
  SE0(wires);
  @(posedge clk);
  SEO(wires);
  @(posedge clk);
  J(wires);
  @(posedge clk);
endtask

task SYNC(input logic clk, usbWires wires);
  repeat (7) begin
    K(wires);
    @(posedge clk);
  end
  J(wires);
  @(posedge clk);
endtask
*/
interface usbWires;
  tri0 DP;
  tri0 DM;
endinterface

module dpdm(
  input logic stream_out, pkt_avail, last,
  input logic clk, rst,
  usbWires wires);
  // ummm you put these wires here right?
  logic dp, dm, en, en_dp, en_dm;

  assign wires.DP = (en) ? en_dp : 1'bz;
  assign wires.DM = (en) ? en_dm : 1'bz;
  assign dp = wires.DP;
  assign dm = wires.DM;
  
  // note to self: tri0 net pull down when not driven

  enum logic [2:0] {IDLE, PACKET, EOP1, EOP2, EOP3
                  } DPDM_state, next_DPDM_state;

  always_ff @(posedge clk) begin
    if (rst)
      DPDM_state <= IDLE;
    else if (~rst)
      DPDM_state <= next_DPDM_state;
  end

  always_comb begin
    next_DPDM_state = DPDM_state;
    en = 1'b1;
    case (DPDM_state)
      IDLE:   begin
        if (pkt_avail) begin
          next_DPDM_state = PACKET;
        end
        else
          next_DPDM_state = IDLE;
      end
      PACKET: begin
        if (stream_out) begin
          en_dp = 1'b1;
          en_dm = 1'b0;
        end
        else if (~stream_out) begin
          en_dp = 1'b0;
          en_dm = 1'b1;
        end
        // last is sent on same as last bit of stream_out
        if (last)
          next_DPDM_state = EOP1;
        else if (~last) 
          next_DPDM_state = PACKET;
      end
      EOP1:   begin
        en_dp = 1'b0;
        en_dm = 1'b0;
        next_DPDM_state = EOP2;
      end
      EOP2:   begin
        en_dp = 1'b0;
        en_dm = 1'b0;
        next_DPDM_state = EOP3;
      end
      EOP3:   begin
        en_dp = 1'b0;
        en_dm = 1'b1;
        next_DPDM_state = IDLE;
      end
    endcase
  end
endmodule: dpdm
