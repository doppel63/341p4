/* usbHost.sv
 * Xiang Lin (xianglin)
 * Doci Mou (dmou)
 * 18-341 P4
 * November 14, 2013
 */

/****************
 *   USBHOST    *
 ****************
 * This module instantiates our bit stream encoder, bit stuffer, nrzi, and
 * dpdm modules in order to create a cohesive host module.  It also includes
 * various tasks that send, read, and write a packet.
 */
module usbHost
  (input logic clk, rst_L, 
  usbWires wires);

  bit pkt_avail, pkt_sent, stall;
  bit start, last;
  bit raw_bit_stream, stuffed_bit_stream, stream_out;
  bit [7:0] pid_in;
  bit [6:0] addr_in;
  bit [3:0] endp_in;
  bit [63:0] data_in;

  bitStreamEncoder bse1(.*, .bit_out(raw_bit_stream));
  bitStuffer bs1(.*, .bit_in(raw_bit_stream), .bit_out(stuffed_bit_stream));
  nrzi n1(.*, .bit_stream(stuffed_bit_stream), .stream_out(stream_out));
  dpdm d1(.*);
  
  // Tasks needed to be finished to run testbenches

  /* PRELABREQUEST */
  // sends an OUT packet with ADDR=5 and ENDP=4
  // packet should have SYNC and EOP too
  task prelabRequest
  (input bit  [7:0] data);

  pid_in <= 8'b1110_0001; addr_in <= 5; endp_in <= 4;
  pkt_avail <= 1;
  @(posedge clk);
  pkt_avail <= 0;
  repeat (40) @(posedge clk);

  endtask: prelabRequest

  /* READDATA */
  // host sends memPage to thumb drive and then gets data back from it
  // then returns data and status to the caller
  task readData
  (input  bit [15:0]  mempage, // Page to write
   output bit [63:0] data, // array of bytes to write
   output bit        success);

  endtask: readData

  /* WRITEDATA */
  // Host sends memPage to thumb drive and then sends data
  // then returns status to the caller
  task writeData
  (input  bit [15:0]  mempage, // Page to write
   input  bit [63:0] data, // array of bytes to write
   output bit        success);

  endtask: writeData

endmodule: usbHost

/*************************
 *    BITSTREAMENCODER   *
 *************************
 * This module takes the information wtihin a packet and converts it to a
 * a serial bit string when sent_sync is toggled.  It also asserts flag 
 * last at the same clock cycle as the last bit of the input.
 */
module bitStreamEncoder(
  input   logic        clk, rst_L,
  input   bit        pkt_avail,
  input   bit [7:0]  pid_in,
  input   bit [6:0]  addr_in,
  input   bit [3:0]  endp_in,
  input   bit [63:0] data_in,
  input   bit        stall,
  output  bit        bit_out, start, last);

  // internal wires
  enum bit [7:0] {OUT = 8'b1110_0001, IN = 8'b0110_1001,
                    DATA0 = 8'b1100_0011,
                    ACK = 8'b1101_0010, NAK = 8'b0101_1010} pid;

  bit [6:0]   addr;
  bit [3:0]   endp;
  bit [63:0]  data;
  bit         crc5_in, crc5, crc16_in, crc16;

  // internal control points; also wires anyway
  bit         crc5_en, crc16_en;
  bit [2:0]   sync_cnt;
  bit [1:0]   endp_cnt;
  bit [2:0]   pid_cnt, addr_cnt;
  bit [5:0]   data_cnt;
  bit [2:0]   crc5_cnt;
  bit [3:0]   crc16_cnt;

  // states for FSM
  enum bit [2:0] {IDLE, SYNC, PID, ADDR, ENDP, CRC5, DATA, CRC16} state;

  // instantiate counters datapath as registers for holding stuff
  counter #(8)  pidReg(.clk(clk), .rst_L(rst_L), .clr(), .ld(pkt_avail), .en(),
                       .up(), .val(pid_in), .cnt(pid));
  counter #(7)  addrReg(.clk(clk), .rst_L(rst_L), .clr(), .ld(pkt_avail), .en(),
                        .up(), .val(addr_in), .cnt(addr));
  counter #(4)  endpReg(.clk(clk), .rst_L(rst_L), .clr(), .ld(pkt_avail), .en(),
                        .up(), .val(endp_in), .cnt(endp));
  counter #(64) dataReg(.clk(clk), .rst_L(rst_L), .clr(), .ld(pkt_avail), .en(),
                        .up(), .val(data_in), .cnt(data));

  crc5Sender  crc5s(.clk(clk), .rst_L(rst_L), .en(~stall && crc5_en),
                      .msg_in(crc5_in), .msg_out(crc5));
  crc16Sender crc16s(.clk(clk), .rst_L(rst_L), .en(~stall && crc16_en),
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

  always_ff @(posedge clk, negedge rst_L) begin
    if (~rst_L) begin
      sync_cnt <= 0;
      pid_cnt <= 0;
      addr_cnt <= 0;
      endp_cnt <= 0;
      data_cnt <= 0;
      crc5_cnt <= 0;
      crc16_cnt <= 0;
      state <= IDLE;
    end
    else if (~stall) begin
      // only change values if no stall signal from bit stuffing
      case (state)
        IDLE:   begin
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
endmodule: bitStreamEncoder

/******************
 *    CRC5CALC    *
 ******************
 * Module calculating the CRC of input, with intended remainder 5'b01100.
 */
module crc5Calc(
  input   logic clk, rst_L,
  input   bit   en, crc_clr, crc_in,
  output  bit [4:0] crc_out);

  always_ff @(posedge clk, negedge rst_L) begin
    if (~rst_L)
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

/*******************
 *    CRC16CALC    *
 *******************
 * Module calculating the CRC of input, with intended remainder 16'h800D.
 */
module crc16Calc(
  input   logic         clk, rst_L,
  input   bit         en, crc_clr, crc_in,
  output  bit [15:0]  crc_out);

  always_ff @(posedge clk, negedge rst_L) begin
    if (~rst_L)
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

/**************
 *   COUNTER  *
 **************
 * Basic counter module with clear, load, enable, up(/down) inputs.  Defaults
 * to four bits of input/output.
 */
module counter
  #(parameter WIDTH = 4)
  (input  logic             clk, rst_L,
  input   bit             clr, ld, en, up,
  input   bit [WIDTH-1:0] val,
  output  bit [WIDTH-1:0] cnt);

  always_ff @(posedge clk, negedge rst_L) begin
    if (~rst_L)
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


/****************
 *   SHIFTREG   *
 ****************
 * Basic left shift register module.  Defaults to eleven bits of input/output.
 */
module shiftReg
  #(parameter WIDTH = 11)
  (input  logic             clk, rst_L,
  input   bit             clr, en, in,
  output  bit [WIDTH-1:0] out);

  always_ff @(posedge clk, negedge rst_L) begin
    if (~rst_L)
      out <= 'd0;
    else if (clr)
      out <= 'd0;
    else if (en) begin
      out <= out << 1;
      out[0] <= in;
    end
  end
endmodule


/********************
 *    CRC5SENDER    *
 ********************
 * Module that sends values to the CRC5 calculator.
 */
module crc5Sender(
  input   logic clk, rst_L,
  input   bit en, msg_in,
  output  bit msg_out);

  bit crc_in;
  bit [4:0] crc_out, com_rem;
  bit crc_clr;
  bit cnt_clr, cnt_en, cnt_up;
  bit rem_ld, rem_en, rem_up;
  bit [3:0] cnt, rem_cnt;
  bit body, firstRem, rem_bit;
  
  enum bit [1:0] {BODY, REM, DONE} cs, ns;

  crc5Calc calc(.*);
  counter msgCnt(clk, rst_L, cnt_clr, , en && cnt_en, cnt_up, , cnt);
  counter remCnt(clk, rst_L, , rem_ld, en && rem_en, rem_up, 4'd3, rem_cnt);
  counter #(5) rem(clk, rst_L, , rem_ld, , , ~crc_out, com_rem);

  // muxes
  assign crc_in = (body) ? msg_in : rem_bit;
  assign rem_bit = (firstRem) ? ~crc_out[4] : com_rem[rem_cnt];
  assign msg_out = crc_in;

  always_ff @(posedge clk, negedge rst_L) begin
    if (~rst_L)
      cs <= BODY;
    else if (en)
      cs <= ns;
  end

  // Next state output logic
  always_comb begin
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
endmodule: crc5Sender


/********************
 *    CRC16SENDER   *
 ********************
 * Module that sends values to the CRC16 calculator.
 */
module crc16Sender(
  input   logic clk, rst_L,
  input   bit en, msg_in,
  output  bit msg_out);

  bit crc_in;
  bit [15:0]  crc_out, com_rem;
  bit crc_clr;
  bit cnt_clr, cnt_en, cnt_up;
  bit rem_ld, rem_en, rem_up;
  bit [6:0] cnt;
  bit [3:0] rem_cnt;
  bit body, firstRem, rem_bit;

  enum bit [1:0] {BODY, REM, DONE} cs, ns;

  crc16Calc calc(.*);
  counter #(7) msgCnt(clk, rst_L, cnt_clr, , en && cnt_en, cnt_up, , cnt);
  counter remCnt(clk, rst_L, , rem_ld, en && rem_en, rem_up, 4'd14, rem_cnt);
  counter #(16) rem(clk, rst_L, , rem_ld, , , ~crc_out, com_rem);

  assign crc_in = (body) ? msg_in : rem_bit;
  assign rem_bit = (firstRem) ? ~crc_out[15] : com_rem[rem_cnt];
  assign msg_out = crc_in;

  // FSM state logic
  always_ff @(posedge clk, negedge rst_L) begin
    if (~rst_L)
      cs <= BODY;
    else if (en)
      cs <= ns;
  end

  // Next state and output logic
  always_comb begin
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
endmodule: crc16Sender


/*********************
 *    CRC5RECEIVER   *
 *********************
 * Module that receives values from the CRC5 calculator.
 */
module crc5Receiver(
  input   logic         clk, rst_L,
  input   bit         en, msg_in,
  output  bit         done, OK,
  output  bit [10:0]  msg);

  bit [4:0] crc_out;
  bit       rcv_en;
  bit       crc_clr;
  bit       cnt_clr, cnt_en, cnt_up;
  bit [3:0] cnt;

  enum bit [1:0] {BODY, REM, DONE} cs, ns;

  crc5Calc calc(clk, rst_L, en, crc_clr, msg_in, crc_out);
  shiftReg rcvd(clk, rst_L, , en && rcv_en, msg_in, msg);
  counter msgCnt(clk, rst_L, cnt_clr, , en && cnt_en, cnt_up, , cnt);

  assign OK = crc_out == 5'b01100;

  // FSM state logic
  always_ff @(posedge clk, negedge rst_L) begin
    if (~rst_L)
      cs <= BODY;
    else if (en)
      cs <= ns;
  end

  // Next state and output logic
  always_comb begin
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
  /*
  always @(posedge done)
    #1 $display("msg received! msg: %b, OK = %b", msg, OK);
  */
endmodule: crc5Receiver


/**********************
 *    CRC16RECEIVER   *
 **********************
 * Module that receives values from the CRC16 calculator.
 */
module crc16Receiver(
  input   logic         clk, rst_L,
  input   bit         en, msg_in,
  output  bit         done, OK,
  output  bit [63:0]  msg);

  bit [15:0]  crc_out;
  bit       rcv_en;
  bit       crc_clr;
  bit       cnt_clr, cnt_en, cnt_up;
  bit [6:0] cnt;
  enum bit [1:0] {BODY, REM, DONE} cs, ns;

  crc16Calc calc(clk, rst_L, en, crc_clr, msg_in, crc_out);
  shiftReg #(64) rcvd(clk, rst_L, , en && rcv_en, msg_in, msg);
  counter #(7) msgCnt(clk, rst_L, cnt_clr, , en && cnt_en, cnt_up, , cnt);

  assign OK = crc_out == 16'h800D;

  // FSM state logic
  always_ff @(posedge clk, negedge rst_L) begin
    if (~rst_L)
      cs <= BODY;
    else if (en)
      cs <= ns;
  end

  // Next state and output logic
  always_comb begin
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
  /*
  always @(posedge done)
    #1 $display("msg received! msg: %h, OK = %b", msg, OK);
  */
endmodule: crc16Receiver


/*******************
 *    BITSTUFFER   *
 *******************
 * When there have been six 1'b1s in a row, a 0 bit is added to the output
 * sequence and the stall flag is toggled.
 */
// for stuffing bits. 
//          NOTE: damnit linky.
module bitStuffer(
  input   logic clk, rst_L,
  input   bit pkt_avail,
  input   bit start,
  input   bit last,
  input   bit bit_in,
  output  bit bit_out,
  output  bit stall);

  bit       clr;
  bit [2:0] cnt;

  enum bit [1:0] {IDLE, COUNTING, STALL} state;

  assign bit_out = (stall) ? 0 : bit_in;
  assign stall = state == STALL;
  assign clr = state == IDLE;

  // counter for counting 1's. sounds like a band name
  //        NOTE:  ...
  counter #(3) onesCnt(.clk(clk), .rst_L(rst_L), .clr(stall|clr|~bit_in), .ld(),
                       .en(bit_in), .up(bit_in), .val(), .cnt(cnt));

  // FSM logic
  always_ff @(posedge clk, negedge rst_L) begin
    if (~rst_L)
      state <= IDLE;
    else if (pkt_avail) 
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
endmodule: bitStuffer


/************
 *   NRZI   *
 ************
 * Module "decodes" input bit_stream into serial output stream_out.  Values
 * remain the same if the current input bit is 1'b1 and flip when the current
 * input bit is 1'b0.
 */
module nrzi(
  input     reg       bit_stream,
  input     bit       pkt_avail,
  input     bit       clk, rst_L,
  input     bit       last,
  output    reg       stream_out);

  reg                   prev_bit;

  enum bit {START, RUN
              } nrzi_state, next_nrzi_state;

  always_ff @(posedge clk or negedge rst_L) begin
    if (~rst_L) begin
      nrzi_state <= START;
      prev_bit <= 1;
    end
    else begin
      nrzi_state <= next_nrzi_state;
      if (nrzi_state == RUN)
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
        else if (pkt_avail)
          next_nrzi_state = RUN;
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


/************
 *   DPDM   *
 ************
 * This module converts the output stream from the nrzi module to D+ and D-
 * values for the usbWires bus, declared in the top module as wires.
 */
module dpdm(
  input   bit   stream_out, pkt_avail, last,
  input   logic clk, rst_L,
  output  bit   pkt_sent,
  usbWires wires);

  bit dp, dm, en, en_dp, en_dm;

  assign wires.DP = (en) ? en_dp : 1'bz;
  assign wires.DM = (en) ? en_dm : 1'bz;
  assign dp = wires.DP;
  assign dm = wires.DM;
  
  enum bit [2:0] {IDLE, PACKET, EOP1, EOP2, EOP3
                  } DPDM_state, next_DPDM_state;

  always_ff @(posedge clk or negedge rst_L) begin
    if (~rst_L)
      DPDM_state <= IDLE;
    else
      DPDM_state <= next_DPDM_state;
  end

  always_comb begin
    next_DPDM_state = DPDM_state;
    en = 1'b0;
    en_dp = 0;
    en_dm = 0;
    pkt_sent = 1'b0;
    case (DPDM_state)
      IDLE:   begin
        if (pkt_avail) begin
          next_DPDM_state = PACKET;
        end
        else
          next_DPDM_state = IDLE;
      end
      PACKET: begin
        en = 1'b1;
        if (stream_out) begin
          en_dp = 1'b1;
          en_dm = 1'b0;
        end
        else if (~stream_out) begin
          en_dp = 1'b0;
          en_dm = 1'b1;
        end
        if (last)
          next_DPDM_state = EOP1;
        else if (~last) 
          next_DPDM_state = PACKET;
      end
      EOP1:   begin
        en = 1'b1;
        en_dp = 1'b0;
        en_dm = 1'b0;
        next_DPDM_state = EOP2;
      end
      EOP2:   begin
        en = 1'b1;
        en_dp = 1'b0;
        en_dm = 1'b0;
        next_DPDM_state = EOP3;
      end
      EOP3:   begin
        en = 1'b1;
        en_dp = 1'b1;
        en_dm = 1'b0;
        pkt_sent = 1'b1;
        next_DPDM_state = IDLE;
      end
    endcase
  end
endmodule: dpdm

/*
task K(usbWires wires);
  wires.DP = 1'b0;
  wires.DM = 1'b1;
endtask

task SE0(usbWires wires);
  wires.DP = 1'b0;
  wires.DP = 1'b0;
entask

task EOP(input bit clk, usbWires wires);
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

interface usbWires;
  tri0 DP;
  tri0 DM;
endinterface
*/
