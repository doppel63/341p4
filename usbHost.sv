/* usbHost.sv
 * Xiang Lin (xianglin)
 * Doci Mou (dmou)
 * 18-341 P4
 * November 14, 2013
 */


`ifndef types
  typedef enum bit [7:0] {PID_OUT = 8'b1110_0001, PID_IN = 8'b0110_1001,
                          PID_DATA = 8'b1100_0011,
                          PID_ACK = 8'b1101_0010, PID_NAK = 8'b0101_1010} pid_t;

  typedef struct packed {
    pid_t       pid;
    bit [6:0]   addr;
    bit [3:0]   endp;
    bit [63:0]  data;
  } pkt_t;
`endif


/****************
 *   USBHOST    *
 ****************
 * This module instantiates our bit stream encoder, bit stuffer, nrzi, and
 * dpdm modules in order to create a cohesive host module.  It also includes
 * various tasks that send, read, and write a packet.
 */
module usbHost(
  input  logic        clk, rst_L, 
  usbWires wires);

  /* 
   bit-level FSMs 
   */

  // handles bit-level protocols for sending a packet
  pkt_t pkt_out;
  bit   pkt_avail, pkt_sent;
  bit   send_stall, send_start, send_last;
  bit   send_raw_bit_stream, send_stuffed_bit_stream, stream_out;
  bit   invalid_input;

  bitStreamEncoder bse1(.*, .bit_out(send_raw_bit_stream));
  bitStuffer bs1(.*, .bit_in(send_raw_bit_stream),
                    .bit_out(send_stuffed_bit_stream));
  nrzi n1(.*, .bit_stream(send_stuffed_bit_stream));

  // handles bit-level protocols for receiving a packet
  pkt_t pkt_in;
  bit   pkt_rcvd, pkt_ok, ack;
  bit   rcv_stall, rcv_start, rcv_last;
  bit   EOP_ok, bit_stuff_ok, sending;
  bit   rcv_raw_bit_stream, rcv_stuffed_bit_stream, stream_in;

  bitStreamDecoder bsd1(.*, .bit_in(rcv_raw_bit_stream));
  bitUnstuffer bu1(.*, .bit_in(rcv_stuffed_bit_stream),
                       .bit_out(rcv_raw_bit_stream));
  nrzi_dec nd1(.*, .bit_stream(rcv_stuffed_bit_stream));

  // dpdm is linked to both parts
  dpdm d1(.*);

  /* 
   protocol level FSM 
   */

  // stuff between FSMs
  bit         transaction, protocol_avail, pStart;
  bit         clear, send_addr, protocol_OK;
  bit [63:0]  data_in, data_out;
  // stuff from rwFSM to tb/OS
  bit         start, read, done, trans_OK;
  bit [15:0]  p_mempage;
  bit [63:0]  write_data, data_tb;

  protocolFSM pFSM(.*);
  rwFSM rFSM(.*);


  /* PRELABREQUEST */
  // sends an OUT packet with ADDR=5 and ENDP=4
  // packet should have SYNC and EOP too
  task prelabRequest(
    input  bit [7:0]   data);

    pkt_out.pid <= PID_OUT; pkt_out.addr <= 5; pkt_out.endp <= 4;
    pkt_avail <= 1;
    @(posedge clk);
    pkt_avail <= 0;
    repeat (40) @(posedge clk);

  endtask: prelabRequest

  /* READDATA */
  // host sends memPage to thumb drive and then gets data back from it
  // then returns data and status to the caller
  task readData(
    input   bit [15:0]  mempage, // Page to write
    output  bit [63:0]  data, // array of bytes to write
    output  bit         success);

    start <= 1; read <= 1; p_mempage <= mempage;
    @(posedge clk);
    start <= 0;
    wait(done)
    data <= data_tb; success <= trans_OK;
    @(posedge clk);
  
  endtask: readData

  /* WRITEDATA */
  // Host sends memPage to thumb drive and then sends data
  // then returns status to the caller
  task writeData(
    input   bit [15:0]  mempage, // Page to write
    input   bit [63:0]  data, // array of bytes to write
    output  bit         success);

    start <= 1; read <= 0; p_mempage <= mempage; write_data <= data;
    @(posedge clk);
    start <= 0;
    wait(done)
    success <= trans_OK;
    @(posedge clk);

  endtask: writeData

endmodule: usbHost


/*****************
 *    R/W FSM    *
 *****************
 * State machine for read/write transactions.
 *
 * Start with an OUT transaction, always.  This sends the address to 
 * protocolFSM.  If it is a read transaction (indicated by read), send the
 * data from protocolFSM (data_out) to the testbench through (data_tb).  If
 * it is a write transaction (~read), send the testbench input (write_data) to
 * the protocolFSM (data_in).  If at any point in the protocolFSM the program
 * is to be reset, clear is toggled and both FSMs return to the IDLE state.
 */

module rwFSM(
  input   logic [15:0]  p_mempage,      // address from task
  input   logic [63:0]  write_data,     // data from writeData task for WRITEs
  input   logic [63:0]  data_out,       // data from protocolFSM (data to tb)
  input   logic         protocol_avail, // data from protocolFSM (transaction complete)
  input   logic         protocol_OK,    // indicates no errors from protocol
  input   logic         read,           // signal from testbench for read/write
  input   logic         start,          // signal from testbench to start
  input   logic         clk, rst_L,
  output  logic         pStart,         // soft start from protocol
  output  logic [63:0]  data_tb,        // data to testbench
  output  logic         done,           // signal to testbench that it's done
  output  logic         trans_OK,       // indicates transaction was OK
  output  logic         transaction,    // data to protocolFSM (1=>in)
  output  logic [63:0]  data_in,        // data to protocolFSM
  output  logic         send_addr);     // data to protocolFSM

  enum    logic [2:0] {IDLE, OUT, OUT_WAIT, OUT2, IN
                      } RWState, nextRWState;

  always_ff @(posedge clk) begin
    if (~rst_L)
      RWState <= IDLE;
    else
      RWState <= nextRWState;
  end

  // Note: some outputs don't have default values because they are don't cares.
  // Inn those situations (their enable signals are off).
  always_comb begin
    trans_OK = protocol_OK;
    nextRWState = RWState;
    pStart = 1'b0;
    done = 1'b0;
    data_tb = data_out; // send data from device to tb
    send_addr = 0;
    case (RWState)
      IDLE:           begin
                        if (start) begin
                          // send address!
                          // assume mempage and write_data from task stays on line
                          data_in = {48'd0, p_mempage};
                          transaction = 1'b0;
                          send_addr = 1'b1;
                          nextRWState = OUT;
                          pStart = 1'b1;
                        end
                      end
      OUT:            begin
                        if (protocol_avail)
                          // signals that protocol is done
                          nextRWState = OUT_WAIT;
                        else
                          nextRWState = OUT;
                      end
      OUT_WAIT:       begin   
                        // this state gives protocolFSM time to get back to IDLE
                        if (~read) begin
                            // send data!
                            data_in = write_data;
                            transaction = 1'b0;
                            send_addr = 1'b0;
                            nextRWState = OUT2;
                            pStart = 1'b1;
                          end
                          else begin
                            // receive data!
                            transaction = 1'b1;
                            send_addr = 1'b0;
                            nextRWState = IN;
                            pStart = 1'b1;
                          end
                      end
      OUT2:           begin
                        if (protocol_avail) begin
                          done = 1;
                          nextRWState = IDLE;
                        end
                        else
                          nextRWState = OUT2;
                      end
      IN:             begin
                        if (protocol_avail) begin
                          done = 1;
                          nextRWState = IDLE;
                        end
                        else
                          nextRWState = IN;
                      end                
    endcase
  end
endmodule: rwFSM


/*****************
 *  PROTOCOLFSM  *
 *****************
 * Receives R/W transaction data from rwFSM and translates into IN/OUT
 * transactions.  Completes transactions, and sends results to rwFSM.
 */
module protocolFSM(
  input   logic         transaction, pkt_ok, pkt_sent, pkt_rcvd, send_addr,
  input   logic         pStart, 
  input   logic         clk, rst_L,
  input   pkt_t         pkt_in,
  input   logic [63:0]  data_in,            // from R/W FSM
  output  logic         pkt_avail, ack,
  output  pkt_t         pkt_out,
  output  logic [63:0]  data_out,           // to R/W FSM
  output  logic         protocol_avail,     // to R/W FSM
  output  logic         protocol_OK);       // to R/W FSM

  enum    logic [2:0] {IDLE, WAIT_IN, IN, WAIT_OUT, WAIT_OUT2, OUT, STANDBY,
                      ERROR} protocolState, nextProtocolState;

  logic                 error;
  logic                 timeoutEn, timeoutClr;
  logic                 totalTimeoutEn, totalTimeoutClr;
  logic                 corruptEn, corruptClr;
  logic [3:0]           totalTimeoutCtr, corruptCtr;
  logic [7:0]           timeoutCtr;

  counter #(8)  timeout(.clk(clk), .rst_L(rst_L), .clr(timeoutClr), .ld(1'b0),
                        .en(timeoutEn), .up(1'b1), .val(8'b0),
                        .cnt(timeoutCtr));
  counter #(4)  totalTimeouts(.clk(clk), .rst_L(rst_L), .clr(totalTimeoutClr),
                        .ld(1'b0), .en(totalTimeoutEn), .up(1'b1), .val(4'b0),
                        .cnt(totalTimeoutCtr));
  counter #(4)  corrupt(.clk(clk), .rst_L(rst_L), .clr(corruptClr), .ld(1'b0),
                        .en(corruptEn), .up(1'b1), .val(4'b0),
                        .cnt(corruptCtr));

  always_ff @(posedge clk or negedge rst_L) begin
    if (~rst_L)
      protocolState <= IDLE;
    else
      protocolState <= nextProtocolState;
  end

  always_ff @(posedge clk, negedge rst_L) begin
    if (~rst_L)
      protocol_OK <= 1;
    else if (pStart)
      protocol_OK <= 1;
    else if (error)
      protocol_OK <= 0;
  end

  always_comb begin
    nextProtocolState = protocolState;
    timeoutClr = 1'b0;
    totalTimeoutClr = 1'b0;
    corruptClr = 1'b0;
    timeoutEn = 1'b0;
    totalTimeoutEn = 1'b0;
    corruptEn = 1'b0;
    pkt_avail = 1'b0;
    protocol_avail = 1'b0;
    error = 0;
    ack = 0;
    data_out = pkt_in.data;
    pkt_out.data = data_in;
    pkt_out.addr = 5;                   // always 5
    pkt_out.endp = (send_addr) ? 4 : 8; // depends if sending address or data
    pkt_out.pid = PID_IN;               // default, should never be used
    case (protocolState)
      IDLE:             begin
                          corruptClr = 1'b1;
                          timeoutClr = 1'b1;
                          totalTimeoutClr = 1'b1;
                          // start transaction
                          if (pStart) begin
                            pkt_avail = 1'b1;
                            // if IN, transmit PKT_IN
                            if (transaction) begin
                              pkt_out.pid = PID_IN;
                              nextProtocolState = WAIT_IN;
                            end
                            // if OUT, transmit PKT_OUT
                            else if (~transaction) begin
                              pkt_out.pid = PID_OUT;
                              nextProtocolState = WAIT_OUT;
                            end
                          end
                        end
      WAIT_IN:          begin
                          if (pkt_sent)
                            nextProtocolState = IN;
                          else
                            nextProtocolState = WAIT_IN;
                        end
      IN:               begin
                          if (pkt_rcvd) begin
                            ack = 1'b1;
                            pkt_avail = 1'b1;
                            // if data is corrupt send it a NAK
                            if (~pkt_ok) begin
                              corruptEn = 1'b1;
                              timeoutClr = 1'b1;
                              pkt_out.pid = PID_NAK;
                              // if 8th time corrupt quit
                              if (corruptCtr == 8) begin
                                error = 1;
                                pkt_avail = 0;
                                nextProtocolState = ERROR;
                              end
                              else
                                nextProtocolState = WAIT_IN;
                            end
                            // if data's ok acknowledge and go to idle
                            else if (pkt_ok) begin
                              pkt_out.pid = PID_ACK;
                              data_out = pkt_in.data;
                              nextProtocolState = STANDBY;
                            end
                          end
                          // packet has not been recieved yet keep waiting
                          else if (~pkt_rcvd) begin
                            timeoutEn = 1'b1;
                            // if the request timed out (255)
                            if (timeoutCtr == 255) begin
                              ack = 1'b1;
                              totalTimeoutEn = 1'b1;
                              pkt_avail = 1'b1;
                              pkt_out.pid = PID_NAK;
                              // if 8th time timed out quit
                              if (totalTimeoutCtr == 8) begin
                                error = 1;
                                pkt_avail = 0;
                                nextProtocolState = ERROR;
                              end
                              else
                                nextProtocolState = WAIT_IN;
                              end
                          end
                        end
      STANDBY:          begin
                          if (pkt_sent) begin
                            protocol_avail = 1'b1;
                            nextProtocolState = IDLE;
                          end   
                          else
                            nextProtocolState = STANDBY;
                        end
      WAIT_OUT:         begin
                          if (pkt_sent) begin
                            nextProtocolState = WAIT_OUT2;
                            pkt_avail = 1'b1;
                            pkt_out.pid = PID_DATA;
                          end
                          else
                            nextProtocolState = WAIT_OUT;
                        end
      WAIT_OUT2:        begin
                          if (pkt_sent)
                            nextProtocolState = OUT;
                          else
                            nextProtocolState = WAIT_OUT2;
                        end
      OUT:              begin
                          if (pkt_rcvd) begin
                            ack = 1'b1;
                            // everything's OK
                            if (pkt_in.pid == PID_ACK) begin
                              nextProtocolState = IDLE;
                              protocol_avail = 1'b1;
                            end
                            // data was corrupted
                            else if (pkt_in.pid == PID_NAK) begin
                              corruptEn = 1'b1;
                              timeoutClr = 1'b1;
                              pkt_avail = 1'b1;
                              pkt_out.pid = PID_DATA;
                              // data was corrupted for 8th time
                              if (corruptCtr == 8) begin
                                error = 1;
                                pkt_avail = 0;
                                nextProtocolState = ERROR;
                              end
                              else
                                nextProtocolState = WAIT_OUT2;
                            end
                          end
                          else begin
                            timeoutEn = 1'b1;
                            // timeout waiting for response
                            if (timeoutCtr == 255) begin
                              ack = 1'b1;
                              totalTimeoutEn = 1'b1;
                              pkt_avail = 1'b1;
                              pkt_out.pid = PID_DATA;
                              // timeout for 8th time
                              if (totalTimeoutCtr == 7) begin
                                error = 1;
                                nextProtocolState = STANDBY;
                              end
                              else
                                nextProtocolState = WAIT_OUT2;
                            end
                            else
                              nextProtocolState = OUT;
                          end
                        end
      ERROR:            begin
                          protocol_avail = 1;
                          nextProtocolState = IDLE;
                        end
      endcase
  end
endmodule: protocolFSM


/*************************
 *    BITSTREAMENCODER   *
 *************************
 * This module takes the information wtihin a packet and converts it to a
 * a serial bit string when sent_sync is toggled.  It also asserts flag 
 * last at the same clock cycle as the last bit of the input.
 */
module bitStreamEncoder(
  input   logic         clk, rst_L,
  input   bit           pkt_avail,
  input   pkt_t         pkt_out,
  input   bit           send_stall,
  output  bit           bit_out, send_start, send_last);

  // internal wires
  enum    bit [7:0]    {OUT = 8'b1110_0001, IN = 8'b0110_1001,
                        DATA0 = 8'b1100_0011,
                        ACK = 8'b1101_0010, NAK = 8'b0101_1010} pid;
  bit [6:0]             addr;
  bit [3:0]             endp;
  bit [63:0]            data;
  bit                   crc5_in, crc5, crc16_in, crc16;

  // internal control points; also wires anyway
  bit                   crc5_en, crc16_en;
  bit [2:0]             sync_cnt;
  bit [1:0]             endp_cnt;
  bit [2:0]             pid_cnt, addr_cnt;
  bit [5:0]             data_cnt;
  bit [2:0]             crc5_cnt;
  bit [3:0]             crc16_cnt;
  bit [1:0]             done_cnt;

  enum    bit [3:0]     {IDLE, SYNC, PID, ADDR, ENDP, CRC5, DATA, CRC16, DONE
                        } state;

  // instantiate counters datapath as registers for holding stuff
  counter #(8)  pidReg(.clk(clk), .rst_L(rst_L), .clr(), .ld(pkt_avail), .en(),
                       .up(), .val(pkt_out.pid), .cnt(pid));
  counter #(7)  addrReg(.clk(clk), .rst_L(rst_L), .clr(), .ld(pkt_avail), .en(),
                        .up(), .val(pkt_out.addr), .cnt(addr));
  counter #(4)  endpReg(.clk(clk), .rst_L(rst_L), .clr(), .ld(pkt_avail), .en(),
                        .up(), .val(pkt_out.endp), .cnt(endp));
  counter #(64) dataReg(.clk(clk), .rst_L(rst_L), .clr(), .ld(pkt_avail), .en(),
                        .up(), .val(pkt_out.data), .cnt(data));

  crc5Sender  crc5s(.en(~send_stall && crc5_en), .msg_in(crc5_in),
                    .msg_out(crc5), .*);
  crc16Sender crc16s(.en(~send_stall && crc16_en), .msg_in(crc16_in),
                    .msg_out(crc16), .*);

  always_comb begin
    // output mux, determined by which state we're in
    bit_out = 'd0; // could be x?
    crc5_en = 0;
    crc16_en = 0;
    case (state)
      SYNC:             
                          bit_out = sync_cnt == 7;
      PID:              
                          bit_out = pid[pid_cnt];
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
    crc16_in = data[63-data_cnt];
    // tell bit stuffer to start checking for 1's on the last bit of PID
    send_start = pid_cnt == 7;
    // last signal asserted when on the last bit of crc5, crc16 or pid depending
    // on packet AND not stalling for bit stuffing
    send_last = ~send_stall && ((crc5_cnt == 4) || (crc16_cnt == 15) ||
                ((pid == ACK || pid == NAK) && pid_cnt == 7));
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
      done_cnt <= 0;
      state <= IDLE;
    end
    else if (~send_stall) begin
      // only change values if no stall signal from bit stuffing
      case (state)
        IDLE:           begin
                          state <= (pkt_avail) ? SYNC : IDLE;
                        end
        SYNC:           begin
                          sync_cnt <= sync_cnt + 1;
                          state <= (sync_cnt == 7) ? PID : SYNC;
                        end
        PID:            begin
                          if (pid_cnt < 7) begin
                            pid_cnt <= pid_cnt + 1;
                            state <= PID;
                          end
                          else begin
                            pid_cnt <= 0;
                            case (pid)
                              IN, OUT:  state <= ADDR;
                              DATA0:    state <= DATA;
                              ACK, NAK: state <= DONE;
                              default:  state <= DONE;
                            endcase
                           end
                        end
        ADDR:           begin
                          addr_cnt <= (addr_cnt < 6) ? addr_cnt + 1 : 0;
                          state <= (addr_cnt < 6) ? ADDR : ENDP;
                        end
        ENDP:           begin
                          endp_cnt <= (endp_cnt < 3) ? endp_cnt + 1 : 0;
                          state <= (endp_cnt < 3) ? ENDP : CRC5;
                        end
        CRC5:           begin
                          crc5_cnt <= (crc5_cnt < 4) ? crc5_cnt + 1 : 0;
                          state <= (crc5_cnt < 4) ? CRC5 : DONE;
                        end
        DATA:           begin
                          data_cnt <= (data_cnt < 63) ? data_cnt + 1 : 0;
                          state <= (data_cnt < 63) ? DATA : CRC16;
                        end
        CRC16:          begin
                          crc16_cnt <= (crc16_cnt < 15) ? crc16_cnt + 1 : 0;
                          state <= (crc16_cnt < 15) ? CRC16 : DONE;
                        end
        DONE:           begin
                          done_cnt <= (done_cnt < 2) ? done_cnt + 1 : 0;
                          state <= (done_cnt < 2) ? DONE : ((pkt_avail) ? SYNC : IDLE);
                        end
      endcase
    end
  end
endmodule: bitStreamEncoder


/**********************
 *  BITSTREAMDECODER  *
 **********************
 * Decodes the incoming stream of data into a packet.
 */
module bitStreamDecoder(
  input   logic clk, rst_L,
  // to rest of receiver stuff
  input   bit   bit_in, rcv_stall, sending,
  input   bit   bit_stuff_ok, EOP_ok, invalid_input,
  output  bit   rcv_start, rcv_last,
  // to protocol FSM
  input   bit   ack,
  output  pkt_t pkt_in,
  output  bit   pkt_rcvd, pkt_ok);

  // internal wires
  enum    bit [7:0]  {OUT = 8'b1110_0001, IN = 8'b0110_1001,
                      DATA0 = 8'b1100_0011,
                      ACK = 8'b1101_0010, NAK = 8'b0101_1010} next_pid;
  bit crc5_en, crc5_done, crc5_ok;
  bit crc16_en, crc16_done, crc16_ok;

  // implicit registers for counting stuff
  bit [2:0] sync_cnt, pid_cnt, addr_cnt;
  bit [1:0] endp_cnt, done_cnt;
  bit [5:0] data_cnt;

  enum bit [3:0] {WAIT, PID, ADDR, ENDP, CRC5, DATA, CRC16, EOP, DONE} state;

  crc5Receiver crc5Rcv(.en(~rcv_stall & crc5_en), .msg_in(bit_in),
                       .done(crc5_done), .OK(crc5_ok), .msg(), .*);
  crc16Receiver crc16Rcv(.en(~rcv_stall & crc16_en), .msg_in(bit_in),
                       .done(crc16_done), .OK(crc16_ok), .msg(), .*);

  always_comb begin
    next_pid = {bit_in, pkt_in.pid[6:0]};
    crc5_en = state == ADDR || state == ENDP || state == CRC5;
    crc16_en = state == DATA || state == CRC16;
    pkt_rcvd = state == DONE;
    // start unstuffing bits after PID
    rcv_start = pid_cnt == 7;
    // last bit to check for unstuffing on last bit of crc5, crc16 or pid
    rcv_last = ~rcv_stall & (crc5_done | crc16_done |
               ((next_pid == ACK || next_pid == NAK) && pid_cnt == 7));
  end

  // FF to hold pkt_ok. pkt is ok until ack is seen because pkt does not change
  // while in the DONE state
  always_ff @(posedge clk, negedge rst_L) begin
    if (~rst_L)
      pkt_ok <= 1;
    else if (ack)
      pkt_ok <= 1;
    else if (~bit_stuff_ok | ~EOP_ok | (state == EOP & (~crc5_ok | ~crc16_ok)))
      pkt_ok <= 0;  // parts of the receiver detect an error
    else if (invalid_input & (state == PID | state == ADDR | state == ENDP |
                              state == CRC5 | state == DATA | state == CRC16))
      pkt_ok <= 0;  // dpdm detects invalid combination of dp and dm
    else if (state == EOP & ~(pkt_in.pid == IN | pkt_in.pid == OUT |
                              pkt_in.pid == DATA0 |
                              pkt_in.pid == ACK | pkt_in.pid == NAK))
      pkt_ok <= 0;  // invalid pid
  end

  // FSM state logic
  always_ff @(posedge clk, negedge rst_L) begin
    if (~rst_L) begin
      state <= WAIT;
      sync_cnt <= 0;
      pid_cnt <= 0;
      addr_cnt <= 0;
      endp_cnt <= 0;
      data_cnt <= 0;
      done_cnt <= 0;
    end
    else if (ack | sending) begin
      state <= WAIT;
      sync_cnt <= 0;
      pid_cnt <= 0;
      addr_cnt <= 0;
      endp_cnt <= 0;
      data_cnt <= 0;
      done_cnt <= 0;
    end
    else if (~rcv_stall) begin
      case (state)
        WAIT:   begin
                  if (sync_cnt < 7) begin
                    state <= WAIT;
                    sync_cnt <= (~bit_in) ? sync_cnt + 1 : 0;
                  end
                  else begin
                    state <= (~bit_in) ? WAIT : PID;
                    sync_cnt <= (~bit_in) ? sync_cnt : 0;
                  end
                end
        PID:    begin
                  pkt_in.pid[pid_cnt] <= bit_in;
                  if (pid_cnt < 7) begin
                    pid_cnt <= pid_cnt + 1;
                    state <= PID;
                  end
                  else begin
                    pid_cnt <= 0;
                    case (next_pid)
                      IN, OUT:  state <= ADDR;
                      DATA0:    state <= DATA;
                      ACK, NAK: state <= EOP;
                      default:  state <= EOP;
                    endcase
                  end
                end
        ADDR:   begin
                  pkt_in.addr[addr_cnt] <= bit_in;
                  addr_cnt <= (addr_cnt < 6) ? addr_cnt + 1 : 0;
                  state <= (addr_cnt < 6) ? ADDR : ENDP;
                end
        ENDP:   begin
                  pkt_in.endp[endp_cnt] <= bit_in;
                  endp_cnt <= (endp_cnt < 3) ? endp_cnt + 1 : 0;
                  state <= (endp_cnt < 3) ? ENDP : CRC5;
                end
        CRC5:   begin
                  state <= (crc5_done) ? EOP : CRC5;
                end
        DATA:   begin
                  pkt_in.data[63-data_cnt] <= bit_in;
                  data_cnt <= (data_cnt < 63) ? data_cnt + 1 : 0;
                  state <= (data_cnt < 63) ? DATA : CRC16;
                end
        CRC16:  begin
                  state <= (crc16_done) ? EOP : CRC16;
                end
        EOP:    begin
                  done_cnt <= (done_cnt < 2) ? done_cnt + 1 : 0;
                  state <= (done_cnt < 2) ? EOP : DONE;
                end
        DONE:   begin
                  state <= (ack) ? WAIT : DONE;
                end
        default: state <= state;
      endcase
    end
  end
endmodule: bitStreamDecoder


/******************
 *    CRC5CALC    *
 ******************
 * Module calculating the CRC of input, with intended remainder 5'b01100.
 */
module crc5Calc(
  input   logic       clk, rst_L,
  input   bit         en, crc_clr, crc_in,
  output  bit [4:0]   crc_out);

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

endmodule: crc5Calc


/*******************
 *    CRC16CALC    *
 *******************
 * Module calculating the CRC of input, with intended remainder 16'h800D.
 */
module crc16Calc(
  input   logic       clk, rst_L,
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

endmodule: crc16Calc


/**************
 *   COUNTER  *
 **************
 * Basic counter module with clear, load, enable, up(/down) inputs.  Defaults
 * to four bits of input/output.
 */
module counter
  #(parameter WIDTH = 4)
  (input  logic           clk, rst_L,
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
endmodule: counter


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
endmodule: shiftReg


/********************
 *    CRC5SENDER    *
 ********************
 * Module that sends values to the CRC5 calculator.
 */
module crc5Sender(
  input   logic       clk, rst_L,
  input   bit         en, msg_in, pkt_avail,
  output  bit         msg_out);

  bit                 crc_in;
  bit [4:0]           crc_out, com_rem;
  bit                 crc_clr;
  bit                 cnt_clr, cnt_en, cnt_up;
  bit                 rem_ld, rem_en, rem_up;
  bit [3:0]           cnt, rem_cnt;
  bit                 body, firstRem, rem_bit;
  
  enum    bit [1:0] {BODY, REM, DONE} cs, ns;

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
    else if (pkt_avail)
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
  input   logic       clk, rst_L,
  input   bit         en, msg_in, pkt_avail,
  output  bit         msg_out);

  bit                 crc_in;
  bit [15:0]          crc_out, com_rem;
  bit                 crc_clr;
  bit                 cnt_clr, cnt_en, cnt_up;
  bit                 rem_ld, rem_en, rem_up;
  bit [6:0]           cnt;
  bit [3:0]           rem_cnt;
  bit                 body, firstRem, rem_bit;

  enum  bit [1:0] {BODY, REM, DONE} cs, ns;

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
    else if (pkt_avail)
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
  input   logic       clk, rst_L,
  input   bit         en, msg_in, ack,
  output  bit         done, OK,
  output  bit [10:0]  msg);

  bit [4:0]           crc_out;
  bit                 rcv_en;
  bit                 crc_clr;
  bit                 cnt_clr, cnt_en, cnt_up;
  bit [3:0]           cnt;

  enum    bit [1:0] {BODY, REM, DONE} cs, ns;

  crc5Calc calc(clk, rst_L, en, crc_clr, msg_in, crc_out);
  shiftReg rcvd(clk, rst_L, , en && rcv_en, msg_in, msg);
  counter msgCnt(clk, rst_L, cnt_clr, , en && cnt_en, cnt_up, , cnt);

  assign OK = cs != DONE || crc_out == 5'b01100;

  // FSM state logic
  always_ff @(posedge clk, negedge rst_L) begin
    if (~rst_L)
      cs <= BODY;
    else if (ack)
      cs <= BODY;
    else if (en)
      cs <= ns;
  end

  // Next state and output logic
  always_comb begin
    crc_clr = 0;
    cnt_clr = 0; cnt_en = 0; cnt_up = 0;
    rcv_en = 0;
    done = 0;
    ns = cs;
    case (cs)
      BODY: begin
              cnt_en = (cnt != 10) ? 1 : 0;
              cnt_up = (cnt != 10) ? 1 : 0;
              cnt_clr = (cnt != 10) ? 0 : 1;
              rcv_en = 1;
              ns = (cnt != 10) ? BODY : REM;
            end
      REM:  begin
              cnt_en = 1;
              cnt_up = 1;
              cnt_clr = (cnt != 4) ? 0 : 1;
              done = (cnt != 4) ? 0 : 1;
              ns = (cnt != 4) ? REM : DONE;
            end
      DONE: begin
              //done = 1;
              cnt_clr = (ack) ? 1 : 0;
              crc_clr = (ack) ? 1 : 0;
              ns = (ack) ? BODY : DONE;
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
  input   logic       clk, rst_L,
  input   bit         en, msg_in, ack,
  output  bit         done, OK,
  output  bit [63:0]  msg);

  bit [15:0]          crc_out;
  bit                 rcv_en;
  bit                 crc_clr;
  bit                 cnt_clr, cnt_en, cnt_up;
  bit [6:0]           cnt;

  enum bit [1:0] {BODY, REM, DONE} cs, ns;

  crc16Calc calc(clk, rst_L, en, crc_clr, msg_in, crc_out);
  shiftReg #(64) rcvd(clk, rst_L, , en && rcv_en, msg_in, msg);
  counter #(7) msgCnt(clk, rst_L, cnt_clr, , en && cnt_en, cnt_up, , cnt);

  assign OK = cs != DONE || crc_out == 16'h800D;

  // FSM state logic
  always_ff @(posedge clk, negedge rst_L) begin
    if (~rst_L)
      cs <= BODY;
    else if (ack)
      cs <= BODY;
    else if (en)
      cs <= ns;
  end

  // Next state and output logic
  always_comb begin
    crc_clr = 0;
    cnt_clr = 0; cnt_en = 0; cnt_up = 0;
    rcv_en = 0;
    done = 0;
    ns = cs;
    case (cs)
      BODY: begin
              cnt_en = (cnt != 63) ? 1 : 0;
              cnt_up = (cnt != 63) ? 1 : 0;
              cnt_clr = (cnt != 63) ? 0 : 1;
              rcv_en = 1;
              ns = (cnt != 63) ? BODY : REM;
            end
      REM:  begin
              cnt_en = 1;
              cnt_up = 1;
              cnt_clr = (cnt != 15) ? 0 : 1;
              done = (cnt != 15) ? 0 : 1;
              ns = (cnt != 15) ? REM : DONE;
            end
      DONE: begin
              cnt_clr = (ack) ? 1 : 0;
              crc_clr = (ack) ? 1 : 0;
              ns = (ack) ? BODY : DONE;
            end
    endcase
  end
endmodule: crc16Receiver


/*******************
 *    BITSTUFFER   *
 *******************
 * When there have been six 1'b1s in a row, a 0 bit is added to the output
 * sequence and the stall flag is toggled.
 */
module bitStuffer(
  input   logic       clk, rst_L,
  input   bit         pkt_avail,
  input   bit         send_start,
  input   bit         send_last,
  input   bit         bit_in,
  output  bit         bit_out,
  output  bit         send_stall);

  bit                 clr;
  bit [2:0]           cnt;

  enum    bit [1:0] {IDLE, COUNTING, STALL} state;

  assign bit_out = (send_stall) ? 0 : bit_in;
  assign send_stall = state == STALL;
  assign clr = state == IDLE;

  counter #(3) onesCnt(.clk(clk), .rst_L(rst_L), .clr(send_stall|clr|~bit_in),
                       .ld(), .en(bit_in), .up(bit_in), .val(), .cnt(cnt));

  // FSM logic
  always_ff @(posedge clk, negedge rst_L) begin
    if (~rst_L)
      state <= IDLE;
    else if (pkt_avail) 
      state <= IDLE;
    else begin
      case (state)
        IDLE:     state <= (send_start) ? COUNTING : IDLE;
        COUNTING: if (send_last)
                    state <= IDLE;
                  else
                    state <= (cnt == 5 && bit_in) ? STALL : COUNTING;
        STALL:    state <= (send_last) ? IDLE : COUNTING;
        default:  state <= state;
      endcase
    end
  end
endmodule: bitStuffer

module bitUnstuffer(
  input   logic clk, rst_L,
  input   bit   ack,
  input   bit   rcv_start,
  input   bit   rcv_last,
  input   bit   bit_in,
  output  bit   bit_out,
  output  bit   rcv_stall,
  output  bit   bit_stuff_ok);

  bit       clr;
  bit [2:0] cnt;

  enum    bit [1:0] {IDLE, COUNTING, STALL, ERROR} state;

  assign bit_out = (rcv_stall) ? 0 : bit_in;
  assign rcv_stall = state == STALL;
  assign clr = state == IDLE;
  assign bit_stuff_ok = state != ERROR;

  counter #(3) onesCnt(.clk(clk), .rst_L(rst_L), .clr(rcv_stall|clr|~bit_in),
                       .ld(), .en(bit_in), .up(bit_in), .val(), .cnt(cnt));

  // FSM logic
  always_ff @(posedge clk, negedge rst_L) begin
    if (~rst_L)
      state <= IDLE;
    else if (ack)
      state <= IDLE;
    else begin
      case (state)
        IDLE:     state <= (rcv_start) ? COUNTING : IDLE;
        COUNTING: state <= (rcv_last) ? IDLE : 
                           ((cnt == 5 && bit_in) ? STALL : COUNTING);
        STALL:    if (~bit_in)
                    state <= (rcv_last) ? IDLE : COUNTING;
                  else
                    state <= ERROR;
        default:  state <= state;
      endcase
    end
  end

endmodule: bitUnstuffer


/************
 *   NRZI   *
 ************
 * Module "decodes" input bit_stream into serial output stream_out.  Values
 * remain the same if the current input bit is 1'b1 and flip when the current
 * input bit is 1'b0.
 */
module nrzi(
  input   reg         bit_stream,
  input   bit         pkt_avail,
  input   bit         clk, rst_L,
  input   bit         send_last,
  output  reg         stream_out);

  reg                 prev_bit;

  enum    bit         {START, RUN
                      } nrzi_state, next_nrzi_state;

  always_ff @(posedge clk or negedge rst_L) begin
    if (~rst_L) begin
      nrzi_state <= START;
      prev_bit <= 1;
    end
    else begin
      nrzi_state <= next_nrzi_state;
      if (nrzi_state == RUN)
        prev_bit <= (send_last) ? 1 : ((bit_stream) ? prev_bit : ~prev_bit);
    end
  end

  always_comb begin
    stream_out = prev_bit;
    next_nrzi_state = nrzi_state;
    case (nrzi_state)
      START: begin
        if (~pkt_avail)
          next_nrzi_state = START;
        else
          next_nrzi_state = RUN;
      end
      RUN: begin
        // output         stays the same on 1   changes on 0
        stream_out = (bit_stream) ? prev_bit : ~prev_bit;
        if (send_last)
          next_nrzi_state = START;
        else
          next_nrzi_state = RUN;
      end
    endcase
  end
endmodule: nrzi


/***************
 *  NRZI_DEC   *
 ***************
 * Decodes nrzi stream if it is not an outgoing packet(~sending).  The 
 * bit_stream is 1 if current stream_in is the same as prev_bit, or 0 if 
 * the current stream_in is different from prev_bit.
 */
module nrzi_dec(
  input   logic clk, rst_L,
  input   bit   stream_in, sending, ack, invalid_input,
  output  bit   bit_stream);

  bit prev_bit;

  always_ff @(posedge clk, negedge rst_L)
    if (~rst_L)
      prev_bit <= 1;
    else if (ack)
      prev_bit <= 1;
    else
      prev_bit <= (sending | invalid_input) ? prev_bit : stream_in;

  assign bit_stream = (sending) ? 0 : ~(stream_in ^ prev_bit) ;

endmodule: nrzi_dec


/************
 *   DPDM   *
 ************
 * This module converts the output stream from the nrzi module to D+ and D-
 * values for the usbWires bus, declared in the top module as wires.
 */
module dpdm(
  input   logic       clk, rst_L,
  // to sender stuff
  input   bit         stream_out, pkt_avail, send_last,
  output  bit         pkt_sent,
  // to receiver stuff
  output  bit         stream_in, EOP_ok, sending, invalid_input,
  input   bit         rcv_last, ack,
  usbWires            wires);

  bit                 dp, dm, en, en_dp, en_dm;

  assign wires.DP = (en) ? en_dp : 1'bz;
  assign wires.DM = (en) ? en_dm : 1'bz;
  assign dp = wires.DP;
  assign dm = wires.DM;
  // incoming bit stream is 0 if K, 1 if J or SE0
  assign stream_in = (~dp & dm) ? 1'b0 : 1'b1;
  
  enum bit [3:0] {IDLE, PACKET, SEOP1, SEOP2, SEOP3, REOP1, REOP2, REOP3, ERROR
                  } DPDM_state, next_DPDM_state;

  // input is invalid if we are seeing something other than J or K
  assign invalid_input = ~((dp & ~dm) | (~dp & dm));


  always_ff @(posedge clk, negedge rst_L)
    if (~rst_L)
      DPDM_state <= IDLE;
    else
      DPDM_state <= next_DPDM_state;

  always_comb begin
    next_DPDM_state = DPDM_state;
    // sending
    en = 1'b0;
    en_dp = 0;
    en_dm = 0;
    pkt_sent = 1'b0;
    // receiving
    // sending is asserted from first bit of SYNC to last bit of EOP
    sending = 0;
    EOP_ok = 1;
    case (DPDM_state)
      IDLE:   begin
        if (pkt_avail)
          next_DPDM_state = PACKET;
        else if (rcv_last)
          next_DPDM_state = REOP1;
        else
          next_DPDM_state = IDLE;
      end
      PACKET: begin
        sending = 1;
        en = 1'b1;
        en_dp = (stream_out) ? 1'b1 : 1'b0;
        en_dm = (stream_out) ? 1'b0 : 1'b1;
        next_DPDM_state = (send_last) ? SEOP1 : PACKET;
      end
      SEOP1:  begin
        sending = 1;
        en = 1'b1;
        en_dp = 1'b0;
        en_dm = 1'b0;
        next_DPDM_state = SEOP2;
      end
      SEOP2:  begin
        sending = 1;
        en = 1'b1;
        en_dp = 1'b0;
        en_dm = 1'b0;
        next_DPDM_state = SEOP3;
      end
      SEOP3:  begin
        sending = 1;
        en = 1'b1;
        en_dp = 1'b1;
        en_dm = 1'b0;
        pkt_sent = 1'b1;
        next_DPDM_state = (pkt_avail) ? PACKET : IDLE;
      end
      REOP1:  begin
        EOP_ok = (~dp & ~dm) ? 1 : 0;
        next_DPDM_state = (~dp & ~dm) ? REOP2 : ERROR;
      end
      REOP2:  begin
        EOP_ok = (~dp & ~dm) ? 1 : 0;
        next_DPDM_state = (~dp & ~dm) ? REOP3 : ERROR;
      end
      REOP3:  begin
        EOP_ok = (dp & ~dm) ? 1 : 0;
        next_DPDM_state = (dp & ~dm) ? IDLE : ERROR;
      end
      ERROR:  begin
        EOP_ok = 0;
        next_DPDM_state = (ack) ? ((pkt_avail) ? PACKET : IDLE) : ERROR;
      end
    endcase
  end
endmodule: dpdm

