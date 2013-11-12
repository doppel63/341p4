// hw3 Xiang Lin, xianglin
// CRC Sender and Receiver


// CRC16 stuff
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
	(input	logic 						clk, rst, clr, ld, en, up,
	input		logic	[WIDTH-1:0]	val,
	output	logic	[WIDTH-1:0]	cnt);

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
	(input	logic							clk, rst, clr, en, in,
	output	logic	[WIDTH-1:0]	out);

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

// CRC16 sender
module crc16Sender(
	input		logic	clk, rst, en, msg_in,
	output	logic msg_out);

	logic	crc_in;
	logic	[15:0]	crc_out, com_rem;
  logic crc_clr;
	logic cnt_clr, cnt_en, cnt_up;
	logic	rem_ld, rem_en, rem_up;
	logic	[6:0]	cnt;
  logic [3:0] rem_cnt;

	logic body, firstRem, rem_bit;
	enum logic [1:0] {BODY, REM, DONE} cs, ns;

	// instantiate stuff
	crc16Calc calc(.*);
	counter #(7) msgCnt(clk, rst, cnt_clr, , en && cnt_en, cnt_up, , cnt);
	counter remCnt(clk, rst, , rem_ld, en && rem_en, rem_up, 4'd14, rem_cnt);
	// counters work as regs too!
	counter	#(16) rem(clk, rst, , rem_ld, , , ~crc_out, com_rem);

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
			BODY:	begin
							cnt_en = (cnt != 64) ? 1 : 0;
							cnt_up = (cnt != 64) ? 1 : 0;
							body = (cnt != 64) ? 1 : 0;
							firstRem = (cnt != 64) ? 0 : 1;
							rem_ld = (cnt != 64) ? 0 : 1;
							ns = (cnt != 64) ? BODY : REM;
						end
			REM:	begin
							body = 0;
							firstRem = 0;
							rem_en = 1;
							rem_up = 0;
							ns = (rem_cnt != 0) ? REM : DONE;
						end
			DONE:	begin
              crc_clr = 1;
							cnt_clr = 1;
							ns = BODY;
						end
		endcase
	end

endmodule

module crc16Receiver(
	input		logic					clk, rst, en, msg_in,
	output	logic					done, OK,
	output	logic	[63:0]	msg);

	logic [15:0]	crc_out;
	logic				rcv_en;
  logic       crc_clr;
	logic				cnt_clr, cnt_en, cnt_up;
	logic	[6:0]	cnt;
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
			DONE:	begin
							done = 1;
							cnt_clr = 1;
							ns = BODY;
						end
    endcase
  end

	always @(posedge done)
		#1 $display("msg received! msg: %h, OK = %b", msg, OK);

endmodule

module tb(
	output	logic					clk, rst, en,
	output	logic					tb_in, 
	input		logic					done, OK,
	input		logic	[63:0]	msg,
	output	logic					error, bit_in_wrong);

	logic [63:0] msgToSend;

	initial begin
		clk = 0;
		forever #5 clk = ~clk;
	end

	// task to write stuff
	task sendMsg(
		input	logic	[63:0] msgToSend);

		#1 rst = 1;	// 1 time delay to allow old results to appear first
		rst <= #1 0;
    @(posedge clk); // do nothing
    @(posedge clk); // again to be sure
    en <= 1;
		for (int i = 0; i < 36 ; i++) begin
			tb_in <= msgToSend[i];
			@(posedge clk);
		end
    en <= 0; repeat (5) @(posedge clk);
    en <= 1;
    for (int i = 36; i < 64; i++) begin
      tb_in <= msgToSend[i];
      @(posedge clk);
    end
		// wait for CRC check
		repeat (16) @(posedge clk);
    en <= 0;
	endtask

	initial begin
		$display("\t\t     sndCalc\trcvCalc\tcom_rem\trcv_msg\t\tdone\tOK");
		$monitor($time,, "%h\t%h\t%h\t%h\t%b\t%b",
		snd.crc_out, rcv.crc_out, snd.com_rem, msg, done, OK);
		error <= 0; en <= 0;
		// test stuff
		sendMsg(64'hcafebabedeadbeef);
		//sendMsg(11'b0000_1000_111);
		//sendMsg(11'b1010_1000_111);

    /*
		// send last msg with error: complement all the bits to the receiver
		#1 rst = 1; // 1 time delay to allow old results to appear first
    rst <= #1 0;
		error <= 1;
		msgToSend = 64'hCAFEBABEDEADBEEF;
    en <= 1;
    for (int i = 63; i >= 0 ; i--) begin
      bit_in_wrong <= ~msgToSend[i];
			tb_in <= msgToSend[i];
      @(posedge clk);
    end
    // wait for CRC check
    repeat (16) @(posedge clk);
    en <= 0;
    */

		@(posedge clk);	// 1 clock cycle before finish
		$finish;
	end
endmodule

module top;
	logic         clk, rst, en;
  logic         tb_in, bit_in, done, OK;
  logic [63:0]  msg;

	tb tb1(clk, rst, en, tb_in, done, OK, msg, error, bit_in_wrong);
	crc16Sender snd(clk, rst, en, tb_in, bit_in);
	assign rcv_bit_in = (error) ? bit_in_wrong : bit_in;
  crc16Receiver rcv(clk, rst, en, rcv_bit_in, done, OK, msg);

endmodule
