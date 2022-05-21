/*
 * audio_pcm.v
 *
 * vim: ts=4 sw=4
 *
 * Copyright (C) 2020  Sylvain Munaut <tnt@246tNt.com>
 * SPDX-License-Identifier: CERN-OHL-P-2.0
 */

`default_nettype none

module audio_pcm_in2 (
	// Wishbone slave
	input  wire [ 1:0] wb_addr,
	output reg  [31:0] wb_rdata,
	input  wire [31:0] wb_wdata,
	input  wire        wb_we,
	input  wire        wb_cyc,
	output wire        wb_ack,

	// USB
	input  wire usb_sof,

	// Clock / Reset
	input  wire clk,
	input  wire rst
);

	// Signals
	// -------

	// Wishbone
	reg  b_ack;
	reg  b_we_csr;
	wire b_rd_rst;

	reg         run;

	// FSM
	localparam
		ST_IDLE  = 0,
		ST_RUN   = 1,
		ST_FLUSH = 2;

	reg  [ 1:0] state;
	reg  [ 1:0] state_nxt;

	wire running;

	// Timebase
	wire        tick;
	reg  [ 9:0] tick_cnt;

	reg  [15:0] tpf_cnt;
	reg  [15:0] tpf_cap;

	// FIFO
    wire [31:0] fw_data;
    wire        fw_ena;
    wire        fw_full;

    wire [31:0] fr_data;
    wire        fr_ena;
    wire        fr_empty;

	reg  [ 9:0] f_lvl;
	wire [ 9:0] f_mod;

	// Audio
	reg [15:0] rx_data;

	// Wishbone interface
	// ------------------

	// Ack
	always @(posedge clk)
		b_ack <= wb_cyc & ~b_ack;

	assign wb_ack = b_ack;

	// Write
	always @(posedge clk)
	begin
		if (b_ack) begin
			b_we_csr    <= 1'b0;
			//b_we_fifo   <= 1'b0;
		end else begin
			b_we_csr    <= wb_cyc & wb_we & (wb_addr == 2'b00);
			//b_we_fifo   <= wb_cyc & wb_we & (wb_addr == 2'b10);
		end
	end

	always @(posedge clk)
		if (rst)
			run <= 1'b0;
		else if (b_we_csr)
			run <= wb_wdata[0];

    assign fw_data = rx_data;
	assign fw_ena  = write & ~fw_full;

	// Read
	wire b_csr;
	wire b_fifo;
	assign b_rd_rst = (~wb_cyc & ~wb_we) | b_ack;

    assign b_csr = wb_addr == 2'b00;
    assign b_fifo = wb_addr == 2'b01;

	always @(posedge clk)
		if (b_rd_rst) begin
		    wb_rdata <= 32'h0;
		end else if (wb_cyc & ~wb_we & b_csr)
            wb_rdata <= { tpf_cap, 2'b00, f_lvl, 2'b00, running, run };
		else if (wb_cyc & ~wb_we & b_fifo)
		    wb_rdata <= fr_data;

	// FSM
	// ---

	// State register
	always @(posedge clk)
		if (rst)
			state <= ST_IDLE;
		else
			state <= state_nxt;

	// Next state
	always @(*)
	begin
		// Default is to stay
		state_nxt = state;

		// Transitions
		case (state)
		ST_IDLE:
			if (run)
				state_nxt = ST_RUN;

		ST_RUN:
			if (~run)
				state_nxt = ST_FLUSH;

		ST_FLUSH:
			if (fr_empty)
				state_nxt = ST_IDLE;
		endcase
	end

	// Misc
	assign running = (state == ST_RUN) | (state == ST_FLUSH);

	// Timebase
	// --------

	// Tick counter
	always @(posedge clk or posedge rst)
		if (rst)
			tick_cnt <= 0;
		else
			tick_cnt <= tick ? 10'd498 : (tick_cnt - 1);

	assign tick = tick_cnt[9];

	// Tick-per-usb frame counter
	always @(posedge clk or posedge rst)
		if (rst)
			tpf_cnt <= 16'h0000;
		else
			tpf_cnt <= tpf_cnt + tick;

	always @(posedge clk or posedge rst)
		if (rst)
			tpf_cap <= 16'h0000;
		else if (usb_sof)
			tpf_cap <= tpf_cnt;


	// FIFO
	// ----

	// Instance
	fifo_sync_ram #(
		.DEPTH(512),
		.WIDTH(32)
	) fifo_I (
		.wr_data  (fw_data),
		.wr_ena   (fw_ena),
		.wr_full  (fw_full),
		.rd_data  (fr_data),
		.rd_ena   (fr_ena),
		.rd_empty (fr_empty),
		.clk      (clk),
		.rst      (rst)
	);

	// Read
	assign fr_ena = ~fr_empty & tick; /*& running;*/

	// Level counter
	always @(posedge clk)
		if (rst)
			f_lvl <= 0;
		else
			f_lvl <= f_lvl + f_mod;

	assign f_mod = { {9{fr_ena & ~fw_ena}}, fr_ena ^ fw_ena };

    // Sine wave output to firmware
    // ----------------------------
    reg [16:0] sine [0:64];
    reg [5:0] i;
    reg [15:0] cnt;
    reg write;
    initial 
        $readmemh("sine.hex", sine);

    always @(posedge clk) begin
        if (cnt==tpf_cnt>>6) begin
            rx_data <= sine[i];
            write <= 1;
            i <= i + 6'd1;
        end else if (cnt==(tpf_cnt>>6)+1) begin
            write <= 0;
            cnt <= 0;
        end
        cnt <= cnt + 1;
    end


endmodule // audio_pcm
