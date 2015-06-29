`timescale 1ns / 1ps

// pid_controller_tf -- mba 2015
// PID controller test fixture.

// TODO
// - add PID coefficients as parameters and connect them to UUT port

module pid_controller_tf;

	// Parameters
	localparam	MASK		= 32'hffffffff;
	localparam 	N_ADC_RD	= 8;
	localparam	T_CYCLE 	= 85;
	localparam	TX_LEN	= W_ADC_DATA*N_ADC_RD/2;

	// Simulation structures
	reg signed [W_ADC_DATA-1:0] chan[0:N_ADC_RD-1];
	reg [TX_LEN-1:0] data_a_tx = 0;
	reg [TX_LEN-1:0] data_b_tx = 0;
	reg [15:0] wire_out = 0;

	// Inputs
	reg clk50_in;
	reg clk17_in;
	reg adc_busy_in;
	wire adc_data_a_in;
	wire adc_data_b_in;

	// Outputs
	wire [2:0] adc_os_out;
	wire adc_convst_out;
	wire adc_reset_out;
	wire adc_sclk_out;
	wire adc_n_cs_out;
	wire dac_nldac_out;
	wire dac_nsync_out;
	wire dac_sclk_out;
	wire dac_din_out;
	wire dac_nclr_out;
	wire [1:0] dds_sclk_out;
	wire [1:0] dds_reset_out;
	wire [1:0] dds_csb_out;
	wire [1:0] dds_sdio_out;
	wire [1:0] dds_io_update_out;
	wire obuf_en_out;

	// Frontpanel
	reg [7:0] hi_in;
	wire [1:0] hi_out;
	wire [15:0] hi_inout;
	wire hi_aa;

	// Instantiate the Unit Under Test (UUT)
	pid_controller uut (
		.clk50_in(clk50_in),
		.clk17_in(clk17_in),
		.adc_busy_in(adc_busy_in),
		.adc_data_a_in(adc_data_a_in),
		.adc_data_b_in(adc_data_b_in),
		.adc_os_out(adc_os_out),
		.adc_convst_out(adc_convst_out),
		.adc_reset_out(adc_reset_out),
		.adc_sclk_out(adc_sclk_out),
		.adc_n_cs_out(adc_n_cs_out),
		.dac_nldac_out(dac_nldac_out),
		.dac_nsync_out(dac_nsync_out),
		.dac_sclk_out(dac_sclk_out),
		.dac_din_out(dac_din_out),
		.dac_nclr_out(dac_nclr_out),
		.dds_sclk_out(dds_sclk_out),
		.dds_reset_out(dds_reset_out),
		.dds_csb_out(dds_csb_out),
		.dds_sdio_out(dds_sdio_out),
		.dds_io_update_out(dds_io_update_out),
		.obuf_en_out(obuf_en_out),
		.hi_in(hi_in),
		.hi_out(hi_out),
		.hi_inout(hi_inout),
		.hi_aa(hi_aa)
	);

	//------------------------------------------------------------------------
	// Begin okHostInterface simulation user configurable  global data
	//------------------------------------------------------------------------
	parameter BlockDelayStates = 5;   // REQUIRED: # of clocks between blocks of pipe data
	parameter ReadyCheckDelay = 5;    // REQUIRED: # of clocks before block transfer before
												 //           host interface checks for ready (0-255)
	parameter PostReadyDelay = 5;     // REQUIRED: # of clocks after ready is asserted and
												 //           check that the block transfer begins (0-255)
	parameter pipeInSize = 2048;      // REQUIRED: byte (must be even) length of default
												 //           PipeIn; Integer 0-2^32
	parameter pipeOutSize = 2048;     // REQUIRED: byte (must be even) length of default
												 //           PipeOut; Integer 0-2^32

	integer k;
	reg  [7:0]  pipeIn [0:(pipeInSize-1)];
	initial for (k=0; k<pipeInSize; k=k+1) pipeIn[k] = 8'h00;

	reg  [7:0]  pipeOut [0:(pipeOutSize-1)];
	initial for (k=0; k<pipeOutSize; k=k+1) pipeOut[k] = 8'h00;

	//------------------------------------------------------------------------
	//  Available User Task and Function Calls:
	//    FrontPanelReset;                  // Always start routine with FrontPanelReset;
	//    SetWireInValue(ep, val, mask);
	//    UpdateWireIns;
	//    UpdateWireOuts;
	//    GetWireOutValue(ep);
	//    ActivateTriggerIn(ep, bit);       // bit is an integer 0-15
	//    UpdateTriggerOuts;
	//    IsTriggered(ep, mask);            // Returns a 1 or 0
	//    WriteToPipeIn(ep, length);        // passes pipeIn array data
	//    ReadFromPipeOut(ep, length);      // passes data to pipeOut array
	//    WriteToBlockPipeIn(ep, blockSize, length);    // pass pipeIn array data; blockSize and length are integers
	//    ReadFromBlockPipeOut(ep, blockSize, length);  // pass data to pipeOut array; blockSize and length are integers
	//
	//    *Pipes operate by passing arrays of data back and forth to the user's
	//    design.  If you need multiple arrays, you can create a new procedure
	//    above and connect it to a differnet array.  More information is
	//    available in Opal Kelly documentation and online support tutorial.
	//------------------------------------------------------------------------

	// generate ~17MHz clock
	always #30 clk17_in = ~clk17_in;

	// generate 50MHz clock
	always #10 clk50_in = ~clk50_in;

	// serial data channels
	assign adc_data_a_in = data_a_tx[TX_LEN-1];
	assign adc_data_b_in = data_b_tx[TX_LEN-1];

	// misc params
	localparam REPS = 10;

	// adc param
	reg [15:0] adc_os = 0;

	// routing params
	reg [15:0] src = 0;
	reg [15:0] dest = 0;
	wire [15:0] src_map = 1 << src;
	wire [15:0] dest_map = 1 << dest;

	// pid parameters
	reg signed [15:0] setpoint = 0;
	reg signed [15:0]	p_coef = 10;
	reg signed [15:0]	i_coef = 3;
	reg signed [15:0]	d_coef = 2;

	// opp parameters
	reg signed [47:0] output_init = 13107;
	reg signed [47:0] output_min = 0;
	reg signed [47:0] output_max = 52428;
	reg signed [15:0] multiplier = 1;
	reg [15:0] right_shift = 2;

	// pid verification
	reg signed [63:0] pid_data_reg = 0;
	reg signed [63:0] error = 0;
	reg signed [63:0]	error_prev = 0;
	reg signed [63:0]	integral = 0;
	reg signed [63:0]	derivative = 0;
	reg signed [63:0]	pid_expected = 0;
	reg signed [63:0]	e_count = 0;
	reg [15:0] target = 0;
	reg lock_en = 1;
	wire [15:0] lock_en_map = (lock_en == 1) ? src_map : 0;
	integer i;

	// output verification
	reg [15:0] dac_data_reg = 0;
	initial dac_data_reg = output_init;

	// pipe verification
	reg signed [15:0] pipeOutWord;
	reg signed [15:0] wireOutValue;
	reg signed [15:0] pipe_expected[REPS-1:0];
	integer rep_count = 0;

	// output verification
	reg signed [127:0] proc_stage[5:0];

	// dac received data verification
	reg [31:0] r_instr = 0;
	wire [15:0] r_data;
	wire [3:0] r_prefix, r_control, r_address, r_feature;
	assign {r_prefix, r_control, r_address, r_data, r_feature} = r_instr;

	// adc channel assignments
	//assign chan[src] = r_data - target;
	initial chan[src] = $random;

	initial begin : main
		// Initialize Inputs
		clk50_in = 0;
		clk17_in = 0;
		adc_busy_in = 0;
		data_a_tx = 0;
		data_b_tx = 0;
		wire_out = 0;

		// Frontpanel reset
		//FrontPanelReset;

		// System reset
		$display("Resetting");
		ActivateTriggerIn(sys_gp_itep, sys_reset_offset);

		// Set ADC oversampling mode
		write_chan_data(adc_os_addr, NULL_CHAN, adc_os);

		// Set OSF ratio and activate source channel
		for (i = 0; i < N_ADC; i = i + 1) begin
			write_chan_data(osf_activate_addr, i, 1);
			write_chan_data(osf_cycle_delay_addr, i, 0);
			write_chan_data(osf_osm_addr, i, 0);
		end

		// Set channel 0 PID params
		write_chan_data(pid_setpoint_addr, src, setpoint);
		write_chan_data(pid_p_coef_addr, src, p_coef);
		write_chan_data(pid_i_coef_addr, src, i_coef);
		write_chan_data(pid_d_coef_addr, src, d_coef);

		// Route source to destination
		write_chan_data(rtr_src_sel_addr, dest, src);

		// Set channel 0 OPP params
		write_chan_data(opp_dac_min_addr, dest, output_min);
		write_chan_data(opp_dac_max_addr, dest, output_max);
		write_chan_data(opp_dac_init_addr, dest, output_init);
		write_chan_data(opp_dac_mult_addr, dest, multiplier);
		write_chan_data(opp_dac_rs_addr, dest, right_shift);

		// set focused channel
		write_chan_data(osf_pipe_chan_addr, NULL_CHAN, src);

		// trigger dac reference set
		ActivateTriggerIn(sys_gp_itep, dac_ref_set_offset);

		// activate pid lock 0
		write_chan_data(pid_lock_en_addr, src, 1);

		// trigger adc cstart
		$display("Starting ADC");
		ActivateTriggerIn(sys_gp_itep, adc_cstart_offset);

		#200;

		fork
			adc_transmit(REPS);
			wire_out_read(REPS);
			check_pid(REPS);
			print_state(REPS);
			change_target(REPS);
			check_output(REPS);
			check_rcv(REPS);
			check_pipe(REPS);
		join

		$stop;

	end

	task write_chan_data;
		input [15:0] addr;
		input [15:0] chan;
		input [47:0] data;
		begin
			SetWireInValue(data2_iwep, data[47:32], MASK);
			SetWireInValue(data1_iwep, data[31:16], MASK);
			SetWireInValue(data0_iwep, data[15:0], MASK);
			SetWireInValue(addr_iwep, addr, MASK);
			SetWireInValue(chan_iwep, chan, MASK);
			UpdateWireIns;
			ActivateTriggerIn(sys_gp_itep, reg_update_offset);
		end
	endtask

	task check_pipe;
		input [31:0] reps;

		begin
			repeat(reps) begin
				@(posedge pid_controller_tf.uut.osf_data_valid[src]) begin
					pipe_expected[rep_count] = pid_controller_tf.uut.osf_data[src][17 -: 16];
					rep_count = rep_count + 1;
				end
			end

			// read pipe data
			ReadFromPipeOut(osf_data_opep, pipeOutSize);
			for(i = 0; i < (REPS); i = i + 1) begin
				pipeOutWord = {pipeOut[i*2+1], pipeOut[i*2]};
				#1;
				$write("#%d: ", i);
				assert_equals(pipe_expected[i], pipeOutWord, "Pipe");
			end
		end
	endtask

	/* Read from opal kelly pipe */
	task wire_out_read;
		input [31:0] reps;

		// simulate pipe continuous read
		repeat(reps) begin
			@(posedge pid_controller_tf.uut.pid_data_valid[src]) begin
				UpdateWireOuts;
				wireOutValue = GetWireOutValue(osf_data_owep);
				//$display("Wire out val: %d", wireOutValue);
			end
		end
	endtask

	/* Verifiy PID values */
	task check_pid;
		input [31:0] reps;

		// check pid value
		repeat(reps) begin
			@(posedge pid_controller_tf.uut.pid_data_valid[src]) begin
				pid_data_reg = pid_controller_tf.uut.pid_data[src];
				assert_equals(pid_expected, pid_data_reg, "PID");
			end
		end
	endtask

	/* Print controller state */
	task print_state;
		input [31:0] reps;

		// print internal state
		repeat(reps) begin
			@(posedge pid_controller_tf.uut.cs_data_valid[src]) begin
				$display("ITERATION #%d", e_count);
				$display("----------------------------------");
				$display("ADC Value:\t%d", $signed(pid_controller_tf.uut.cs_data_a));
			end
			@(posedge pid_controller_tf.uut.pid_data_valid[src]) begin
				$display("PID Value:\t%d", $signed(pid_controller_tf.uut.pid_data[src]));
			end
			@(posedge pid_controller_tf.uut.opp_dac_dv[dest]) begin
				$display("OPP Value:\t%d", pid_controller_tf.uut.opp_dac_data[dest]);
				$display("Target Value:\t%d", target);
				$display("----------------------------------");
			end
		end
	endtask

	task change_target;
		input [31:0] reps;

		// change target after 15 reps
		repeat(reps) begin
			@(posedge pid_controller_tf.uut.pid_data_valid[src]) begin
				if(e_count == 15) begin
					//target = 26214;
				end
			end
		end
	endtask

	/* Verify output */
	task check_output;
		input [31:0] reps;

		repeat(reps) begin

			@(posedge pid_controller_tf.uut.opp_dac_dv[dest]) begin
				$display("***************EXP PIPE***************");
				$display("Prev: %d", dac_data_reg);
				$display("Min:  %d", output_min);
				$display("Init: %d", output_init);
				$display("Max:  %d", output_max);
				$display(proc_stage[0]);
				$display(proc_stage[1]);
				$display(proc_stage[2]);
				$display(proc_stage[3]);
				$display(proc_stage[4]);
				$display(proc_stage[5]);
				$display("**************************************");

				//$display("***************RCV PIPE***************");
				//$display("Prev: %d", pid_controller_tf.uut.dac_opp_array[dest].dac_opp.data_out_prev);
				//$display("Min:  %d", pid_controller_tf.uut.dac_opp_array[dest].dac_opp.output_min_in);
				//$display("Init: %d", pid_controller_tf.uut.dac_opp_array[dest].dac_opp.output_init_in);
				//$display("Max:  %d", pid_controller_tf.uut.dac_opp_array[dest].dac_opp.output_max_in);
				//$display(pid_controller_tf.uut.dac_opp_array[dest].dac_opp.proc_stage_0);
				//$display(pid_controller_tf.uut.dac_opp_array[dest].dac_opp.proc_stage_1);
				//$display(pid_controller_tf.uut.dac_opp_array[dest].dac_opp.proc_stage_2);
				//$display(pid_controller_tf.uut.dac_opp_array[dest].dac_opp.proc_stage_3);
				//$display(pid_controller_tf.uut.dac_opp_array[dest].dac_opp.proc_stage_4);
				//$display(pid_controller_tf.uut.dac_opp_array[dest].dac_opp.proc_stage_5);
				//$display("**************************************");
				dac_data_reg = pid_controller_tf.uut.opp_dac_data[dest];

			end

			#1 assert_equals(proc_stage[5], dac_data_reg, "Output");

			//right_shift = $random;
			//multiplier = $random;

		end
	endtask

	`include "ep_map.vh"
	`include "sim_tasks.v"
	`include "parameters.vh"
	`include "ok_sim/okHostCalls.v"
	`include "verification_tasks.v"


endmodule

