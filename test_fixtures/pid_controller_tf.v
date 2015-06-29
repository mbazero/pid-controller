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

	// sim params
	localparam REPS = 10;
	localparam N_ACT = 1;

	// adc params
	reg [15:0] adc_os = 0;

	// routing params
	localparam src = 0;
	localparam dest = 0;

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
	reg signed [63:0]	pid_exp = 0;
	reg signed [63:0] pid_rcv = 0;
	reg signed [63:0] error = 0;
	reg signed [63:0]	error_prev = 0;
	reg signed [63:0]	integral = 0;
	reg signed [63:0]	derivative = 0;
	reg signed [63:0]	e_count = 0;
	reg [15:0] target = 0;
	reg lock_en = 1;
	integer i;

	// output verification
	reg [15:0] dac_data_reg = 0;
	initial dac_data_reg = output_init;

	// wire-out verification
	reg signed [15:0] wire_out_exp;
	reg signed [15:0] wire_out_rcv;

	// pipe verification
	reg signed [15:0] pipeOutWord;
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
		FrontPanelReset;

		// Configure channels
		configure_chan(10, 0, 0, 1, 0, 10, 3, 2, 13107, 0, 52428, 1, 2, "DAC", 1);

		// Reset system
		ActivateTriggerIn(sys_gp_itep, sys_reset_offset);

		// Trigger dac reference set
		ActivateTriggerIn(sys_gp_itep, dac_ref_set_offset);

		// Set adc oversample mode and trigger cstart
		write_chan_data(adc_os_addr, NULL_CHAN, adc_os);
		ActivateTriggerIn(sys_gp_itep, adc_cstart_offset);

		#200;

		fork
			adc_transmit(REPS);
			check_wire_out(REPS);
			check_pid(REPS);
			change_target(REPS);
			check_output(REPS);
			check_rcv(REPS);
			check_pipe(REPS);
			print_state(REPS);
		join

		$stop;

	end

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

	/* Verify wire-out values */
	task check_wire_out;
		input [31:0] reps;

		repeat(reps) begin
			@(posedge pid_controller_tf.uut.osf_data_valid[src]) begin
				wire_out_exp = pid_controller_tf.uut.osf_data[src][17:2];
			end
			@(posedge pid_controller_tf.uut.pid_data_valid[src]) begin
				UpdateWireOuts;
				wire_out_rcv = GetWireOutValue(osf_data_owep);
				//assert_equals(wire_out_exp, wire_out_rcv, "Wire-out");
			end
		end
	endtask

	/* Verify PID values */
	task check_pid;
		input [31:0] reps;

		repeat(reps) begin
			@(posedge pid_controller_tf.uut.pid_data_valid[src]) begin
				pid_rcv = pid_controller_tf.uut.pid_data[src];
				assert_equals(pid_exp, pid_rcv, "PID");
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

	/* Print controller state */
	task print_state;
		input [31:0] reps;

		// print internal state
		repeat(reps) begin
			@(posedge pid_controller_tf.uut.cs_data_valid[src]) begin
				$display(">>> ADC Value:\t%d <<<", $signed(pid_controller_tf.uut.cs_data_a));
			end
			@(posedge pid_controller_tf.uut.osf_data_valid[src]) begin
				$display(">>> OSF Value:\t%d <<<", $signed(pid_controller_tf.uut.osf_data[src]));
			end
			@(posedge pid_controller_tf.uut.pid_data_valid[src]) begin
				$display(">>> PID Value:\t%d <<<", $signed(pid_controller_tf.uut.pid_data[src]));
			end
			@(posedge pid_controller_tf.uut.opp_dac_data_valid[dest]) begin
				$display(">>> OPP Value:\t%d <<<", pid_controller_tf.uut.opp_dac_data[dest]);
				//$display("Target Value:\t%d", target);
			end
		end
	endtask

	/* Verify output */
	task check_output;
		input [31:0] reps;

		repeat(reps) begin

			@(posedge pid_controller_tf.uut.opp_dac_data_valid[dest]) begin
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

				$display("***************RCV PIPE***************");
				$display("Prev: %d", pid_controller_tf.uut.dac_opp_array[dest].dac_opp.data_out_prev);
				$display("Min:  %d", pid_controller_tf.uut.dac_opp_array[dest].dac_opp.output_min_in);
				$display("Init: %d", pid_controller_tf.uut.dac_opp_array[dest].dac_opp.output_init_in);
				$display("Max:  %d", pid_controller_tf.uut.dac_opp_array[dest].dac_opp.output_max_in);
				$display(pid_controller_tf.uut.dac_opp_array[dest].dac_opp.proc_stage_0);
				$display(pid_controller_tf.uut.dac_opp_array[dest].dac_opp.proc_stage_1);
				$display(pid_controller_tf.uut.dac_opp_array[dest].dac_opp.proc_stage_2);
				$display(pid_controller_tf.uut.dac_opp_array[dest].dac_opp.proc_stage_3);
				$display(pid_controller_tf.uut.dac_opp_array[dest].dac_opp.proc_stage_4);
				$display(pid_controller_tf.uut.dac_opp_array[dest].dac_opp.proc_stage_5);
				$display("**************************************");
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
	`include "channel_sim.v"

endmodule

