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
	reg signed [W_ADC_DATA-1:0] adc_val[0:N_ADC_RD-1];
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

	// simulation reps
	localparam REPS = 100;

	// adc params
	reg [15:0] adc_os = 0;

	//////////////////////////////////////////
	// Sim Params (change these)
	//////////////////////////////////////////

	// channel params
	localparam NAC = 2;
	reg [15:0] focused_chan = 1;
	reg [15:0] chan_no = 0;

	// pid parameters
	reg signed [15:0] setpoint[0:NAC-1];
	reg signed [15:0]	p_coef[0:NAC-1];
	reg signed [15:0]	i_coef[0:NAC-1];
	reg signed [15:0]	d_coef[0:NAC-1];
	reg lock_en[0:NAC-1];

	initial begin : set_pid_params
		chan_no = 0;
		setpoint[chan_no] = 0;
		p_coef[chan_no] = 10;
		i_coef[chan_no] = 3;
		d_coef[chan_no] = 2;
		lock_en[chan_no] = 1;

		chan_no = 1;
		setpoint[chan_no] = 0;
		p_coef[chan_no] = 10;
		i_coef[chan_no] = 3;
		d_coef[chan_no] = 2;
		lock_en[chan_no] = 1;
	end

	// routing params
	reg[15:0] src[0:NAC-1];
	reg[15:0] dest[0:NAC-1];

	initial begin : set_routing
		chan_no = 0;
		src[chan_no] = 0;
		dest[chan_no] = 0;

		chan_no = 1;
		src[chan_no] = 1;
		dest[chan_no] = 1;
	end

	// opp parameters
	reg signed [47:0] output_init[0:NAC-1];
	reg signed [47:0] output_min[0:NAC-1];
	reg signed [47:0] output_max[0:NAC-1];
	reg signed [15:0] multiplier[0:NAC-1];
	reg [15:0] right_shift[0:NAC-1];
	reg [5*8-1:0] dest_type[0:NAC-1];
	reg focused[0:NAC-1];

	initial begin : set_opp_params
		chan_no = 0;
		output_init[chan_no] = 13107;
		output_min[chan_no] = 0;
		output_max[chan_no] = 52428;
		multiplier[chan_no] = 1;
		right_shift[chan_no] = 2;
		dest_type[chan_no] = "DAC";
		focused[chan_no] = (focused_chan == chan_no) ? 1 : 0;

		chan_no = 1;
		output_init[chan_no] = 20000;
		output_min[chan_no] = 0;
		output_max[chan_no] = 52428;
		multiplier[chan_no] = 1;
		right_shift[chan_no] = 10;
		dest_type[chan_no] = "DAC";
		focused[chan_no] = (focused_chan == chan_no) ? 1 : 0;
	end

	//////////////////////////////////////////
	// Verification Params
	//////////////////////////////////////////

	// pid verification
	reg signed [63:0]	pid_exp[0:NAC-1];
	reg signed [63:0] pid_rcv[0:NAC-1];
	reg signed [63:0] error[0:NAC-1];
	reg signed [63:0]	error_prev[0:NAC-1];
	reg signed [63:0]	integral[0:NAC-1];
	reg signed [63:0]	derivative[0:NAC-1];
	reg signed [63:0]	e_count[0:NAC-1];
	reg [15:0] target[0:NAC-1];
	integer pc_count;
	integer adc_count = 0;
	integer pc_chan = 0;
	integer src_count = 0;

	initial begin
		for (pc_chan = 0; pc_chan < NAC; pc_chan = pc_chan+1) begin
			pid_exp[pc_chan] = 0;
			pid_rcv[pc_chan] = 0;
			error[pc_chan] = 0;
			error_prev[pc_chan] = 0;
			integral[pc_chan] = 0;
			derivative[pc_chan] = 0;
			e_count[pc_chan] = 0;
			target[pc_chan] = 0;
		end
	end

	// opp verification
	reg signed [127:0] opp_exp[0:NAC-1];
	reg [15:0] opp_rcv[0:NAC-1];
	integer oc_count = 0;
	integer dac_count = 0;
	integer oc_chan = 0;

	initial begin
		for (oc_chan = 0; oc_chan < NAC; oc_chan = oc_chan+1) begin
			opp_exp[oc_chan] = 0;
			opp_rcv[oc_chan] = output_init[oc_chan];
		end
	end

	// dac received data verification
	reg [31:0] r_instr = 0;
	wire [15:0] r_data;
	wire [3:0] r_prefix, r_control, r_address, r_feature;
	integer rc = 0;
	assign {r_prefix, r_control, r_address, r_data, r_feature} = r_instr;

	// wire-out verification
	reg signed [15:0] wire_out_exp;
	reg signed [15:0] wire_out_rcv;
	integer wc = 0;

	// pipe verification
	reg signed [15:0] pipeOutWord;
	reg signed [15:0] pipe_expected[REPS-1:0];
	integer rep_count = 0;
	integer ppc = 0;

	// adc channel assignments
	//assign adc_val[src] = r_data - target;

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
		configure_chans();

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
			check_pid(REPS);
			check_opp(REPS);
			check_dac_rcv(REPS);
			//check_wire_out(REPS);
			check_pipe(REPS);

			//print_state(REPS);
		join

		$display("SIMULATION SUCCESSFUL.");
		$stop;

	end

	/* Verify PID values */
	task check_pid;
		input [31:0] reps;

		repeat(reps) begin

			pc_count = 0;
			while (pc_count < NAC) begin

				@(posedge |pid_controller_tf.uut.pid_data_valid) begin

					for (adc_count = 0; adc_count < N_ADC; adc_count = adc_count + 1) begin
						if (pid_controller_tf.uut.pid_data_valid[adc_count] == 1) begin

							for (src_count = 0; src_count < NAC; src_count = src_count+1) begin
								if (src[src_count] == adc_count) begin
									pc_chan = src_count;
									pc_count = pc_count + 1;

									// compute expected PID value
									e_count[pc_chan] = e_count[pc_chan] + 1;
									error[pc_chan] = setpoint[pc_chan] - adc_val[src[pc_chan]];
									#1 integral[pc_chan] = integral[pc_chan] + error[pc_chan];
									derivative[pc_chan] = error[pc_chan] - error_prev[pc_chan];
									#1 pid_exp[pc_chan] = (p_coef[pc_chan] * error[pc_chan]) + (i_coef[pc_chan] * integral[pc_chan]) + (d_coef[pc_chan] * derivative[pc_chan]);
									#1 pid_exp[pc_chan] = (lock_en[pc_chan]) ? pid_exp[pc_chan] : 0;
									error_prev[pc_chan] = error[pc_chan];
									// compare with received value
									pid_rcv[pc_chan] = pid_controller_tf.uut.pid_data[src[pc_chan]];
									#1 assert_equals(pid_exp[pc_chan], pid_rcv[pc_chan], "PID", pc_chan);
								end
							end
						end
					end

				end

			end

		end
	endtask

	/* Verify output */
	task check_opp;
		input [31:0] reps;

		repeat(reps) begin

			oc_count = 0;
			while (oc_count < NAC) begin

				@(posedge |pid_controller_tf.uut.opp_dac_data_valid) begin

					for (dac_count = 0; dac_count < N_DAC; dac_count = dac_count + 1) begin
						if (pid_controller_tf.uut.opp_dac_data_valid[dac_count] == 1) begin
							oc_chan = dac_to_chan(dac_count);
							oc_count = oc_count + 1;

							// compute expected output value
							$display("pid_exp[%d] : %d", oc_chan, pid_exp[oc_chan]);
							#1 opp_exp[oc_chan] = pid_exp[oc_chan] * multiplier[oc_chan];
							#1 $display(opp_exp[oc_chan]);
							#1 opp_exp[oc_chan] = opp_exp[oc_chan] >>> right_shift[oc_chan];
							#1 $display(opp_exp[oc_chan]);
							#1 opp_exp[oc_chan] = opp_exp[oc_chan] + opp_rcv[oc_chan];
							#1 $display(opp_exp[oc_chan]);
							#1 opp_exp[oc_chan] = (lock_en[oc_chan] == 1) ? opp_exp[oc_chan] : output_init[oc_chan];
							#1 $display(opp_exp[oc_chan]);
							#1 opp_exp[oc_chan] = (opp_exp[oc_chan] > output_max[oc_chan]) ? output_max[oc_chan] : opp_exp[oc_chan];
							#1 $display(opp_exp[oc_chan]);
							#1 opp_exp[oc_chan] = (opp_exp[oc_chan] < output_min[oc_chan]) ? output_min[oc_chan] : opp_exp[oc_chan];
							// compare with received value
							opp_rcv[oc_chan] = pid_controller_tf.uut.opp_dac_data[dest[oc_chan]];
							#1 assert_equals(opp_exp[oc_chan], opp_rcv[oc_chan], "OPP", oc_chan);
						end
					end

				end

			end

		end
	endtask

	/* Verify received data */
	task check_dac_rcv;
		input [31:0] reps;
		integer sim_chan;
		integer n_active_dacs;

		begin
			n_active_dacs = nac_type("DAC");

			repeat(reps * n_active_dacs) begin
				// simulate dac receiving data
				@(negedge dac_nsync_out) begin
					repeat(32) begin
						@(negedge dac_sclk_out) begin
							r_instr = {r_instr[30:0], dac_din_out}; // shift data in
						end
					end
				end

				// check recevied data
				#1 sim_chan = dac_to_chan(r_address);
				#1 assert_equals(opp_rcv[sim_chan], r_data, "RCV", sim_chan);
			end
		end
	endtask

	task check_pipe;
		input [31:0] reps;

		begin
			repeat(reps) begin
				@(posedge pid_controller_tf.uut.osf_data_valid[src[focused_chan]]) begin
					pipe_expected[rep_count] = pid_controller_tf.uut.osf_data[src[focused_chan]][17 -: 16];
					rep_count = rep_count + 1;
				end
			end

			// read pipe data
			ReadFromPipeOut(osf_data_opep, pipeOutSize);
			for(ppc = 0; ppc < (REPS); ppc = ppc + 1) begin
				pipeOutWord = {pipeOut[ppc*2+1], pipeOut[ppc*2]};
				#1;
				$write("#%d: ", ppc);
				assert_equals(pipe_expected[ppc], pipeOutWord, "Pipe", focused_chan);
			end
		end
	endtask

	/* Verify wire-out values */
	task check_wire_out;
		input [31:0] reps;
		input [15:0] wc;

		repeat(reps) begin
			@(posedge pid_controller_tf.uut.osf_data_valid[src[wc]]) begin
				wire_out_exp = pid_controller_tf.uut.osf_data[src[wc]][17:2];
			end
			@(posedge pid_controller_tf.uut.pid_data_valid[src[wc]]) begin
				UpdateWireOuts;
				wire_out_rcv = GetWireOutValue(osf_data_owep);
				//assert_equals(wire_out_exp, wire_out_rcv, "Wire-out");
			end
		end
	endtask


	/* Print controller state */
	task print_state;
		input [31:0] reps;
		input [15:0] chan;

		// print internal state
		repeat(reps) begin
			@(posedge pid_controller_tf.uut.cs_data_valid[src[chan]]) begin
				$display(">>> ADC Value:\t%d <<<", $signed(pid_controller_tf.uut.cs_data_a));
			end
			@(posedge pid_controller_tf.uut.osf_data_valid[src[chan]]) begin
				$display(">>> OSF Value:\t%d <<<", $signed(pid_controller_tf.uut.osf_data[src[chan]]));
			end
			@(posedge pid_controller_tf.uut.pid_data_valid[src[chan]]) begin
				$display(">>> PID Value:\t%d <<<", $signed(pid_controller_tf.uut.pid_data[src[chan]]));
			end
			@(posedge pid_controller_tf.uut.opp_dac_data_valid[dest[chan]]) begin
				$display(">>> OPP Value:\t%d <<<", pid_controller_tf.uut.opp_dac_data[dest[chan]]);
				//$display("Target Value:\t%d", target);
			end
		end
	endtask

	task assert_equals;
		input [127:0] expected;
		input [127:0] received;
		input [20*8-1:0] test_name;
		input [15:0] chan_no;

		begin

			$display("%s [%d] Test:", test_name, chan_no);
			$display("Expected: %d", $signed(expected));
			$display("Received: %d", $signed(received));

			if(expected == received) begin
				$display("Success");
			end else begin
				$display("Failure");
				$stop;
			end
		end
	endtask

	function [15:0] adc_to_chan;
		input [15:0] adc;
		integer x = 0;
		reg hit = 0;
		reg [15:0] chan;
		begin
			for (x = 0; x < NAC; x = x+1) begin
				if (src[x] == adc) begin
					chan = x;
					hit = 1;
				end
			end
			adc_to_chan = (hit) ? chan : 9;
		end
	endfunction

	function [15:0] dac_to_chan;
		input [15:0] dac;
		integer x = 0;
		reg hit = 0;
		reg [15:0] chan;
		begin
			for (x = 0; x < NAC; x = x+1) begin
				if (dest_type[x] == "DAC" & dest[x] == dac) begin
					chan = x;
					hit = 1;
				end
			end
			dac_to_chan = (hit) ? chan : 9;
		end
	endfunction

	function [15:0] nac_type;
		input [5*8-1:0] type;
		reg [15:0] num = 0;
		integer x = 0;
		begin
			for (x = 0; x < NAC; x = x+1) begin
				if (dest_type[x] == type) begin
					num = num + 1;
				end
			end
			nac_type = num;
		end
	endfunction


	`include "ep_map.vh"
	`include "adc_transmit.v"
	`include "parameters.vh"
	`include "ok_sim/okHostCalls.v"
	`include "channel_sim.v"

endmodule

