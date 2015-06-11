`timescale 1ns / 1ps

// pid_controller_tf -- mba 2015
// PID controller test fixture.

// TODO
// - add PID coefficients as parameters and connect them to UUT port

module pid_controller_tf;
	// Endpoint map
	`include "ep_map.vh"
	`include "parameters.vh"

	// Parameters
	localparam	W_DATA	= 18;
	localparam 	N_CHAN	= 8;
	localparam	T_CYCLE 	= 85;
	localparam	TX_LEN	= W_DATA*N_CHAN/2;

	// Simulation structures
	wire signed [W_DATA-1:0]	chan[0:N_CHAN-1];
	reg [TX_LEN-1:0] 	data_a_tx = 0;
	reg [TX_LEN-1:0] 	data_b_tx = 0;
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
	wire out_buf_en;

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
		.n_out_buf_en(out_buf_en),
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
	localparam REPS = 100;

	// pid parameters
	reg signed [15:0] setpoint = 0;
	reg signed [15:0]	p_coef = 0;
	reg signed [15:0]	i_coef = 0;
	reg signed [15:0]	d_coef = 0;

	// opp parameters
	reg signed [47:0] output_init = 0;
	reg signed [47:0] output_min = 0;
	reg signed [47:0] output_max = 0;
	reg signed [15:0] multiplier;
	reg signed [15:0] right_shift;

	// pid verification
	reg signed [63:0] pid_data_reg = 0;
	reg signed [63:0] error = 0;
	reg signed [63:0]	error_prev = 0;
	reg signed [63:0]	integral = 0;
	reg signed [63:0]	derivative = 0;
	reg signed [63:0]	u_expected = 0;
	reg signed [63:0]	e_count = 0;
	reg [15:0] target = 0;
	wire pid_dv = pid_controller_tf.uut.pid_data_valid[0];
	reg lock_en = 0;
	integer i;

	// output verification
	reg [15:0] dac_data_reg = 0;

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
	reg signed [15:0] chan_1_reg = 0;
	//assign chan[0] = r_data - target;
	//assign chan[0] = 655;
	assign chan[0] = chan_1_reg;
	assign chan[1] = 0;
	assign chan[2] = 0;
	assign chan[3] = 0;
	assign chan[4] = 0;
	assign chan[5] = 0;
	assign chan[6] = 0;
	assign chan[7] = 0;

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

		// System reset
		ActivateTriggerIn(sys_reset_tep, 0);

		// Set ADC oversampling mode
		SetWireInValue(adc_os_wep, 1, mask);	// os = 0

		UpdateWireIns;
		ActivateTriggerIn(module_update_tep, 0);

		// Set OSF ratio and activate channel 0
		SetWireInValue(osf_activate_wep, 1, mask); // set OSF[0] activation
		SetWireInValue(osf_cycle_delay_wep, 0, mask); // cycle delay = 0
		SetWireInValue(osf_osm_wep, 0, mask); // log ovr = 0
		SetWireInValue(osf_update_en_wep, 1, mask); // sensitize OSF channel 0

		UpdateWireIns;
		ActivateTriggerIn(module_update_tep, 0);


		// Set channel 0 PID params
		setpoint = 0;
		p_coef = 10;
		i_coef = 3;
		d_coef = 2;
		SetWireInValue(pid_setpoint_wep, setpoint, mask);	// setpoint = 3
		SetWireInValue(pid_p_coef_wep, p_coef, mask);	// p = 10
		SetWireInValue(pid_i_coef_wep, i_coef, mask);	// i = 3
		SetWireInValue(pid_d_coef_wep, d_coef, mask);	// d = 0
		SetWireInValue(pid_update_en_wep, 16'd1, mask);	// sensitize PID channel 0

		UpdateWireIns;
		ActivateTriggerIn(module_update_tep, 0);

		// Route input channel 0 to output channel 0 and activate output 0
		SetWireInValue(rtr_src_sel_wep, 16'd0, mask);	// router source
		SetWireInValue(rtr_dest_sel_wep, 16'd0, mask);	// router destination
		SetWireInValue(rtr_output_active_wep, 16'd1, mask);	// activate channel 0

		UpdateWireIns;
		ActivateTriggerIn(module_update_tep, 0);

		// Set channel 0 OPP params
		output_init = 13107;
		output_min = 1;
		output_max = 52428;
		multiplier = 1;
		right_shift = 9;

		SetWireInValue(opp_init0_wep, output_init[15:0], mask); // set output init
		SetWireInValue(opp_init1_wep, output_init[31:16], mask);
		SetWireInValue(opp_init2_wep, output_init[47:32], mask);

		SetWireInValue(opp_min0_wep, output_min[15:0], mask); // set output min
		SetWireInValue(opp_min1_wep, output_min[31:16], mask);
		SetWireInValue(opp_min2_wep, output_min[47:32], mask);

		SetWireInValue(opp_max0_wep, output_max[15:0], mask); // set output max
		SetWireInValue(opp_max1_wep, output_max[31:16], mask);
		SetWireInValue(opp_max2_wep, output_max[47:32], mask);

		SetWireInValue(opp_multiplier_wep, multiplier, mask); // set multiplier

		SetWireInValue(opp_right_shift_wep, right_shift, mask); // set right_shift

		SetWireInValue(opp_update_en_wep, 16'd1, mask);	// sensitize OPP channel 0
		UpdateWireIns;
		ActivateTriggerIn(module_update_tep, 0);

		// trigger adc reference set
		ActivateTriggerIn(dac_ref_set_tep, 0);

		// activate pid lock 0
		lock_en = 1;
		SetWireInValue(pid_lock_en_wep, lock_en, mask);
		UpdateWireIns;

		// activate adc channel 0
		SetWireInValue(osf_activate_wep, 16'd1, mask);
		UpdateWireIns;

		// trigger adc cstart
		#200;
		ActivateTriggerIn(adc_cstart_tep, 0);

		fork
			adc_transmit(REPS);
			pipe_read(REPS);
			check_pid(REPS);
			print_state(REPS);
			change_target(REPS);
			check_output(REPS);
			check_rcv(REPS);
			check_pipe(REPS);
		join

		$stop;

	end

	task check_pipe;
		input [31:0] reps;

		begin
			repeat(reps) begin
				@(posedge pid_controller_tf.uut.osf_data_valid[0]) begin
					pipe_expected[rep_count] = pid_controller_tf.uut.osf_data[0][17 -: 16];
					rep_count = rep_count + 1;
				end
			end

			// read pipe data
			ReadFromPipeOut(osf_block_data_pep, pipeOutSize);
			for(i = 0; i < (REPS); i = i + 1) begin
				pipeOutWord = {pipeOut[i*2+1], pipeOut[i*2]};
				#1;
				$write("#%d: ", i);
				assert_equals(pipe_expected[i], pipeOutWord, "Pipe");
			end
		end
	endtask

	task adc_transmit;
		input [31:0] reps;

		begin
			// adc data transmission simulation
			repeat(reps) begin
				// wait for convst_out to pulse and then assert busy
				@(posedge adc_convst_out) begin
					@(posedge clk17_in) adc_busy_in = 1;
				end

				chan_1_reg = $random;

				// simulate serial transmission from adc to fpga
				@(negedge adc_n_cs_out) begin
					data_a_tx = {chan[0], chan[1], chan[2], chan[3]};
					data_b_tx = {chan[4], chan[5], chan[6], chan[7]};
				end

				// wait one cycle before transmitting
				@(posedge clk17_in);

				// simulate serial data transmission
				repeat (71) begin
					@(negedge clk17_in)
					data_a_tx = data_a_tx << 1;
					data_b_tx = data_b_tx << 1;
				end

				// simulate conversion end
				#200;
				@(posedge clk17_in) adc_busy_in = 0;

			end
		end
	endtask

	task pipe_read;
		input [31:0] reps;

		// simulate pipe continuous read
		repeat(reps) begin
			@(posedge pid_dv) begin
				UpdateWireOuts;
				wireOutValue = GetWireOutValue(osf_data0_owep);
				//$display("Wire out val: %d", wireOutValue);
			end
		end
	endtask

	task check_pid;
		input [31:0] reps;

		// check pid value
		repeat(reps) begin
			@(posedge pid_dv) begin
				pid_data_reg = pid_controller_tf.uut.pid_data[0];
				e_count = e_count + 1;
				error = setpoint - chan[0];
				#1;
				integral = integral + error;
				derivative = error - error_prev;
				#1;
				u_expected = (p_coef * error) + (i_coef * integral) + (d_coef * derivative);
				error_prev = error;
				#1;
				assert_equals(u_expected, pid_data_reg, "PID");
			end
		end
	endtask

	task print_state;
		input [31:0] reps;

		// print internal state
		repeat(reps) begin
			@(posedge pid_controller_tf.uut.cs_data_valid[0]) begin
				$display("ITERATION #%d", e_count);
				$display("----------------------------------");
				$display("ADC Value:\t%d", $signed(pid_controller_tf.uut.cs_data_a));
			end
			@(posedge pid_controller_tf.uut.pid_data_valid[0]) begin
				$display("PID Value:\t%d", $signed(pid_controller_tf.uut.pid_data[0]));
			end
			@(posedge pid_controller_tf.uut.opp_dac_data_valid[0]) begin
				$display("OPP Value:\t%d", pid_controller_tf.uut.opp_dac_data[0]);
				$display("Target Value:\t%d", target);
				$display("----------------------------------");
			end
		end
	endtask

	task change_target;
		input [31:0] reps;

		// change target after 15 reps
		repeat(reps) begin
			@(posedge pid_dv) begin
				if(e_count == 15) begin
					//target = 26214;
				end
			end
		end
	endtask

	task check_output;
		input [31:0] reps;

		repeat(reps) begin
			// compute expected output value
			@(posedge pid_dv) begin
				#1;
				proc_stage[0] = pid_data_reg * multiplier;
				proc_stage[1] = proc_stage[0] / (2**right_shift);
				proc_stage[2] = proc_stage[1] + dac_data_reg;
				proc_stage[3] = (lock_en == 1) ? proc_stage[2] : output_init;
				proc_stage[4] = (proc_stage[3] > output_max) ? output_max : proc_stage[3];
				proc_stage[5] = (proc_stage[4] < output_min) ? output_min : proc_stage[4];
			end

			@(posedge pid_controller_tf.uut.opp_dac_data_valid[0]) begin
				dac_data_reg = pid_controller_tf.uut.opp_dac_data[0];
			end

			#1 assert_equals(proc_stage[5], dac_data_reg, "Output");

			//right_shift = $random;
			//multiplier = $random;

		end
	endtask

	task check_rcv;
		input [31:0] reps;

		repeat(reps) begin
			// simulate dac receiving data
			@(negedge dac_nsync_out) begin
				repeat(32) begin
					@(negedge dac_sclk_out) begin
						r_instr = {r_instr[30:0], dac_din_out}; // shift data in
					end
				end
			end
			#1 assert_equals(dac_data_reg, r_data, "Receive");
		end
	endtask

	task assert_equals;
		input [127:0] expected;
		input [127:0] received;
		input [20*8-1:0] test_name;

		begin

			$display("%s Test:", test_name);
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


	`include "ok_sim/okHostCalls.v"

endmodule
