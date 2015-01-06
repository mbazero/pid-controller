`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   14:32:40 12/15/2014
// Design Name:   pid_controller
// Module Name:   Y:/Documents/MIST/pid_controller/tle_adc_controller_tf.v
// Project Name:  pid_controller
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: pid_controller
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module tle_adc_controller_tf;
	// Parameters
	localparam	W_DATA	= 18;
	localparam 	N_CHAN	= 6;
	localparam	T_CYCLE 	= 85;
	localparam	TX_LEN	= W_DATA*N_CHAN/2;

	// Simulation structures
	reg [W_DATA-1:0]	chan[0:N_CHAN-1]; 
	reg [TX_LEN-1:0] 	data_a_tx;
	reg [TX_LEN-1:0] 	data_b_tx; 
	reg [15:0] wire_out; 

	// Inputs
	reg clk50_in;
	reg clk17_in;
	reg adc_busy_in;
	wire adc_data_a_in;
	wire adc_data_b_in;
	reg [7:0] hi_in;

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
	wire [1:0] hi_out;
	wire adc_busy_db;
	wire adc_data_a_db;
	wire adc_data_b_db;
	wire [2:0] adc_os_db;
	wire adc_convst_db;
	wire adc_reset_db;
	wire adc_sclk_db;
	wire adc_n_cs_db;
	wire [6:0] adc_data_a_vect_db;
	wire adc_data_valid_db;
	wire adc_cstart_db;
	wire mod_update_db;
	wire clk17_db;

	// Bidirs
	wire [15:0] hi_inout;

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
		.adc_busy_db(adc_busy_db), 
		.adc_data_a_db(adc_data_a_db), 
		.adc_data_b_db(adc_data_b_db), 
		.adc_os_db(adc_os_db), 
		.adc_convst_db(adc_convst_db), 
		.adc_reset_db(adc_reset_db), 
		.adc_sclk_db(adc_sclk_db), 
		.adc_n_cs_db(adc_n_cs_db), 
		.adc_data_a_vect_db(adc_data_a_vect_db), 
		.adc_data_valid_db(adc_data_valid_db), 
		.adc_cstart_db(adc_cstart_db), 
		.mod_update_db(mod_update_db), 
		.clk17_db(clk17_db)
	);

	//------------------------------------------------------------------------
	// Begin okHostInterface simulation user configurable  global data
	//------------------------------------------------------------------------
	parameter BlockDelayStates = 5;   // REQUIRED: # of clocks between blocks of pipe data
	parameter ReadyCheckDelay = 5;    // REQUIRED: # of clocks before block transfer before
												 //           host interface checks for ready (0-255)
	parameter PostReadyDelay = 5;     // REQUIRED: # of clocks after ready is asserted and
												 //           check that the block transfer begins (0-255)
	parameter pipeInSize = 1024;      // REQUIRED: byte (must be even) length of default
												 //           PipeIn; Integer 0-2^32
	parameter pipeOutSize = 1024;     // REQUIRED: byte (must be even) length of default
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
	always #30 clk17_in = !clk17_in; 
	
	// generate 50MHz clock
	always #10 clk50_in = !clk50_in; 

	// serial data channels
	assign adc_data_a_in = data_a_tx[TX_LEN-1]; 
	assign adc_data_b_in = data_b_tx[TX_LEN-1];
	
	// ok local params
	localparam	mask = 32'hffffffff,
				mod_update_ep = 8'h57;
	
	initial begin
		// Initialize Inputs
		clk50_in = 0;
		clk17_in = 0;
		adc_busy_in = 0;
		wire_out = 0;
		
		// Issue frontpanel reset
		FrontPanelReset; 
		
		// set test channel values
		chan[0] = 1111;
		chan[1] = 2222; 
		chan[2] = 3333;
		chan[3] = 4444;
		chan[4] = 5555;
		chan[5] = 6666; 

		#100;
		
		// System reset
		ActivateTriggerIn(8'h58, 0);
		
		// Set ADC oversampling mode
		SetWireInValue(8'h01, 16'd0, mask);	// os = 0
		
		UpdateWireIns;
		ActivateTriggerIn(mod_update_ep, 0);

		// Set OSF ratio and activate channel 0
		SetWireInValue(8'h15, 16'd1, mask); // set OSF[0] activation 
		SetWireInValue(8'h02, 16'd0, mask); // cycle delay = 0
		SetWireInValue(8'h03, 16'd0, mask); // log ovr = 0
		SetWireInValue(8'h04, 16'd1, mask); // sensitize OSF channel 0

		UpdateWireIns;
		ActivateTriggerIn(mod_update_ep, 0);


		// Set channel 0 PID params
		SetWireInValue(8'h04, 16'd3, 16'hffff);	// setpoint = 3
		SetWireInValue(8'h05, 16'd10, 16'hffff);	// p = 10
		SetWireInValue(8'h06, 16'd3, 16'hffff);	// i = 3
		SetWireInValue(8'h07, 16'd0, 16'hffff);	// d = 0
		SetWireInValue(8'h08, 16'd1, 16'hffff);	// sensitize PID channel 0

		UpdateWireIns;
		ActivateTriggerIn(mod_update_ep, 0);

		// Set channel 5 PID params
//		SetWireInValue(8'h02, 16'd0, 16'hffff);	// setpoint = 0
//		SetWireInValue(8'h03, 16'd1, 16'hffff);	// p = 1
//		SetWireInValue(8'h04, 16'd2, 16'hffff);	// i = 2
//		SetWireInValue(8'h05, 16'd3, 16'hffff);	// d = 3
//		SetWireInValue(8'h06, 16'h20, 16'hffff);	// sensitize PID channel 5
//
//		UpdateWireIns;
//		ActivateTriggerIn(8'h54, 0);

		// Route input channel 0 to output channel 0
		SetWireInValue(8'h09, 16'd0, 16'hffff);	// router source
		SetWireInValue(8'h0a, 16'd0, 16'hffff);	// router destination

		UpdateWireIns;
		ActivateTriggerIn(mod_update_ep, 0);

		// Route input channel 5 to output channel 0 
//		SetWireInValue(8'h0b, 16'd5, 16'hffff);	// router source
//		SetWireInValue(8'h0c, 16'd0, 16'hffff);	// router destination
//
//		UpdateWireIns;
//		ActivateTriggerIn(8'h54, 0);

		// Set channel 0 OPP params
		SetWireInValue(8'h0b, 16'd500, 16'hffff);	// opp init p1 = 0 
		SetWireInValue(8'h0c, 16'd0, 16'hffff); 	// opp init p2 = 0 
		SetWireInValue(8'h0d, 16'd500, 16'hffff);	// opp init p3 = 500
		SetWireInValue(8'h14, 16'd1, 16'hffff);	// sensitize OPP channel 0

		UpdateWireIns;
		ActivateTriggerIn(mod_update_ep, 0);

		// Set channel 1 OPP params
//		SetWireInValue(8'h07, 16'd100, 16'hffff);	// opp init p3 = 100
//		SetWireInValue(8'h0a, 16'd2, 16'hffff);	// sensitize OPP channel 1
//
//		UpdateWireIns;
//		ActivateTriggerIn(8'h54, 0);
		
		// trigger adc cstart
		ActivateTriggerIn(8'h53, 0);

		// transission simulation
		repeat(4) begin
			chan[0] = {$random} % 15; 
		
			// wait for convst_out to pulse and then assert busy
			@(posedge adc_convst_out) begin
				@(posedge clk17_in) adc_busy_in = 1; 
			end

			// simulate serial transmission from adc to fpga
			@(negedge adc_n_cs_out) begin 
				data_a_tx = {chan[0], chan[1], chan[2]};
				data_b_tx = {chan[3], chan[4], chan[5]}; 
			end
			
			// wait one cycle before transmitting 
			@(posedge clk17_in); 
	
			// simulate serial data transmission
			repeat (53) begin
				@(negedge clk17_in) 
					data_a_tx = data_a_tx << 1; 
					data_b_tx = data_b_tx << 1; 
			end
			
			// simulate data read
			UpdateWireOuts();
			wire_out = GetWireOutValue(8'd32);
			
			//$display("Read Value: %d", test);
			
			// simulate conversion end
			#200;
			@(posedge clk17_in) adc_busy_in = 0; 
		end
		
		$stop;

	end
	
	`include "./oksim/okHostCalls.v"
      
endmodule

