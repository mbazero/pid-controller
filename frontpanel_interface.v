`timescale 1ns / 1ps

// frontpanel_interface -- mba13

// TODO
// - add multiplexer for osf data active
// - consolodate triggers to single end point
// - parameterize all input widths
// - implement better wireOr functionality (right now you can only read adc_data from first channel)
// - paramaterize all output widths
// - if time: consolodate smaller signals to single endpoint

module frontpanel_interface #(
	// parameters
	parameter N_ADC		= 8,									// number of active adc channels
	parameter N_OUT		= 8,									// number of output channels
	parameter W_ADC_DATA	= 18,									// width of adc channels
	parameter W_OSF_CD	= 16,									// width of osf cycle delay signal
	parameter W_OSF_OSM	= 6,									// width of oversample ratio signal
	parameter W_MLT		= 10,									// width of opp multiplier
	parameter W_EP			= 16,									// width of opal kelly endpoint
	parameter N_DAC		= 8,									// number of dac channels
	parameter W_DAC		= 16									// width of dac data channel
	)(
	// inputs <- top level entity
	input	wire										clk50_in,
	input wire										clk17_in,

	// inputs <- oversample filter
	input wire				[N_ADC-1:0]					osf_data_valid_in,
	input wire				[N_ADC*W_ADC_DATA-1:0]	osf_data_packed_in,

	// outputs -> adc controller
	output wire				[2:0]					adc_os_out,						// dm
	output wire										adc_cstart_out, 				// dm

	// outputs -> oversample filter
	output wire				[N_ADC-1:0]			osf_activate_out,				// dm
	output wire				[W_OSF_CD-1:0]		osf_cycle_delay_out, 		// dm
	output wire				[W_OSF_OSM-1:0]	osf_osm_out, 					// computed from osf_ovr
	output wire				[N_ADC-1:0]			osf_update_en_out, 			// computed on osf param change

	// outputs -> pid core
	output wire				[N_ADC-1:0]			pid_lock_en_out,
	output wire				[N_ADC-1:0]			pid_clear_out,
	output wire				[15:0]				pid_setpoint_out,				// dm
	output wire signed	[15:0]				pid_p_coef_out,				// dm
	output wire signed	[15:0]				pid_i_coef_out,				// dm
	output wire signed	[15:0]				pid_d_coef_out,				// dm
	output wire				[N_ADC-1:0]			pid_update_en_out, 			// computed on osf param change

	// outputs -> router
	output wire				[3:0]					rtr_src_sel_out,				// dm (in osf module)
	output wire				[3:0]					rtr_dest_sel_out,				// dm (in output module)
	output wire				[N_OUT-1:0]			rtr_output_active_out,		// dm

	// outputs -> output preprocessor
	output wire				[47:0]				opp_min_out,					// dm
	output wire				[47:0]				opp_max_out,					// dm
	output wire				[47:0]				opp_init_out,					// dm
	output wire				[W_MLT-1:0]			opp_multiplier_out, 			// dm
	output wire				[W_EP-1:0]			opp_right_shift_out,
	output wire				[N_OUT-1:0]			opp_update_en_out, 			// computed on opp param change

	// outputs -> dac controller
	output wire										dac_ref_set_out, 				// dm (in global module)

	// outputs -> all modules
	output wire										module_update_out,			// triggered when any parameter is updated
	output wire										sys_reset_out, 				// system reset

	// inouts <-> frontpanel host interface
	input wire				[7:0]					hi_in,
	output wire				[1:0]					hi_out,
	inout wire				[15:0]				hi_inout,
	inout wire										hi_aa,

	output wire										i2c_sda,
	output wire										i2c_scl,
	output wire										hi_muxsel
   );

//////////////////////////////////////////
// includes
//////////////////////////////////////////

`include "ep_map.vh"
`include "parameters.vh"

//////////////////////////////////////////
// local parameters
//////////////////////////////////////////

localparam N_PIPES = 2; // number of opal kelly pipes

//////////////////////////////////////////
// internal structures
//////////////////////////////////////////

/* split osf data */
wire	[W_ADC_DATA-1:0]	osf_data[N_ADC-1:0];
wire	[W_ADC_DATA-1:0]	osf_data_active;	// osf data channel active for bulk transfer mode
wire							osf_dv_active;

/* host interface */
wire 						ticlk;
wire	[30:0]			ok1;
wire	[16:0] 			ok2;
//wire	[17*(N_ADC+N_PIPES)-1:0] ok2x;	// must have space for continuous update adc registers for each channel a number of block update pipes
wire	[17*(N_ADC+1)-1:0] ok2x;	// must have space for continuous update adc registers for each channel a number of block update pipes

/* adc controller */
wire	[15:0] 		adc_os_wire;
wire	[15:0] 		adc_cstart_trig;
reg	[W_ADC_DATA-1:0]	adc_data[0:N_ADC-1];

/* oversample filter */
wire	[15:0]	osf_activate_wire;
wire	[15:0]	osf_update_en_wire;
wire	[15:0]	osf_cycle_delay_wire;
wire	[15:0]	osf_osm_wire;

/* pid core */
wire	[15:0] 	pid_clear_trig;
wire	[15:0]	pid_lock_en_wire;
wire	[15:0] 	pid_setpoint_wire;
wire	[15:0] 	pid_p_coef_wire;
wire	[15:0] 	pid_i_coef_wire;
wire	[15:0] 	pid_d_coef_wire;
wire	[15:0] 	pid_update_en_wire;

/* router */
wire	[15:0]	rtr_src_sel_wire;
wire	[15:0]	rtr_dest_sel_wire;
wire	[15:0]	rtr_output_active_wire;

/* dds preprocessor */
wire	[15:0]	opp_max_wire[0:3];
wire	[15:0]	opp_min_wire[0:3];
wire	[15:0] 	opp_init_wire[0:3];
wire	[15:0]	opp_multiplier_wire;
wire	[15:0]	opp_right_shift_wire;
wire	[15:0] 	opp_update_en_wire;

/* dac controller */
wire	[15:0]	dac_ref_set_trig;

/* all modules */
wire	[15:0] 	module_update_trig;
wire	[15:0]	sys_reset_trig;


//////////////////////////////////////////
// combinational logic
//////////////////////////////////////////

/* split osf data */
genvar a;
generate
	for ( a = 0; a < N_ADC; a = a + 1) begin : osf_data_split
		assign osf_data[a] 	= osf_data_packed_in[ a*W_ADC_DATA +: W_ADC_DATA ];
	end
endgenerate

/* multiplex active osf data channel */
assign osf_data_active		= osf_data[0];
assign osf_dv_active			= osf_data_valid_in[0];

/* adc controller */
assign adc_os_out 			= adc_os_wire[2:0];
assign adc_cstart_out		= adc_cstart_trig[0];

/* oversample filter */
assign osf_activate_out		= osf_activate_wire[N_ADC-1:0];
assign osf_cycle_delay_out	= osf_cycle_delay_wire[W_OSF_CD-1:0];
assign osf_osm_out			= osf_osm_wire[W_OSF_OSM-1:0];
assign osf_update_en_out	= osf_update_en_wire[N_ADC-1:0];

/* pid core */
assign pid_clear_out			= pid_clear_trig[N_ADC-1:0];
assign pid_lock_en_out		= pid_lock_en_wire[N_ADC-1:0];
assign pid_setpoint_out		= pid_setpoint_wire;
assign pid_p_coef_out 		= pid_p_coef_wire;
assign pid_i_coef_out 		= pid_i_coef_wire;
assign pid_d_coef_out 		= pid_d_coef_wire;
assign pid_update_en_out	= pid_update_en_wire[N_ADC-1:0];

/* router */
assign rtr_src_sel_out		= rtr_src_sel_wire[3:0];
assign rtr_dest_sel_out		= rtr_dest_sel_wire[3:0];
assign rtr_output_active_out	= rtr_output_active_wire[N_OUT-1:0];

/* output preprocessor */
assign opp_min_out			= {opp_min_wire[2], opp_min_wire[1], opp_min_wire[0]};
assign opp_max_out			= {opp_max_wire[2], opp_max_wire[1], opp_max_wire[0]};
assign opp_init_out			= {opp_init_wire[2], opp_init_wire[1], opp_init_wire[0]};
assign opp_multiplier_out	= opp_multiplier_wire[W_MLT-1:0];
assign opp_right_shift_out	= opp_right_shift_wire;
assign opp_update_en_out	= opp_update_en_wire[N_OUT-1:0];

/* dac controller */
assign dac_ref_set_out 		= dac_ref_set_trig[0];

/* all modules */
assign module_update_out	= module_update_trig[0];
assign sys_reset_out			= sys_reset_trig[0];

/* frontpanel */
assign i2c_sda   = 1'bz;
assign i2c_scl   = 1'bz;
assign hi_muxsel = 1'b0;

//////////////////////////////////////////
// modules
//////////////////////////////////////////

/* frontpanel host interface */
okHost hostIf (
	.hi_in			(hi_in),
	.hi_out			(hi_out),
	.hi_inout		(hi_inout),
	.hi_aa			(hi_aa),
	.ti_clk			(ticlk),
	.ok1				(ok1),
	.ok2				(ok2)
	);

//okWireOR # (.N(N_ADC+N_PIPES)) wireOR (.ok2(ok2), .ok2s(ok2x));
okWireOR # (.N(N_ADC+1)) wireOR (.ok2(ok2), .ok2s(ok2x));

/* oversample filter data pipe */
wire osf_pipe_read;
wire [15:0] osf_pipe_dout;

okPipeOut osf_pipe (
		.ok1			(ok1),
		.ok2			(ok2x[N_ADC*17 +: 17]),
		.ep_addr		(osf_block_data_pep),
		.ep_datain	(osf_pipe_dout),
		.ep_read		(osf_pipe_read)
		);

pipe_tx_fifo osf_pipe_fifo (
		.ti_clk_in		(ticlk),
		.sys_clk_in		(clk50_in),
		.reset_in		(sys_reset_out), // TODO add reset when active channel changes
		.data_valid_in	(osf_dv_active),
		.data_in			(osf_data_active[W_ADC_DATA-1 -: W_EP]),
		.pipe_read_in	(osf_pipe_read),
		.data_out		(osf_pipe_dout)
		);

/* dac opp data pipe */
//wire opp_pipe_read;
//wire [15:0] opp_pipe_dout;

//okPipeOut opp_pipe (
		//.ok1			(ok1),
		//.ok2			(ok2x[(N_ADC+1)*17 +: 17]),
		//.ep_addr		(8'ha4),
		//.ep_datain	(opp_pipe_dout),
		//.ep_read		(opp_pipe_read)
		//);

//pipe_tx_fifo opp_pipe_fifo (
		//.ti_clk_in		(ticlk),
		//.sys_clk_in		(clk50_in),
		//.reset_in		(sys_reset_out),
		//.data_valid_in	(opp_dac_data_valid_in[0]),	//DEBUG
		//.data_in			(opp_dac_data0_in),	//DEBUG
		//.pipe_read_in	(opp_pipe_read),
		//.data_out		(opp_pipe_dout)
		//);

/* adc continous update wire outs */
genvar j;
generate
	for ( j = 0; j < N_ADC; j = j + 1 ) begin : adc_data_owo_arr
		okWireOut adc_data_owo (
			.ok1				(ok1),
			.ok2				(ok2x[j*17 +: 17]),
			.ep_addr			(osf_data0_owep + j[7:0]),
			.ep_datain		(osf_data[j][W_ADC_DATA-1 -: W_EP])
			);
	end
endgenerate

/* adc controller */
okWireIn adc_os_owi (
	.ok1				(ok1),
	.ep_addr			(adc_os_wep),
	.ep_dataout		(adc_os_wire)
	);

okTriggerIn adc_cstart_ti (
	.ok1				(ok1),
	.ep_addr			(adc_cstart_tep),
	.ep_clk			(clk17_in),
	.ep_trigger		(adc_cstart_trig)
	);

/* oversample filter */
okWireIn osf_activate_owi (
	.ok1				(ok1),
	.ep_addr			(osf_activate_wep),
	.ep_dataout		(osf_activate_wire)
	);

okWireIn osf_cycle_delay_owi (
	.ok1				(ok1),
	.ep_addr			(osf_cycle_delay_wep),
	.ep_dataout		(osf_cycle_delay_wire)
	);

okWireIn osf_osm_owi (
	.ok1				(ok1),
	.ep_addr			(osf_osm_wep),
	.ep_dataout		(osf_osm_wire)
	);

okWireIn osf_update_en_owi (
	.ok1				(ok1),
	.ep_addr			(osf_update_en_wep),
	.ep_dataout		(osf_update_en_wire)
	);

/* pid core */
okTriggerIn pid_clear_ti (
	.ok1				(ok1),
	.ep_addr			(pid_clear_tep),
	.ep_clk			(clk50_in),
	.ep_trigger		(pid_clear_trig)
	);

okWireIn pid_lock_en_owi (
	.ok1				(ok1),
	.ep_addr			(pid_lock_en_wep),
	.ep_dataout		(pid_lock_en_wire)
	);

okWireIn pid_setpoint_owi (
	.ok1				(ok1),
	.ep_addr			(pid_setpoint_wep),
	.ep_dataout		(pid_setpoint_wire)
	);

okWireIn pid_p_coef_owi (
	.ok1				(ok1),
	.ep_addr			(pid_p_coef_wep),
	.ep_dataout		(pid_p_coef_wire)
	);

okWireIn pid_i_coef_owi (
	.ok1				(ok1),
	.ep_addr			(pid_i_coef_wep),
	.ep_dataout		(pid_i_coef_wire)
	);

okWireIn pid_d_coef_owi (
	.ok1				(ok1),
	.ep_addr			(pid_d_coef_wep),
	.ep_dataout		(pid_d_coef_wire)
	);

okWireIn pid_update_en_owi (
	.ok1				(ok1),
	.ep_addr			(pid_update_en_wep),
	.ep_dataout		(pid_update_en_wire)
	);

/* router */
okWireIn rtr_src_sel_owi (
	.ok1				(ok1),
	.ep_addr			(rtr_src_sel_wep),
	.ep_dataout		(rtr_src_sel_wire)
	);

okWireIn rtr_dest_sel_owi (
	.ok1				(ok1),
	.ep_addr			(rtr_dest_sel_wep),
	.ep_dataout		(rtr_dest_sel_wire)
	);

okWireIn rtr_output_active_owi (
	.ok1				(ok1),
	.ep_addr			(rtr_output_active_wep),
	.ep_dataout		(rtr_output_active_wire)
	);

/* output preprocessor */
okWireIn opp_init0_owi (
	.ok1				(ok1),
	.ep_addr			(opp_init0_wep),
	.ep_dataout		(opp_init_wire[0])
	);

okWireIn opp_init1_owi (
	.ok1				(ok1),
	.ep_addr			(opp_init1_wep),
	.ep_dataout		(opp_init_wire[1])
	);

okWireIn opp_init2_owi (
	.ok1				(ok1),
	.ep_addr			(opp_init2_wep),
	.ep_dataout		(opp_init_wire[2])
	);

okWireIn opp_min0_owi (
	.ok1				(ok1),
	.ep_addr			(opp_min0_wep),
	.ep_dataout		(opp_min_wire[0])
	);

okWireIn opp_min1_owi (
	.ok1				(ok1),
	.ep_addr			(opp_min1_wep),
	.ep_dataout		(opp_min_wire[1])
	);

okWireIn opp_min2_owi (
	.ok1				(ok1),
	.ep_addr			(opp_min2_wep),
	.ep_dataout		(opp_min_wire[2])
	);

okWireIn opp_max0_owi (
	.ok1				(ok1),
	.ep_addr			(opp_max0_wep),
	.ep_dataout		(opp_max_wire[0])
	);

okWireIn opp_max1_owi (
	.ok1				(ok1),
	.ep_addr			(opp_max1_wep),
	.ep_dataout		(opp_max_wire[1])
	);

okWireIn opp_max2_owi (
	.ok1				(ok1),
	.ep_addr			(opp_max2_wep),
	.ep_dataout		(opp_max_wire[2])
	);

okWireIn opp_multiplier_owi (
	.ok1				(ok1),
	.ep_addr			(opp_multiplier_wep),
	.ep_dataout		(opp_multiplier_wire)
	);

okWireIn opp_right_shift_owi (
	.ok1				(ok1),
	.ep_addr			(opp_right_shift_wep),
	.ep_dataout		(opp_right_shift_wire)
	);

okWireIn opp_update_en_owi (
	.ok1				(ok1),
	.ep_addr			(opp_update_en_wep),
	.ep_dataout		(opp_update_en_wire)
	);

/* dac controller */
okTriggerIn dac_ref_set_ti (
	.ok1				(ok1),
	.ep_addr			(dac_ref_set_tep),
	.ep_clk			(clk50_in),
	.ep_trigger		(dac_ref_set_trig)
	);

/* all modules */
okTriggerIn module_update_ti (
	.ok1				(ok1),
	.ep_addr			(module_update_tep),
	.ep_clk			(clk50_in),
	.ep_trigger		(module_update_trig)
	);

okTriggerIn sys_reset_ti (
	.ok1				(ok1),
	.ep_addr			(sys_reset_tep),
	.ep_clk			(clk17_in),
	.ep_trigger		(sys_reset_trig)
	);

endmodule
