//////////////////////////////////////////
// wire-in endpoints
//////////////////////////////////////////
parameter addr_iwep					= 8'h01;
parameter chan_iwep					= 8'h02;

parameter data2_iwep					= 8'h03;
parameter data1_iwep					= 8'h04;
parameter data0_iwep					= 8'h05;

//////////////////////////////////////////
// wire-in data addresses
//////////////////////////////////////////
parameter adc_os_addr				= 16'h01;

parameter chan_active_addr  		= 16'h02;
parameter chan_src_sel_addr		= 16'h03;

parameter osf_activate_addr		= 16'h05;
parameter osf_os_addr				= 16'h07;

parameter pid_setpoint_addr		= 16'h08;
parameter pid_p_coef_addr			= 16'h09;
parameter pid_i_coef_addr			= 16'h0a;
parameter pid_d_coef_addr			= 16'h0b;
parameter pid_lock_en_addr			= 16'h0c;

parameter opp_min_addr				= 16'h0d;
parameter opp_max_addr				= 16'h0e;
parameter opp_init_addr				= 16'h0f;
parameter opp_mult_addr				= 16'h10;
parameter opp_rs_addr				= 16'h11;

parameter pipe_chan_addr         = 16'h04;

//////////////////////////////////////////
// trigger-in endpoints
//////////////////////////////////////////
parameter sys_gp_itep				= 8'h40;

parameter opp_inject1_itep			= 8'h41;
parameter opp_inject0_itep			= 8'h42;

//////////////////////////////////////////
// multipurpose trigger offsets
//////////////////////////////////////////
parameter sys_rst_offset			= 0;
parameter adc_cstart_offset		= 1;
parameter write_data_offset		= 2;
parameter dac_ref_set_offset		= 3;

//////////////////////////////////////////
// wire/pipe-out endpoints
//////////////////////////////////////////
parameter osf_data0_owep			= 8'h20;
parameter osf_data_opep				= 8'ha3;

