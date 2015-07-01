//////////////////////////////////////////
// wire-in endpoints
//////////////////////////////////////////
parameter data2_iwep					= 8'h01;
parameter data1_iwep					= 8'h02;
parameter data0_iwep					= 8'h03;

parameter addr_iwep					= 8'h04;
parameter ochan_iwep					= 8'h05;

//////////////////////////////////////////
// wire-in data addresses
//////////////////////////////////////////
parameter adc_os_addr				= 16'h01;

parameter osf_activate_addr		= 16'h02;
parameter osf_cycle_delay_addr	= 16'h03;
parameter osf_osm_addr				= 16'h04;

parameter pid_lock_en_addr			= 16'h05;
parameter pid_setpoint_addr		= 16'h06;
parameter pid_p_coef_addr			= 16'h07;
parameter pid_i_coef_addr			= 16'h08;
parameter pid_d_coef_addr			= 16'h09;
parameter pid_update_en_addr		= 16'h0a;

parameter ochan_src_sel_addr		= 16'h0b;

parameter opp_min_addr				= 16'h0c;
parameter opp_max_addr				= 16'h0d;
parameter opp_init_addr				= 16'h0e;
parameter opp_mult_addr				= 16'h0f;
parameter opp_rs_addr				= 16'h10;

parameter focused_chan_addr		= 16'h11;

//////////////////////////////////////////
// trigger-in endpoints
//////////////////////////////////////////
parameter sys_gp_itep				= 8'h40;

parameter opp_dac_inj_itep			= 8'h42;
parameter opp_freq_inj_itep		= 8'h43;
parameter opp_phase_inj_itep		= 8'h44;
parameter opp_amp_inj_itep			= 8'h45;

//////////////////////////////////////////
// multipurpose trigger offsets
//////////////////////////////////////////
parameter sys_reset_offset			= 0;
parameter adc_cstart_offset		= 1;
parameter write_data_offset		= 2;
parameter dac_ref_set_offset		= 3;

//////////////////////////////////////////
// wire/pipe-out endpoints
//////////////////////////////////////////
parameter osf_data_owep				= 8'h20;
parameter osf_data_opep				= 8'ha3;
