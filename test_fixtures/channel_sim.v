task configure_chan;
	input integer reps;
	input [15:0] src;
	input [15:0] dest;
	input lock_en;
	input signed [15:0] setpoint;
	input signed [15:0] p_coef;
	input signed [15:0] i_coef;
	input signed [15:0] d_coef;
	input signed [47:0] output_init;
	input signed [47:0] output_min;
	input signed [47:0] output_max;
	input signed [15:0] multiplier;
	input [15:0] right_shift;
	input [8*5-1:0] dest_type;
	input focused;
	begin
		/**********************CONFIG***************************/

		// Set OSF ratio and activate source channel
		write_chan_data(osf_activate_addr, src, 1);
		write_chan_data(osf_cycle_delay_addr, src, 0);
		write_chan_data(osf_osm_addr, src, 0);

		// Set PID params
		write_chan_data(pid_setpoint_addr, src, setpoint);
		write_chan_data(pid_p_coef_addr, src, p_coef);
		write_chan_data(pid_i_coef_addr, src, i_coef);
		write_chan_data(pid_d_coef_addr, src, d_coef);

		// Route source to destination
		if (dest_type == "DAC") begin
			$display("Routing DAC");
			write_chan_data(rtr_src_sel_addr, dest, src);
		end else if (dest_type == "FREQ") begin
			$display("Routing FREQ");
			write_chan_data(rtr_src_sel_addr, dest, src);
		end else if (dest_type == "PHASE") begin
			write_chan_data(rtr_phase_src_addr, dest, src);
			$display("Routing PHASE");
		end else if (dest_type == "AMP") begin
			$display("Routing AMP");
			write_chan_data(rtr_amp_src_addr, dest, src);
		end

		// Set OPP params
		write_chan_data(opp_dac_min_addr, dest, output_min);
		write_chan_data(opp_dac_max_addr, dest, output_max);
		write_chan_data(opp_dac_init_addr, dest, output_init);
		write_chan_data(opp_dac_mult_addr, dest, multiplier);
		write_chan_data(opp_dac_rs_addr, dest, right_shift);

		// Set focused
		if (focused == 1) begin
			write_chan_data(osf_pipe_chan_addr, NULL_CHAN, src);
		end

		// Activate PID lock
		write_chan_data(pid_lock_en_addr, src, lock_en);

	end
endtask

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

