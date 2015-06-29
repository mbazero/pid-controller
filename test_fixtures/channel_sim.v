task configure_chans;
	begin : cchan
		integer c;
		for (c = 0; c < NAC; c = c+1) begin
			// Set OSF ratio and activate source channel
			write_chan_data(osf_activate_addr, src[c], 1);
			write_chan_data(osf_cycle_delay_addr, src[c], 0);
			write_chan_data(osf_osm_addr, src[c], 0);

			// Set PID params
			write_chan_data(pid_setpoint_addr, src[c], setpoint[c]);
			write_chan_data(pid_p_coef_addr, src[c], p_coef[c]);
			write_chan_data(pid_i_coef_addr, src[c], i_coef[c]);
			write_chan_data(pid_d_coef_addr, src[c], d_coef[c]);

			// Route source to destination
			if (dest_type[c] == "DAC") begin
				$display("Routing DAC");
				write_chan_data(rtr_src_sel_addr, dest[c], src[c]);
			end else if (dest_type[c] == "FREQ") begin
				$display("Routing FREQ");
				write_chan_data(rtr_src_sel_addr, dest[c], src[c]);
			end else if (dest_type[c] == "PHASE") begin
				write_chan_data(rtr_phase_src_addr, dest[c], src[c]);
				$display("Routing PHASE");
			end else if (dest_type[c] == "AMP") begin
				$display("Routing AMP");
				write_chan_data(rtr_amp_src_addr, dest[c], src[c]);
			end

			// Set OPP params
			write_chan_data(opp_dac_min_addr, dest[c], output_min[c]);
			write_chan_data(opp_dac_max_addr, dest[c], output_max[c]);
			write_chan_data(opp_dac_init_addr, dest[c], output_init[c]);
			write_chan_data(opp_dac_mult_addr, dest[c], multiplier[c]);
			write_chan_data(opp_dac_rs_addr, dest[c], right_shift[c]);

			// Set focused
			if (focused[c] == 1) begin
				write_chan_data(osf_pipe_chan_addr, NULL_CHAN, src[c]);
			end

			// Activate PID lock
			write_chan_data(pid_lock_en_addr, src[c], lock_en[c]);
		end
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

