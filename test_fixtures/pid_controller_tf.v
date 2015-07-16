`timescale 1ns / 1ps

// pid_controller_tf -- mba 2015
// PID controller test fixture.

// TODO
// - add PID coefficients as parameters and connect them to UUT port

module pid_controller_tf;

    // Parameters
    localparam  MASK = 32'hffffffff;
    localparam  N_CHAN = N_PID_CHAN;
    localparam  N_ADC_RD = 8;
    localparam  T_CYCLE = 85;
    localparam  TX_LEN = W_ADC_DATA*N_ADC_RD/2;

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
    wire [N_DDS-1:0] dds_sclk_out;
    wire [N_DDS-1:0] dds_reset_out;
    wire [N_DDS-1:0] dds_csb_out;
    wire [N_DDS-1:0] dds_sdio_out;
    wire [N_DDS-1:0] dds_io_update_out;
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

    //////////////////////////////////////////
    // Sim Params
    //////////////////////////////////////////

    // simulation reps
    localparam REPS = 1055;

    // adc params
    reg [15:0] adc_os = 0;

    // channel params
    localparam NAC = 1; // number of active channels
    reg [15:0] chan_focus;

    // routing params
    reg[15:0] src[0:NAC-1];
    reg[15:0] dest[0:NAC-1];

    // ovr params
    reg [15:0] os[0:NAC-1];

    // pid parameters
    reg signed [15:0] setpoint[0:NAC-1];
    reg signed [15:0] p_coef[0:NAC-1];
    reg signed [15:0] i_coef[0:NAC-1];
    reg signed [15:0] d_coef[0:NAC-1];
    reg inv_error[0:NAC-1];

    // opp parameters
    reg signed [47:0] output_init[0:NAC-1];
    reg signed [47:0] output_min[0:NAC-1];
    reg signed [47:0] output_max[0:NAC-1];
    reg signed [15:0] multiplier[0:NAC-1];
    reg [15:0] right_shift[0:NAC-1];
    reg [15:0] add_chan[0:NAC-1];

    //////////////////////////////////////////
    // Verification Params
    //////////////////////////////////////////

    // pid verification
    reg signed [63:0] pid_exp[0:NAC-1];
    reg signed [63:0] pid_rcv[0:NAC-1];
    reg signed [63:0] error[0:NAC-1];
    reg signed [63:0] error_prev[0:NAC-1];
    reg signed [63:0] integral[0:NAC-1];
    reg signed [63:0] derivative[0:NAC-1];
    reg signed [63:0] e_count[0:NAC-1];
    reg [15:0] target[0:NAC-1];
    integer pc_count;
    integer dv_count = 0;
    integer pc_chan = 0;
    integer ac_count = 0;

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

    // adc verification
    integer ac = 0;

    // opp verification
    reg signed [127:0] opt_exp[0:NAC-1];
    reg signed [127:0] opt_mtrs[0:NAC-1];
    reg [15:0] opt_rcv[0:NAC-1];
    integer oc_count = 0;
    integer out_count = 0;
    integer oc_chan = 0;

    // dac received data verification
    reg [31:0] r_instr = 0;
    wire [15:0] r_data;
    wire [3:0] r_prefix, r_control, r_address, r_feature;
    integer rc = 0;
    assign {r_prefix, r_control, r_address, r_data, r_feature} = r_instr;

    // wire-out verification
    reg signed [15:0] wire_out_exp[0:NAC-1];
    reg signed [15:0] wire_out_rcv;
    integer cwo_chan;
    integer wc = 0;

    // pipe verification
    reg signed [15:0] pipeOutWord;
    reg signed [15:0] pipe_expected[REPS-1:0];
    integer rep_count = 0;
    integer ppc = 0;

    // adc channel assignments
    //assign adc_val[src] = r_data - target;
    integer x;
    integer pcx = 0;
    integer opx = 0;
    integer ldx = 0;
    integer cdx = 0;

    initial begin : main
        // Initialize Inputs
        clk50_in = 0;
        clk17_in = 0;
        adc_busy_in = 0;
        data_a_tx = 0;
        data_b_tx = 0;
        wire_out = 0;

        //-----------------------------------------
        // Set Parameters
        //-----------------------------------------
        for ( x = 0; x < NAC; x = x + 1 ) begin
            // Routing
            src[x] = $unsigned($random) % N_ADC;
            dest[x] = 8;

            // Oversample
            os[x] = 0;

            // PID
            setpoint[x] = 0;
            p_coef[x] = $random % 1000;
            i_coef[x] = $random % 1000;
            d_coef[x] = $random % 1000;
            inv_error[x] = 0;

            // OPT
            output_init[x] = 332;
            output_min[x] = 0;
            output_max[x] = 512;

            multiplier[x] = $random % 1000;
            right_shift[x] = 0;

            add_chan[x] = N_CHAN + 1;

            $display("--------------------------------------");
            $display("Chan %d config:", x);
            $display("--------------------------------------");
            $display("src = %d", src[x]);
            $display("dst = %d", dest[x]);
            $display("os = %d", os[x]);
            $display("");
            $display("setpoint = %d", setpoint[x]);
            $display("p_coef = %d", p_coef[x]);
            $display("i_coef = %d", i_coef[x]);
            $display("d_coef = %d", d_coef[x]);
            $display("inv_error = %d", inv_error[x]);
            $display("");
            $display("multiplier = %d", multiplier[x]);
            $display("right shift = %d", right_shift[x]);
            $display("add chan = %d", add_chan[x]);
            $display("--------------------------------------");
        end
        //-----------------------------------------


        // Initialize simulation structures
        for (x = 0; x < NAC; x = x+1) begin
            opt_exp[x] = 0;
            opt_mtrs[x] = 0;
            opt_rcv[x] = output_init[x];
        end

        // Frontpanel reset
        FrontPanelReset;

        // Configure channels
        configure_chans();

        // Set focus
        chan_focus = $unsigned($random) % NAC;
        write_data(pipe_cset_rqst, dest[chan_focus], 0);

        // Reset system
        ActivateTriggerIn(sys_gp_itep, sys_rst_offset);

        // Trigger dac reference set
        ActivateTriggerIn(sys_gp_itep, dac_rset_offset);

        // Set adc oversample mode and trigger cstart
        write_data(adc_os_addr, 0, adc_os);
        ActivateTriggerIn(sys_gp_itep, adc_cstart_offset);

        // inject write instruction and verify
        for (x = 0; x < NAC; x = x+1) begin
            write_data(opt_inj_rqst, dest[x], 0);
            opt_rcv[x] = output_init[x];

            $display("type: %s", get_dest_type(x));
            if (get_dest_type(x) == "DAC") begin
                check_dac_rcv(1);
            end else if (get_dest_type(x) == "FREQ") begin
                check_freq_rcv(x, 1);
            end else if (get_dest_type(x) == "PHASE") begin
                check_phase_rcv(x, 1);
            end else begin
                check_amp_rcv(x, 1);
            end
        end

        #200;

        fork
            adc_transmit(REPS);
            check_pid(NAC * REPS);
            check_opp(NAC * REPS);
            //check_dac_rcv(nac_of_type("DAC") * REPS);
            //check_amp_rcv(0, REPS);
            //log_data(NAC * REPS);
            //check_data_log(REPS);
        join

        // Clear oversample memory and verify
        for (x = 0; x < NAC; x=x+1) begin
            write_data(ovr_clr_rqst, dest[x], 0);
            assert_equals(0, pid_controller_tf.uut.pid_pipe.ovr.sum_mem[dest[x]], "OVR Clear: sum_mem", x);
            assert_equals(0, pid_controller_tf.uut.pid_pipe.ovr.count_mem[dest[x]], "OVR Clear: count_mem", x);
        end

        // Clear pid memory and verify
        for (x = 0; x < NAC; x=x+1) begin
            write_data(pid_clr_rqst, dest[x], 0);
            assert_equals(0, pid_controller_tf.uut.pid_pipe.pid.error_prev1_mem[dest[x]], "PID Clear: error_prev1", x);
            assert_equals(0, pid_controller_tf.uut.pid_pipe.pid.error_prev2_mem[dest[x]], "PID Clear: error_prev2", x);
            assert_equals(0, pid_controller_tf.uut.pid_pipe.pid.dout_prev_mem[dest[x]], "PID Clear: dout_prev", x);
        end

        // Clear output memory and verify
        for (x = 0; x < NAC; x=x+1) begin
            write_data(opt_clr_rqst, dest[x], 0);
            assert_equals(0, pid_controller_tf.uut.pid_pipe.opt.dmtrs_prev_mem[dest[x]], "OVR Clear: dmtrs_prev", x);
            assert_equals(output_init[x], pid_controller_tf.uut.pid_pipe.opt.dout_prev_mem[dest[x]], "OVR Clear: dout_prev", x);
        end

        $display("SIMULATION SUCCESSFUL.");
        $stop;

    end

    /* configure channel */
    task configure_chans;
        begin : cchan
            integer c;
            for (c = 0; c < NAC; c = c+1) begin
                // Route and activate channel
                write_data(chan_src_sel_addr, dest[c], src[c]);
                write_data(pid_lock_en_addr, dest[c], 1);

                // Set OSF ratio and activate source channel
                write_data(ovr_os_addr, dest[c], os[c]);

                // Set PID params
                write_data(pid_setpoint_addr, dest[c], setpoint[c]);
                write_data(pid_p_coef_addr, dest[c], p_coef[c]);
                write_data(pid_i_coef_addr, dest[c], i_coef[c]);
                write_data(pid_d_coef_addr, dest[c], d_coef[c]);
                write_data(pid_inv_error_addr, dest[c], inv_error[c]);

                // Set OPP params
                write_data(opt_min_addr, dest[c], output_min[c]);
                write_data(opt_max_addr, dest[c], output_max[c]);
                write_data(opt_init_addr, dest[c], output_init[c]);
                write_data(opt_mult_addr, dest[c], multiplier[c]);
                write_data(opt_rs_addr, dest[c], right_shift[c]);
                write_data(opt_add_chan_addr, dest[c], add_chan[c]);
            end
        end
    endtask

    /* write data to PID controller */
    task write_data;
        input [15:0] addr;
        input [15:0] chan;
        input [63:0] data;
        begin
            SetWireInValue(addr_iwep, addr, MASK);
            SetWireInValue(chan_iwep, chan, MASK);
            SetWireInValue(data3_iwep, data[63:48], MASK);
            SetWireInValue(data2_iwep, data[47:32], MASK);
            SetWireInValue(data1_iwep, data[31:16], MASK);
            SetWireInValue(data0_iwep, data[15:0], MASK);
            UpdateWireIns;
            ActivateTriggerIn(sys_gp_itep, wr_en_offset);
        end
    endtask

    /* Transmit random data words to the ADC controller */
    task adc_transmit;
        input [31:0] reps;

        begin : adc_tx
            integer adc_count = 0, j = 0;

            repeat(reps) begin
                // wait for convst_out to pulse and then assert busy
                @(posedge adc_convst_out) begin
                    @(posedge clk17_in) adc_busy_in = 1;
                end

                for ( j = 0; j < N_ADC; j = j+1 ) begin
                    adc_val[j] = 5555;
                end

                // simulate serial transmission from adc to fpga
                @(negedge adc_n_cs_out) begin
                    data_a_tx = {adc_val[0], adc_val[1], adc_val[2], adc_val[3]};
                    data_b_tx = {adc_val[4], adc_val[5], adc_val[6], adc_val[7]};
                end

                // wait one cycle before transmitting
                @(posedge clk17_in);

                $display("======================================");
                $display("Iteration #%d", adc_count);
                $display("======================================");

                for ( ac = 0; ac < NAC; ac = ac + 1 ) begin
                    $display("ADC %d val: %d", src[ac], adc_val[src[ac]]);
                end

                // simulate serial data transmission
                repeat (71) begin
                    @(negedge clk17_in)
                    data_a_tx = data_a_tx << 1;
                    data_b_tx = data_b_tx << 1;
                end

                // simulate conversion end
                #2000;
                @(posedge clk17_in) adc_busy_in = 0;

                adc_count = adc_count + 1;

            end
        end
    endtask

    /* Verify PID values */
    task check_pid;
        input [31:0] reps;

        begin
        for(pcx = 0; pcx < reps;) begin
            @(posedge clk50_in) begin
                if (pid_controller_tf.uut.pid_pipe.pid_dv) begin
                    pc_chan = out_to_chan(pid_controller_tf.uut.pid_pipe.pid_chan);
                    pid_rcv[pc_chan] = pid_controller_tf.uut.pid_pipe.pid_data;

                    // compute expected PID value
                    e_count[pc_chan] = e_count[pc_chan] + 1;
                    error[pc_chan] = setpoint[pc_chan] - adc_val[src[pc_chan]];
                    #1 error[pc_chan] = (inv_error[pc_chan]) ? -error[pc_chan] : error[pc_chan];
                    #1 integral[pc_chan] = integral[pc_chan] + error[pc_chan];
                    derivative[pc_chan] = error[pc_chan] - error_prev[pc_chan];
                    #1 pid_exp[pc_chan] = (p_coef[pc_chan] * error[pc_chan]) + (i_coef[pc_chan] * integral[pc_chan]) + (d_coef[pc_chan] * derivative[pc_chan]);
                    error_prev[pc_chan] = error[pc_chan];

                    // compare with received value
                    #1 assert_equals(pid_exp[pc_chan], pid_rcv[pc_chan], "PID", pc_chan);

                    pcx = pcx + 1;
                end
            end
        end
    end
    endtask

    /* Verify output */
    task check_opp;
        input [31:0] reps;

        for(opx = 0; opx < reps;) begin
            @(posedge clk50_in) begin
                if (pid_controller_tf.uut.pid_pipe.opt_dv) begin
                    oc_chan = out_to_chan(pid_controller_tf.uut.pid_pipe.opt_chan);
                    opt_rcv[oc_chan] = pid_controller_tf.uut.pid_pipe.opt_data;

                    // compute expected output value
                    $display("pid_exp[%d] : %d", oc_chan, pid_exp[oc_chan]);
                    #1 opt_exp[oc_chan] = pid_exp[oc_chan] * multiplier[oc_chan];
                    #1 $display(opt_exp[oc_chan]);
                    #1 opt_exp[oc_chan] = opt_exp[oc_chan] >>> right_shift[oc_chan];
                    #1 opt_mtrs[oc_chan] = opt_exp[oc_chan];
                    #1 $display(opt_exp[oc_chan]);
                    #1;
                    if (add_chan[oc_chan] < N_CHAN) begin
                        opt_exp[oc_chan] = opt_exp[oc_chan] + opt_mtrs[out_to_chan(add_chan[oc_chan])];
                    end
                    #1 opt_exp[oc_chan] = opt_exp[oc_chan] + opt_rcv[oc_chan];
                    #1 $display(opt_exp[oc_chan]);
                    #1 opt_exp[oc_chan] = (opt_exp[oc_chan] > output_max[oc_chan]) ? output_max[oc_chan] : opt_exp[oc_chan];
                    #1 $display(opt_exp[oc_chan]);
                    #1 opt_exp[oc_chan] = (opt_exp[oc_chan] < output_min[oc_chan]) ? output_min[oc_chan] : opt_exp[oc_chan];

                    // compare with received value
                    #1 assert_equals(opt_exp[oc_chan], opt_rcv[oc_chan], "OPP", oc_chan);

                    opx = opx + 1;
                end
            end
        end
    endtask

    /* Verify dac received data */
    task check_dac_rcv;
        input [31:0] reps;
        integer sim_chan;

        repeat(reps) begin
            // simulate dac receiving data
            @(negedge dac_nsync_out) begin
                repeat(32) begin
                    @(negedge dac_sclk_out) begin
                        r_instr = {r_instr[30:0], dac_din_out}; // shift data in
                    end
                end
            end

            // check recevied data
            #1 sim_chan = out_to_chan(r_address);
            #1 assert_equals(opt_rcv[sim_chan], r_data, "RCV", sim_chan);
        end
    endtask

    /* Verify dds frequency received data */
    task check_freq_rcv;
        input [15:0] chan;
        input [31:0] reps;
        reg [63:0] freq_instr;
        reg [47:0] freq_data;
        integer dds_chan;

        repeat(reps) begin
            dds_chan = (dest[chan] - N_DAC) / 3;
            @(negedge dds_csb_out[dds_chan]) begin
                #1;
                repeat(64) begin
                    @(posedge dds_sclk_out[dds_chan]) begin
                        freq_instr = {freq_instr[62:0], dds_sdio_out[dds_chan]};
                    end
                end
            end
            #1 freq_data = freq_instr[47:0];
            #1 assert_equals(opt_rcv[chan], freq_data, "FREQ RCV", chan);
        end
    endtask

    /* Verify dds phase received data */
    task check_phase_rcv;
        input [15:0] chan;
        input [31:0] reps;
        reg [31:0] phase_instr;
        reg [13:0] phase_data;
        integer dds_chan;

        repeat(reps) begin
            dds_chan = (dest[chan] - N_DAC) / 3;
            @(negedge dds_csb_out[dds_chan]) begin
                repeat(32) begin
                    @(posedge dds_sclk_out[dds_chan]) begin
                        phase_instr = {phase_instr[30:0], dds_sdio_out[dds_chan]};
                    end
                end
            end
            #1 phase_data = phase_instr[13:0];
            #1 assert_equals(opt_rcv[chan], phase_data, "PHASE RCV", chan);
        end
    endtask

    /* Verify dds amp received data */
    task check_amp_rcv;
        input [15:0] chan;
        input [31:0] reps;
        reg [31:0] amp_instr;
        reg [9:0] amp_data;
        integer dds_chan;

        repeat(reps) begin
            dds_chan = (dest[chan] - N_DAC) / 3;
            @(negedge dds_csb_out[dds_chan]) begin
                repeat(32) begin
                    @(posedge dds_sclk_out[dds_chan]) begin
                        amp_instr = {amp_instr[30:0], dds_sdio_out[dds_chan]};
                    end
                end
            end
            #1 amp_data = amp_instr[9:0];
            #1 assert_equals(opt_rcv[chan], amp_data, "PHASE RCV", chan);
        end
    endtask


    /* Simulation data logging */
    task log_data;
        input [31:0] reps;

        for(ldx = 0; ldx < reps;) begin
            @(posedge clk50_in) begin
                if(pid_controller_tf.uut.pid_pipe.ovr_dv) begin
                    // Log for wire outs
                    wire_out_exp[out_to_chan(pid_controller_tf.uut.pid_pipe.ovr_chan)] =
                        pid_controller_tf.uut.pid_pipe.ovr_data[17:2];

                    // Log for pipe
                    if(pid_controller_tf.uut.pid_pipe.ovr_chan == dest[chan_focus]) begin
                        pipe_expected[rep_count] = pid_controller_tf.uut.pid_pipe.ovr_data[17 -: 16];
                        rep_count = rep_count + 1;
                    end

                    ldx = ldx + 1;
                end
            end
        end
    endtask

    /* Verify wire-out and pipe values */
    task check_data_log;
        input [31:0] reps;

        begin
            // check wires
            repeat(reps) begin
                @(negedge adc_busy_in) begin
                    UpdateWireOuts;
                    for(cdx = 0; cdx < NAC; cdx = cdx + 1) begin
                        wire_out_rcv = GetWireOutValue(data_log0_owep + dest[cdx]);
                        assert_equals(wire_out_exp[cdx], wire_out_rcv, "Wire-out", cdx);
                    end
                end
            end

            // check pipe
            ReadFromPipeOut(data_log_opep, pipeOutSize);
            for(ppc = 0; ppc < REPS; ppc = ppc + 1) begin
                pipeOutWord = {pipeOut[ppc*2+1], pipeOut[ppc*2]};
                #1;
                $write("#%d: ", ppc);
                assert_equals(pipe_expected[ppc], pipeOutWord, "Pipe", chan_focus);
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

    function [15:0] out_to_chan;
        input [15:0] out;
        integer x = 0;
        reg hit = 0;
        reg [15:0] chan;
        begin
            for (x = 0; x < NAC; x = x+1) begin
                if (dest[x] == out) begin
                    chan = x;
                    hit = 1;
                end
            end
            out_to_chan = (hit) ? chan : 9;
        end
    endfunction

    function [5*8-1:0] get_dest_type;
        input [15:0] chan;
        reg [15:0] dst;
        reg [5*8-1:0] temp;
        begin
            dst = dest[chan];
            if (dst < FREQ0_ADDR) begin
                temp = "DAC";
            end else if (dst < PHASE0_ADDR) begin
                temp = "FREQ";
            end else if (dst < AMP0_ADDR) begin
                temp = "PHASE";
            end else begin
                temp = "AMP";
            end
            get_dest_type = temp;
        end
    endfunction


    function [15:0] nac_of_type;
        input [5*8-1:0] type;
        reg [15:0] num = 0;
        integer x = 0;
        begin
            num = 0;
            for (x = 0; x < NAC; x = x+1) begin
                if (get_dest_type(x) == type) begin
                    num = num + 1;
                end
            end
            nac_of_type = num;
        end
    endfunction


    `include "../ep_map.vh"
    `include "../parameters.vh"
    `include "assert_equals.v"
    `include "../ok_sim/okHostCalls.v"

endmodule

