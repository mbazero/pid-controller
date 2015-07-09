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
    reg sys_clk_in;
    reg adc_clk_in;
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
        .sys_clk_in(sys_clk_in),
        .adc_clk_in(adc_clk_in),
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
    always #30 adc_clk_in = ~adc_clk_in;

    // generate 50MHz clock
    always #10 sys_clk_in = ~sys_clk_in;

    // serial data channels
    assign adc_data_a_in = data_a_tx[TX_LEN-1];
    assign adc_data_b_in = data_b_tx[TX_LEN-1];

    //////////////////////////////////////////
    // Sim Params
    //////////////////////////////////////////

    // simulation reps
    localparam REPS = 100;

    // adc params
    reg [15:0] adc_os = 0;

    // channel params
    localparam NAC = 2; // number of active channels
    reg [15:0] chan_focus = N_CHAN + 1;

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
    reg [5*8-1:0] dest_type[0:NAC-1];

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

    initial begin : main
        // Initialize Inputs
        sys_clk_in = 0;
        adc_clk_in = 0;
        adc_busy_in = 0;
        data_a_tx = 0;
        data_b_tx = 0;
        wire_out = 0;

        //-----------------------------------------
        // Set Parameters
        //-----------------------------------------
        for ( x = 0; x < NAC; x = x + 1 ) begin
            // Routing
            src[x] = $random % N_ADC;
            dest[x] = x;

            // Oversample
            os[x] = 0;

            // PID
            setpoint[x] = 0;
            p_coef[x] = $random % 1000;
            i_coef[x] = $random % 1000;
            d_coef[x] = $random % 1000;
            inv_error[x] = 0;

            // OPT
            output_init[x] = 30000;
            output_min[x] = 0;
            output_max[x] = 65535;

            multiplier[x] = $random % 1000;
            right_shift[x] = 0;

            add_chan[x] = N_CHAN + 1;
            dest_type[x] = "DAC";

            $display("--------------------------------------");
            $display("Chan %d config:", x);
            $display("--------------------------------------");
            $display("src = %d", src[x]);
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
        chan_focus = dest[$random % NAC];
        write_data(pipe_chan_addr, chan_focus, 1);

        // Reset system
        ActivateTriggerIn(sys_gp_itep, sys_rst_offset);

        // Trigger dac reference set
        ActivateTriggerIn(sys_gp_itep, dac_rset_offset);

        // Set adc oversample mode and trigger cstart
        write_data(adc_os_addr, 0, adc_os);
        ActivateTriggerIn(sys_gp_itep, adc_cstart_offset);

        #200;

        fork
            adc_transmit(REPS);
            check_pid(REPS);
            check_opp(REPS);
            check_dac_rcv(REPS);
            log_data(REPS);
            check_data_log(REPS);

        join

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
                write_data(chan_en_addr, dest[c], 1);

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
                    @(posedge adc_clk_in) adc_busy_in = 1;
                end

                for ( j = 0; j < N_ADC; j = j+1 ) begin
                    adc_val[j] = $random % 1000;
                end

                // simulate serial transmission from adc to fpga
                @(negedge adc_n_cs_out) begin
                    data_a_tx = {adc_val[0], adc_val[1], adc_val[2], adc_val[3]};
                    data_b_tx = {adc_val[4], adc_val[5], adc_val[6], adc_val[7]};
                end

                // wait one cycle before transmitting
                @(posedge adc_clk_in);

                $display("======================================");
                $display("Iteration #%d", adc_count);
                $display("======================================");

                for ( ac = 0; ac < NAC; ac = ac + 1 ) begin
                    $display("ADC %d val: %d", src[ac], adc_val[src[ac]]);
                end

                // simulate serial data transmission
                repeat (71) begin
                    @(negedge adc_clk_in)
                    data_a_tx = data_a_tx << 1;
                    data_b_tx = data_b_tx << 1;
                end

                // simulate conversion end
                #2000;
                @(posedge adc_clk_in) adc_busy_in = 0;

                adc_count = adc_count + 1;

            end
        end
    endtask

    /* Verify PID values */
    task check_pid;
        input [31:0] reps;

        repeat(reps * NAC) begin
            @(posedge pid_controller_tf.uut.pid_pipe.dv_pid) begin
                pc_chan = out_to_chan(pid_controller_tf.uut.pid_pipe.chan_pid);

                // compute expected PID value
                e_count[pc_chan] = e_count[pc_chan] + 1;
                error[pc_chan] = setpoint[pc_chan] - adc_val[src[pc_chan]];
                #1 error[pc_chan] = (inv_error[pc_chan]) ? -error[pc_chan] : error[pc_chan];
                #1 integral[pc_chan] = integral[pc_chan] + error[pc_chan];
                derivative[pc_chan] = error[pc_chan] - error_prev[pc_chan];
                #1 pid_exp[pc_chan] = (p_coef[pc_chan] * error[pc_chan]) + (i_coef[pc_chan] * integral[pc_chan]) + (d_coef[pc_chan] * derivative[pc_chan]);
                error_prev[pc_chan] = error[pc_chan];

                // compare with received value
                pid_rcv[pc_chan] = pid_controller_tf.uut.pid_pipe.data_pid;
                #1 assert_equals(pid_exp[pc_chan], pid_rcv[pc_chan], "PID", pid_controller_tf.uut.pid_pipe.chan_pid);
            end
        end
    endtask

    /* Verify output */
    task check_opp;
        input [31:0] reps;

        repeat (reps * NAC) begin
            @(posedge pid_controller_tf.uut.pid_pipe.dv_opt) begin
                oc_chan = out_to_chan(pid_controller_tf.uut.pid_pipe.chan_opt);

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
                opt_rcv[oc_chan] = pid_controller_tf.uut.pid_pipe.data_opt;
                #1 assert_equals(opt_exp[oc_chan], opt_rcv[oc_chan], "OPP", oc_chan);
            end
        end
    endtask

    /* Verify received data */
    task check_dac_rcv;
        input [31:0] reps;
        integer sim_chan;
        integer n_active_dacs;

        begin
            n_active_dacs = nac_of_type("DAC");

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
                #1 sim_chan = out_to_chan(r_address);
                #1 assert_equals(opt_rcv[sim_chan], r_data, "RCV", sim_chan);
            end
        end
    endtask

    /* Simulation data logging */
    task log_data;
        input [31:0] reps;

        repeat(reps * NAC) begin
            @(posedge pid_controller_tf.uut.pid_pipe.dv_ovr) begin
                // Log for wire outs
                wire_out_exp[out_to_chan(pid_controller_tf.uut.pid_pipe.chan_ovr)] =
                    pid_controller_tf.uut.pid_pipe.data_ovr[17:2];

                // Log for pipe
                if(pid_controller_tf.uut.pid_pipe.chan_ovr == dest[chan_focus]) begin
                    pipe_expected[rep_count] = pid_controller_tf.uut.pid_pipe.data_ovr[17 -: 16];
                    rep_count = rep_count + 1;
                end
            end
        end
    endtask

    /* Verify wire-out and pipe values */
    task check_data_log;
        input [31:0] reps;

        begin
            // check wires
            repeat(reps * NAC) begin
                @(posedge pid_controller_tf.uut.pid_pipe.dv_pid) begin
                    cwo_chan = out_to_chan(pid_controller_tf.uut.pid_pipe.chan_pid);
                    UpdateWireOuts;
                    wire_out_rcv = GetWireOutValue(data_log0_owep + dest[cwo_chan]);
                    assert_equals(wire_out_exp[cwo_chan], wire_out_rcv, "Wire-out", cwo_chan);
                end
            end

            // check pipe
            ReadFromPipeOut(data_log_opep, pipeOutSize);
            for(ppc = 0; ppc < (REPS); ppc = ppc + 1) begin
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

    function [15:0] nac_of_type;
        input [5*8-1:0] type;
        reg [15:0] num = 0;
        integer x = 0;
        begin
            for (x = 0; x < NAC; x = x+1) begin
                if (dest_type[x] == type) begin
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

