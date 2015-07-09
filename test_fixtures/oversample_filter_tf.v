`timescale 1ns / 1ps

// oversample_filter_tf

module oversample_filter_tf;

    // Parameters
    localparam W_CHAN = 5;
    localparam N_CHAN = 4;
    localparam W_DATA = 18;
    localparam W_SUM = 20;
    localparam W_OS = 3;
    localparam W_WR_ADDR = W_EP;
    localparam W_WR_CHAN = W_EP;
    localparam W_WR_DATA = W_EP*4;

    localparam W_EP = 16;

    // Inputs
    reg clk_in;
    reg rst_in;

    reg dv_in;
    reg [W_CHAN-1:0] chan_in;
    reg signed [W_DATA-1:0] data_in;

    reg wr_en;
    reg [W_WR_ADDR-1:0] wr_addr;
    reg [W_WR_CHAN-1:0] wr_chan;
    reg [W_WR_DATA-1:0] wr_data;

    // Outputs
    wire dv_out;
    wire [W_CHAN-1:0] chan_out;
    wire signed [W_DATA-1:0] data_out;

    oversample_filter #(
        .W_CHAN(W_CHAN),
        .N_CHAN(N_CHAN),
        .W_DATA(W_DATA),
        .W_SUM(W_SUM),
        .W_OS(W_OS),
        .W_WR_ADDR(W_WR_ADDR),
        .W_WR_CHAN(W_WR_CHAN),
        .W_WR_DATA(W_WR_DATA))
    uut (
        .clk_in(clk_in),
        .rst_in(rst_in),
        .dv_in(dv_in),
        .chan_in(chan_in),
        .data_in(data_in),
        .wr_en(wr_en),
        .wr_addr(wr_addr),
        .wr_chan(wr_chan),
        .wr_data(wr_data),
        .dv_out(dv_out),
        .chan_out(chan_out),
        .data_out(data_out)
    );

    // simulation structures
    localparam signed [W_SUM-1:0] MAX_SUM = {W_SUM{1'b1}} >> 1;
    localparam signed [W_SUM-1:0] MIN_SUM = ~MAX_SUM;

    reg signed [W_DATA-1:0] data_rcv = 0;
    reg signed [W_DATA-1:0] data_exp = 0;
    integer delta = 0;

    integer num_samples[N_CHAN-1:0];
    reg signed [127:0] sum[N_CHAN-1:0];
    reg [W_OS-1:0] os[N_CHAN-1:0];

    integer i;
    initial begin
        for ( i = 0; i < N_CHAN; i = i + 1 ) begin
            num_samples[i] = 0;
            sum[i] = 0;
            os[i] = 0;
        end
    end

    // misc structures
    integer chan_int = 0;
    integer max_samples = 0;
    integer total_samples = 0;
    integer scount;
    integer cc1, cc2;

    // generate 50MHz clock
    always #10 clk_in = ~clk_in;

    initial begin
        clk_in = 0;
        rst_in = 0;
        dv_in = 0;
        chan_in = 0;
        data_in = 0;
        wr_en = 0;
        wr_addr = 0;
        wr_chan = -1;
        wr_data = 0;

        #100;

        repeat(50) begin
            scount = 0;
            cc1 = 0;
            cc2 = 0;
            total_samples = 0;

            // assign random oversample modes to each channel
            repeat(N_CHAN) begin
                os[cc1] = $random % (2**W_OS - 1);
                wr_addr = ovr_os_addr;
                wr_chan = cc1;
                wr_data = os[cc1];

                num_samples[cc1] = 2**os[cc1];

                if(num_samples[cc1] > max_samples) begin
                    max_samples = num_samples[cc1];
                end

                total_samples = total_samples + num_samples[cc1];

                @(posedge clk_in) wr_en = 1;
                @(posedge clk_in) wr_en = 0;

                $display("######################################");
                $display("CHAN %d OS = %d", cc1, os[cc1]);
                $display("######################################");

                cc1 = cc1 + 1;
            end

            fork
                repeat(max_samples) begin
                    cc2 = 0;

                    // send data into module
                    repeat(N_CHAN) begin
                        if(num_samples[cc2] != 0) begin
                            @(posedge clk_in) begin
                                chan_in = cc2;
                                data_in = $random;
                                dv_in = 1;
                                sum[chan_in] = sum[chan_in] + data_in;

                                if (sum[chan_in] > MAX_SUM) begin
                                    $display("Overflow");
                                    $display("Max sum: %d", MAX_SUM);
                                    $display("Act sum: %d", sum[chan_in]);
                                    sum[chan_in] = MAX_SUM;
                                end else if (sum[chan_in] < MIN_SUM) begin
                                    $display("Underflow");
                                    $display("Min sum: %d", MIN_SUM);
                                    $display("Act sum: %d", sum[chan_in]);
                                    sum[chan_in] = MIN_SUM;
                                end
                            end

                            num_samples[cc2] = num_samples[cc2] - 1;
                        end else begin
                            @(posedge clk_in) dv_in = 0;
                        end

                        @(posedge clk_in) dv_in = 0;
                        cc2 = cc2 + 1;
                    end
                end


                // check internal sum
                repeat(total_samples) begin
                    @(posedge oversample_filter_tf.uut.dv_p2) begin
                        chan_int = oversample_filter_tf.uut.chan_p2;
                        $display("----------------------------------------------------------");
                        $display("INTERNAL TEST -- chan %d", chan_int);
                        $display("----------------------------------------------------------");
                        assert_equals(sum[chan_int], oversample_filter_tf.uut.sum_p2, "SUM", chan_int);
                    end
                end

                // receive data
                repeat(N_CHAN) begin
                    @(posedge dv_out) begin
                        data_rcv = data_out;
                        data_exp = sum[chan_out] / (2**os[chan_out]);
                        sum[chan_out] = 0;

                        delta = data_exp - data_rcv;

                        $display("----------------------------------------------------------");
                        $display("RECEIVE TEST -- chan %d", chan_out);
                        $display("----------------------------------------------------------");
                        $display("Received: %d\tExpected: %d", data_rcv, data_exp);
                        $display("OSM: %d", os);
                        $display("num_samples: %d", num_samples[chan_out]);
                        if(delta == 0 | delta == -1 | delta == 1) begin // data data_rcv might vary by +/- 1
                            $display("SUCCESS");
                        end else begin
                            $display("FAILURE");
                            $stop;
                        end
                        $display("----------------------------------------------------------");
                    end
                end
            join

        end

        $display("*****SIMULATION COMPLETED SUCCESSFULLY*****");

        $stop;

    end

   `include "../ep_map.vh"
   `include "assert_equals.v"

endmodule
