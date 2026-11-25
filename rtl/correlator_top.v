
module correlator_top (
    input  wire clk,
    input  wire btnC,
    input  wire [4:0] sw,
    output wire [6:0] seg,
    output wire [3:0] an,
    output wire led_locked
);

    localparam MAX_CORRELATORS = 32;
    localparam TOTAL_OFFSETS   = 1023;

    wire btnC_clean;
    debounce debounce_inst (
        .clk     (clk),
        .btn_in  (btnC),
        .btn_out (btnC_clean)
    );

    wire [5:0] active_count = sw + 1;
    wire [10:0] range_size = TOTAL_OFFSETS / active_count;

    wire chip_clk;
    clk_divider #(
        .DIVIDE_COUNT(100_000)
    ) divider_inst (
        .clk_in  (clk),
        .clk_out (chip_clk)
    );

    wire local_chip;
    prn_generator #(
        .G2_TAP_A(2),
        .G2_TAP_B(6)
    ) local_prn_inst (
        .clk      (chip_clk),
        .reset    (btnC_clean),
        .chip_out (local_chip)
    );

    wire incoming_chip;
    delay_line #(
        .DELAY_CHIPS(800)
    ) delay_inst (
        .clk      (chip_clk),
        .chip_in  (local_chip),
        .chip_out (incoming_chip)
    );

    wire [MAX_CORRELATORS-1:0] slice_locked;
    wire [10:0] slice_locked_offset [MAX_CORRELATORS-1:0];

    genvar i;
    generate
        for (i = 0; i < MAX_CORRELATORS; i = i + 1) begin : slice_array
            wire slice_enable = (i < active_count);
            wire slice_reset  = btnC_clean | ~slice_enable;
            wire [10:0] this_range_start = i * range_size;

            correlator_slice #(
                .LOCK_THRESHOLD (950)
            ) slice_inst (
                .clk            (chip_clk),
                .reset          (slice_reset),
                .local_chip     (local_chip),
                .incoming_chip  (incoming_chip),
                .range_start    (this_range_start),
                .range_size     (range_size),
                .locked         (slice_locked[i]),
                .locked_offset  (slice_locked_offset[i]),
                .corr_count_out ()
            );
        end
    endgenerate

    reg overall_locked = 0;
    reg [10:0] overall_offset = 0;
    integer j;
    always @(posedge chip_clk) begin
        overall_locked = 0;
        overall_offset = 0;
        for (j = MAX_CORRELATORS - 1; j >= 0; j = j - 1) begin
            if (j < active_count && slice_locked[j]) begin
                overall_locked = 1;
                overall_offset = slice_locked_offset[j];
            end
        end
    end

    wire [10:0] display_value = overall_locked ? overall_offset : 11'd0;

    seven_seg_driver display_inst (
        .clk   (clk),
        .value (display_value),
        .seg   (seg),
        .an    (an)
    );

    assign led_locked = overall_locked;

endmodule