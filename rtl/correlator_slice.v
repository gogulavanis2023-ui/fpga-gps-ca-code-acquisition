module correlator_slice #(
    parameter LOCK_THRESHOLD = 950
)(
    input  wire clk,
    input  wire reset,
    input  wire local_chip,
    input  wire incoming_chip,
    input  wire [10:0] range_start,
    input  wire [10:0] range_size,
    output wire locked,
    output wire [10:0] locked_offset,
    output wire [10:0] corr_count_out
);

    reg [3:0] settle_counter = 0;
    reg settled = 0;
    always @(posedge clk) begin
        if (reset) begin
            settle_counter <= 0;
            settled        <= 0;
        end else if (!settled) begin
            if (settle_counter == 4'd8) begin
                settled <= 1;
            end else begin
                settle_counter <= settle_counter + 1;
            end
        end
    end

    wire correlator_reset = reset | ~settled;

    wire [10:0] local_delay_select;
    wire [10:0] absolute_delay = range_start + local_delay_select;

    wire local_chip_candidate;
    variable_delay_line #(
        .MAX_DELAY(1023)
    ) var_delay_inst (
        .clk          (clk),
        .chip_in      (local_chip),
        .delay_select (absolute_delay),
        .chip_out     (local_chip_candidate)
    );

    wire [10:0] corr_count;
    wire cycle_done;
    correlator correlator_inst (
        .clk           (clk),
        .reset         (correlator_reset),
        .local_chip    (local_chip_candidate),
        .incoming_chip (incoming_chip),
        .corr_count    (corr_count),
        .cycle_done    (cycle_done)
    );

    reg [10:0] delay_select_reg = 0;
    reg locked_reg = 0;
    reg [10:0] locked_offset_reg = 0;

    always @(posedge clk) begin
        if (reset) begin
            delay_select_reg  <= 0;
            locked_reg        <= 0;
            locked_offset_reg <= 0;
        end else if (cycle_done && !locked_reg) begin
            if (corr_count >= LOCK_THRESHOLD) begin
                locked_reg        <= 1;
                locked_offset_reg <= range_start + delay_select_reg + 1;
            end else begin
                if (delay_select_reg >= range_size - 1) begin
                    delay_select_reg <= 0;
                end else begin
                    delay_select_reg <= delay_select_reg + 1;
                end
            end
        end
    end

    assign local_delay_select = delay_select_reg;
    assign locked             = locked_reg;
    assign locked_offset      = locked_offset_reg;
    assign corr_count_out     = corr_count;

endmodule