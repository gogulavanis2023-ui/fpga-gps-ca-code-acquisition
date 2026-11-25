module delay_line #(
    parameter DELAY_CHIPS = 50   // how many chips of delay to introduce (the "true" code phase offset)
)(
    input  wire clk,
    input  wire chip_in,
    output wire chip_out
);

    reg [DELAY_CHIPS-1:0] shift_reg = 0;

    always @(posedge clk) begin
        shift_reg <= {shift_reg[DELAY_CHIPS-2:0], chip_in};
    end

    assign chip_out = shift_reg[DELAY_CHIPS-1];

endmodule