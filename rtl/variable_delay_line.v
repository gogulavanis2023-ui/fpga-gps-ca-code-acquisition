module variable_delay_line #(
    parameter MAX_DELAY = 1023
)(
    input  wire clk,
    input  wire chip_in,
    input  wire [10:0] delay_select,
    output wire chip_out
);

    reg [MAX_DELAY-1:0] shift_reg = 0;

    always @(posedge clk) begin
        shift_reg <= {shift_reg[MAX_DELAY-2:0], chip_in};
    end

    assign chip_out = shift_reg[delay_select];

endmodule