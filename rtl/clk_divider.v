module clk_divider #(
    parameter DIVIDE_COUNT = 100_000   // 100MHz / 25,000,000 = 4 Hz toggle rate -> 2Hz output period
)(
    input  wire clk_in,
    output reg  clk_out = 0
);

    reg [24:0] count = 0;   // 25 bits comfortably holds up to 25,000,000

    always @(posedge clk_in) begin
        if (count == DIVIDE_COUNT - 1) begin
            count   <= 0;
            clk_out <= ~clk_out;
        end else begin
            count <= count + 1;
        end
    end

endmodule