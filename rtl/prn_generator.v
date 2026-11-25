module prn_generator #(
    parameter G2_TAP_A = 1,
    parameter G2_TAP_B = 5
)(
    input  wire clk,
    input  wire reset,
    output wire chip_out
);

    // Bit k of each register holds LFSR stage k+1 (GPS ICD numbering):
    // new bits enter stage 1 (bit 0) and the G1 output is stage 10 (bit 9).
    reg [9:0] g1 = 10'b1111111111;
    reg [9:0] g2 = 10'b1111111111;

    wire g1_feedback = g1[2] ^ g1[9];
    wire g2_feedback = g2[1] ^ g2[2] ^ g2[5] ^ g2[7] ^ g2[8] ^ g2[9];
    wire g2_select = g2[G2_TAP_A-1] ^ g2[G2_TAP_B-1];

    assign chip_out = g1[9] ^ g2_select;

    always @(posedge clk) begin
        if (reset) begin
            g1 <= 10'b1111111111;
            g2 <= 10'b1111111111;
        end else begin
            g1 <= {g1[8:0], g1_feedback};
            g2 <= {g2[8:0], g2_feedback};
        end
    end

endmodule