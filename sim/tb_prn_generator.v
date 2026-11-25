`timescale 1ns / 1ps
// ---------------------------------------------------------------------------
// tb_prn_generator.v - self-checking testbench for prn_generator.v
//
// For PRN 1..8 (G2 phase taps from the GPS ICD) it checks that:
//   1. the first 10 chips match the ICD table (octal 1440, 1620, 1710, 1744,
//      1133, 1455, 1131, 1454)
//   2. one 1023-chip period contains 512 ones (C/A code balance)
//   3. the sequence repeats with period 1023 (chip[i] == chip[i+1023])
// Prints PASS or FAIL per PRN and an overall result.
// ---------------------------------------------------------------------------
module tb_prn_generator;
    reg clk = 1'b0;
    reg reset = 1'b1;
    always #5 clk = ~clk;

    localparam integer NCH = 2046;
    wire [7:0] chip;

    // G2 phase-select taps for PRN 1..8
    localparam [8*4-1:0] TAP_A = {4'd2, 4'd1, 4'd2, 4'd1, 4'd5, 4'd4, 4'd3, 4'd2}; // PRN8..PRN1
    localparam [8*4-1:0] TAP_B = {4'd9, 4'd8, 4'd10, 4'd9, 4'd9, 4'd8, 4'd7, 4'd6};

    genvar p;
    generate
        for (p = 0; p < 8; p = p + 1) begin : prn
            prn_generator #(.G2_TAP_A(TAP_A[p*4 +: 4]), .G2_TAP_B(TAP_B[p*4 +: 4])) dut (
                .clk(clk), .reset(reset), .chip_out(chip[p])
            );
        end
    endgenerate

    reg seq [0:7][0:NCH-1];
    reg [9:0] expected [0:7];
    integer i, k, ones, pm, errs, total_errs;
    reg [9:0] first10;

    initial begin
        expected[0] = 10'o1440; expected[1] = 10'o1620; expected[2] = 10'o1710; expected[3] = 10'o1744;
        expected[4] = 10'o1133; expected[5] = 10'o1455; expected[6] = 10'o1131; expected[7] = 10'o1454;
        total_errs = 0;

        repeat (2) @(posedge clk);
        @(negedge clk) reset = 1'b0;

        // Chip 0 is the output of the reset state. Sample on each falling edge,
        // i.e. half a cycle after the rising edge that produced the chip.
        for (i = 0; i < NCH; i = i + 1) begin
            #1;
            for (k = 0; k < 8; k = k + 1) seq[k][i] = chip[k];
            @(negedge clk);
        end

        for (k = 0; k < 8; k = k + 1) begin
            errs = 0;
            first10 = 10'd0;
            for (i = 0; i < 10; i = i + 1) first10 = {first10[8:0], seq[k][i]};
            if (first10 !== expected[k]) errs = errs + 1;
            ones = 0;
            for (i = 0; i < 1023; i = i + 1) ones = ones + seq[k][i];
            if (ones != 512) errs = errs + 1;
            pm = 0;
            for (i = 0; i < 1023; i = i + 1) if (seq[k][i] !== seq[k][i + 1023]) pm = pm + 1;
            errs = errs + pm;
            $display("PRN %0d: first10 = %o (expected %o), ones = %0d, period-1023 mismatches = %0d -> %s",
                     k + 1, first10, expected[k], ones, pm, errs == 0 ? "PASS" : "FAIL");
            total_errs = total_errs + errs;
        end

        if (total_errs == 0) $display("RESULT: PASS - all 8 PRNs match the GPS ICD");
        else                 $display("RESULT: FAIL - %0d errors", total_errs);
        $finish;
    end
endmodule
