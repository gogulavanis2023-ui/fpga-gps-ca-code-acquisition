`timescale 1ns / 1ps
// ---------------------------------------------------------------------------
// tb_acquisition_slice.v - end-to-end check of the acquisition chain
//
// prn_generator (PRN 1) -> delay_line (800 chips) -> correlator_slice
// Two slices are simulated with the N = 32 partitioning (range = 31):
//   slice 25 covers offsets 776..806 and must lock at offset 800
//            after 25 correlation periods;
//   slice  0 covers offsets 1..31 and must never lock (no false lock).
// The chip clock runs at full simulation speed here (no clk_divider).
// ---------------------------------------------------------------------------
module tb_acquisition_slice;
    reg clk = 1'b0;
    reg reset = 1'b1;
    always #5 clk = ~clk;

    wire local_chip, incoming_chip;
    prn_generator #(.G2_TAP_A(2), .G2_TAP_B(6)) prn (.clk(clk), .reset(reset), .chip_out(local_chip));
    delay_line #(.DELAY_CHIPS(800)) dl (.clk(clk), .chip_in(local_chip), .chip_out(incoming_chip));

    wire lock25, lock0;
    wire [10:0] off25, off0;
    correlator_slice #(.LOCK_THRESHOLD(950)) s25 (
        .clk(clk), .reset(reset), .local_chip(local_chip), .incoming_chip(incoming_chip),
        .range_start(11'd775), .range_size(11'd31), .locked(lock25), .locked_offset(off25), .corr_count_out());
    correlator_slice #(.LOCK_THRESHOLD(950)) s0 (
        .clk(clk), .reset(reset), .local_chip(local_chip), .incoming_chip(incoming_chip),
        .range_start(11'd0), .range_size(11'd31), .locked(lock0), .locked_offset(off0), .corr_count_out());

    integer cycles = 0;
    integer lock_cycle = -1;
    always @(posedge clk) if (!reset) begin
        cycles <= cycles + 1;
        if (lock25 && lock_cycle < 0) lock_cycle <= cycles;
    end

    initial begin
        repeat (4) @(posedge clk);
        reset = 1'b0;
        // Let the delay line fill, then allow 40 full periods: enough for slice 25 (needs 25).
        repeat (1023 * 40 + 1000) @(posedge clk);
        $display("slice 25: locked = %0d, offset = %0d (expected 800), lock after %0d cycles (~%0d periods)",
                 lock25, off25, lock_cycle, lock_cycle / 1023);
        $display("slice  0: locked = %0d (expected 0)", lock0);
        if (lock25 && off25 == 800 && !lock0) $display("RESULT: PASS");
        else                                    $display("RESULT: FAIL");
        $finish;
    end
endmodule
