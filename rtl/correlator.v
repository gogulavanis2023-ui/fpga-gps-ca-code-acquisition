module correlator (
    input  wire clk,             // chip-rate clock
    input  wire reset,           // resets chip counter and accumulator at start of each cycle
    input  wire local_chip,      // local PRN replica chip (our own generator)
    input  wire incoming_chip,   // "incoming" chip (simulated satellite signal)
    output reg  [10:0] corr_count = 0,  // final correlation result for the completed 1023-chip cycle (0-1023)
    output reg  cycle_done = 0   // pulses high for one clock when a full 1023-chip cycle has just completed
);

    reg  [10:0] chip_index = 0;       // counts 0..1022 within current correlation cycle
    reg  [10:0] accumulator = 0;      // running count of agreeing chips this cycle

    // Chips "agree" when XOR result is 0 (i.e., local_chip == incoming_chip)
    wire chip_match = ~(local_chip ^ incoming_chip);

    always @(posedge clk) begin
        cycle_done <= 0;   // default: not done, unless we set it below this cycle

        if (reset) begin
            chip_index  <= 0;
            accumulator <= 0;
        end else begin
            if (chip_index == 11'd1022) begin
                // This is the last chip of the cycle: finalize the count and report it
                corr_count  <= accumulator + chip_match;
                cycle_done  <= 1;
                chip_index  <= 0;
                accumulator <= 0;
            end else begin
                accumulator <= accumulator + chip_match;
                chip_index  <= chip_index + 1;
            end
        end
    end

endmodule