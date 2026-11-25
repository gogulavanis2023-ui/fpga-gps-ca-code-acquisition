module debounce (
    input  wire clk,
    input  wire btn_in,
    output reg  btn_out = 0
);

    localparam STABLE_COUNT = 100_000;

    reg [16:0] counter = 0;
    reg btn_sync_0 = 0, btn_sync_1 = 0;
    reg btn_last = 0;

    always @(posedge clk) begin
        btn_sync_0 <= btn_in;
        btn_sync_1 <= btn_sync_0;

        if (btn_sync_1 != btn_last) begin
            counter  <= 0;
            btn_last <= btn_sync_1;
        end else if (counter < STABLE_COUNT) begin
            counter <= counter + 1;
        end else begin
            btn_out <= btn_last;
        end
    end

endmodule