module seven_seg_driver (
    input  wire clk,
    input  wire [10:0] value,
    output reg  [6:0] seg,
    output reg  [3:0] an = 4'b1110
);

    wire [3:0] digit0 = value % 10;
    wire [3:0] digit1 = (value / 10) % 10;
    wire [3:0] digit2 = (value / 100) % 10;
    wire [3:0] digit3 = (value / 1000) % 10;

    reg [1:0] digit_select = 0;
    reg [16:0] refresh_count = 0;

    always @(posedge clk) begin
        refresh_count <= refresh_count + 1;
        if (refresh_count == 0) begin
            digit_select <= digit_select + 1;
        end
    end

    reg [3:0] current_digit;

    always @(*) begin
        case (digit_select)
            2'b00: begin an = 4'b1110; current_digit = digit0; end
            2'b01: begin an = 4'b1101; current_digit = digit1; end
            2'b10: begin an = 4'b1011; current_digit = digit2; end
            2'b11: begin an = 4'b0111; current_digit = digit3; end
        endcase
    end

    always @(*) begin
        case (current_digit)
            4'd0: seg = 7'b1000000;
            4'd1: seg = 7'b1111001;
            4'd2: seg = 7'b0100100;
            4'd3: seg = 7'b0110000;
            4'd4: seg = 7'b0011001;
            4'd5: seg = 7'b0010010;
            4'd6: seg = 7'b0000010;
            4'd7: seg = 7'b1111000;
            4'd8: seg = 7'b0000000;
            4'd9: seg = 7'b0010000;
            default: seg = 7'b1111111;
        endcase
    end

endmodule