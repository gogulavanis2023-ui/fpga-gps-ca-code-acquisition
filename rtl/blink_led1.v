`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08.06.2026 11:57:59
// Design Name: 
// Module Name: blink_led1
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module blink_led1 (
    input  wire clk,      // 100 MHz clock from the board
    output reg  led       // one LED output
);

    reg [26:0] counter = 0;   // 27-bit counter: counts up to ~134 million

    always @(posedge clk) begin
        if (counter == 27'd49_999_999) begin
            counter <= 0;
            led <= ~led;      // flip the LED state
        end else begin
            counter <= counter + 1;
        end
    end

endmodule
