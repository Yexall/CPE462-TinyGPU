`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/03/2025 07:06:50 AM
// Design Name: 
// Module Name: PC_NZPRegister
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


module PC_NZPRegister#(
    parameter logic [2:0] RESET_NZP = 3'b010    // Z set on reset
)(
    input  logic       CLK,
    input  logic       RST,
    input  logic       NZPWE,                   // write enable
    input  logic [2:0] ALUOut,                  // expected order {N,Z,P}
    output logic [2:0] NZPOut                   // current flags {N,Z,P}
);
    always_ff @(posedge CLK) begin
        if (RST)            NZPOut <= RESET_NZP;
        else if (NZPWE)     NZPOut <= ALUOut;
    end
endmodule
