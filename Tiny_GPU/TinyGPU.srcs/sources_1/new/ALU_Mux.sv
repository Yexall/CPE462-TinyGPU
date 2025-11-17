`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/03/2025 08:07:34 AM
// Design Name: 
// Module Name: ALU_Mux
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


module ALU_Mux#(
    parameter int WIDTH = 8
)(
    input  logic               ALUOutputMux,   // 0: Arithmetic; 1: Comparison flags
    input  logic [WIDTH-1:0]   ArithOut,       // from Arithmetic
    input  logic [2:0]         CmpOut,         // {N,Z,P} from Comparison
    output logic [WIDTH-1:0]   ALUOut          // single ALU bus
);
    
    wire [WIDTH-1:0] FlagsPZN = {
        {(WIDTH-3){1'b0}},
        CmpOut[0]/*P*/, CmpOut[1]/*Z*/, CmpOut[2]/*N*/
    };

    always_comb ALUOut = ALUOutputMux ? FlagsPZN : ArithOut;
endmodule

