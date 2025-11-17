`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/03/2025 06:01:55 AM
// Design Name: 
// Module Name: PC_MuxTwo
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


module PC_MuxTwo#(
    parameter int PC_WIDTH = 8
)(
    input logic PCMux,
    input logic [PC_WIDTH-1:0] Branch,
    input logic [PC_WIDTH-1:0] CurrentPCPlus,
    output logic [PC_WIDTH-1:0] NextPC
);

    always_comb NextPC = PCMux ? Branch : CurrentPCPlus;
endmodule
