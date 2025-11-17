`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/03/2025 06:01:55 AM
// Design Name: 
// Module Name: PC_MuxOne
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


module PC_MuxOne#(
    parameter int PC_WIDTH = 8
)(
    input logic NZPSelect,
    input logic [PC_WIDTH-1:0] CurrentPCPlus,
    input logic [PC_WIDTH-1:0] Immediate,
    output logic [PC_WIDTH-1:0] Branch
);
    
    always_comb Branch = NZPSelect ? Immediate : CurrentPCPlus;
endmodule
