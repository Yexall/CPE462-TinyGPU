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
// Revision 0.01  -  File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module PC_MuxOne#(
    parameter int PC_WIDTH = 8
)(
    input logic NZPSelect,				                        //NZP match select
    input logic [PC_WIDTH - 1:0] CurrentPCPlus,		            //PC+1
    input logic [PC_WIDTH - 1:0] Immediate,			            //Immediate target
    output logic [PC_WIDTH - 1:0] Branch			                //Selected branch candidate
);
    always_comb Branch = NZPSelect ? Immediate : CurrentPCPlus; //NZPSelect chooses immediate vs PC+1
endmodule
