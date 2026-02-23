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
    input logic PCMux,					                    //PC mux select
    input logic [PC_WIDTH - 1:0] Branch,			            //Branch candidate
    input logic [PC_WIDTH - 1:0] CurrentPCPlus,		        //Sequential candidate
    output logic [PC_WIDTH - 1:0] NextPC			            //Final next PC
);

    always_comb NextPC = PCMux ? Branch : CurrentPCPlus;    //PCMux chooses branch vs sequential
endmodule
