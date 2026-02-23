`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/03/2025 05:54:33 AM
// Design Name: 
// Module Name: PC_PlusOne
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


module PC_PlusOne#(
    parameter int PC_WIDTH = 8
)(
    input logic [PC_WIDTH - 1:0] CurrentPC,		//Current PC
    output logic [PC_WIDTH - 1:0] NextPC		    //PC+1
);
    assign NextPC = CurrentPC + 1'd1;			//Increment PC
endmodule
