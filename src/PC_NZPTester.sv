`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/03/2025 07:07:09 AM
// Design Name: 
// Module Name: PC_NZPTester
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


module PC_NZPTester(
    input  logic [2:0] NZPOut,          //(N,Z,P)
    input  logic [2:0] NZP,             //(N,Z,P) from BR instruction
    output logic TesterOut
);
    assign TesterOut = |(NZPOut & NZP); //True if ANY requested flag is set
endmodule
