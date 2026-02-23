`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/03/2025 08:01:25 AM
// Design Name: 
// Module Name: ALU_Comparison
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


module ALU_Comparison #(
    parameter int WIDTH  = 8
)(
    input  logic [WIDTH - 1:0] rs,                  //Operand A
    input  logic [WIDTH - 1:0] rt,                  //Operand B
    output logic [2:0] CmpOut                       //Flags as (N,Z,P)
);

    logic signed [WIDTH:0] diff;                    //Extra bit prevents overflow on subtract

    // NZP Flag Generation  //
    always_comb begin
        diff = $signed({1'b0, rs})  -  $signed({1'b0, rt}); //Signed compare via signed subtract

        if (diff == 0) CmpOut = 3'b010;                     //Z
        else if (diff < 0) CmpOut = 3'b100;                 //N
        else CmpOut = 3'b001;                               //P
    end
endmodule
