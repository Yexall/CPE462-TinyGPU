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
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module ALU_Comparison#(
    parameter int WIDTH  = 8,
    parameter bit SIGNED = 1'b0
)(
    input  logic [WIDTH-1:0] rs,
    input  logic [WIDTH-1:0] rt,
    output logic [2:0]       CmpOut   // {N,Z,P}
);
    logic LessThan = SIGNED ? ($signed(rs) <  $signed(rt)) : (rs <  rt);
    logic Equal = (rs == rt);
    logic GreaterThan = SIGNED ? ($signed(rs) >  $signed(rt)) : (rs >  rt);

    always_comb begin
        if (Equal) CmpOut = 3'b010;              // Z
        else if (LessThan) CmpOut = 3'b100;      // N
        else CmpOut = 3'b001;                    // P
    end
endmodule
