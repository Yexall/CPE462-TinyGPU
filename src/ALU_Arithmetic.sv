`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/03/2025 08:00:33 AM
// Design Name: 
// Module Name: ALU_Arithmetic
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


module ALU_Arithmetic #(
    parameter int WIDTH = 8
)(
    input logic [1:0] AluArithmeticMux,        //Opcode select for arithmetic
    input logic [WIDTH-1:0] rs,                //Operand A
    input logic [WIDTH-1:0] rt,                //Operand B
    output logic [WIDTH-1:0] ArithOut          //Arithmetic result
);

    // Opcodes  //
    localparam logic [1:0] ADD = 2'b00,
                           SUB = 2'b01,
                           MUL = 2'b10,
                           DIV = 2'b11;

    // Combinational Arithmetic  //
    always_comb begin
        case (AluArithmeticMux)
            ADD: ArithOut = rs + rt;        //Add
            SUB: ArithOut = rs - rt;        //Subtract
            MUL: ArithOut = rs * rt;        //Multiply
            DIV: ArithOut = rs / rt;        //Divide
            default: ArithOut = '0;
        endcase
    end
endmodule


