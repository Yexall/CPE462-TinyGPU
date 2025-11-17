`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/03/2025 08:24:47 AM
// Design Name: 
// Module Name: ALU
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


module ALU#(
    parameter int WIDTH = 8,
    parameter logic [2:0] EXECUTE = 3'b101
)(
    input  logic             CLK,
    input  logic             RST,
    input  logic             ENABLE,        // some ALUs may be inactive
    input  logic [2:0]       CoreState,     // state machine from your core

    // Controls
    input  logic [1:0]       AluArithmeticMux,  // 00=ADD 01=SUB 10=MUL 11=DIV
    input  logic             ALUOutputMux,      // 0: Arithmetic, 1: Comparison flags

    // Operands
    input  logic [WIDTH-1:0] rs,
    input  logic [WIDTH-1:0] rt,

    // Single external output
    output logic [WIDTH-1:0] ALUOut
);
    logic [WIDTH-1:0] arith_y;
    logic [2:0]       cmp_nzp;
    logic [WIDTH-1:0] alu_bus_comb;

    // Submodules
    ALU_Arithmetic #(.WIDTH(WIDTH)) u_arith (
        .AluArithmeticMux(AluArithmeticMux), .rs(rs), .rt(rt), .ArithOut(arith_y)
    );

    ALU_Comparison #(.WIDTH(WIDTH), .SIGNED(1'b0)) u_cmp (
        .rs(rs), .rt(rt), .CmpOut(cmp_nzp)    // {N,Z,P}
    );

    ALU_Mux #(.WIDTH(WIDTH)) u_mux (
        .ALUOutputMux(ALUOutputMux),
        .ArithOut(arith_y),
        .CmpOut(cmp_nzp),     // {N,Z,P}
        .ALUOut(alu_bus_comb) // LSBs = {P,Z,N} when compare selected
    );

    // Register the ALU bus during EXECUTE
    always_ff @(posedge CLK) begin
        if (RST) begin
            ALUOut <= '0;
        end else if (ENABLE && (CoreState == EXECUTE)) begin
            ALUOut <= alu_bus_comb;
        end
        // else: hold last ALUOut
    end
endmodule

