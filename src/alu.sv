`default_nettype none
`timescale 1ns/1ns

// ARITHMETIC-LOGIC UNIT
// > Executes computations on register values
// > In this minimal implementation, the ALU supports the 4 basic arithmetic operations
// > Each thread in each core has it's own ALU
// > ADD, SUB, MUL, DIV instructions are all executed here
module alu (
    input wire clk,
    input wire reset,
    input wire enable,                              //Per-thread ALU enable (inactive lanes disabled)

    input reg [2:0] core_state,                     //Core pipeline state (controls when ALU updates)

    input reg [1:0] decoded_alu_arithmetic_mux,     //Selects ADD/SUB/MUL/DIV
    input reg decoded_alu_output_mux,               //Selects arithmetic vs compare flags

    input reg [7:0] rs,                             //Operand A
    input reg [7:0] rt,                             //Operand B
    output wire [7:0] alu_out                       //ALU result bus
);

    localparam logic [2:0] EXECUTE = 3'b101;        //ALU writes output only during EXECUTE

    // Internal Signals //
    logic [7:0] arith_out;                          //Arithmetic result
    logic [2:0] cmp_nzp;                            //Compare flags as (N,Z,P)
    logic [7:0] alu_bus_comb;                       //Muxed ALU bus
    logic [7:0] alu_out_reg;                        //Output register

    assign alu_out = alu_out_reg;                   //External ALU output is registered

    // Arithmetic Submodule //
    ALU_Arithmetic #(
        .WIDTH(8)
    ) u_arith (
        .rs(rs),
        .rt(rt),
        .AluArithmeticMux(decoded_alu_arithmetic_mux),
        .ArithOut(arith_out)
    );

    // Comparison Submodule //
    ALU_Comparison #(
        .WIDTH(8)
    ) u_cmp (
        .rs(rs),
        .rt(rt),
        .CmpOut(cmp_nzp)                            //Outputs (N,Z,P)
    );

    // Output Select (Arithmetic vs Flags) //
    ALU_Mux #(
        .WIDTH(8)
    ) u_mux (
        .ALUOutputMux(decoded_alu_output_mux),      //0: Arithmetic, 1: Flags
        .ArithOut(arith_out),
        .CmpOut(cmp_nzp),                           //(N,Z,P)
        .ALUOut(alu_bus_comb)                       //LSBs become (P,Z,N) when flags selected
    );

    // Output Register //
    always@ (posedge clk) begin
        if (reset) begin
            alu_out_reg <= 8'b0;                    //Reset output
        end else if (enable && (core_state == EXECUTE)) begin
            alu_out_reg <= alu_bus_comb;            //Change result only when lane active + EXECUTE
        end
        //Else: Hold previous alu_out_reg
    end
endmodule
