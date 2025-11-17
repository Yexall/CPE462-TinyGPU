`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/03/2025 07:24:46 AM
// Design Name: 
// Module Name: PC
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


module PC#(
    parameter int  PC_WIDTH  = 8,
    parameter logic [PC_WIDTH-1:0] RESET_PC = '0,
    parameter int  ALU_WIDTH = 8              
)(
    input  logic                    CLK,
    input  logic                    RST,

    // BR decode/control
    input  logic                    PCMux,          // 1 only for BR instruction
    input  logic [2:0]              NZP,            // {N,Z,P} mask from BR

    // Immediate target (absolute)
    input  logic [PC_WIDTH-1:0]     Immediate,

    // ALU interface (single-output ALU bus + compare-active)
    input  logic [ALU_WIDTH-1:0]    ALUOutBus,      // LSBs = {P,Z,N} when compare is selected
    input  logic                    CmpActive,      // 1 when compare flags are valid this cycle

    // Outputs
    output logic [PC_WIDTH-1:0]     PC
);
    // --- Internal wires ---
    logic [PC_WIDTH-1:0] plus1, branch, next_pc;
    logic [2:0]          NZPOut;       // {N,Z,P} from NZP register
    logic                nzp_match;

    // PZN (on ALU bus) -> NZP for the NZP register input
    wire [2:0] ALU_NZP = { ALUOutBus[0]/*N*/, ALUOutBus[1]/*Z*/, ALUOutBus[2]/*P*/ };
    wire       NZPWE   = CmpActive;

    // --- NZP register & tester ---
    PC_NZPRegister u_nzp_reg (
        .CLK   (CLK),
        .RST   (RST),
        .NZPWE (NZPWE),
        .ALUOut(ALU_NZP),   // {N,Z,P}
        .NZPOut(NZPOut)     // {N,Z,P}
    );

    PC_NZPTester u_nzp_test (
        .NZPOut   (NZPOut), // {N,Z,P}
        .NZP      (NZP),    // {N,Z,P} from BR instruction
        .TesterOut(nzp_match)
    );

    // --- pc + 1 ---
    PC_PlusOne #(.PC_WIDTH(PC_WIDTH)) u_inc (
        .CurrentPC(PC),
        .NextPC   (plus1)
    );

    // --- MuxOne: (nzp_match ? Immediate : pc+1) -> Branch ---
    PC_MuxOne #(.PC_WIDTH(PC_WIDTH)) u_mux1 (
        .Select        (nzp_match),
        .CurrentPCPlus (plus1),
        .Immediate     (Immediate),
        .Branch        (branch)
    );

    // --- MuxTwo: (PCMux ? Branch : pc+1) -> NextPC ---
    PC_MuxTwo #(.PC_WIDTH(PC_WIDTH)) u_mux2 (
        .PCMux         (PCMux),
        .Branch        (branch),
        .CurrentPCPlus (plus1),
        .NextPC        (next_pc)
    );

    // --- PC register (state) ---
    always_ff @(posedge CLK) begin
        if (RST) PC <= RESET_PC;
        else     PC <= next_pc;
    end
endmodule
