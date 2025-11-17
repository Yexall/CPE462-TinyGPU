`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/17/2025 07:59:05 AM
// Design Name: 
// Module Name: Mux
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


module Mux #(
    parameter int DATA_BITS = 8
) (
    input  wire [1:0]               sel,            // 00 ALU, 01 MEMORY, 10 CONSTANT
    input  wire [DATA_BITS-1:0]     alu_out,
    input  wire [DATA_BITS-1:0]     lsu_out,
    input  wire [DATA_BITS-1:0]     imm,
    output logic [DATA_BITS-1:0]    wb_data
);
    localparam [1:0] ARITHMETIC = 2'b00,
                     MEMORY     = 2'b01,
                     CONSTANT   = 2'b10;

    always_comb begin
        unique case (sel)
            ARITHMETIC: wb_data = alu_out;
            MEMORY:     wb_data = lsu_out;
            CONSTANT:   wb_data = imm;
            default:    wb_data = '0;
        endcase
    end
endmodule
