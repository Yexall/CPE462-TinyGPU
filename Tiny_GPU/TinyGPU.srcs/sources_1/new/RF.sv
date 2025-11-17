`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/17/2025 07:32:31 AM
// Design Name: 
// Module Name: RF
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


module RF #(
    parameter int DATA_BITS = 8
) (
    input  wire                    CLK,
    input  wire                    RST,

    // Core state (e.g., 3'b110 = WRITEBACK)
    input  wire [2:0]              core_state,

    // Read addresses
    input  wire [3:0]              decoded_rs_address,
    input  wire [3:0]              decoded_rt_address,

    // Write controls
    input  wire                    decoded_reg_write_enable,   // write enable for r0..r12
    input  wire [3:0]              decoded_rd_address,         // 0..12 writable
    input  wire [1:0]              decoded_reg_input_mux,      // 00=ALU, 01=MEM, 10=CONST

    // Writeback sources
    input  wire [DATA_BITS-1:0]    alu_out,
    input  wire [DATA_BITS-1:0]    lsu_out,
    input  wire [DATA_BITS-1:0]    decoded_immediate,

    // Read-only execution context (mapped to r13..r15)
    input  wire [DATA_BITS-1:0]    blockIdx_in,   // r13
    input  wire [DATA_BITS-1:0]    blockDim_in,   // r14
    input  wire [DATA_BITS-1:0]    threadIdx_in,  // r15

    // Read data
    output wire [DATA_BITS-1:0]    rs,
    output wire [DATA_BITS-1:0]    rt
);

    // 16 x DATA_BITS register file
    logic [DATA_BITS-1:0] regs [0:15];

    // Write-back mux
    wire [DATA_BITS-1:0] wb_data;
    Mux #(.DATA_BITS(DATA_BITS)) u_wb_mux (
        .sel     (decoded_reg_input_mux),  // 00=ALU, 01=MEM, 10=CONST
        .alu_out (alu_out),
        .lsu_out (lsu_out),
        .imm     (decoded_immediate),
        .wb_data (wb_data)
    );

    // Reset and updates: r0..r12 writable; r13..r15 reflect context
    integer i;
    always_ff @(posedge CLK or posedge RST) begin
        if (RST) begin
            for (i = 0; i < 13; i++) begin
                regs[i] <= '0;
            end
            regs[13] <= blockIdx_in;
            regs[14] <= blockDim_in;
            regs[15] <= threadIdx_in;
        end else begin
            // Writeback stage
            if (core_state == 3'b110) begin
                if (decoded_reg_write_enable && (decoded_rd_address < 4'd13)) begin
                    regs[decoded_rd_address] <= wb_data;
                end
                // Keep context registers current
                regs[13] <= blockIdx_in;
                regs[14] <= blockDim_in;
                regs[15] <= threadIdx_in;
            end
        end
    end

    // Asynchronous reads
    assign rs = regs[decoded_rs_address];
    assign rt = regs[decoded_rt_address];

endmodule

