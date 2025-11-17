`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/17/2025 07:11:37 AM
// Design Name: 
// Module Name: LSU_Store
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


module LSU_Store #(
    parameter int DATA_WIDTH = 8,
    parameter int ADDR_WIDTH = 8,
    parameter logic [2:0] ST_REQUEST = 3'b011,
    parameter logic [2:0] ST_UPDATE  = 3'b110
)(
    input  logic                   CLK,
    input  logic                   RST,

    input  logic [2:0]             core_state,
    input  logic                   decoded_mem_write_enable,

    input  logic [ADDR_WIDTH-1:0]  rs,   // write address
    input  logic [DATA_WIDTH-1:0]  rt,   // write data

    output logic                   mem_write_valid,
    output logic [ADDR_WIDTH-1:0]  mem_write_address,
    output logic [DATA_WIDTH-1:0]  mem_write_data,
    input  logic                   mem_write_ready,

    output logic [1:0]             lsu_state          // 00 IDLE, 01 REQ, 10 WAIT, 11 DONE
);
    typedef enum logic [1:0] { IDLE=2'b00, REQUESTING=2'b01, WAITING=2'b10, DONE=2'b11 } lsu_fsm_e;
    lsu_fsm_e state_q, state_d;

    // Next-state and memory drive
    always_comb begin
        state_d           = state_q;
        mem_write_valid   = 1'b0;
        mem_write_address = '0;
        mem_write_data    = '0;

        unique case (state_q)
            IDLE: begin
                if (core_state == ST_REQUEST && decoded_mem_write_enable) state_d = REQUESTING;
            end
            REQUESTING: begin
                mem_write_valid   = 1'b1;     // single-cycle strobe
                mem_write_address = rs;
                mem_write_data    = rt;
                state_d           = WAITING;
            end
            WAITING: begin
                if (mem_write_ready) state_d = DONE;
            end
            DONE: begin
                if (core_state == ST_UPDATE) state_d = IDLE;
            end
        endcase
    end

    // State register
    always_ff @(posedge CLK) begin
        if (RST) state_q <= IDLE;
        else     state_q <= state_d;
    end

    assign lsu_state = state_q;
endmodule
