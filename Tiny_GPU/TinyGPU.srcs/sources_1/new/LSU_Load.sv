`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/17/2025 06:35:24 AM
// Design Name: 
// Module Name: LSU_Load
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


module LSU_Load #(
    parameter int DATA_WIDTH = 8,
    parameter int ADDR_WIDTH = 8,

    // Core state encodings (override to match your core constants)
    parameter logic [2:0] ST_REQUEST = 3'b011,
    parameter logic [2:0] ST_UPDATE  = 3'b110
)(
    input  logic                   CLK,
    input  logic                   RST,

    // From core/decode
    input  logic [2:0]             core_state,
    input  logic                   decoded_mem_read_enable,  // LDR enable

    // Address source
    input  logic [ADDR_WIDTH-1:0]  rs,   // effective address

    // Memory (data) interface - Valid/Ready
    output logic                   mem_read_valid,
    output logic [ADDR_WIDTH-1:0]  mem_read_address,
    input  logic                   mem_read_ready,
    input  logic [DATA_WIDTH-1:0]  mem_read_data,

    // Outputs to core
    output logic [DATA_WIDTH-1:0]  lsu_out,
    output logic [1:0]             lsu_state                 // 00 IDLE, 01 REQ, 10 WAIT, 11 DONE
);
    // FSM
    typedef enum logic [1:0] { IDLE=2'b00, REQUESTING=2'b01, WAITING=2'b10, DONE=2'b11 } lsu_fsm_e;
    lsu_fsm_e state_q, state_d;

    // Combinational next-state and memory drive
    always_comb begin
        // Defaults
        state_d          = state_q;
        mem_read_valid   = 1'b0;
        mem_read_address = '0;

        unique case (state_q)
            IDLE: begin
                // Kick off a load when entering REQUEST with LDR enabled
                if (core_state == ST_REQUEST && decoded_mem_read_enable) begin
                    state_d = REQUESTING;
                end
            end

            REQUESTING: begin
                // One-cycle valid pulse with the requested address
                mem_read_valid   = 1'b1;
                mem_read_address = rs;
                state_d          = WAITING;
            end

            WAITING: begin
                // Wait for memory to return data
                if (mem_read_ready) begin
                    state_d = DONE;
                end
            end

            DONE: begin
                // Hold result until UPDATE, then go idle
                if (core_state == ST_UPDATE) begin
                    state_d = IDLE;
                end
            end
        endcase
    end

    // State/data registers
    always_ff @(posedge CLK) begin
        if (RST) begin
            state_q <= IDLE;
            lsu_out <= '0;
        end else begin
            state_q <= state_d;

            // Capture memory data precisely when it arrives
            if (state_q == WAITING && mem_read_ready) begin
                lsu_out <= mem_read_data;
            end
        end
    end

    // Encode state to 2-bit output
    assign lsu_state = state_q;

endmodule
