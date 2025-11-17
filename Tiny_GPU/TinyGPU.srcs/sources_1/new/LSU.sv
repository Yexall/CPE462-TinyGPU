`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/17/2025 07:16:33 AM
// Design Name: 
// Module Name: LSU
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


module LSU #(
    parameter int DATA_WIDTH = 8,
    parameter int ADDR_WIDTH = 8,
    parameter logic [2:0] ST_REQUEST = 3'b011,
    parameter logic [2:0] ST_UPDATE  = 3'b110
)(
    input  logic                   CLK,
    input  logic                   RST,

    input  logic [2:0]             core_state,
    input  logic                   mem_read_enable,
    input  logic                   mem_write_enable,

    input  logic [ADDR_WIDTH-1:0]  rs,
    input  logic [DATA_WIDTH-1:0]  rt,

    // Read interface
    output logic                   mem_read_valid,
    output logic [ADDR_WIDTH-1:0]  mem_read_address,
    input  logic                   mem_read_ready,
    input  logic [DATA_WIDTH-1:0]  mem_read_data,

    // Write interface
    output logic                   mem_write_valid,
    output logic [ADDR_WIDTH-1:0]  mem_write_address,
    output logic [DATA_WIDTH-1:0]  mem_write_data,
    input  logic                   mem_write_ready,

    output logic [DATA_WIDTH-1:0]  lsu_out,   // from Load only
    output logic [1:0]             lsu_state
);
    // Load submodule
    logic [DATA_WIDTH-1:0] load_out;
    logic [1:0]            load_state;

    LSU_Load #(
        .DATA_WIDTH (DATA_WIDTH),
        .ADDR_WIDTH (ADDR_WIDTH),
        .ST_REQUEST (ST_REQUEST),
        .ST_UPDATE  (ST_UPDATE)
    ) u_load (
        .CLK                   (CLK),
        .RST                   (RST),
        .core_state            (core_state),
        .decoded_mem_read_enable(mem_read_enable),
        .rs                    (rs),
        .mem_read_valid        (mem_read_valid),
        .mem_read_address      (mem_read_address),
        .mem_read_ready        (mem_read_ready),
        .mem_read_data         (mem_read_data),
        .lsu_out               (load_out),
        .lsu_state             (load_state)
    );

    // Store submodule
    logic [1:0] store_state;

    LSU_Store #(
        .DATA_WIDTH (DATA_WIDTH),
        .ADDR_WIDTH (ADDR_WIDTH),
        .ST_REQUEST (ST_REQUEST),
        .ST_UPDATE  (ST_UPDATE)
    ) u_store (
        .CLK                 (CLK),
        .RST                 (RST),
        .core_state          (core_state),
        .decoded_mem_write_enable(mem_write_enable),
        .rs                  (rs),
        .rt                  (rt),
        .mem_write_valid     (mem_write_valid),
        .mem_write_address   (mem_write_address),
        .mem_write_data      (mem_write_data),
        .mem_write_ready     (mem_write_ready),
        .lsu_state           (store_state)
    );

    // Data output from Load only
    assign lsu_out = load_out;

    // State selection: prefer non-IDLE state from Load, else Store, else IDLE
    assign lsu_state =
        (load_state  != 2'b00) ? load_state  :
        (store_state != 2'b00) ? store_state : 2'b00;

endmodule

