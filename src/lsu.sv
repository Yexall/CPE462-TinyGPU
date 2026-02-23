`default_nettype none
`timescale 1ns/1ns

// LOAD-STORE UNIT
// > Handles asynchronous memory load and store operations and waits for response
// > Each thread in each core has it's own LSU
// > LDR, STR instructions are executed here
module lsu (
    input wire clk,
    input wire reset,
    input wire enable,					                    //Thread active enable

    input reg [2:0] core_state,				                //Core pipeline state

    // Memory Control Signals //
    input reg decoded_mem_read_enable,			            //Enable load request
    input reg decoded_mem_write_enable,		                //Enable store request

    // Registers //
    input reg [7:0] rs,					                    //Address / source
    input reg [7:0] rt,					                    //Store data

    // Data Memory //
    output wire mem_read_valid,			                    //Read request valid
    output wire [7:0] mem_read_address,			            //Read address
    input reg mem_read_ready,				                //Read response ready
    input reg [7:0] mem_read_data,			                //Read response data
    output wire mem_write_valid,			                //Write request valid
    output wire [7:0] mem_write_address,			        //Write address
    output wire [7:0] mem_write_data,			            //Write data
    input reg mem_write_ready,				                //Write response ready

    // LSU Outputs //
    output wire [1:0] lsu_state,				            //Visible LSU state
    output wire [7:0] lsu_out				                //Writeback data (loads only)
);

    localparam IDLE = 2'b00;				                //No LSU activity

    // Load Submodule Wires //
    wire [1:0] load_state;				                    //Load FSM state
    wire [7:0] load_out;					                //Load result

    // Load Submodule //
    LSU_Load u_load(
        .clk(clk),
        .reset(reset),
        .enable(enable),
        .core_state(core_state),
        .decoded_mem_read_enable(decoded_mem_read_enable),
        .rs(rs),
        .mem_read_valid(mem_read_valid),
        .mem_read_address(mem_read_address),
        .mem_read_ready(mem_read_ready),
        .mem_read_data(mem_read_data),
        .load_state(load_state),
        .load_out(load_out)
    );

    // Store Submodule Wires //
    wire [1:0] store_state;				                    //Store FSM state

    // Store Submodule //
    LSU_Store u_store(
        .clk(clk),
        .reset(reset),
        .enable(enable),
        .core_state(core_state),
        .decoded_mem_write_enable(decoded_mem_write_enable),
        .rs(rs),
        .rt(rt),
        .mem_write_valid(mem_write_valid),
        .mem_write_address(mem_write_address),
        .mem_write_data(mem_write_data),
        .mem_write_ready(mem_write_ready),
        .store_state(store_state)
    );

    assign lsu_out = load_out;				                //Only loads produce writeback data

    // State Output Select//
    assign lsu_state = decoded_mem_read_enable?load_state :	//Prefer load state when active
                     decoded_mem_write_enable?store_state :	//Else store state when active
                     IDLE;


endmodule