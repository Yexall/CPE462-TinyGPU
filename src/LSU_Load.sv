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


module LSU_Load (
    input wire clk,
    input wire reset,
    input wire enable,					    //Thread active enable

    input wire [2:0] core_state,	        //Core pipeline state
    input wire decoded_mem_read_enable,		//Load enable

    input wire [7:0] rs,					//Load address

    output reg mem_read_valid,				//Read request valid
    output reg [7:0] mem_read_address,	    //Read address
    input wire mem_read_ready,				//Memory ready w/ read data
    input wire [7:0] mem_read_data,			//Read data bus

    output reg [1:0] load_state,			//Load FSM state
    output reg [7:0] load_out				//Captured load result
);

    // State Encoding //
    localparam IDLE = 2'b00, REQUESTING = 2'b01, WAITING = 2'b10, DONE = 2'b11;	//Load FSM states

    // Sequential Fsm//
    always@ (posedge clk) begin
        if (reset) begin
            load_state <= IDLE;				            //Reset FSM
            load_out <= 8'h00;				            //Clear output data
            mem_read_valid <= 1'b0;				        //Deassert read valid
            mem_read_address <= 8'h00;			        //Clear address
        end else if (enable) begin
            if (decoded_mem_read_enable) begin
                case (load_state)
                    IDLE: begin
                        if (core_state == 3'b011) begin
                            load_state <= REQUESTING;   //Arm request on REQUEST phase
                        end
                    end
                    REQUESTING: begin
                        mem_read_valid <= 1'b1;			//Raise read valid
                        mem_read_address <= rs;			//Drive address from rs
                        load_state <= WAITING;			//Wait for memory response
                    end
                    WAITING: begin
                        if (mem_read_ready) begin
                            mem_read_valid <= 1'b0;		//Drop valid after accept
                            load_out <= mem_read_data;	//Capture returned data
                            load_state <= DONE;			//Mark done until UPDATE phase
                        end
                    end
                    DONE: begin
                        if (core_state == 3'b110) begin
                            load_state <= IDLE;			//Return idle on UPDATE phase
                        end
                    end

                endcase
            end
        end
    end
endmodule
