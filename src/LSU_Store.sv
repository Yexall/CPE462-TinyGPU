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


module LSU_Store (
    input wire clk,
    input wire reset,
    input wire enable,					//Thread active enable

    input wire [2:0] core_state,				//Core pipeline state
    input wire decoded_mem_write_enable,		//Store enable

    input wire [7:0] rs,					//Store address
    input wire [7:0] rt,					//Store data

    output reg mem_write_valid,			//Write request valid
    output reg [7:0] mem_write_address,			//Write address
    output reg [7:0] mem_write_data,			//Write data
    input wire mem_write_ready,			//Memory ready/ack

    output reg [1:0] store_state			//Store FSM state
);

    // State Encoding //
    localparam IDLE = 2'b00, REQUESTING = 2'b01, WAITING = 2'b10, DONE = 2'b11;	//Store FSM states

    // Sequential Fsm //
    always@ (posedge clk) begin
        if (reset) begin
            store_state <= IDLE;				    //Reset FSM
            mem_write_valid <= 1'b0;				//Deassert write valid
            mem_write_address <= 8'h00;			    //Clear address
            mem_write_data <= 8'h00;				//Clear data
        end else if (enable) begin
            if (decoded_mem_write_enable) begin
                case (store_state)
                    IDLE: begin
                        if (core_state == 3'b011) begin
                            store_state <= REQUESTING;		//Arm request on REQUEST phase
                        end
                    end
                    REQUESTING: begin
                        mem_write_valid <= 1'b1;			//Raise write valid
                        mem_write_address <= rs;			//Drive address from rs
                        mem_write_data <= rt;			    //Drive data from rt
                        store_state <= WAITING;			    //Wait for memory ack
                    end
                    WAITING: begin
                        if (mem_write_ready) begin
                            mem_write_valid <= 1'b0;		//Drop valid after ack
                            store_state <= DONE;			//Mark done until UPDATE phase
                        end
                    end
                    DONE: begin
                        if (core_state == 3'b110) begin
                            store_state <= IDLE;			//Return idle on UPDATE phase
                        end
                    end
                endcase
            end
        end
    end
endmodule
