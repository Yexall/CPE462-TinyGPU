`default_nettype none
`timescale 1ns/1ns

// REGISTER FILE
// > Each thread within each core has it's own register file with 13 free registers and 3 read-only registers
// > Read-only registers hold the familiar %blockIdx, %blockDim, and %threadIdx values critical to SIMD
module registers#(
    parameter THREADS_PER_BLOCK = 4,
    parameter THREAD_ID = 0,
    parameter DATA_BITS = 8
)(
    input wire clk,
    input wire reset,
    input wire enable,					        //Thread active enable

    // Kernel Execution //
    input reg [7:0] block_id,				    //%blockIdx value

    // State //
    input reg [2:0] core_state,				    //Core pipeline state

    // Instruction Signals //
    input reg [3:0] decoded_rd_address,			//Destination register
    input reg [3:0] decoded_rs_address,			//Source register A
    input reg [3:0] decoded_rt_address,			//Source register B

    // Control Signals //
    input reg decoded_reg_write_enable,			//Writeback enable
    input reg [1:0] decoded_reg_input_mux,		//Select writeback source
    input reg [DATA_BITS-1:0] decoded_immediate, //Immediate value

    // Thread Unit Outputs //
    input reg [DATA_BITS-1:0] alu_out,			//ALU result
    input reg [DATA_BITS-1:0] lsu_out,			//LSU result

    // Registers //
    output reg [7:0] rs,				            //Latched operand A
    output reg [7:0] rt				            //Latched operand B
);

// Parameter Definitions //
localparam logic [1:0] ARITHMETIC = 2'b00;
localparam logic [1:0] MEMORY = 2'b01;
localparam logic [1:0] CONSTANT = 2'b10;

localparam logic [2:0] ST_REQUEST = 3'b011;
localparam logic [2:0] ST_UPDATE = 3'b110;

// Register Storage //
reg [7:0] registers [0:15];			            //16 registers per thread
integer i;					                    //Loop iterator

// Sequential Logic //
always_ff@ (posedge clk) begin
    if (reset) begin

        //Reset Outputs//
        rs <= 0;					                //Clear rs
        rt <= 0;					                //Clear rt

        // Clear Free Registers //
        for (i = 0; i < 16; i = i + 1) begin
            registers[i] <= 0;			        //Initialize all registers
        end

        //Initialize Read-Only Registers//
        registers[14] <= THREADS_PER_BLOCK[7:0];  //%blockDim
        registers[15] <= THREAD_ID[7:0];		    //%threadIdx
    end else if (enable) begin

        // Update Context Registers //
        registers[13] <= block_id;		        //%blockIdx
        registers[14] <= THREADS_PER_BLOCK[7:0];	//%blockDim
        registers[15] <= THREAD_ID[7:0];	        //%threadIdx

        // Read Operands During Request //
        if (core_state == ST_REQUEST) begin
            rs <= registers[decoded_rs_address];	//Latch rs
            rt <= registers[decoded_rt_address];	//Latch rt
        end

        // Writeback During Update //
        if (core_state == ST_UPDATE) begin
            if (decoded_reg_write_enable && (decoded_rd_address < 4'd13)) begin
                unique case(decoded_reg_input_mux)
                    ARITHMETIC: registers[decoded_rd_address] <= alu_out;		    //ALU writeback
                    MEMORY: registers[decoded_rd_address] <= lsu_out;		        //Memory writeback
                    CONSTANT: registers[decoded_rd_address] <= decoded_immediate;	//Immediate writeback
                    default: registers[decoded_rd_address] <= registers[decoded_rd_address];   //Hold value
                endcase
            end
        end
    end
end
endmodule
