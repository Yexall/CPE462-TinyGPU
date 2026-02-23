`default_nettype none
`timescale 1ns/1ns

// PROGRAM COUNTER
// > Calculates the next PC for each thread to update to (but currently we assume all threads
//   update to the same PC and don't support branch divergence)
// > Currently, each thread in each core has it's own calculation for next PC
// > The NZP register value is set by the CMP instruction (based on >/=/< comparison) to 
//   initiate the BRnzp instruction for branching
module pc #(
    parameter DATA_MEM_DATA_BITS = 8,
    parameter PROGRAM_MEM_ADDR_BITS = 8
) (
    input wire clk,
    input wire reset,
    input wire enable,

    // State //
    input reg [2:0] core_state,

    // Control Signals //
    input reg [2:0] decoded_nzp,                              //PZN mask
    input reg [DATA_MEM_DATA_BITS - 1:0] decoded_immediate,
    input reg decoded_nzp_write_enable,
    input reg decoded_pc_mux,

    // ALU Output //
    input reg [DATA_MEM_DATA_BITS - 1:0] alu_out,               //LSBs hold flags

    // Current And Next PCs //
    input reg [PROGRAM_MEM_ADDR_BITS - 1:0] current_pc,
    output wire [PROGRAM_MEM_ADDR_BITS - 1:0] next_pc
);

    // State Encoding //
    localparam logic [2:0] EXECUTE = 3'b101;				    //Latch next_pc on EXECUTE
    localparam logic [2:0] UPDATE = 3'b110;				        //Write NZP on UPDATE

    // Flag Reorder //
    wire [2:0] alu_nzp = {alu_out[0], alu_out[1], alu_out[2]};  //Convert P,Z,N to N,Z,P

    // NZP Register //
    wire [2:0] nzp_flags;							            //Current NZP flags
    wire nzp_we = enable && (core_state == UPDATE) && decoded_nzp_write_enable;
    PC_NZPRegister u_nzp (
        .CLK(clk),
        .RST(reset),
        .NZPWE(nzp_we),
        .ALUOut(alu_nzp),
        .NZPOut(nzp_flags)
    );

    // PC Plus One //
    wire [PROGRAM_MEM_ADDR_BITS  -  1:0] pc_plus1;					//Sequential PC
    PC_PlusOne #(.PC_WIDTH(PROGRAM_MEM_ADDR_BITS)) u_inc (
        .CurrentPC(current_pc),
        .NextPC(pc_plus1)
    );

    // Branch Candidate //
    wire nzp_match = |(decoded_nzp & nzp_flags);				//True if any requested flag is set
    wire [PROGRAM_MEM_ADDR_BITS  -  1:0] branch_pc;					//Branch PC
    PC_MuxOne #(.PC_WIDTH(PROGRAM_MEM_ADDR_BITS)) u_mux1 (
        .NZPSelect(nzp_match),
        .CurrentPCPlus(pc_plus1),
        .Immediate(decoded_immediate[PROGRAM_MEM_ADDR_BITS - 1:0]),
        .Branch(branch_pc)
    );

    // Next PC Select //
    wire [PROGRAM_MEM_ADDR_BITS  -  1:0] next_pc_comb;				//Combinational next_pc
    PC_MuxTwo #(.PC_WIDTH(PROGRAM_MEM_ADDR_BITS)) u_mux2 (
        .PCMux(decoded_pc_mux),
        .Branch(branch_pc),
        .CurrentPCPlus(pc_plus1),
        .NextPC(next_pc_comb)
    );

    reg [PROGRAM_MEM_ADDR_BITS  -  1:0] next_pc_r;					//Registered next_pc

    always_ff @(posedge clk) begin
        if (reset) begin
            next_pc_r <= 0;							        //Reset next_pc
        end else if (enable) begin
            if (core_state == EXECUTE) begin
                next_pc_r <= next_pc_comb;					    //Latch next_pc on EXECUTE
            end
        end
    end

    assign next_pc = next_pc_r;							        //Drive next_pc output

endmodule
