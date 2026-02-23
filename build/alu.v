module ALU (
	clk,
	reset,
	enable,
	core_state,
	decoded_alu_arithmetic_mux,
	decoded_alu_output_mux,
	rs,
	rt,
	alu_out
);
	parameter signed [31:0] WIDTH = 8;
	parameter [2:0] EXECUTE = 3'b101;
	input wire clk;
	input wire reset;
	input wire enable;
	input wire [2:0] core_state;
	input wire [1:0] decoded_alu_arithmetic_mux;
	input wire decoded_alu_output_mux;
	input wire [WIDTH - 1:0] rs;
	input wire [WIDTH - 1:0] rt;
	output reg [WIDTH - 1:0] alu_out;
	wire [WIDTH - 1:0] arith_out;
	wire [2:0] cmp_nzp;
	wire [WIDTH - 1:0] alu_bus_comb;
	ALU_Arithmetic #(.WIDTH(WIDTH)) u_arith(
		.rs(rs),
		.rt(rt),
		.AluArithmeticMux(decoded_alu_arithmetic_mux),
		.ArithOut(arith_out)
	);
	ALU_Comparison #(
		.WIDTH(WIDTH),
		.SIGNED(1'b0)
	) u_cmp(
		.rs(rs),
		.rt(rt),
		.CmpOut(cmp_nzp)
	);
	ALU_Mux #(.WIDTH(WIDTH)) u_mux(
		.ALUOutputMux(decoded_alu_output_mux),
		.ArithOut(arith_out),
		.CmpOut(cmp_nzp),
		.ALUOut(alu_bus_comb)
	);
	always @(posedge clk)
		if (reset)
			alu_out <= 1'sb0;
		else if (enable && (core_state == EXECUTE))
			alu_out <= alu_bus_comb;
endmodule