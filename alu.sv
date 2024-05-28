module alu (a, b, ALUControl, Result, ALUFlags, clk);
	
	input logic [31:0] a, b;
	input logic [1:0] ALUControl;
	output logic [31:0] Result;
	output logic [3:0] ALUFlags;
	input logic clk;
	
	logic [32:0] temp;
	
	initial ALUFlags = 4'b0000;
	assign temp = {1'b0, a} + {1'b0, b};


	always_comb begin
	
		case (ALUControl)
			2'b00	:	begin	//add
					Result = a + b;
			end
			
			2'b01	:	begin	//minus
					Result = a + ~b + 1;
			end
			
			2'b10	:	begin	//AND
					Result = a & b;
			end
			
			2'b11	:	begin	//OR
					Result = a | b;
			end
			
		endcase
	end

	always_comb begin
//			//negative
//			if (Result[31] == 1)
//				ALUFlags[3] = 1;
//			else
//				ALUFlags[3] = 0;
//			
//			//zero
//			if (Result == 0)
//				ALUFlags[2] = 1;
//			else
//				ALUFlags[2] = 0;
//				
//			
//			if (ALUControl[1] == 0) begin
//				//carry is ab + cin (a^b)
//				if ((a & b) | (ALUControl[0] & (a ^ b)))
//					ALUFlags[1] = 1;
//				//overflow is a & b have matching sign but result doesn't
//				if ((a[31] & b[31]) & ((a[31] != Result[31]) | (b[31] != Result[31])))
//					ALUFlags[0] = 1;
//			end else begin
//				ALUFlags[1] = 0;
//				ALUFlags[0] = 0;
//			end //if

		
		ALUFlags[3] = Result[31];		//negative
		ALUFlags[2] = &(~Result);		//zero
		ALUFlags[1] = ~ALUControl[1] & temp[32];	//carry over
		ALUFlags[0] = ~ALUControl[1] & (a[31] ^ Result[31]) & ~(a[31] ^ ALUControl[0] ^ b[31]);	//overflow
	end //always block
		

endmodule

module alu_testbench();

	logic [31:0] a, b;
	logic [1:0] ALUControl;
	logic [31:0] Result;
	logic [3:0] ALUFlags;
	logic [103:0] testvectors [1000:0];
	logic clk;
	
	alu dut (.a, .b, .ALUControl, .Result, .ALUFlags, .clk);
	
	parameter CLOCK_PERIOD = 100;
	
	initial clk = 1;
	always begin
		#(CLOCK_PERIOD / 2);
		clk = ~clk;
	end
	
	initial begin
		$readmemh("alu.tv", testvectors);
		
		for (int i = 0; i < 18; i++) begin
			{ALUControl, a, b, Result, ALUFlags} = testvectors[i];	repeat(3) @(posedge clk);
			ALUFlags = 4'b0000;
		end
			
		
	$stop;
	end
endmodule 