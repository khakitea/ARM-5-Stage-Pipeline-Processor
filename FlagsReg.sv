module FlagsReg(clk, FlagWrite, addr, Flags, ALUFlags);
	
	input logic clk;
   input logic [1:0] FlagWrite;
   input logic [3:0] addr;
   input logic [3:0] Flags;
   output logic [3:0] ALUFlags;
                    
   logic [1:0] [3:0] memory; 
    
	assign ALUFlags = memory[addr[3:0]];
    
	 
	
   always_ff @(posedge clk) begin
        if (FlagWrite[1]) begin
            memory[addr[3:2]] <= Flags[3:2];
        end
        else if (FlagWrite[0]) begin
            memory[addr[1:0]] <= Flags[1:0]; 
        end
   end            
 

 
endmodule
