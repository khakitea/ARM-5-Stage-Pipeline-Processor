/* top is a structurally made toplevel module. It consists of 3 instantiations, as well as the signals that link them. 
** It is almost totally self-contained, with no outputs and two system inputs: clk and rst. clk represents the clock 
** the system runs on, with one instruction being read and executed every cycle. rst is the system reset and should 
** be run for at least a cycle when simulating the system.
*/

// clk - system clock
// rst - system reset. Technically unnecessary
module top(
    input logic clk, rst
);
    
    // processor io signals
    logic [31:0] Instr;
    logic [31:0] ReadData;
    logic [31:0] WriteData;
    logic [31:0] PC, ALUResult;
    logic        MemWrite;

    // our single cycle arm processor
    arm processor (
        .clk        (clk        ), 
        .rst        (rst        ),
        .Instr      (Instr      ),
        .ReadDataM   (ReadData   ),
        .WriteDataM  (WriteData  ), 
        .PCF         (PC         ), 
        .ALUOutM  (ALUResult  ),
        .MemWriteM   (MemWrite   )
    );

    // instruction memory
    // contained machine code instructions which instruct processor on which operations to make
    // effectively a rom because our processor cannot write to it
    imem imemory (
        .addr   (PC     ),
        .instr  (Instr  )
    );

    // data memory
    // containes data accessible by the processor through ldr and str commands
    dmem dmemory (
        .clk     (clk       ), 
        .wr_en   (MemWrite  ),
        .addr    (ALUResult ),
        .wr_data (WriteData ),
        .rd_data (ReadData  )
    );


endmodule

module top_testbench();
	logic        clk, rst;
	logic [31:0] Instr;
	logic [31:0] ReadData;
	logic [31:0] WriteData;
	logic [31:0] PC, ALUResult;
	logic        MemWrite;

	top cpu(.*);
	
	// Clock setup
	parameter clock_period = 100;
	initial begin
		clk <= 0;
		forever #(clock_period /2) clk <= ~clk;
	end
	
	
	
	
	initial begin
        // start with a basic reset
        rst = 1; @(posedge clk);
        rst <= 0; @(posedge clk);

        // repeat for 50 cycles. Not all 50 are necessary, however a loop at the end of the program will keep anything weird from happening
        repeat(50) @(posedge clk);
		  

        // task 2:
        assert(cpu.processor.u_reg_file.memory[8] == 32'd1)  $display("Task 2 Passed");
        else $display("Task 2 Failed");
		
		$stop;
	end 
	
	
	
	
endmodule 