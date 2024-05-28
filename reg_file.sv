module reg_file(clk, wr_en, write_data, write_addr, read_addr1, read_addr2, read_data1, read_data2);

	input logic clk, wr_en;
	input logic [31:0] write_data;
	input logic [3:0] write_addr;
	input logic [3:0] read_addr1, read_addr2;
	output logic [31:0] read_data1, read_data2;

		logic [15:0][31:0] memory;
		
		assign read_data1 = memory[read_addr1];
		assign read_data2 = memory[read_addr2];

	always_ff @(posedge clk) begin
		if (wr_en) begin
			memory[write_addr] <= write_data;
		end //if
	end

endmodule

module reg_file_testbench();

	logic clk, wr_en;
	logic [31:0] write_data;
	logic [3:0] write_addr;
	logic [3:0] read_addr1, read_addr2;
	logic [31:0] read_data1, read_data2;
	
	reg_file dut (.clk(clk), .wr_en(wr_en), .write_data(write_data), .write_addr(write_addr), .read_addr1(read_addr1), .read_addr2(read_addr2), .read_data1(read_data1), .read_data2(read_data2));
	
	parameter CLOCK_PERIOD = 100;
	
	initial begin
		clk <= 0;
		forever	#(CLOCK_PERIOD/2) clk <= ~ clk;
	end
	
	initial begin
	
		//data written clock cycle after wr_en is asserted
		wr_en = 0;	write_data = 52;	@(posedge clk);
						write_addr = 1;			
		wr_en = 1;			  repeat(5) @(posedge clk);
		wr_en = 0;
		
		//read_data updated at the same time as read_addr
		read_addr1 = 1;	  repeat(5) @(posedge clk);
		read_addr2 = 2;	  repeat(5) @(posedge clk);
		
		//read_data updated the cycle after the write_addr given and data updated
		wr_en = 0;	write_addr = 2;	@(posedge clk);
						write_data = 62;	
		read_addr1 = 2;	  repeat(5) @(posedge clk);
		wr_en = 1;			  repeat(5) @(posedge clk);
		read_addr1 = 2;	  repeat(5) @(posedge clk);
		wr_en = 0;			  repeat(5) @(posedge clk);
		
		$stop;
		end
endmodule 