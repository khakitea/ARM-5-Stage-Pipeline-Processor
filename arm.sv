/* arm is the spotlight of the show and contains the bulk of the datapath and control logic. This module is split into two parts, the datapath and control. 
*/

// clk - system clock
// rst - system reset
// Instr - incoming 32 bit instruction from imem, contains opcode, condition, addresses and or immediates
// ReadData - data read out of the dmem
// WriteData - data to be written to the dmem
// MemWrite - write enable to allowed WriteData to overwrite an existing dmem word
// PC - the current program count value, goes to imem to fetch instruciton
// ALUResult - result of the ALU operation, sent as address to the dmem

module arm (
    input  logic        clk, rst,
    input  logic [31:0] Instr,
    input  logic [31:0] ReadDataM,
    output logic [31:0] WriteDataM, 
    output logic [31:0] PCF, ALUOutM,
    output logic        MemWriteM
);

    // datapath buses and signals
    logic [31:0] PCPrime, PCPlus4F, PCPlus8D, PCPlus8E; // pc signals
    logic [ 3:0] RA1, RA2;                  // regfile input addresses
    logic [31:0] RD1, RD2;                  // raw regfile outputs
    logic [ 3:0] ALUFlags;                  // alu combinational flag outputs
    logic [31:0] ExtImm, SrcA, SrcB;        // immediate and alu inputs 
    logic [31:0] ResultW;                   // computed or fetched value to be written into regfile or pc
	 logic [31:0] WriteData, ReadData;		  // write back and read back
	 logic [31:0] ALUResultE, ALUOutW;		  // 
	 logic [3:0] WA3D, WA3E, WA3M, WA3W;	  // addresses for write
	 logic [3:0] FlagsReg;						  // flags register

	 //-------------------------------------------------------------------------------
    // 										control signals
	 //-------------------------------------------------------------------------------
	 
	 // decode
    logic PCSrcD, MemtoRegD, ALUSrcD, RegWriteD, BranchD;
    logic [1:0] RegSrcD, ImmSrcD, ALUControlD;
	 logic [1:0] FlagWriteD;
	 logic MemWriteD;
	 
	 // execute
	 logic MemWriteE;
	 logic [1:0] FlagWriteE;
	 logic PCSrcE, MemtoRegE, ALUSrcE, RegWriteE, BranchE, CondExE;
	 logic [1:0] ALUControlE;
	 logic [3:0] CondE;
	 logic [3:0] FlagsE;
	 logic [3:0] RA1E, RA2E;
	 logic [31:0] RD1E, RD2E;
	 logic [31:0] InstrD, InstrF;
	 logic [31:0] WriteDataE;
	 logic [31:0] ExtImmE;
	 
	 // memory
	 logic PCSrcM, MemtoRegM, RegWriteM;
	 
	 //writeback
	 logic PCSrcW, MemtoRegW, RegWriteW;
	 
	 // hazards
	 logic Match_1E_M, Match_2E_M, Match_1E_W, Match_2E_W, Match_12D_E;
	 logic [1:0] ForwardAE, ForwardBE;
	 logic StallF, StallD, FlushE, FlushD, Ldrstall, LdrstallD, PCWrPendingF;


    /* The datapath consists of a PC as well as a series of muxes to make decisions about which data words to pass forward and operate on. It is 
    ** noticeably missing the register file and alu, which you will fill in using the modules made in lab 1. To correctly match up signals to the 
    ** ports of the register file and alu take some time to study and understand the logic and flow of the datapath.
    */
    //-------------------------------------------------------------------------------
    //                                      DATAPATH
    //-------------------------------------------------------------------------------

	 logic [31:0] tempMux;
	 logic BranchTakenE;
	 
	 assign InstrF = Instr;

    assign PCPrime = BranchTakenE ? ALUResultE : (PCSrcW ? ResultW : PCPlus8D);  // mux, use either default or newly computed value
    assign PCPlus4F = PCF + 'd4;                  // default value to access next instruction
    assign PCPlus8D = PCPlus4F;						  // value read when reading from reg[15]

    // update the PC, at rst initialize to 0
    always_ff @(posedge clk) begin
        if (rst) PCF <= '0;
		  else if (StallF) PCF <= PCF;
        else     PCF <= PCPrime;
    end

    // determine the register addresses based on control signals
    // RegSrc[0] is set if doing a branch instruction
    // RefSrc[1] is set when doing memory instructions
    assign RA1 = RegSrcD[0] ? 4'd15         : InstrD[19:16];
    assign RA2 = RegSrcD[1] ? InstrD[15:12] : InstrD[ 3: 0];

    // TODO: insert your reg file here
    // TODO: instantiation comment
    reg_file u_reg_file (
        .clk       (clk), 
        .wr_en     (RegWriteW),
        .write_data(ResultW),
        .write_addr(WA3W),
        .read_addr1(RA1), 
        .read_addr2(RA2),
        .read_data1(RD1), 
        .read_data2(RD2)
    );

    // two muxes, put together into an always_comb for clarity
    // determines which set of instruction bits are used for the immediate
    always_comb begin
        if      (ImmSrcD == 'b00) ExtImm = {{24{InstrD[7]}},InstrD[7:0]};          // 8 bit immediate - reg operations
        else if (ImmSrcD == 'b01) ExtImm = {20'b0, InstrD[11:0]};                 // 12 bit immediate - mem operations
        else                      ExtImm = {{6{InstrD[23]}}, InstrD[23:0], 2'b00}; // 24 bit immediate - branch operation
    end

    // WriteData and SrcA are direct outputs of the register file, wheras SrcB is chosen between reg file output and the immediate
//    assign WriteDataE = (RA2 == 'd15) ? PCPlus8D : RD2;           // substitute the 15th regfile register for PC 
//    assign SrcA       = (RA1 == 'd15) ? PCPlus8E : RD1;           // substitute the 15th regfile register for PC 
//    assign SrcB       = ALUSrcD        ? ExtImm  : WriteData;     // determine alu operand to be either from reg file or from immediate
	 assign SrcB = ALUSrcE ? ExtImmE : WriteDataE;

    // TODO: insert your alu here
    // TODO: instantiation comment
	 // ALU can add, subtract, OR, AND. ALU outputs the result and flags
	 
    alu u_alu (
        .a          (SrcA), 
        .b          (SrcB),
        .ALUControl (ALUControlE),
        .Result     (ALUResultE),
        .ALUFlags   (ALUFlags)
    );

    // determine the result to run back to PC or the register file based on whether we used a memory instruction
    assign ResultW = MemtoRegW ? ReadData : ALUOutW;    // determine whether final writeback result is from dmemory or alu


    /* The control conists of a large decoder, which evaluates the top bits of the instruction and produces the control bits 
    ** which become the select bits and write enables of the system. The write enables (RegWrite, MemWrite and PCSrc) are 
    ** especially important because they are representative of your processors current state. 
    */
    //-------------------------------------------------------------------------------
    //                                      CONTROL
    //-------------------------------------------------------------------------------
    
    always_ff begin
        casez (Instr[27:20])

            // ADD (Imm or Reg)
            8'b00?_0100_0 : begin   // note that we use wildcard "?" in bit 25. That bit decides whether we use immediate or reg, but regardless we add
                PCSrcD    = 0;
                MemtoRegD = 0; 
                MemWriteD = 0; 
                ALUSrcD   = Instr[25]; // may use immediate
                RegWriteD = 1;
                RegSrcD   = 'b00;
                ImmSrcD   = 'b00; 
                ALUControlD = 'b00;
					 FlagWriteD = 2'b00;
					 BranchD = 0;
            end

            // SUB (Imm or Reg)
            8'b00?_0010_0 : begin   // note that we use wildcard "?" in bit 25. That bit decides whether we use immediate or reg, but regardless we sub
                PCSrcD    = 0; 
                MemtoRegD = 0; 
                MemWriteD = 0; 
                ALUSrcD   = Instr[25]; // may use immediate
                RegWriteD = 1;
                RegSrcD   = 'b00;
                ImmSrcD   = 'b00; 
                ALUControlD = 'b01;
					 FlagWriteD = 2'b00;
					 BranchD = 0;
            end
				
				// CMP (Imm or Reg)
				8'b00?_0010_1 : begin   // note that we use wildcard "?" in bit 25. That bit decides whether we use immediate or reg, but regardless we sub 
					 PCSrcD =  0;
					 MemtoRegD = 0; 
					 MemWriteD = 0; 
					 ALUSrcD   = InstrD[25]; // may use immediate
					 RegWriteD = 0;
					 RegSrcD   = 'b00;
					 ImmSrcD   = 'b00; 
					 ALUControlD = 'b01;
					 FlagWriteD = 2'b11;
					 BranchD = 0;
				end

            // AND
            8'b000_0000_0 : begin
                PCSrcD    = 0; 
                MemtoRegD = 0; 
                MemWriteD = 0; 
                ALUSrcD   = 0; 
                RegWriteD = 1;
                RegSrcD   = 'b00;
                ImmSrcD   = 'b00;    // doesn't matter
                ALUControlD = 'b10;
					 FlagWriteD = 2'b00;
					 BranchD = 0;	 
            end

            // ORR
            8'b000_1100_0 : begin
                PCSrcD    = 0; 
                MemtoRegD = 0; 
                MemWriteD = 0; 
                ALUSrcD   = 0; 
                RegWriteD = 1;
                RegSrcD   = 'b00;
                ImmSrcD   = 'b00;    // doesn't matter
                ALUControlD = 'b11;
					 FlagWriteD = 2'b00;
					 BranchD = 0;
            end

            // LDR
            8'b010_1100_1 : begin
                PCSrcD    = 0; 
                MemtoRegD = 1; 
                MemWriteD = 0; 
                ALUSrcD   = 1;
                RegWriteD = 1;
                RegSrcD   = 'b10;    // msb doesn't matter
                ImmSrcD   = 'b01; 
                ALUControlD = 'b00;  // do an add
					 FlagWriteD = 2'b00;
					 BranchD = 0;
            end

            // STR
            8'b010_1100_0 : begin
                PCSrcD    = 0; 
                MemtoRegD = 0; // doesn't matter
                MemWriteD = 1; 
                ALUSrcD   = 1;
                RegWriteD = 0;
                RegSrcD   = 'b10;    // msb doesn't matter
                ImmSrcD   = 'b01; 
                ALUControlD = 'b00;  // do an add
					 FlagWriteD = 2'b00;
					 BranchD = 0;
            end

            // B
            8'b1010_???? : begin
                    PCSrcD    = 1; 
                    MemtoRegD = 0;
                    MemWriteD = 0; 
                    ALUSrcD   = 1;
                    RegWriteD = 0;
                    RegSrcD   = 'b01;
                    ImmSrcD   = 'b10; 
                    ALUControlD = 'b00;  // do an add
						  FlagWriteD = 2'b00;
					 //BranchD = 0;
            end

			default: begin
					PCSrcD    = 0; 
                    MemtoRegD = 0; // doesn't matter
                    MemWriteD = 0; 
                    ALUSrcD   = 0;
                    RegWriteD = 0;
                    RegSrcD   = 'b00;
                    ImmSrcD   = 'b00; 
                    ALUControlD = 'b00;  // do an add
						  FlagWriteD = 2'b00;
						  BranchD = 0;
			end
        endcase
		  
		  case(CondE)
				4'b0000 : CondExE = FlagsE[2];
				4'b0001 : CondExE = ~FlagsE[2];
				4'b1010 : CondExE = ~(FlagsE[3] ^ FlagsE[0]); 
				4'b1100 : CondExE = (~FlagsE[2] & ~(FlagsE[3] ^ FlagsE[0]));
				4'b1101 : CondExE = (FlagsE[2] | (FlagsE[3] ^ FlagsE[0]));
				4'b1011 : CondExE = (FlagsE[3] ^ FlagsE[0]);
				4'b1110 : CondExE = 1;
				default : CondExE = 0;
		  endcase
		  
    end
	 
	 
	 // decode
	 always_ff @(posedge clk) begin
			if (FlushD) InstrD <= '0;
			else if (StallD) InstrD <= InstrD;
			else InstrD <= InstrF;
	 end
	 
	 //execute
	 always_ff @(posedge clk) begin
		if (rst | FlushE) begin
			FlagWriteE <= 0;
			PCSrcE <= 0;
			MemtoRegE <= 0;
			RegWriteE <=0;
			BranchE <= 0;
			BranchTakenE <= 0;	
			CondE <= 0;
			ALUControlE <=0;
			FlagsE <= 0;
			RA1E <= 0;
			RA2E <= 0;
			WA3E <= 0;
			ExtImmE <= 0;
			PCPlus8E <= 0;
		end else begin
			PCSrcE <= PCSrcD;
			RegWriteE <= RegWriteD;
			MemtoRegE <= MemtoRegD;
			ALUControlE <= ALUControlD;
			BranchE <= BranchD;
			ALUSrcE <= ALUSrcD;
			FlagWriteE <= FlagWriteD;
			CondE <= InstrD[31:28];
			RA1E <= RA1;
			RA2E <= RA2;
			RD1E <= RD1;
			RD2E <= RD2;
			WA3E <= InstrD[15:12];
			ExtImmE <= ExtImm;
			PCPlus8E <= PCPlus8D;
			BranchTakenE <= CondExE & BranchE;
			if (FlagWriteE[1]) 
				FlagsE[3:2] <= ALUFlags[3:2];
			else if (FlagWriteE[0]) 
				FlagsE[1:0] <= ALUFlags[1:0];
		end
	 end
	 
	 //memory
	 always_ff @(posedge clk) begin
			if (rst) begin
				PCSrcM <= 0;
				RegWriteM <= 0;
				MemtoRegM <= 0;
				MemWriteM <= 0;
				WA3M <= 0;
				WriteData <= 0;
				ALUOutM <= 0;
			end	
			else begin
				PCSrcM <= PCSrcE & CondExE;
				RegWriteM <= RegWriteE & CondExE;
				MemtoRegM <= MemtoRegE;
				MemWriteM <= MemWriteE & CondExE;
				WA3M <= WA3E;
				WriteDataM <= WriteDataE;
				ALUOutM <= ALUResultE;
			end
	  end
	  
	  //writeback
	  always_ff @(posedge clk) begin
			if(rst) begin
				PCSrcW <= 0;
				RegWriteW <= 0;
				MemtoRegW <= 0;
				WA3W <= 0;
				ALUOutW <= 0;
				ReadData <= 0;
			end
			else begin
				PCSrcW <= PCSrcM;
				RegWriteW <= RegWriteM;
				MemtoRegW <= MemtoRegM;
				WA3W <= WA3M;
				ALUOutW <= ALUOutM;
				ReadData <= ReadDataM;
			end
		end
		
		
		
		always_comb begin
		// Match Memory stage registers
		Match_1E_M = (RA1E == WA3M);
		Match_2E_M = (RA2E == WA3M);
		
		// Matche Writeback stage registers
		Match_1E_W = (RA1E == WA3W);
		Match_2E_W = (RA2E == WA3W);
		
		// match -> forward	
		if (Match_1E_M & RegWriteM)
			ForwardAE = 2'b10;
		else if(Match_1E_W & RegWriteW)
			ForwardAE = 2'b01;
		else
			ForwardAE = 2'b00;	
			
		// match -> forward result	
		if (Match_2E_M & RegWriteM)
			ForwardBE = 2'b10;
		else if(Match_2E_W & RegWriteW)
			ForwardBE = 2'b01;
		else
			ForwardBE = 2'b00;	
			
	end
	
	
	//hazards
	// fetch hazards
	always_comb begin
		  Match_12D_E = (RA1 == WA3E) | (RA2 == WA3E);
		LdrstallD = Match_12D_E & MemtoRegE;
			PCWrPendingF = PCSrcD | PCSrcE | PCSrcM;
			StallF = LdrstallD | PCWrPendingF;
			StallD = LdrstallD;
			FlushE = LdrstallD | BranchTakenE;
			FlushD = PCWrPendingF | PCSrcW | BranchTakenE;
	end
	// execute hazards
	always_comb begin
			case(ForwardAE) 
				2'b00: SrcA <= (RA1E =='d15) ? PCPlus8E : RD1E;
				2'b01: SrcA <= ResultW;
				2'b10: SrcA <= ALUOutM;
				default: SrcA <= RD1E;
			endcase
	end
	
	always_comb begin
			case(ForwardBE)
				2'b00: WriteDataE <= (RA2E == 'd15) ? PCPlus8E:RD2E;
				2'b01: WriteDataE <= ResultW;
				2'b10: WriteDataE <= ALUOutM;
				default: WriteDataE <= RD2E;
			endcase
	end
			
			
			
endmodule

module arm_testbench();
		logic clk, rst;
		logic [31:0] Instr;
		logic [31:0] ReadDataM;
		logic [31:0] WriteDataM;
		logic [31:0] PCF, ALUOutM;
		logic MemWriteM;
		
		logic [31:0] SrcA, SrcB;       
		logic [31:0] ResultW;
		logic RegWriteD;
		
		
		logic [31:0] testvectors [1000:0];
		
		arm dut(.clk, .rst, .Instr, .ReadDataM, .WriteDataM, .PCF, .ALUOutM, .MemWriteM);
		
		// set clock
		parameter clock_period = 100;
		initial begin
			clk <= 0;
			forever #(clock_period /2) clk <= ~clk;
		end
		
		
		
		initial begin
			rst <= 0; 											@(posedge clk);
			rst <= 1; 											@(posedge clk);
			rst <= 0; 											@(posedge clk);
			$readmemb("memfile3.dat", testvectors);
			for(int i = 0; i < 30; i++) begin
				{Instr} = testvectors[i]; 				@(posedge clk);
				$display(ResultW); 							@(posedge clk);
																	@(posedge clk);
			end
			
			$stop;
		end
		
		
endmodule 