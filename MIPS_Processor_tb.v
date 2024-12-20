`timescale 1ns/1ns
module MIPS_Processor_tb;

    // Clock and reset
    reg clk;
    reg rst;

    // Instantiate the processor
    MIPS_Processor uut (
        .clk(clk),
        .rst(rst)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 10 ns clock period
    end

    /* 
        IMPORTANT: The test bench loads a sample program that test the following supported instruction:
        R-type:
        - ADD
        - SUB
        - AND
        - OR
        - SLT
        - NOR

        I-type:
        - ADDI
        
        J-type:
        - J

        Branch:
        - BEQ
        - BNE

        Memory: The memory is initialized from the attached project files, the memory
        is byte-addressable big endian, if you want to test it make sure you only write
        one byte per line in the data.txt and instructions.txt files.
    */

    /*
        Sample program:
        addi $t1, $zero, 0xc8  // t1 = 200
        addi $t2, $t1, -100  // t2 =  100

        add $t3, $t1, $t2   // t3 = 300
        and $t4, $t3, $t1   // t4 = 8
        or  $t5, $t2, $t4   // t5 = 108
        slt $t6, $t5, $t3   // t6 = -1
        nor $t7, $t6, $t1   // t7 = 0

        j helo
        add $t0, $zero, 5
        helo:

        bne $t3, $t1, lol
        addi $t0, $zero, 5
        lol:

        beq $t3, $t3, hey
        add $t0, $zero, 5
        hey:

        Final Values:
        t0 = 0, t1 = 200, t2 = 100, t3 = 300, t4 = 8, t5 = 108, t6 = -1, t7 = 0
    */

	reg [31:0] temp [0:12];

	initial begin
		temp[0] = 32'h200900C8;
		temp[1] = 32'h212AFF9C;
		temp[2] = 32'h012A5820;
		temp[3] = 32'h01696024;
		temp[4] = 32'h014C6825;
		temp[5] = 32'h01AB702A;
		temp[6] = 32'h01C97827;
		temp[7] = 32'h08000009;
		temp[8] = 32'h20080005;
		temp[9] = 32'h15690001;
		temp[10] = 32'h20080005;
		temp[11] = 32'h116B0001;
		temp[12] = 32'h20080005;
	end



    // Test sequence
    integer i;
    initial begin
        i = 0;
        // Initialize reset
        rst = 1;
        #10;
        rst = 0;

        // Wait for program execution
        #200;

        // Check final values of sample program
        if(uut.datapath.RF.registers[8] != 32'b0) begin
            $display("Error: t0 = %h, expected 0", uut.datapath.RF.registers[8]);
            i = 1;
        end
        if(uut.datapath.RF.registers[9] != 32'h000000c8) begin
            $display("Error: t1 = %h, expected 200", uut.datapath.RF.registers[9]);
            i = 1;
        end
        if(uut.datapath.RF.registers[10] != 32'h00000064) begin
            $display("Error: t2 = %h, expected 100", uut.datapath.RF.registers[10]);
            i = 1;
        end
        if(uut.datapath.RF.registers[11] != 32'h0000012c) begin
            $display("Error: t3 = %h, expected 300", uut.datapath.RF.registers[11]);
            i = 1;
        end
        if(uut.datapath.RF.registers[12] != 32'h00000008) begin
            $display("Error: t4 = %h, expected 8", uut.datapath.RF.registers[12]);
            i = 1;
        end
        if(uut.datapath.RF.registers[13] != 32'h0000006c) begin
            $display("Error: t5 = %h, expected 108", uut.datapath.RF.registers[13]);
            i = 1;
        end
        if(uut.datapath.RF.registers[14] != 32'hffffffff) begin
            $display("Error: t6 = %h, expected -1", uut.datapath.RF.registers[14]);
            i = 1;
        end
        if(uut.datapath.RF.registers[15] != 32'h00000000) begin
            $display("Error: t7 = %h, expected 0", uut.datapath.RF.registers[15]);
            i = 1;
        end
        if(i != 1) begin
            $display("All tests passed successfully!");
        end
        // Finish simulation
        $finish;
    end

	integer j;
    // Memory initialization for IMEM
    initial begin
        // Instruction file contains various instructions to test processor functionality
        for(j = 0; j < 12; j = j + 1) begin
			{uut.Imem.mem[j * 4], uut.Imem.mem[j * 4 + 1], uut.Imem.mem[j * 4 + 2], uut.Imem.mem[j * 4 + 3]} =  temp[j];
		end
        
		for(j = 0; j < 4096; j = j + 1) begin
			uut.Dmem.mem[j] = j ;
		end
    end

    // Optional: Monitor signals
    initial begin
        $monitor($time, " PC=%h, Instruction=%h, alu_result = %d", uut.pc, uut.instruction, uut.alu_result);
    end

endmodule




module MIPS_Processor(
    input clk,
    input rst
);

// Internal signals, implicit declaration gave errors in simulation
wire [31:0] instruction, read_data, write_data, alu_result;
wire [31:0] pc;
wire Zero, RegDst, Jump, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite, pc_src;
wire [3:0] ALUControl;

// Instruction Memory
Imem Imem(
    .addr(pc[11:0]),
    .instruction(instruction)
);

// Controller 
controller controller(
    .op_code(instruction[31:26]),
    .funct(instruction[5:0]),
    .Zero(Zero),
    .RegDst(RegDst),
    .Jump(Jump),
    .MemRead(MemRead),
    .MemtoReg(MemtoReg),
    .ALUControl(ALUControl),
    .MemWrite(MemWrite),
    .ALUSrc(ALUSrc),
    .RegWrite(RegWrite),
    .pc_src(pc_src)
);

// Datapath 
datapath datapath(
    .clk(clk),
    .rst(rst),
    .instruction(instruction),
    .read_data(read_data),
    .ALUControl(ALUControl),
    .RegDst(RegDst),
    .Jump(Jump),
    .pc_src(pc_src),
    .MemtoReg(MemtoReg),
    .ALUSrc(ALUSrc),
    .RegWrite(RegWrite),
    .Zero(Zero),
    .alu_result(alu_result),
    .write_data(write_data),
    .pc(pc)
);

// Data Memory
Dmem Dmem(
    .clk(clk),
    .addr(alu_result[13:0]),
    .write_data(write_data),
    .MemRead(MemRead),
    .MemWrite(MemWrite),
    .read_data(read_data)
);



endmodule

module Dmem(
    input clk,
    input [13:0] addr,
    input [31:0] write_data,
    input MemRead,
    input MemWrite,
    output reg [31:0] read_data
);

    // 16 k x 8-bit memory
    reg [7:0] mem[0:16383];


    always @(*) begin
        // Read data from memory if MemRead is enabled
        if (MemRead) begin
            read_data = {mem[addr], mem[addr+1], mem[addr+2], mem[addr+3]};
        end
        else begin
            // Output high impedance if MemRead is disabled
            read_data = {32{1'bz}};
        end
    end

    /*
        Make sure when writing to memory, you write the data in big-endian format.
        And when indexing the memory you index on word boundaries. (multiples of 4)
    */

    // Sequential write operation
    always @(posedge clk) begin
        // Write data to memory if MemWrite is enabled, 
        if(MemWrite) begin
            mem[addr] <= write_data[31:24];
            mem[addr+1] <= write_data[23:16];
            mem[addr+2] <= write_data[15:8];
            mem[addr+3] <= write_data[7:0];
        end
    end

endmodule

module Imem(
    input [11:0] addr,
    output [31:0] instruction
);
    // 4k x 8-bit memory
    reg [7:0] mem[0:4095];

    /*
        Big-endian memory scheme
    */

    // Read data from memory
    assign instruction = {mem[addr], mem[addr+1], mem[addr+2], mem[addr+3]};

endmodule

module controller(
    input [5:0] op_code,
    input [5:0] funct,
    input Zero,

    output wire RegDst,
    output wire Jump,
    output wire MemRead,
    output wire MemtoReg,
    output wire [3:0] ALUControl,
    output wire MemWrite,
    output wire ALUSrc,
    output wire RegWrite,
    output wire pc_src
);

    // Internal signals
    wire [1:0] ALUOp;
    wire Branch, BNE;


    decoder dec(
        .op_code(op_code),
        .RegDst(RegDst),
        .Jump(Jump),
        .Branch(Branch),
        .MemRead(MemRead),
        .MemtoReg(MemtoReg),    
        .ALUOp(ALUOp),
        .MemWrite(MemWrite),
        .ALUSrc(ALUSrc),
        .RegWrite(RegWrite),
        .BNE(BNE)
    );

    alu_controller alu_ctrl(
        .funct(funct),
        .ALUOp(ALUOp),
        .ALUControl(ALUControl)
    );

    assign pc_src = (Branch && Zero) || (BNE && !Zero); 

endmodule

module decoder(
    input [5:0] op_code,
    output reg RegDst,
    output reg Jump,
    output reg Branch,
    output reg MemRead,
    output reg MemtoReg,
    output reg [1:0] ALUOp,
    output reg MemWrite,
    output reg ALUSrc,
    output reg RegWrite,
    output reg BNE
);

always @(*) begin
    case(op_code)
        // R-type
        6'b000000: begin
            RegDst = 1'b1;
            Jump = 1'b0;
            Branch = 1'b0;
            MemRead = 1'b0;
            MemtoReg = 1'b0;
            ALUOp = 2'b10;
            MemWrite = 1'b0;
            ALUSrc = 1'b0;
            RegWrite = 1'b1;
            BNE = 1'b0;
        end
        // J-type
        6'b000010: begin
            RegDst = 1'b0;
            Jump = 1'b1;
            Branch = 1'b0;
            MemRead = 1'b0;
            MemtoReg = 1'b0;
            ALUOp = 2'b00;
            MemWrite = 1'b0;
            ALUSrc = 1'b0;
            RegWrite = 1'b0;
            BNE = 1'b0;
        end
        // BEQ
        6'b000100: begin
            RegDst = 1'b0;
            Jump = 1'b0;
            Branch = 1'b1;
            MemRead = 1'b0;
            MemtoReg = 1'b0;
            ALUOp = 2'b01;
            MemWrite = 1'b0;
            ALUSrc = 1'b0;
            RegWrite = 1'b0;
            BNE = 1'b0;
        end
        // BNE
        6'b000101: begin
            RegDst = 1'b0;
            Jump = 1'b0;
            Branch = 1'b0;
            MemRead = 1'b0;
            MemtoReg = 1'b0;
            ALUOp = 2'b01;
            MemWrite = 1'b0;
            ALUSrc = 1'b0;
            RegWrite = 1'b0;
            BNE = 1'b1;
        end
        // LW
        6'b100011: begin
            RegDst = 1'b0;
            Jump = 1'b0;
            Branch = 1'b0;
            MemRead = 1'b1;
            MemtoReg = 1'b1;
            ALUOp = 2'b00;
            MemWrite = 1'b0;
            ALUSrc = 1'b1;
            RegWrite = 1'b1;
            BNE = 1'b0;
        end
        // SW
        6'b101011: begin
            RegDst = 1'b0;
            Jump = 1'b0;
            Branch = 1'b0;
            MemRead = 1'b0;
            MemtoReg = 1'b0;
            ALUOp = 2'b00;
            MemWrite = 1'b1;
            ALUSrc = 1'b1;
            RegWrite = 1'b0;
            BNE = 1'b0;
        end
        // ADDI
        6'b001000: begin
            RegDst = 1'b0;
            Jump = 1'b0;
            Branch = 1'b0;
            MemRead = 1'b0;
            MemtoReg = 1'b0;
            ALUOp = 2'b00;
            MemWrite = 1'b0;
            ALUSrc = 1'b1;
            RegWrite = 1'b1;
            BNE = 1'b0;
        end
        default: begin
            RegDst = 1'b0;
            Jump = 1'b0;
            Branch = 1'b0;
            MemRead = 1'b0;
            MemtoReg = 1'b0;
            ALUOp = 2'b00;
            MemWrite = 1'b0;
            ALUSrc = 1'b0;
            RegWrite = 1'b0;
            BNE = 1'b0;
        end

    endcase
end
endmodule


module alu_controller(
    input [5:0] funct,
    input [1:0] ALUOp,
    output reg [3:0] ALUControl
);

always @(*) begin
    case(ALUOp)
        2'b00: ALUControl = 4'b0010; // ADD
        2'b01: ALUControl = 4'b0110; // SUB
        2'b10: begin
            case(funct)
                6'b100000: ALUControl = 4'b0010; // ADD
                6'b100010: ALUControl = 4'b0110; // SUB
                6'b100100: ALUControl = 4'b0000; // AND
                6'b100101: ALUControl = 4'b0001; // OR
                6'b101010: ALUControl = 4'b0111; // SLT
                6'b100111: ALUControl = 4'b1100; // NOR
                default: ALUControl = 4'b0010; // ADD
            endcase
        end
    endcase
end
endmodule

module datapath(
    input clk,
    input rst,
    input [31:0] instruction,       // Instruction from Imem
    input [31:0] read_data,         // Data from Dmem
    input [3:0] ALUControl,         // Control signal for ALU
    input RegDst,                   // Control signal for RegDst
    input Jump,                     // Control signal for Jump
    input pc_src,                   // Control signal for pc_src
    input MemtoReg,                 // Control signal for MemtoReg
    input ALUSrc,                   // Control signal for ALUSrc
    input RegWrite,                 // Control signal for RegWrite

    output wire Zero,                    // Zero flag
    output wire [31:0] alu_result,       // Result from ALU
    output wire [31:0] write_data,       // Data to Dmem
    output reg [31:0] pc                // Program Counter
);

// Internal signals
wire [31:0] read_data1, read_data2, rf_write_data, alu_input, imm_ext;
wire [4:0] write_reg;

wire [31:0] pc_plus_4;


    // RF write register address and write data
    assign write_reg = (RegDst) ? instruction[15:11] : instruction[20:16];
    assign rf_write_data = (MemtoReg) ? read_data : alu_result;

    // output
    assign write_data = read_data2;


    // select ALU input
    assign alu_input = (ALUSrc) ? imm_ext : read_data2;
    
    // pc_plus_4
    assign pc_plus_4 = (rst) ? 32'b0 : pc + 4;




// Instantiate register file
Register_File RF (
    .clk(clk),
    .rst(rst),
    .RegWrite(RegWrite),
    .ReadReg1(instruction[25:21]),
    .ReadReg2(instruction[20:16]),
    .WriteReg(write_reg),
    .WriteData(rf_write_data),
    .ReadData1(read_data1),
    .ReadData2(read_data2)
);




// ALU
ALU alu(
    .a(read_data1),
    .b(alu_input),
    .ALUControl(ALUControl),
    .result(alu_result),
    .Zero(Zero)
);

// Immediate extender
imm_extender imm(
    .imm(instruction[15:0]),
    .imm_ext(imm_ext)
);


// PC logic
always @(posedge clk or posedge rst) begin
    // Reset
    if(rst) begin
        pc <= 32'b0;
    end
    // Jump 
    else if(Jump) begin
        pc <= {pc_plus_4[31:28] ,instruction[25:0], 2'b00};
    end
    // Branch
    else if(pc_src) begin
        pc <= pc_plus_4 + (imm_ext << 2);
    end
    // Default
    else begin
        pc <= pc_plus_4;
    end
end

endmodule

module ALU(
    input [31:0] a,
    input [31:0] b,
    input [3:0] ALUControl,
    output reg [31:0] result,
    output reg Zero
);

always @(*) begin
    case(ALUControl)
        4'b0000: result = a & b;
        4'b0001: result = a | b;
        4'b0010: result = a + b;
        4'b0110: result = a - b;
        4'b0111: result = (a < b) ? 32'b1 : 32'b0;
        4'b1100: result = a << b;
        4'b1101: result = a >> b;
        default: result = 32'b0;
    endcase

    Zero = (result == 32'b0);
end

endmodule

module imm_extender(
    input [15:0] imm,
    output reg [31:0] imm_ext
);

always @(*) begin
    imm_ext = {{16{imm[15]}}, imm};
end

endmodule


module Register_File(
    input clk,
    input rst,
    input RegWrite,
    input [4:0] WriteReg,
    input [4:0] ReadReg1,
    input [4:0] ReadReg2,
    input [31:0] WriteData,
    output [31:0] ReadData1,
    output [31:0] ReadData2
);
    integer i;
    reg [31:0] registers [0:31];

    assign ReadData1 = registers[ReadReg1];
    assign ReadData2 = registers[ReadReg2];

    always @(posedge clk or posedge rst) begin
        if(rst) begin
            for(i = 0; i < 32; i = i + 1) begin
                registers[i] <= 0;
            end
        end
        else if(RegWrite) begin
            registers[WriteReg] <= WriteData;
        end
    end

endmodule