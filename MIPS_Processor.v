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