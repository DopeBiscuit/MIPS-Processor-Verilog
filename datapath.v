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