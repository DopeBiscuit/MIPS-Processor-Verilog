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
    wire Branch;


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
        .RegWrite(RegWrite)
    );

    alu_controller alu_ctrl(
        .funct(funct),
        .ALUOp(ALUOp),
        .ALUControl(ALUControl)
    );

    assign pc_src = Branch && Zero;

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
    output reg RegWrite
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
        end
        // SUBI
        6'b001001: begin
            RegDst = 1'b0;
            Jump = 1'b0;
            Branch = 1'b0;
            MemRead = 1'b0;
            MemtoReg = 1'b0;
            ALUOp = 2'b01;
            MemWrite = 1'b0;
            ALUSrc = 1'b1;
            RegWrite = 1'b1;
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