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