module Dmem(
    input clk,
    input [13:0] addr,
    input [31:0] write_data,
    input MemRead,
    input MemWrite,
    output reg [31:0] read_data
);

    reg [7:0] mem[0:16383];

    // Read from data.txt
    initial begin
        $readmemh("data.txt", mem);
    end

    always @(*) begin
        if (MemRead) begin
            read_data = {mem[addr+3], mem[addr+2], mem[addr+1], mem[addr]};
        end
        else begin
            read_data = {32{1'bz}};
        end
    end

    always @(posedge clk) begin
        if(MemWrite) begin
            mem[addr] <= write_data[7:0];
            mem[addr+1] <= write_data[15:8];
            mem[addr+2] <= write_data[23:16];
            mem[addr+3] <= write_data[31:24];
        end
    end

endmodule