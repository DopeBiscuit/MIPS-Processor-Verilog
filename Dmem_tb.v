module Dmem_tb();

reg [13: 0] addr;
reg [31: 0] write_data;
reg MemRead;
reg MemWrite;
wire [31: 0] read_data;

Dmem Dmem (
    .addr(addr),
    .write_data(write_data),
    .MemRead(MemRead),
    .MemWrite(MemWrite),
    .read_data(read_data)
);

initial begin: test
    integer i;
    for(i = 0;i < 10; i = i + 1) begin
        addr = i;
        write_data = i * 2;
        MemRead = 0;
        MemWrite = 1;
        #10;
    end

    for(i = 0;i < 10; i = i + 1) begin
        addr = i;
        write_data = 0;
        MemRead = 1;
        MemWrite = 0;
        #10;
    end
end

initial begin
    $monitor("addr=%h, write_data=%h, MemRead=%b, MemWrite=%b, read_data=%h", addr, write_data, MemRead, MemWrite, read_data);
end

endmodule