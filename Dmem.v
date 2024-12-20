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