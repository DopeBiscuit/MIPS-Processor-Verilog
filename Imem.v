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