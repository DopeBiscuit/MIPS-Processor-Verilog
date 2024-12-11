module Imem(
    input [11:0] addr,
    output [31:0] instruction
);

    reg [7:0] mem[0:4095];

    integer i;
    // Read from instructions.txt
    initial begin
        // open "instructions.txt" file
        $readmemh("instructions.txt", mem);
    end

    assign instruction = {mem[addr+3], mem[addr+2], mem[addr+1], mem[addr]};

endmodule