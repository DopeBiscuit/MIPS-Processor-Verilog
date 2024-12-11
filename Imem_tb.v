module Imem_tb();

    reg [11 : 0] addr;
    wire [31 : 0] instruction;

    Imem Imem(
        .addr(addr),
        .instruction(instruction)
    );

    integer i;
    initial begin
        for(i = 0; i < 20; i = i + 1) begin
            addr = i;
            #1;
        end
    end

    initial begin
        $monitor("addr = %d, instruction = %d", addr, instruction);
    end
endmodule