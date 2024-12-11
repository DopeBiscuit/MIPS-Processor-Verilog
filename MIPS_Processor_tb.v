module MIPS_Processor_tb;

    // Clock and reset
    reg clk;
    reg rst;

    // Instantiate the processor
    // Assuming your processor has clk and rst inputs
    MIPS_Processor uut (
        .clk(clk),
        .rst(rst)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 10 ns clock period
    end

    // Test sequence
    initial begin
        // Initialize reset
        rst = 1;
        #10;
        rst = 0;

        // Wait for program execution
        #200;

        // Finish simulation
        $finish;
    end

    // Memory initialization for IMEM
    initial begin
        // Instruction file contains various instructions to test processor functionality
        
        $readmemh("instructions.txt", uut.Imem.mem); 
        $readmemh("data.txt", uut.Dmem.mem); 
    end

    // Optional: Monitor signals
    initial begin
        $monitor($time, " PC=%h, Instruction=%h, alu_result =%d", uut.pc, uut.instruction, uut.alu_result);
    end

endmodule
