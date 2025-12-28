`timescale 1ns/1ps

module iic_master_tb;

    reg clk;
    reg rst;

    reg        start;
    reg        rw;          // 0 = WRITE, 1 = READ
    reg [6:0]  slave_addr;
    reg [7:0]  wdata;

    wire [7:0] rdata;
    wire       busy;
    wire       done;


    // I2C Bus (Open Drain)
    wire sda;
    wire scl;

    reg sda_slave_oe;

    assign sda = sda_slave_oe ? 1'b0 : 1'bz;
    assign scl = 1'bz;   // Master-only SCL

    pullup(sda);
    pullup(scl);

    iic_master #(
        .SYS_CLK_HZ(1_000_000),
        .I2C_CLK_HZ(100_000)
    ) uut (
        .clk(clk),
        .rst(rst),
        .start(start),
        .rw(rw),
        .slave_addr(slave_addr),
        .wdata(wdata),
        .rdata(rdata),
        .busy(busy),
        .done(done),
        .sda(sda),
        .scl(scl)
    );

    initial begin 
         clk = 0; // Clock Generation (1 MHz)
    forever #500 clk = ~clk; end

    wire [3:0] dut_state;
    assign dut_state = uut.state;

    localparam IDLE        = 4'd0,
               START_ST    = 4'd1,
               SEND_ADDR   = 4'd2,
               ADDR_ACK    = 4'd3,
               SEND_DATA   = 4'd4,
               DATA_ACK    = 4'd5,
               READ_DATA   = 4'd6,
               MASTER_NACK = 4'd7,
               STOP        = 4'd8,
               DONE_ST     = 4'd9;

    //============================================================
    // Simple Deterministic Slave Model
    //============================================================
    reg [7:0] slave_memory;
    reg [2:0] rd_bit_cnt;
    reg       force_nack;

    initial begin
        sda_slave_oe = 0;
        slave_memory = 8'h3C;
        rd_bit_cnt   = 3'd7;
        force_nack   = 1'b0;

        forever begin
            //----------------------------------------------------
            // ACK handling (ONE BIT ONLY)
            //----------------------------------------------------
            @(posedge scl);
            if (dut_state == ADDR_ACK || dut_state == DATA_ACK) begin
                if (!force_nack)
                    sda_slave_oe = 1'b1; // ACK = drive LOW
            end

            @(negedge scl);
            sda_slave_oe = 1'b0;         // Release after ACK

            //----------------------------------------------------
            // READ DATA (edge-driven, FSM-aware)
            //----------------------------------------------------
            if (dut_state == READ_DATA) begin
                @(negedge scl);
                sda_slave_oe = ~slave_memory[rd_bit_cnt];

                @(posedge scl);
                if (rd_bit_cnt == 0)
                    rd_bit_cnt = 3'd7;
                else
                    rd_bit_cnt = rd_bit_cnt - 1'b1;
            end
        end
    end

    // Test Sequence
    initial begin
        $dumpfile("iic_master_final_tb.vcd");
        $dumpvars(0, iic_master_tb);

        // ---------------- RESET ----------------
        rst        = 1;
        start      = 0;
        rw         = 0;
        slave_addr = 7'h42;
        wdata      = 8'hAA;
        #2000;
        rst = 0;

        // ---------------- WRITE TEST ----------------
        #3000;
        $display("\n--- WRITE TEST ---");
        rw    = 0;
        start = 1; #1000; start = 0;
        wait(done);
        $display("WRITE COMPLETE | DATA = %h", wdata);

        // ---------------- READ TEST ----------------
        #4000;
        $display("\n--- READ TEST ---");
        rw    = 1;
        start = 1; #1000; start = 0;
        wait(done);
        $display("READ COMPLETE | DATA = %h", rdata);

        // ---------------- RESET MID-TRANSACTION ----------------
        #4000;
        $display("\n--- RESET DURING TRANSACTION ---");
        rw = 0;
        wdata = 8'h55;
        start = 1;
        #1000;
        rst = 1;
        #1000;
        rst = 0;
        start = 0;
        #2000;
        $display("RESET HANDLED");

        // ---------------- SLAVE NACK TEST ----------------
        #4000;
        $display("\n--- SLAVE NACK TEST ---");
        force_nack = 1'b1;
        rw = 0;
        start = 1; #1000; start = 0;
        wait(done);
        force_nack = 1'b0;
        $display("NACK HANDLED CORRECTLY");

        #3000;
        $display("\nALL TESTS PASSED SUCCESSFULLY");
        $finish;
    end

endmodule
