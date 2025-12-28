`timescale 1ns / 1ps

module iic_master #(
    parameter SYS_CLK_HZ = 1_000_000,
    parameter I2C_CLK_HZ = 100_000
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        start,
    input  wire        rw,          // 0 = WRITE, 1 = READ
    input  wire [6:0]  slave_addr,
    input  wire [7:0]  wdata,
    output reg  [7:0]  rdata,
    output reg         busy,
    output reg         done,
    inout  wire        sda,
    inout  wire        scl
);

    //============================================================
    // Clock Divider
    //============================================================
    localparam integer DIV = SYS_CLK_HZ / (I2C_CLK_HZ * 2);
    reg [$clog2(DIV)-1:0] div_cnt;
    reg scl_req;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            div_cnt <= 0;
            scl_req <= 1'b1;
        end
        else if (div_cnt == DIV-1) begin
            div_cnt <= 0;
            scl_req <= ~scl_req;
        end
        else begin
            div_cnt <= div_cnt + 1'b1;
        end
    end

    //============================================================
    // Open-drain I2C Bus
    //============================================================
    reg sda_oe;
    assign sda = sda_oe ? 1'b0 : 1'bz;
    assign scl = scl_req ? 1'bz : 1'b0;

    pullup(sda);
    pullup(scl);

    wire scl_bus;
    assign scl_bus = scl;

    //============================================================
    // FSM States
    //============================================================
    localparam IDLE        = 4'd0,
               START       = 4'd1,
               SEND_ADDR   = 4'd2,
               ADDR_ACK    = 4'd3,
               SEND_DATA   = 4'd4,
               DATA_ACK    = 4'd5,
               READ_DATA   = 4'd6,
               MASTER_NACK = 4'd7,
               STOP        = 4'd8,
               DONE        = 4'd9;

    reg [3:0] state;
    reg [7:0] shift_reg;
    reg [2:0] bit_cnt;

    //============================================================
    // FSM
    //============================================================
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state   <= IDLE;
            sda_oe  <= 1'b0;
            busy    <= 1'b0;
            done    <= 1'b0;
            bit_cnt <= 3'd0;
            rdata   <= 8'd0;
        end
        else begin
            done <= 1'b0;

            case (state)

            // IDLE: Wait for start
            IDLE: begin
                busy   <= 1'b0;
                sda_oe <= 1'b0;
                if (start) begin
                    busy      <= 1'b1;
                    shift_reg <= {slave_addr, rw};
                    bit_cnt   <= 3'd7;
                    state     <= START;
                end
            end

            // START: SDA goes LOW while SCL HIGH
            START: begin
                if (scl_bus) begin
                    sda_oe <= 1'b1;
                    state  <= SEND_ADDR;
                end
            end

            // SEND_ADDR: Shift out 7-bit address + R/W
            SEND_ADDR: begin
                if (!scl_bus)
                    sda_oe <= ~shift_reg[bit_cnt];
                else begin
                    if (bit_cnt == 0)
                        state <= ADDR_ACK;
                    else
                        bit_cnt <= bit_cnt - 1'b1;
                end
            end

            // ADDR_ACK: Sample ACK/NACK from slave
            ADDR_ACK: begin
                if (!scl_bus)
                    sda_oe <= 1'b0;  // Release SDA for slave ACK
                else begin
                    if (sda == 1'b0) begin  // ACK
                        bit_cnt   <= 3'd7;
                        shift_reg <= wdata;
                        state     <= rw ? READ_DATA : SEND_DATA;
                    end
                    else begin              // NACK
                        state <= STOP;
                    end
                end
            end

            // SEND_DATA: Transmit data bits
            SEND_DATA: begin
                if (!scl_bus)
                    sda_oe <= ~shift_reg[bit_cnt];
                else begin
                    if (bit_cnt == 0)
                        state <= DATA_ACK;
                    else
                        bit_cnt <= bit_cnt - 1'b1;
                end
            end

            // DATA_ACK: Sample ACK/NACK from slave
            DATA_ACK: begin
                if (!scl_bus)
                    sda_oe <= 1'b0;  // Release SDA
                else begin
                    // Minimal single-byte write
                    state <= STOP;
                end
            end

            // READ_DATA: Capture data from slave
            READ_DATA: begin
                if (!scl_bus)
                    sda_oe <= 1'b0;
                else begin
                    rdata[bit_cnt] <= sda;
                    if (bit_cnt == 0)
                        state <= MASTER_NACK;
                    else
                        bit_cnt <= bit_cnt - 1'b1;
                end
            end

            // MASTER_NACK: Master sends NACK after single-byte read
            MASTER_NACK: begin
                if (!scl_bus)
                    sda_oe <= 1'b0; // SDA HIGH → NACK
                else
                    state <= STOP;
            end

            // STOP: SDA LOW→HIGH while SCL HIGH
            STOP: begin
                sda_oe <= 1'b1;   // Drive LOW first
                if (scl_bus) begin
                    sda_oe <= 1'b0; // Release → HIGH
                    state <= DONE;
                end
            end

            // DONE: Transaction complete
            DONE: begin
                busy  <= 1'b0;
                done  <= 1'b1;
                state <= IDLE;
            end

            default: state <= IDLE;

            endcase
        end
    end

endmodule 