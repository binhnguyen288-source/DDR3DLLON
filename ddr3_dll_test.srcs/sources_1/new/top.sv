`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/20/2023 09:06:55 AM
// Design Name: 
// Module Name: top
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module oserdes_wrap(
    input rst_i,
    input clk,
    input clk_ddr,
    input D1,
    input D2,
    input D3,
    input D4,
    output DOUT
);
    OSERDESE2 #(
      .DATA_RATE_OQ("DDR"),   // DDR, SDR
      .DATA_RATE_TQ("SDR"),   // DDR, BUF, SDR
      .DATA_WIDTH(4),         // Parallel data width (2-8,10,14)
      .INIT_OQ(1'b0),         // Initial value of OQ output (1'b0,1'b1)
      .INIT_TQ(1'b0),         // Initial value of TQ output (1'b0,1'b1)
      .SERDES_MODE("MASTER"), // MASTER, SLAVE
      .SRVAL_OQ(1'b0),        // OQ output value when SR is used (1'b0,1'b1)
      .SRVAL_TQ(1'b0),        // TQ output value when SR is used (1'b0,1'b1)
      .TBYTE_CTL("FALSE"),    // Enable tristate byte operation (FALSE, TRUE)
      .TBYTE_SRC("FALSE"),    // Tristate byte source (FALSE, TRUE)
      .TRISTATE_WIDTH(1)      // 3-state converter width (1,4)
   )
   OSERDESE2_inst (
      .OQ(DOUT),               // 1-bit output: Data path output
      // SHIFTOUT1 / SHIFTOUT2: 1-bit (each) output: Data output expansion (1-bit each)
      
      
      .CLK(clk_ddr),             // 1-bit input: High speed clock
      .CLKDIV(clk),       // 1-bit input: Divided clock
      // D1 - D8: 1-bit (each) input: Parallel data inputs (1-bit each)
      .D1(D1),
      .D2(D2),
      .D3(D3),
      .D4(D4),
      .OCE(1'b1),             // 1-bit input: Output data clock enable
      .RST(rst_i),             // 1-bit input: Reset
      // SHIFTIN1 / SHIFTIN2: 1-bit (each) input: Data input expansion (1-bit each)
      .OFB(),             // 1-bit output: Feedback path for data
      .SHIFTOUT1(),
      .SHIFTOUT2(),
      .TBYTEOUT(),   // 1-bit output: Byte group tristate
      .TFB(),             // 1-bit output: 3-state control
      .TQ(),               // 1-bit output: 3-state control
      .D5(1'b0),
      .D6(1'b0),
      .D7(1'b0),
      .D8(1'b0),
      .SHIFTIN1(1'b0),
      .SHIFTIN2(1'b0),
      // T1 - T4: 1-bit (each) input: Parallel 3-state inputs
      .T1(1'b0),
      .T2(1'b0),
      .T3(1'b0),
      .T4(1'b0),
      .TBYTEIN(1'b0),     // 1-bit input: Byte group tristate
      .TCE(1'b0)              // 1-bit input: 3-state clock enable
   );

endmodule

module top(
    input osc,
    output ddr3_reset_n,
    output ddr3_cke,
    output ddr3_ck_p,
    output ddr3_ck_n,
    output ddr3_cs_n,
    output ddr3_ras_n,
    output ddr3_cas_n,
    output ddr3_we_n,
    output[2:0] ddr3_ba,
    output[13:0] ddr3_addr,
    output ddr3_odt,
    output[1:0] ddr3_dm,
    inout[1:0] ddr3_dqs_p,
    inout[1:0] ddr3_dqs_n,
    inout[15:0] ddr3_dq,
    output[3:0] led
);
    assign ddr3_cs_n = 1'b0;
    assign ddr3_odt = 1'b0;
    assign ddr3_dm = 2'b00;
    wire clk, clk_90, clk_ref, clk_ddr;
    wire locked;
    clkgen clkgen0(
        .osc(osc),
        .clk(clk),
        .clk_90(clk_90),
        .clk_ref(clk_ref),
        .clk_ddr(clk_ddr),
        .locked(locked)
    );
    
    reg rst_i = 1'b1;
    always_ff @(posedge clk)
        rst_i <= locked ? 1'b0 : 1'b1;
        
    wire ck_ser;
    oserdes_wrap ser_ck(.rst_i(rst_i), .clk(clk), .clk_ddr(clk_ddr), .D1(1'b0), .D2(1'b1), .D3(1'b0), .D4(1'b1), .DOUT(ck_ser));
    OBUFDS #(
      .IOSTANDARD("DIFF_SSTL135"), // Specify the output I/O standard
      .SLEW("SLOW")           // Specify the output slew rate
   ) OBUFDS_inst (
      .O(ddr3_ck_p),     // Diff_p output (connect directly to top-level port)
      .OB(ddr3_ck_n),   // Diff_n output (connect directly to top-level port)
      .I(ck_ser)      // Buffer input
   );
    
    
    localparam real clk_period_ns = 3.125;
    function real max(input real a, input real b);
        return a > b ? a : b;
    endfunction
    
    function int ns_to_clk(input real ns);
        return $ceil(ns / clk_period_ns);
    endfunction
    
    localparam tRFC = 160;
    localparam cRFC = ns_to_clk(160);
    localparam cMRD = 4;
    localparam cMOD = ns_to_clk(max(12 * clk_period_ns, 15));
    localparam cXPR = ns_to_clk(max(5 * clk_period_ns, tRFC + 10));
    localparam cDLLK = 512;
    localparam cZQInit = 512;
    localparam cRCD = ns_to_clk(14); // active to read/write
    localparam cRP = ns_to_clk(14);  // precharge period
    `ifndef XILINX_SIMULATOR
    localparam cRESET  = ns_to_clk(200_000) + 1;
    localparam cRSTCKE = ns_to_clk(500_000) + 1;
    `else
    localparam cRESET  = 100;
    localparam cRSTCKE = 100;
    `endif
    
    
    localparam CMD_NOP            = 4'b0111;
    localparam CMD_ACTIVE         = 4'b0011;
    localparam CMD_READ           = 4'b0101;
    localparam CMD_WRITE          = 4'b0100;
    localparam CMD_PRECHARGE      = 4'b0010;
    localparam CMD_REFRESH        = 4'b0001;
    localparam CMD_LOAD_MODE      = 4'b0000;
    localparam CMD_ZQCL           = 4'b0110;
    localparam cMPRRead = 100;
    localparam cMPRLoad = cMPRRead + cMOD;
    localparam cZQCL = cMPRLoad + cZQInit;
    localparam cLOAD0Init = cZQCL + cMOD;
    localparam cLOAD1Init = cLOAD0Init + cMRD;
    localparam cLOAD3Init = cLOAD1Init + cMRD;
    localparam cLOAD2Init = cLOAD3Init + cMRD;
    localparam cCKEN = cLOAD2Init + cXPR;
    localparam cRSTNEN = cCKEN + cRSTCKE;
    localparam STATECTR_INIT = cRSTNEN + cRESET;
    reg[15:0] state_ctr_q;
    always_ff @(posedge clk) begin
        if (rst_i)
            state_ctr_q <= STATECTR_INIT;
        else if (|state_ctr_q)
            state_ctr_q <= state_ctr_q - 1;
    end
   
    reg[3:0] command_r;
    reg[2:0] bank_r;
    reg[13:0] addr_r;
    reg reset_n_r;
    reg cke_r;
    always_comb begin
        command_r = CMD_NOP;
        bank_r = 3'd0;
        addr_r = 14'd0;
        reset_n_r = state_ctr_q > cRSTNEN ? 1'b0 : 1'b1;
        cke_r = state_ctr_q > cCKEN ? 1'b0 : 1'b1;
            
                if (state_ctr_q == cLOAD2Init) begin
                    command_r = CMD_LOAD_MODE;
                    bank_r    = 3'b010;
                    addr_r    = 14'h0000;
                end
                if (state_ctr_q == cLOAD3Init) begin
                    command_r = CMD_LOAD_MODE;
                    bank_r    = 3'b011;
                    addr_r    = 14'h0000;
                end
                if (state_ctr_q == cLOAD1Init) begin
                    command_r = CMD_LOAD_MODE;
                    bank_r    = 3'b001;
                    addr_r    = 14'h0000;
                end
                if (state_ctr_q == cLOAD0Init) begin
                    command_r = CMD_LOAD_MODE;
                    bank_r    = 3'b000;
                    addr_r    = 14'h0310;
                end
                if (state_ctr_q == cZQCL) begin
                    command_r = CMD_ZQCL;
                    addr_r[10] = 1'b1;
                end
                if (state_ctr_q == cMPRLoad) begin
                    command_r = CMD_LOAD_MODE;
                    bank_r    = 3'b011;
                    addr_r    = 14'h0004;
                end
                if (state_ctr_q == cMPRRead) begin
                    command_r = CMD_READ;
                    bank_r    = 3'b000;
                    addr_r    = 14'h0000;
                end
    end
    
    reg cke_q, reset_n_q;
    always_ff @(posedge clk)
        {cke_q, reset_n_q} <= rst_i ? 2'b00 : {cke_r, reset_n_r};
    assign ddr3_reset_n = reset_n_q;
    assign ddr3_cke = cke_q;
    
    oserdes_wrap ras_n_ser(.rst_i(rst_i), .clk(clk), .clk_ddr(clk_ddr), .D1(command_r[2]), .D2(command_r[2]), .D3(1'b1), .D4(1'b1), .DOUT(ddr3_ras_n));
    oserdes_wrap cas_n_ser(.rst_i(rst_i), .clk(clk), .clk_ddr(clk_ddr), .D1(command_r[1]), .D2(command_r[1]), .D3(1'b1), .D4(1'b1), .DOUT(ddr3_cas_n));
    oserdes_wrap we_n_ser(.rst_i(rst_i), .clk(clk), .clk_ddr(clk_ddr), .D1(command_r[0]), .D2(command_r[0]), .D3(1'b1), .D4(1'b1), .DOUT(ddr3_we_n));
    generate
        for (genvar i = 0; i < 3; i = i + 1)
            oserdes_wrap bank_ser(.rst_i(rst_i), .clk(clk), .clk_ddr(clk_ddr), .D1(bank_r[i]), .D2(bank_r[i]), .D3(1'b0), .D4(1'b0), .DOUT(ddr3_ba[i]));
        for (genvar i = 0; i < 14; i = i + 1)
            oserdes_wrap addr_ser(.rst_i(rst_i), .clk(clk), .clk_ddr(clk_ddr), .D1(addr_r[i]), .D2(addr_r[i]), .D3(1'b0), .D4(1'b0), .DOUT(ddr3_addr[i]));
    endgenerate
    
    
    wire en_wr_n_w = 1'b1;
    
    wire[1:0] dqs_in;
    wire[1:0] dqs_out;
    generate
        for (genvar i = 0; i < 2; ++i) begin
            IOBUFDS #(
              .DIFF_TERM("FALSE"),     // Differential Termination ("TRUE"/"FALSE")
              .IBUF_LOW_PWR("FALSE"),   // Low Power - "TRUE", High Performance = "FALSE" 
              .IOSTANDARD("DIFF_SSTL135"), // Specify the I/O standard
              .SLEW("SLOW")            // Specify the output slew rate
           ) IOBUFDS_inst (
              .O(dqs_in[i]),     // Buffer output
              .IO(ddr3_dqs_p[i]),   // Diff_p inout (connect directly to top-level port)
              .IOB(ddr3_dqs_n[i]), // Diff_n inout (connect directly to top-level port)
              .I(dqs_out[i]),     // Buffer input
              .T(en_wr_n_w)      // 3-state enable input, high=input, low=output
           );
       end
    endgenerate
    
    wire[15:0] dq_in;
    wire[15:0] dq_out;
    generate
        for (genvar i = 0; i < 16; ++i) begin
            IOBUF #(
              .DRIVE(12), // Specify the output drive strength
              .IBUF_LOW_PWR("FALSE"),  // Low Power - "TRUE", High Performance = "FALSE" 
              .IOSTANDARD("SSTL135"), // Specify the I/O standard
              .SLEW("SLOW") // Specify the output slew rate
           ) IOBUF_inst (
              .O(dq_in[i]),     // Buffer output
              .IO(ddr3_dq[i]),   // Buffer inout port (connect directly to top-level port)
              .I(dq_out[i]),     // Buffer input
              .T(en_wr_n_w)      // 3-state enable input, high=input, low=output
           );
        end
    endgenerate
   

   
   wire dqs0_in = dqs_in[0];
   wire dq0_in = dq_in[0];
   
   
   wire dqs0_delayed;
   wire dq0_delayed;
   
   (* IODELAY_GROUP = "inputdelaygroup" *) // Specifies group name for associated IDELAYs/ODELAYs and IDELAYCTRL
    wire idelay_rdy;
   IDELAYCTRL IDELAYCTRL_inst (
      .RDY(idelay_rdy),       // 1-bit output: Ready output
      .REFCLK(clk_ref), // 1-bit input: Reference clock input
      .RST(rst_i)        // 1-bit input: Active high reset input
   );
   
   
   (* IODELAY_GROUP = "inputdelaygroup" *) // Specifies group name for associated IDELAYs/ODELAYs and IDELAYCTRL
   IDELAYE2 #(
      .CINVCTRL_SEL("FALSE"),          // Enable dynamic clock inversion (FALSE, TRUE)
      .DELAY_SRC("IDATAIN"),           // Delay input (IDATAIN, DATAIN)
      .HIGH_PERFORMANCE_MODE("TRUE"), // Reduced jitter ("TRUE"), Reduced power ("FALSE")
      .IDELAY_TYPE("FIXED"),           // FIXED, VARIABLE, VAR_LOAD, VAR_LOAD_PIPE
      .IDELAY_VALUE(12),                // Input delay tap setting (0-31)
      .PIPE_SEL("FALSE"),              // Select pipelined mode, FALSE, TRUE
      .REFCLK_FREQUENCY(200.0),        // IDELAYCTRL clock input frequency in MHz (190.0-210.0, 290.0-310.0).
      .SIGNAL_PATTERN("CLOCK")          // DATA, CLOCK input signal
   )
   IDELAYE2_inst (
      .CNTVALUEOUT(), // 5-bit output: Counter value output
      .DATAOUT(dqs0_delayed),         // 1-bit output: Delayed data output
      .C(1'b0),                     // 1-bit input: Clock input
      .CE(1'b0),                   // 1-bit input: Active high enable increment/decrement input
      .CINVCTRL(1'b0),       // 1-bit input: Dynamic clock inversion input
      .CNTVALUEIN(5'd0),   // 5-bit input: Counter value input
      .DATAIN(1'b0),           // 1-bit input: Internal delay data input
      .IDATAIN(dqs0_in),         // 1-bit input: Data input from the I/O
      .INC(1'b0),                 // 1-bit input: Increment / Decrement tap delay input
      .LD(1'b0),                   // 1-bit input: Load IDELAY_VALUE input
      .LDPIPEEN(1'b0),       // 1-bit input: Enable PIPELINE register to load data input
      .REGRST(1'b0)            // 1-bit input: Active-high reset tap-delay input
   );
   (* IODELAY_GROUP = "inputdelaygroup" *) // Specifies group name for associated IDELAYs/ODELAYs and IDELAYCTRL
   IDELAYE2 #(
      .CINVCTRL_SEL("FALSE"),          // Enable dynamic clock inversion (FALSE, TRUE)
      .DELAY_SRC("IDATAIN"),           // Delay input (IDATAIN, DATAIN)
      .HIGH_PERFORMANCE_MODE("TRUE"), // Reduced jitter ("TRUE"), Reduced power ("FALSE")
      .IDELAY_TYPE("FIXED"),           // FIXED, VARIABLE, VAR_LOAD, VAR_LOAD_PIPE
      .IDELAY_VALUE(10),                // Input delay tap setting (0-31)
      .PIPE_SEL("FALSE"),              // Select pipelined mode, FALSE, TRUE
      .REFCLK_FREQUENCY(200.0),        // IDELAYCTRL clock input frequency in MHz (190.0-210.0, 290.0-310.0).
      .SIGNAL_PATTERN("DATA")          // DATA, CLOCK input signal
   )
   IDELAYE2_dq0 (
      .CNTVALUEOUT(), // 5-bit output: Counter value output
      .DATAOUT(dq0_delayed),         // 1-bit output: Delayed data output
      .C(1'b0),                     // 1-bit input: Clock input
      .CE(1'b0),                   // 1-bit input: Active high enable increment/decrement input
      .CINVCTRL(1'b0),       // 1-bit input: Dynamic clock inversion input
      .CNTVALUEIN(5'd0),   // 5-bit input: Counter value input
      .DATAIN(1'b0),           // 1-bit input: Internal delay data input
      .IDATAIN(dq0_in),         // 1-bit input: Data input from the I/O
      .INC(1'b0),                 // 1-bit input: Increment / Decrement tap delay input
      .LD(1'b0),                   // 1-bit input: Load IDELAY_VALUE input
      .LDPIPEEN(1'b0),       // 1-bit input: Enable PIPELINE register to load data input
      .REGRST(1'b0)            // 1-bit input: Active-high reset tap-delay input
   );
    wire[3:0] iserdese2_dq0;
    wire dqs0_buf;
    BUFG BUFIO_inst (
      .O(dqs0_buf), // 1-bit output: Clock output (connect to I/O clock loads).
      .I(dqs0_delayed)  // 1-bit input: Clock input (connect to an IBUF or BUFMR).
   );
    ISERDESE2 #(
      .DATA_RATE("DDR"),           // DDR, SDR
      .DATA_WIDTH(4),              // Parallel data width (2-8,10,14)
      .DYN_CLKDIV_INV_EN("FALSE"), // Enable DYNCLKDIVINVSEL inversion (FALSE, TRUE)
      .DYN_CLK_INV_EN("FALSE"),    // Enable DYNCLKINVSEL inversion (FALSE, TRUE)
      // INIT_Q1 - INIT_Q4: Initial value on the Q outputs (0/1)
      .INIT_Q1(1'b0),
      .INIT_Q2(1'b0),
      .INIT_Q3(1'b0),
      .INIT_Q4(1'b0),
      .INTERFACE_TYPE("MEMORY"),   // MEMORY, MEMORY_DDR3, MEMORY_QDR, NETWORKING, OVERSAMPLE
      .IOBDELAY("IFD"),           // NONE, BOTH, IBUF, IFD
      .NUM_CE(1),                  // Number of clock enables (1,2)
      .OFB_USED("FALSE"),          // Select OFB path (FALSE, TRUE)
      .SERDES_MODE("MASTER"),      // MASTER, SLAVE
      // SRVAL_Q1 - SRVAL_Q4: Q output values when SR is used (0/1)
      .SRVAL_Q1(1'b0),
      .SRVAL_Q2(1'b0),
      .SRVAL_Q3(1'b0),
      .SRVAL_Q4(1'b0)
   )
   ISERDESE2_inst (
      .O(),                       // 1-bit output: Combinatorial output
      // Q1 - Q8: 1-bit (each) output: Registered data outputs
      .Q1(iserdese2_dq0[3]),
      .Q2(iserdese2_dq0[2]),
      .Q3(iserdese2_dq0[1]),
      .Q4(iserdese2_dq0[0]),
      .Q5(),
      .Q6(),
      .Q7(),
      .Q8(),
      // SHIFTOUT1, SHIFTOUT2: 1-bit (each) output: Data width expansion output ports
      .SHIFTOUT1(),
      .SHIFTOUT2(),
      .BITSLIP(1'b0),           // 1-bit input: The BITSLIP pin performs a Bitslip operation synchronous to
                                   // CLKDIV when asserted (active High). Subsequently, the data seen on the Q1
                                   // to Q8 output ports will shift, as in a barrel-shifter operation, one
                                   // position every time Bitslip is invoked (DDR operation is different from
                                   // SDR).

      // CE1, CE2: 1-bit (each) input: Data register clock enable inputs
      .CE1(1'b1),
      .CE2(1'b1),
      .CLKDIVP(1'b0),           // 1-bit input: TBD
      // Clocks: 1-bit (each) input: ISERDESE2 clock input ports
      .CLK(dqs0_buf),                   // 1-bit input: High-speed clock
      .CLKB(~dqs0_buf),                 // 1-bit input: High-speed secondary clock
      .CLKDIV(clk),             // 1-bit input: Divided clock
      .OCLK(clk_ddr),                 // 1-bit input: High speed output clock used when INTERFACE_TYPE="MEMORY" 
      // Dynamic Clock Inversions: 1-bit (each) input: Dynamic clock inversion pins to switch clock polarity
      .DYNCLKDIVSEL(1'b0), // 1-bit input: Dynamic CLKDIV inversion
      .DYNCLKSEL(1'b0),       // 1-bit input: Dynamic CLK/CLKB inversion
      // Input Data: 1-bit (each) input: ISERDESE2 data input ports
      .D(1'b0),                       // 1-bit input: Data input
      .DDLY(dq0_delayed),                 // 1-bit input: Serial data from IDELAYE2
      .OFB(1'b0),                   // 1-bit input: Data feedback from OSERDESE2
      .OCLKB(~clk_ddr),               // 1-bit input: High speed negative edge output clock
      .RST(rst_i),                   // 1-bit input: Active high asynchronous reset
      // SHIFTIN1, SHIFTIN2: 1-bit (each) input: Data width expansion input ports
      .SHIFTIN1(1'b0),
      .SHIFTIN2(1'b0)
   );
    localparam rd_latency = 5;
    localparam rd_width = rd_latency + 2;
    
    reg[rd_width-1:0] rd_en_q;
    always_ff @(posedge clk) begin
        if (rst_i)
            rd_en_q <= {rd_width{1'b0}};
        else if (command_r == CMD_READ)
            rd_en_q <= {2'b11, rd_en_q[rd_latency:1]};
        else
            rd_en_q <= {1'b0, rd_en_q[rd_width-1:1]};
    end
    
    
    
    
    
    
    
    reg[3:0] led_q;
    always_ff @(posedge clk) begin
        if (rst_i)
            led_q <= 4'h0;
        else if (rd_en_q[0])
            led_q <= iserdese2_dq0;
    end
    assign led = led_q;
    
        
    
    
endmodule
