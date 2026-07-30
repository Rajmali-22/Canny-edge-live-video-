`timescale 1ns / 1ps

module top_module(
    input wire clk, rst_n,
    input wire sw_canny, // Switch R7: 0 = Normal Video, 1 = Canny Edge
    input wire [3:0] key, // key[1:0] for brightness control , key[3:2] for contrast control
    
    // camera pinouts
    input wire cmos_pclk, cmos_href, cmos_vsync,
    input wire [7:0] cmos_db,
    inout cmos_sda, cmos_scl, 
    output wire cmos_rst_n, cmos_pwdn, cmos_xclk,
    
    // Debugging
    output [3:0] led, 
    
    // controller to sdram
    output wire sdram_clk,
    output wire sdram_cke, 
    output wire sdram_cs_n, sdram_ras_n, sdram_cas_n, sdram_we_n, 
    output wire [12:0] sdram_addr,
    output wire [1:0] sdram_ba, 
    output wire [1:0] sdram_dqm, 
    inout [15:0] sdram_dq,
    
    // VGA output
    output wire [3:0] vga_out_r,
    output wire [3:0] vga_out_g,
    output wire [3:0] vga_out_b,
    output wire vga_out_vs, vga_out_hs
);
     
    wire f2s_data_valid;
    wire [9:0] data_count_r;
    wire [15:0] dout, din;
    wire clk_sdram;
    wire empty_fifo;
    wire clk_vga;
    wire state;
    wire rd_en;

    // Wires to receive raw pixel colors from vga_interface m2
    wire [3:0] raw_vga_r;
    wire [3:0] raw_vga_g;
    wire [3:0] raw_vga_b;

    // Control logic for retrieving data from camera, storing data to asyn_fifo, and sending data to sdram
    camera_interface m0 
    (
        .clk(clk),
        .clk_100(clk_sdram),
        .rst_n(rst_n),
        .key(key),
        // asyn_fifo IO
        .rd_en(f2s_data_valid),
        .data_count_r(data_count_r),
        .dout(dout),
        // camera pinouts
        .cmos_pclk(cmos_pclk),
        .cmos_href(cmos_href),
        .cmos_vsync(cmos_vsync),
        .cmos_db(cmos_db),
        .cmos_sda(cmos_sda),
        .cmos_scl(cmos_scl), 
        .cmos_rst_n(cmos_rst_n),
        .cmos_pwdn(cmos_pwdn),
        .cmos_xclk(cmos_xclk),
        // Debugging
        .led(led)
    );
     
    // Control logic for writing the pixel-data from camera to sdram and reading pixel-data from sdram to vga
    sdram_interface m1 
    (
        .clk(clk_sdram),
        .rst_n(rst_n),
        // asyn_fifo IO
        .clk_vga(clk_vga),
        .rd_en(rd_en),
        .data_count_r(data_count_r),
        .f2s_data(dout),
        .f2s_data_valid(f2s_data_valid),
        .empty_fifo(empty_fifo),
        .dout(din),
        // controller to sdram
        .sdram_cke(sdram_cke), 
        .sdram_cs_n(sdram_cs_n),
        .sdram_ras_n(sdram_ras_n),
        .sdram_cas_n(sdram_cas_n),
        .sdram_we_n(sdram_we_n), 
        .sdram_addr(sdram_addr),
        .sdram_ba(sdram_ba), 
        .sdram_dqm(sdram_dqm),
        .sdram_dq(sdram_dq)
    );
     
    // Control logic for retrieving data from sdram, storing data to asyn_fifo, and sending data to vga
    vga_interface m2 
    (
        .clk(clk),
        .rst_n(rst_n),
        // asyn_fifo IO
        .empty_fifo(empty_fifo),
        .din(din),
        .clk_vga(clk_vga),
        .rd_en(rd_en),
        // VGA output routed to internal wires
        .vga_out_r(raw_vga_r),
        .vga_out_g(raw_vga_g),
        .vga_out_b(raw_vga_b),
        .vga_out_vs(vga_out_vs),
        .vga_out_hs(vga_out_hs)
    );
     
    // Active display area condition (when sync signals are active high)
    wire active_video = vga_out_vs & vga_out_hs;

    // Canny / Sobel Edge Detector Module with Dynamic Threshold Control
    canny_sobel_edge_detector m4 (
        .clk(clk_vga),
        .rst_n(rst_n),
        .sw_canny(sw_canny),         // Switch R7 input
        .key_threshold(key[3:2]),    // key[3] increases threshold, key[2] decreases threshold
        .video_on(active_video),     // Active display area
        .raw_r(raw_vga_r),
        .raw_g(raw_vga_g),
        .raw_b(raw_vga_b),
        .out_r(vga_out_r),
        .out_g(vga_out_g),
        .out_b(vga_out_b)
    );

    // ERROR APPEARS IF ODDR2 IS ROUTED INSIDE THE FPGA INSTEAD OF BEING DIRECTLY CONNECTED TO OUTPUT
    ODDR2#(.DDR_ALIGNMENT("NONE"), .INIT(1'b0),.SRTYPE("SYNC")) oddr2_primitive
    (
        .D0(1'b0),
        .D1(1'b1),
        .C0(clk_sdram),
        .C1(~clk_sdram),
        .CE(1'b1),
        .R(1'b0),
        .S(1'b0),
        .Q(sdram_clk)
    );

    wire RESET;
    assign RESET = ~rst_n;

    // SDRAM clock generator
    dcm_165MHz m3
    (
        .clk(clk),              // IN
        .clk_sdram(clk_sdram),  // OUT
        .RESET(RESET),          // IN
        .LOCKED(LOCKED)         // OUT
    );

endmodule