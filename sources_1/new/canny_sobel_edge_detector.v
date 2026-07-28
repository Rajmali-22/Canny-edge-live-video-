`timescale 1ns / 1ps

module canny_sobel_edge_detector (
    input  wire        clk,           // VGA Clock (25 MHz)
    input  wire        rst_n,         // Active Low Reset
    input  wire        sw_canny,      // Switch R7 input
    input  wire        video_on,      // Active VGA Region
    input  wire [3:0]  raw_r,         // Raw Input Red (4-bit)
    input  wire [3:0]  raw_g,         // Raw Input Green (4-bit)
    input  wire [3:0]  raw_b,         // Raw Input Blue (4-bit)
    output reg  [3:0]  out_r,         // Muxed Output Red to Pin
    output reg  [3:0]  out_g,         // Muxed Output Green to Pin
    output reg  [3:0]  out_b          // Muxed Output Blue to Pin
);

    localparam THRESHOLD   = 8'd90;   // Sensitivity threshold (Adjustable)
    localparam FRAME_WIDTH = 640;

    //-------------------------------------------------------------
    // Step 1: RGB444 to Grayscale Conversion
    //-------------------------------------------------------------
    wire [7:0] gray_pixel;
    assign gray_pixel = ({4'b0, raw_r} * 8'd17) + 
                         ({4'b0, raw_g} * 8'd10) + 
                         ({4'b0, raw_b} * 8'd4);

    //-------------------------------------------------------------
    // Step 2: BRAM Line Buffers for 3x3 Window
    //-------------------------------------------------------------
    reg [7:0] line_buf_0 [0:FRAME_WIDTH-1];
    reg [7:0] line_buf_1 [0:FRAME_WIDTH-1];
    
    reg [9:0] col_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            col_cnt <= 0;
        else if (video_on) begin
            if (col_cnt == FRAME_WIDTH - 1)
                col_cnt <= 0;
            else
                col_cnt <= col_cnt + 1'b1;
        end else begin
            col_cnt <= 0;
        end
    end

    reg [7:0] lb0_out, lb1_out;
    always @(posedge clk) begin
        if (video_on) begin
            lb0_out <= line_buf_0[col_cnt];
            lb1_out <= line_buf_1[col_cnt];
            line_buf_0[col_cnt] <= gray_pixel;
            line_buf_1[col_cnt] <= lb0_out;
        end
    end

    // 3x3 Matrix Registers
    reg [7:0] p00, p01, p02;
    reg [7:0] p10, p11, p12;
    reg [7:0] p20, p21, p22;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            {p00, p01, p02} <= 24'b0;
            {p10, p11, p12} <= 24'b0;
            {p20, p21, p22} <= 24'b0;
        end else if (video_on) begin
            p00 <= p01; p01 <= p02; p02 <= lb1_out;
            p10 <= p11; p11 <= p12; p12 <= lb0_out;
            p20 <= p21; p21 <= p22; p22 <= gray_pixel;
        end
    end

    //-------------------------------------------------------------
    // Step 3: Fixed Sobel Gradient Calculation (Gx and Gy)
    //-------------------------------------------------------------
    reg signed [10:0] gx, gy;
    reg        [10:0] abs_gx, abs_gy;
    reg        [10:0] g_sum;

    always @(posedge clk) begin
        // Gx = (p02 + 2*p12 + p22) - (p00 + 2*p10 + p20)
        gx <= ($signed({3'b0, p02}) + $signed({2'b0, p12, 1'b0}) + $signed({3'b0, p22})) -
              ($signed({3'b0, p00}) + $signed({2'b0, p10, 1'b0}) + $signed({3'b0, p20}));

        // Gy = (p00 + 2*p01 + p02) - (p20 + 2*p21 + p22)
        gy <= ($signed({3'b0, p00}) + $signed({2'b0, p01, 1'b0}) + $signed({3'b0, p02})) -
              ($signed({3'b0, p20}) + $signed({2'b0, p21, 1'b0}) + $signed({3'b0, p22}));
              
        // Absolute values
        abs_gx <= (gx < 0) ? -gx : gx;
        abs_gy <= (gy < 0) ? -gy : gy;
        
        // Magnitude approximation
        g_sum  <= abs_gx + abs_gy;
    end

    // Binary Edge Output
    wire [11:0] edge_pixel = (g_sum > THRESHOLD) ? 12'hFFF : 12'h000;

    //-------------------------------------------------------------
    // Step 4: Delay Raw Video to Match Pipeline Latency
    //-------------------------------------------------------------
    reg [3:0] r_d1, r_d2, r_d3;
    reg [3:0] g_d1, g_d2, g_d3;
    reg [3:0] b_d1, b_d2, b_d3;

    always @(posedge clk) begin
        r_d1 <= raw_r; r_d2 <= r_d1; r_d3 <= r_d2;
        g_d1 <= raw_g; g_d2 <= g_d1; g_d3 <= g_d2;
        b_d1 <= raw_b; b_d2 <= b_d1; b_d3 <= b_d2;
    end

    //-------------------------------------------------------------
    // Step 5: MUX Output Logic based on Switch R7
    //-------------------------------------------------------------
    always @(posedge clk) begin
        if (!video_on) begin
            out_r <= 4'h0;
            out_g <= 4'h0;
            out_b <= 4'h0;
        end else if (sw_canny) begin
            out_r <= edge_pixel[11:8];
            out_g <= edge_pixel[7:4];
            out_b <= edge_pixel[3:0];
        end else begin
            out_r <= r_d3;
            out_g <= g_d3;
            out_b <= b_d3;
        end
    end

endmodule