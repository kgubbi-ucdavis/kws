// SPDX-FileCopyrightText: 2020 Efabless Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// SPDX-License-Identifier: Apache-2.0

`default_nettype none
/*
 *-------------------------------------------------------------
 *
 * user_project_wrapper
 *
 * This wrapper enumerates all of the pins available to the
 * user for the user project.
 *
 * The example user project has been removed and replaced
 * with the actual user project.
 *
 *-------------------------------------------------------------
 */

module WeightBuffer(
    input clk,
    input reset,
    input [31:0] weight_data,
    input write_enable,
    input [4:0] write_addr,
    output [31:0] weight_out0, weight_out1, weight_out2, weight_out3, weight_out4,
    output [31:0] weight_out5, weight_out6, weight_out7, weight_out8, weight_out9,
    output [31:0] weight_out10, weight_out11, weight_out12, weight_out13, weight_out14,
    output [31:0] weight_out15, weight_out16, weight_out17, weight_out18, weight_out19,
    output [31:0] weight_out20, weight_out21, weight_out22, weight_out23, weight_out24,
    output [31:0] weight_out25, weight_out26, weight_out27, weight_out28, weight_out29,
    output [31:0] weight_out30, weight_out31
);
    reg [31:0] weights [0:31];
    integer i;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 32; i = i + 1)
                weights[i] <= 32'b0;
        end else if (write_enable) begin
            weights[write_addr] <= weight_data;
        end
    end

    assign weight_out0 = weights[0];
    assign weight_out1 = weights[1];
    assign weight_out2 = weights[2];
    assign weight_out3 = weights[3];
    assign weight_out4 = weights[4];
    assign weight_out5 = weights[5];
    assign weight_out6 = weights[6];
    assign weight_out7 = weights[7];
    assign weight_out8 = weights[8];
    assign weight_out9 = weights[9];
    assign weight_out10 = weights[10];
    assign weight_out11 = weights[11];
    assign weight_out12 = weights[12];
    assign weight_out13 = weights[13];
    assign weight_out14 = weights[14];
    assign weight_out15 = weights[15];
    assign weight_out16 = weights[16];
    assign weight_out17 = weights[17];
    assign weight_out18 = weights[18];
    assign weight_out19 = weights[19];
    assign weight_out20 = weights[20];
    assign weight_out21 = weights[21];
    assign weight_out22 = weights[22];
    assign weight_out23 = weights[23];
    assign weight_out24 = weights[24];
    assign weight_out25 = weights[25];
    assign weight_out26 = weights[26];
    assign weight_out27 = weights[27];
    assign weight_out28 = weights[28];
    assign weight_out29 = weights[29];
    assign weight_out30 = weights[30];
    assign weight_out31 = weights[31];
endmodule

module LineBuffer(
    input clk,
    input reset,
    input [31:0] data_in,
    input write_enable,
    input [4:0] write_addr,
    input [4:0] read_addr,
    output [31:0] data_out
);
    reg [31:0] line_data [0:31];
    integer i;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 32; i = i + 1)
                line_data[i] <= 32'b0;
        end else if (write_enable) begin
            line_data[write_addr] <= data_in;
        end
    end
    
    assign data_out = line_data[read_addr];
endmodule

module MAC(
    input [31:0] a,
    input [31:0] b,
    output [31:0] y
);
    assign y = a * b;
endmodule

module MACArray(
    input [31:0] a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15,
    input [31:0] a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31,
    input [31:0] b0, b1, b2, b3, b4, b5, b6, b7, b8, b9, b10, b11, b12, b13, b14, b15,
    input [31:0] b16, b17, b18, b19, b20, b21, b22, b23, b24, b25, b26, b27, b28, b29, b30, b31,
    output [31:0] product0, product1, product2, product3, product4, product5, product6, product7,
    output [31:0] product8, product9, product10, product11, product12, product13, product14, product15,
    output [31:0] product16, product17, product18, product19, product20, product21, product22, product23,
    output [31:0] product24, product25, product26, product27, product28, product29, product30, product31
);
    MAC mac0 (.a(a0), .b(b0), .y(product0));
    MAC mac1 (.a(a1), .b(b1), .y(product1));
    MAC mac2 (.a(a2), .b(b2), .y(product2));
    MAC mac3 (.a(a3), .b(b3), .y(product3));
    MAC mac4 (.a(a4), .b(b4), .y(product4));
    MAC mac5 (.a(a5), .b(b5), .y(product5));
    MAC mac6 (.a(a6), .b(b6), .y(product6));
    MAC mac7 (.a(a7), .b(b7), .y(product7));
    MAC mac8 (.a(a8), .b(b8), .y(product8));
    MAC mac9 (.a(a9), .b(b9), .y(product9));
    MAC mac10 (.a(a10), .b(b10), .y(product10));
    MAC mac11 (.a(a11), .b(b11), .y(product11));
    MAC mac12 (.a(a12), .b(b12), .y(product12));
    MAC mac13 (.a(a13), .b(b13), .y(product13));
    MAC mac14 (.a(a14), .b(b14), .y(product14));
    MAC mac15 (.a(a15), .b(b15), .y(product15));
    MAC mac16 (.a(a16), .b(b16), .y(product16));
    MAC mac17 (.a(a17), .b(b17), .y(product17));
    MAC mac18 (.a(a18), .b(b18), .y(product18));
    MAC mac19 (.a(a19), .b(b19), .y(product19));
    MAC mac20 (.a(a20), .b(b20), .y(product20));
    MAC mac21 (.a(a21), .b(b21), .y(product21));
    MAC mac22 (.a(a22), .b(b22), .y(product22));
    MAC mac23 (.a(a23), .b(b23), .y(product23));
    MAC mac24 (.a(a24), .b(b24), .y(product24));
    MAC mac25 (.a(a25), .b(b25), .y(product25));
    MAC mac26 (.a(a26), .b(b26), .y(product26));
    MAC mac27 (.a(a27), .b(b27), .y(product27));
    MAC mac28 (.a(a28), .b(b28), .y(product28));
    MAC mac29 (.a(a29), .b(b29), .y(product29));
    MAC mac30 (.a(a30), .b(b30), .y(product30));
    MAC mac31 (.a(a31), .b(b31), .y(product31));
endmodule

module AdderTree(
    input [31:0] in0,
    input [31:0] in1,
    input [31:0] in2,
    input [31:0] in3,
    input [31:0] in4,
    input [31:0] in5,
    input [31:0] in6,
    input [31:0] in7,
    output [31:0] sum
);
    wire [31:0] sum1, sum2, sum3, sum4;

    assign sum1 = in0 + in1;
    assign sum2 = in2 + in3;
    assign sum3 = in4 + in5;
    assign sum4 = in6 + in7;

    wire [31:0] sum5, sum6;

    assign sum5 = sum1 + sum2;
    assign sum6 = sum3 + sum4;

    assign sum = sum5 + sum6;
endmodule

module ConfigurableAdderTree(
    input [31:0] products0, products1, products2, products3, products4,
    input [31:0] products5, products6, products7, products8, products9,
    input [31:0] products10, products11, products12, products13, products14,
    input [31:0] products15, products16, products17, products18, products19,
    input [31:0] products20, products21, products22, products23, products24,
    input [31:0] products25, products26, products27, products28, products29,
    input [31:0] products30, products31,
    input sum_mode,
    output [31:0] final_sum0, final_sum1, final_sum2, final_sum3, final_sum4,
    output [31:0] final_sum5, final_sum6, final_sum7, final_sum8, final_sum9,
    output [31:0] final_sum10, final_sum11, final_sum12, final_sum13, final_sum14,
    output [31:0] final_sum15, final_sum16, final_sum17, final_sum18, final_sum19,
    output [31:0] final_sum20, final_sum21, final_sum22, final_sum23, final_sum24,
    output [31:0] final_sum25, final_sum26, final_sum27, final_sum28, final_sum29,
    output [31:0] final_sum30, final_sum31
);
    wire [31:0] sum_depthwise [0:31];
    wire [31:0] sum_pointwise [0:7];

    assign sum_depthwise[0] = products0;
    assign sum_depthwise[1] = products1;
    assign sum_depthwise[2] = products2;
    assign sum_depthwise[3] = products3;
    assign sum_depthwise[4] = products4;
    assign sum_depthwise[5] = products5;
    assign sum_depthwise[6] = products6;
    assign sum_depthwise[7] = products7;
    assign sum_depthwise[8] = products8;
    assign sum_depthwise[9] = products9;
    assign sum_depthwise[10] = products10;
    assign sum_depthwise[11] = products11;
    assign sum_depthwise[12] = products12;
    assign sum_depthwise[13] = products13;
    assign sum_depthwise[14] = products14;
    assign sum_depthwise[15] = products15;
    assign sum_depthwise[16] = products16;
    assign sum_depthwise[17] = products17;
    assign sum_depthwise[18] = products18;
    assign sum_depthwise[19] = products19;
    assign sum_depthwise[20] = products20;
    assign sum_depthwise[21] = products21;
    assign sum_depthwise[22] = products22;
    assign sum_depthwise[23] = products23;
    assign sum_depthwise[24] = products24;
    assign sum_depthwise[25] = products25;
    assign sum_depthwise[26] = products26;
    assign sum_depthwise[27] = products27;
    assign sum_depthwise[28] = products28;
    assign sum_depthwise[29] = products29;
    assign sum_depthwise[30] = products30;
    assign sum_depthwise[31] = products31;

    assign sum_pointwise[0] = products0;
    assign sum_pointwise[1] = products1;
    assign sum_pointwise[2] = products2;
    assign sum_pointwise[3] = products3;
    assign sum_pointwise[4] = products4;
    assign sum_pointwise[5] = products5;
    assign sum_pointwise[6] = products6;
    assign sum_pointwise[7] = products7;

    assign final_sum0 = sum_mode ? sum_depthwise[0] : sum_pointwise[0];
    assign final_sum1 = sum_mode ? sum_depthwise[1] : sum_pointwise[1];
    assign final_sum2 = sum_mode ? sum_depthwise[2] : sum_pointwise[2];
    assign final_sum3 = sum_mode ? sum_depthwise[3] : sum_pointwise[3];
    assign final_sum4 = sum_mode ? sum_depthwise[4] : sum_pointwise[4];
    assign final_sum5 = sum_mode ? sum_depthwise[5] : sum_pointwise[5];
    assign final_sum6 = sum_mode ? sum_depthwise[6] : sum_pointwise[6];
    assign final_sum7 = sum_mode ? sum_depthwise[7] : sum_pointwise[7];
    assign final_sum8 = sum_mode ? sum_depthwise[8] : 32'b0;
    assign final_sum9 = sum_mode ? sum_depthwise[9] : 32'b0;
    assign final_sum10 = sum_mode ? sum_depthwise[10] : 32'b0;
    assign final_sum11 = sum_mode ? sum_depthwise[11] : 32'b0;
    assign final_sum12 = sum_mode ? sum_depthwise[12] : 32'b0;
    assign final_sum13 = sum_mode ? sum_depthwise[13] : 32'b0;
    assign final_sum14 = sum_mode ? sum_depthwise[14] : 32'b0;
    assign final_sum15 = sum_mode ? sum_depthwise[15] : 32'b0;
    assign final_sum16 = sum_mode ? sum_depthwise[16] : 32'b0;
    assign final_sum17 = sum_mode ? sum_depthwise[17] : 32'b0;
    assign final_sum18 = sum_mode ? sum_depthwise[18] : 32'b0;
    assign final_sum19 = sum_mode ? sum_depthwise[19] : 32'b0;
    assign final_sum20 = sum_mode ? sum_depthwise[20] : 32'b0;
    assign final_sum21 = sum_mode ? sum_depthwise[21] : 32'b0;
    assign final_sum22 = sum_mode ? sum_depthwise[22] : 32'b0;
    assign final_sum23 = sum_mode ? sum_depthwise[23] : 32'b0;
    assign final_sum24 = sum_mode ? sum_depthwise[24] : 32'b0;
    assign final_sum25 = sum_mode ? sum_depthwise[25] : 32'b0;
    assign final_sum26 = sum_mode ? sum_depthwise[26] : 32'b0;
    assign final_sum27 = sum_mode ? sum_depthwise[27] : 32'b0;
    assign final_sum28 = sum_mode ? sum_depthwise[28] : 32'b0;
    assign final_sum29 = sum_mode ? sum_depthwise[29] : 32'b0;
    assign final_sum30 = sum_mode ? sum_depthwise[30] : 32'b0;
    assign final_sum31 = sum_mode ? sum_depthwise[31] : 32'b0;
endmodule

module Norm(
    input [31:0] data_in,
    input [31:0] norm_param,
    output [31:0] data_out
);
    assign data_out = (data_in * norm_param) >> 16;
endmodule

module ReLU(
    input [31:0] data_in,
    output [31:0] data_out
);
    assign data_out = (data_in < 0) ? 0 : data_in;
endmodule

module Pooling(
    input [31:0] data_in0,
    input [31:0] data_in1,
    input [31:0] data_in2,
    input [31:0] data_in3,
    output [31:0] data_out
);
    assign data_out = (data_in0 + data_in1 + data_in2 + data_in3) >> 2;
endmodule

module ControlFSM(
    input clk,
    input reset,
    input start,
    input [31:0] norm_param,
    output reg weight_write_enable,
    output reg line_write_enable,
    output reg sum_mode,
    output reg [31:0] weight_data,
    output reg [4:0] weight_write_addr,
    output reg [31:0] line_data,
    output reg [4:0] line_write_addr,
    output reg norm_enable,
    output reg relu_enable,
    output reg pool_enable,
    output reg done
);
    // FSM Implementation
endmodule

module CNN_Accelerator(
    input clk,
    input reset,
    input [31:0] weight_data0, weight_data1, weight_data2, weight_data3, weight_data4,
    input [31:0] weight_data5, weight_data6, weight_data7, weight_data8, weight_data9,
    input [31:0] weight_data10, weight_data11, weight_data12, weight_data13, weight_data14,
    input [31:0] weight_data15, weight_data16, weight_data17, weight_data18, weight_data19,
    input [31:0] weight_data20, weight_data21, weight_data22, weight_data23, weight_data24,
    input [31:0] weight_data25, weight_data26, weight_data27, weight_data28, weight_data29,
    input [31:0] weight_data30, weight_data31,
    input weight_write_enable,
    input [4:0] weight_write_addr,
    input [31:0] line_data,
    input line_write_enable,
    input [4:0] line_write_addr,
    input sum_mode,
    input [31:0] norm_param,
    input norm_enable,
    input relu_enable,
    input pool_enable,
    output [31:0] result0, result1, result2, result3, result4, result5, result6, result7,
    output [31:0] result8, result9, result10, result11, result12, result13, result14, result15,
    output [31:0] result16, result17, result18, result19, result20, result21, result22, result23,
    output [31:0] result24, result25, result26, result27, result28, result29, result30, result31
);
    // Implementation of CNN_Accelerator integrating sub-modules
endmodule

module CNN_Accelerator_Top(
    input clk,
    input reset,
    input start,
    input [31:0] norm_param,
    output [31:0] result0, result1, result2, result3, result4, result5, result6, result7,
    output [31:0] result8, result9, result10, result11, result12, result13, result14, result15,
    output [31:0] result16, result17, result18, result19, result20, result21, result22, result23,
    output [31:0] result24, result25, result26, result27, result28, result29, result30, result31,
    output done
);
    // Integrate ControlFSM, WeightBuffer, and CNN_Accelerator
endmodule

/*
 *-------------------------------------------------------------
 *
 * user_project_wrapper
 *
 * This wrapper enumerates all of the pins available to the
 * user for the user project.
 *
 *-------------------------------------------------------------
 */

module user_project_wrapper #(
    parameter BITS = 32
) (
`ifdef USE_POWER_PINS
    inout vdda1,    // User area 1 3.3V supply
    inout vdda2,    // User area 2 3.3V supply
    inout vssa1,    // User area 1 analog ground
    inout vssa2,    // User area 2 analog ground
    inout vccd1,    // User area 1 1.8V supply
    inout vccd2,    // User area 2 1.8v supply
    inout vssd1,    // User area 1 digital ground
    inout vssd2,    // User area 2 digital ground
`endif

    // Wishbone Slave ports (WB MI A)
    input wb_clk_i,
    input wb_rst_i,
    input wbs_stb_i,
    input wbs_cyc_i,
    input wbs_we_i,
    input [3:0] wbs_sel_i,
    input [31:0] wbs_dat_i,
    input [31:0] wbs_adr_i,
    output wbs_ack_o,
    output [31:0] wbs_dat_o,

    // Logic Analyzer Signals
    input  [127:0] la_data_in,
    output [127:0] la_data_out,
    input  [127:0] la_oenb,

    // IOs
    input  [`MPRJ_IO_PADS-1:0] io_in,
    output [`MPRJ_IO_PADS-1:0] io_out,
    output [`MPRJ_IO_PADS-1:0] io_oeb,

    // Analog (direct connection to GPIO pad---use with caution)
    // Note that analog I/O is not available on the 7 lowest-numbered
    // GPIO pads, and so the analog_io indexing is offset from the
    // GPIO indexing by 7 (also upper 2 GPIOs do not have analog_io).
    inout [`MPRJ_IO_PADS-10:0] analog_io,

    // Independent clock (on independent integer divider)
    input   user_clock2,

    // User maskable interrupt signals
    output [2:0] user_irq
);

/*--------------------------------------*/
/* User project is instantiated  here   */
/*--------------------------------------*/

// Additional signals for weight loading
wire [31:0] weight_data;
wire [4:0] weight_write_addr;
wire weight_write_enable;

// Assigning specific IO pins for weight loading
assign weight_data = io_in[31:0];
assign weight_write_addr = io_in[36:32];
assign weight_write_enable = io_in[37];

CNN_Accelerator_Top mprj (
`ifdef USE_POWER_PINS
    .vccd1(vccd1),  // User area 1 1.8V power
    .vssd1(vssd1),  // User area 1 digital ground
`endif

    .clk(wb_clk_i),
    .reset(wb_rst_i),
    .start(wbs_stb_i & wbs_cyc_i),  // Example start condition
    .norm_param(wbs_dat_i),         // Example parameter passing
    .result0(io_out[0]), .result1(io_out[1]), .result2(io_out[2]), .result3(io_out[3]),
    .result4(io_out[4]), .result5(io_out[5]), .result6(io_out[6]), .result7(io_out[7]),
    .result8(io_out[8]), .result9(io_out[9]), .result10(io_out[10]), .result11(io_out[11]),
    .result12(io_out[12]), .result13(io_out[13]), .result14(io_out[14]), .result15(io_out[15]),
    .result16(io_out[16]), .result17(io_out[17]), .result18(io_out[18]), .result19(io_out[19]),
    .result20(io_out[20]), .result21(io_out[21]), .result22(io_out[22]), .result23(io_out[23]),
    .result24(io_out[24]), .result25(io_out[25]), .result26(io_out[26]), .result27(io_out[27]),
    .result28(io_out[28]), .result29(io_out[29]), .result30(io_out[30]), .result31(io_out[31]),
    .done(user_irq[0]),             // Mapping the done signal to user IRQ

    // Connecting weight loading signals
    .weight_data0(weight_data), .weight_data1(weight_data), .weight_data2(weight_data), .weight_data3(weight_data),
    .weight_data4(weight_data), .weight_data5(weight_data), .weight_data6(weight_data), .weight_data7(weight_data),
    .weight_data8(weight_data), .weight_data9(weight_data), .weight_data10(weight_data), .weight_data11(weight_data),
    .weight_data12(weight_data), .weight_data13(weight_data), .weight_data14(weight_data), .weight_data15(weight_data),
    .weight_data16(weight_data), .weight_data17(weight_data), .weight_data18(weight_data), .weight_data19(weight_data),
    .weight_data20(weight_data), .weight_data21(weight_data), .weight_data22(weight_data), .weight_data23(weight_data),
    .weight_data24(weight_data), .weight_data25(weight_data), .weight_data26(weight_data), .weight_data27(weight_data),
    .weight_data28(weight_data), .weight_data29(weight_data), .weight_data30(weight_data), .weight_data31(weight_data),
    .weight_write_enable(weight_write_enable),
    .weight_write_addr(weight_write_addr),

    // Assuming other signals as required by the top module
    .line_data(io_in[31:0]), // Example, connect accordingly
    .line_write_enable(io_in[38]),
    .line_write_addr(io_in[42:39]),
    .sum_mode(io_in[43]),
    .norm_enable(io_in[44]),
    .relu_enable(io_in[45]),
    .pool_enable(io_in[46])
);

endmodule  // user_project_wrapper




`default_nettype wire
