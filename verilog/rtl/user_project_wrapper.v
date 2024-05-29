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

module CNN_Accelerator_Top(
    input clk,
    input reset,
    input start,
    input [31:0] norm_param,
    input [7:0] serial_weight_data,  // Serial weight data from off-chip memory
    input serial_weight_valid,       // Valid signal for serial weight data
    input [7:0] serial_line_data,    // Serial line data from off-chip memory
    input serial_line_valid,         // Valid signal for serial line data
    output reg [7:0] serial_result,  // Serial output for result data
    output reg serial_result_valid,  // Valid signal for serial result data
    output done
);

    // Registers to store the results
    reg [7:0] result [15:0];
    reg [3:0] result_index;

    // Control FSM signals
    wire weight_write_enable;
    wire line_write_enable;
    wire sum_mode;
    wire [7:0] weight_data;
    wire [3:0] weight_write_addr;
    wire [7:0] line_data;
    wire [3:0] line_write_addr;
    wire norm_enable;
    wire relu_enable;
    wire pool_enable;

    // Intermediate signals
    wire [255:0] products;
    wire [127:0] sum;
    wire [127:0] norm_out;

    // Instantiate the Control FSM
    ControlFSM control_fsm (
        .clk(clk),
        .reset(reset),
        .start(start),
        .norm_param(norm_param),
        .weight_write_enable(weight_write_enable),
        .line_write_enable(line_write_enable),
        .sum_mode(sum_mode),
        .weight_data(weight_data),
        .weight_write_addr(weight_write_addr),
        .line_data(line_data),
        .line_write_addr(line_write_addr),
        .norm_enable(norm_enable),
        .relu_enable(relu_enable),
        .pool_enable(pool_enable),
        .done(done),
        .serial_weight_data(serial_weight_data),
        .serial_weight_valid(serial_weight_valid),
        .serial_line_data(serial_line_data),
        .serial_line_valid(serial_line_valid)
    );

    // Instantiate the Weight Buffer
    WeightBuffer weight_buffer (
        .clk(clk),
        .reset(reset),
        .weight_data(weight_data),
        .write_enable(weight_write_enable),
        .write_addr(weight_write_addr),
        .weight_out(weight_out)
    );

    // Instantiate the Line Buffer
    LineBuffer line_buffer (
        .clk(clk),
        .reset(reset),
        .data_in(line_data),
        .write_enable(line_write_enable),
        .write_addr(line_write_addr),
        .read_addr(line_write_addr), // Example read address, should be controlled by FSM
        .data_out(line_out)
    );

    // Instantiate the MAC Array
    MACArray mac_array (
        .line_data(line_out),
        .weight_data(weight_out),
        .products(products)
    );

    // Instantiate the Configurable Adder Tree
    ConfigurableAdderTree adder_tree (
        .products(products),
        .sum_mode(sum_mode),
        .final_sum(sum)
    );

    // State machine for serializing the results
    localparam IDLE = 2'b00,
               SERIALIZE = 2'b01;

    reg [1:0] state, next_state;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= IDLE;
            result_index <= 0;
            serial_result_valid <= 0;
        end else begin
            state <= next_state;
            if (state == SERIALIZE) begin
                serial_result <= result[result_index];
                serial_result_valid <= 1;
                result_index <= result_index + 1;
                if (result_index == 15)
                    next_state <= IDLE;
            end else begin
                serial_result_valid <= 0;
                result_index <= 0;
            end
        end
    end

    always @(*) begin
        next_state = state;
        case (state)
            IDLE: if (done) next_state = SERIALIZE;
            SERIALIZE: if (result_index == 15) next_state = IDLE;
        endcase
    end

    // Instantiate the Norm module
    Norm norm_inst (
        .data_in(sum),
        .norm_param(norm_param[7:0]),  // Only using 8-bit part of norm_param for this example
        .data_out(norm_out)
    );

endmodule

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
/* User project is instantiated here    */
/*--------------------------------------*/

// Additional signals for weight and line data
wire [7:0] serial_weight_data = io_in[7:0];
wire serial_weight_valid = io_in[8];
wire [7:0] serial_line_data = io_in[15:8];
wire serial_line_valid = io_in[16];
wire [7:0] serial_result;
wire serial_result_valid;

CNN_Accelerator_Top mprj (
`ifdef USE_POWER_PINS
    .vccd1(vccd1),  // User area 1 1.8V power
    .vssd1(vssd1),  // User area 1 digital ground
`endif

    .clk(wb_clk_i),
    .reset(wb_rst_i),
    .start(wbs_stb_i & wbs_cyc_i),  // Example start condition
    .norm_param(wbs_dat_i),         // Example parameter passing
    .serial_weight_data(serial_weight_data),
    .serial_weight_valid(serial_weight_valid),
    .serial_line_data(serial_line_data),
    .serial_line_valid(serial_line_valid),
    .serial_result(serial_result),
    .serial_result_valid(serial_result_valid),
    .done(user_irq[0])             // Mapping the done signal to user IRQ
);

// Connecting the serial result to output pins
assign io_out[7:0] = serial_result;
assign io_out[8] = serial_result_valid;
assign io_oeb = {`MPRJ_IO_PADS{1'b0}}; // Set all IOs to output

endmodule  // user_project_wrapper

module ControlFSM(
    input clk,
    input reset,
    input start,
    input [31:0] norm_param,
    output reg weight_write_enable,
    output reg line_write_enable,
    output reg sum_mode,
    output reg [7:0] weight_data,
    output reg [3:0] weight_write_addr,
    output reg [7:0] line_data,
    output reg [3:0] line_write_addr,
    output reg norm_enable,
    output reg relu_enable,
    output reg pool_enable,
    output reg done,
    input [7:0] serial_weight_data,  // Serial weight data from top module
    input serial_weight_valid,       // Valid signal for serial weight data
    input [7:0] serial_line_data,    // Serial line data from top module
    input serial_line_valid          // Valid signal for serial line data
);
    reg [3:0] state, next_state;

    localparam IDLE = 4'b0000,
               LOAD_WEIGHTS = 4'b0001,
               LOAD_LINE = 4'b0010,
               COMPUTE = 4'b0011,
               NORM = 4'b0100,
               RELU = 4'b0101,
               POOL = 4'b0110,
               DONE = 4'b0111;

    always @(posedge clk or posedge reset) begin
        if (reset)
            state <= IDLE;
        else
            state <= next_state;
    end

    always @(*) begin
        next_state = state;
        weight_write_enable = 0;
        line_write_enable = 0;
        sum_mode = 0;
        weight_data = 0;
        weight_write_addr = 0;
        line_data = 0;
        line_write_addr = 0;
        norm_enable = 0;
        relu_enable = 0;
        pool_enable = 0;
        done = 0;

        case (state)
            IDLE: begin
                if (start)
                    next_state = LOAD_WEIGHTS;
            end
            LOAD_WEIGHTS: begin
                if (serial_weight_valid) begin
                    weight_write_enable = 1;
                    weight_data = serial_weight_data;
                    weight_write_addr = weight_write_addr + 1;
                    if (weight_write_addr == 4'b1111)
                        next_state = LOAD_LINE;
                end
            end
            LOAD_LINE: begin
                if (serial_line_valid) begin
                    line_write_enable = 1;
                    line_data = serial_line_data;
                    line_write_addr = line_write_addr + 1;
                    if (line_write_addr == 4'b1111)
                        next_state = COMPUTE;
                end
            end
            COMPUTE: begin
                sum_mode = 1;
                next_state = NORM;
            end
            NORM: begin
                norm_enable = 1;
                next_state = RELU;
            end
            RELU: begin
                relu_enable = 1;
                next_state = POOL;
            end
            POOL: begin
                pool_enable = 1;
                next_state = DONE;
            end
            DONE: begin
                done = 1;
                next_state = IDLE;
            end
        endcase
    end
endmodule

module Norm(
    input [127:0] data_in,
    input [7:0] norm_param,
    output [127:0] data_out
);
    genvar i;
    generate
        for (i = 0; i < 16; i = i + 1) begin : norm_loop
            assign data_out[i*8 +: 8] = (data_in[i*8 +: 8] * norm_param) >> 16; // Example normalization
        end
    endgenerate
endmodule

module ReLU(
    input [127:0] data_in,
    output [127:0] data_out
);
    genvar i;
    generate
        for (i = 0; i < 16; i = i + 1) begin : relu_loop
            assign data_out[i*8 +: 8] = (data_in[i*8 + 7] == 1'b1) ? 8'b0 : data_in[i*8 +: 8]; // Check if the MSB (sign bit) is 1
        end
    endgenerate
endmodule

module Pooling(
    input [7:0] data_in0,
    input [7:0] data_in1,
    input [7:0] data_in2,
    input [7:0] data_in3,
    output [7:0] data_out
);
    assign data_out = (data_in0 + data_in1 + data_in2 + data_in3) >> 2; // Average pooling
endmodule

module AdderTree(
    input [7:0] in0,
    input [7:0] in1,
    input [7:0] in2,
    input [7:0] in3,
    input [7:0] in4,
    input [7:0] in5,
    input [7:0] in6,
    input [7:0] in7,
    output [7:0] sum
);
    wire [7:0] sum1, sum2, sum3, sum4;
    assign sum1 = in0 + in1;
    assign sum2 = in2 + in3;
    assign sum3 = in4 + in5;
    assign sum4 = in6 + in7;
    assign sum = sum1 + sum2 + sum3 + sum4;
endmodule

module ConfigurableAdderTree(
    input [255:0] products, // Flattened array for products (16 x 8-bit)
    input sum_mode, // 0 for pointwise, 1 for depthwise
    output [127:0] final_sum // Flattened array for final sums (16 x 8-bit)
);

    wire [7:0] sum_depthwise [15:0];
    wire [7:0] sum_pointwise [1:0];

    // Unflattened products for easier readability
    wire [7:0] products_array [15:0];
    genvar i;
    generate
        for (i = 0; i < 16; i = i + 1) begin : products_loop
            assign products_array[i] = products[i*16 +: 16];
        end
    endgenerate

    // Depthwise summation
    assign sum_depthwise[0] = products_array[0] + products_array[1] + products_array[2] + products_array[3];
    assign sum_depthwise[1] = products_array[4] + products_array[5] + products_array[6] + products_array[7];
    assign sum_depthwise[2] = products_array[8] + products_array[9] + products_array[10] + products_array[11];
    assign sum_depthwise[3] = products_array[12] + products_array[13] + products_array[14] + products_array[15];

    // Pointwise summation
    AdderTree adder_tree_0 (.in0(products_array[0]), .in1(products_array[1]), .in2(products_array[2]), .in3(products_array[3]), .in4(products_array[4]), .in5(products_array[5]), .in6(products_array[6]), .in7(products_array[7]), .sum(sum_pointwise[0]));
    AdderTree adder_tree_1 (.in0(products_array[8]), .in1(products_array[9]), .in2(products_array[10]), .in3(products_array[11]), .in4(products_array[12]), .in5(products_array[13]), .in6(products_array[14]), .in7(products_array[15]), .sum(sum_pointwise[1]));

    // Flatten the final sums
    wire [7:0] final_sum_array [15:0];
    assign final_sum_array[0] = sum_mode ? sum_depthwise[0] : sum_pointwise[0];
    assign final_sum_array[1] = sum_mode ? sum_depthwise[1] : sum_pointwise[1];
    assign final_sum_array[2] = sum_mode ? sum_depthwise[2] : 8'b0;
    assign final_sum_array[3] = sum_mode ? sum_depthwise[3] : 8'b0;
    assign final_sum_array[4] = sum_mode ? sum_depthwise[4] : 8'b0;
    assign final_sum_array[5] = sum_mode ? sum_depthwise[5] : 8'b0;
    assign final_sum_array[6] = sum_mode ? sum_depthwise[6] : 8'b0;
    assign final_sum_array[7] = sum_mode ? sum_depthwise[7] : 8'b0;
    assign final_sum_array[8] = sum_mode ? sum_depthwise[8] : 8'b0;
    assign final_sum_array[9] = sum_mode ? sum_depthwise[9] : 8'b0;
    assign final_sum_array[10] = sum_mode ? sum_depthwise[10] : 8'b0;
    assign final_sum_array[11] = sum_mode ? sum_depthwise[11] : 8'b0;
    assign final_sum_array[12] = sum_mode ? sum_depthwise[12] : 8'b0;
    assign final_sum_array[13] = sum_mode ? sum_depthwise[13] : 8'b0;
    assign final_sum_array[14] = sum_mode ? sum_depthwise[14] : 8'b0;
    assign final_sum_array[15] = sum_mode ? sum_depthwise[15] : 8'b0;

    assign final_sum = {final_sum_array[0], final_sum_array[1], final_sum_array[2], final_sum_array[3],
                        final_sum_array[4], final_sum_array[5], final_sum_array[6], final_sum_array[7],
                        final_sum_array[8], final_sum_array[9], final_sum_array[10], final_sum_array[11],
                        final_sum_array[12], final_sum_array[13], final_sum_array[14], final_sum_array[15]};
endmodule

module WeightBuffer(
    input clk,
    input reset,
    input [7:0] weight_data,
    input write_enable,
    input [3:0] write_addr,
    output [127:0] weight_out  // Flattened array for weight data (16 x 8-bit)
);
    reg [7:0] weights [15:0];
    integer i;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 16; i = i + 1) begin
                weights[i] <= 8'b0;
            end
        end else if (write_enable) begin
            weights[write_addr] <= weight_data;
        end
    end

    // Flatten the output array
    assign weight_out = {weights[0], weights[1], weights[2], weights[3],
                         weights[4], weights[5], weights[6], weights[7],
                         weights[8], weights[9], weights[10], weights[11],
                         weights[12], weights[13], weights[14], weights[15]};
endmodule

module LineBuffer(
    input clk,
    input reset,
    input [7:0] data_in,
    input write_enable,
    input [3:0] write_addr,
    input [3:0] read_addr,
    output [127:0] data_out  // Flattened array for line data (16 x 8-bit)
);
    reg [7:0] line_data [15:0];
    integer i;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 16; i = i + 1) begin
                line_data[i] <= 8'b0;
            end
        end else if (write_enable) begin
            line_data[write_addr] <= data_in;
        end
    end
    
    // Flatten the output array
    assign data_out = {line_data[0], line_data[1], line_data[2], line_data[3],
                       line_data[4], line_data[5], line_data[6], line_data[7],
                       line_data[8], line_data[9], line_data[10], line_data[11],
                       line_data[12], line_data[13], line_data[14], line_data[15]};
endmodule

module MAC(
    input [7:0] a,
    input [7:0] b,
    output [15:0] product
);
    assign product = a * b;
endmodule

module MACArray(
    input [127:0] line_data,  // Flattened array for line data (16 x 8-bit)
    input [127:0] weight_data,  // Flattened array for weight data (16 x 8-bit)
    output [255:0] products  // Flattened array for products (16 x 16-bit)
);
    wire [7:0] line_data_array [15:0];
    wire [7:0] weight_data_array [15:0];
    wire [15:0] products_array [15:0];

    // Unflatten the input arrays
    genvar i;
    generate
        for (i = 0; i < 16; i = i + 1) begin : input_unflatten
            assign line_data_array[i] = line_data[i*8 +: 8];
            assign weight_data_array[i] = weight_data[i*8 +: 8];
        end
    endgenerate

    // Instantiate the MAC units
    generate
        for (i = 0; i < 16; i = i + 1) begin : mac
            MAC mac_unit (
                .a(line_data_array[i]),
                .b(weight_data_array[i]),
                .product(products_array[i])
            );
        end
    endgenerate

    // Flatten the output array
    generate
        for (i = 0; i < 16; i = i + 1) begin : output_flatten
            assign products[i*16 +: 16] = products_array[i];
        end
    endgenerate
endmodule












// module CNN_Accelerator(
//     input clk,
//     input reset,
//     input [7:0] serial_weight_data,  // Serial weight data from top module
//     input serial_weight_valid,       // Valid signal for serial weight data
//     input [7:0] serial_line_data,    // Serial line data from top module
//     input serial_line_valid,         // Valid signal for serial line data
//     input start,
//     input [31:0] norm_param,
//     output [7:0] result0, result1, result2, result3, result4, result5, result6, result7,
//     output [7:0] result8, result9, result10, result11, result12, result13, result14, result15,
//     output [7:0] result16, result17, result18, result19, result20, result21, result22, result23,
//     output [7:0] result24, result25, result26, result27, result28, result29, result30, result31,
//     output done
// );

//     wire weight_write_enable;
//     wire line_write_enable;
//     wire sum_mode;
//     wire [4:0] weight_write_addr;
//     wire [4:0] line_write_addr;
//     wire [7:0] weight_data;
//     wire [7:0] line_data;
//     wire norm_enable;
//     wire relu_enable;
//     wire pool_enable;

//     ControlFSM control_fsm (
//         .clk(clk),
//         .reset(reset),
//         .start(start),
//         .norm_param(norm_param),
//         .weight_write_enable(weight_write_enable),
//         .line_write_enable(line_write_enable),
//         .sum_mode(sum_mode),
//         .weight_data(weight_data),
//         .weight_write_addr(weight_write_addr),
//         .line_data(line_data),
//         .line_write_addr(line_write_addr),
//         .norm_enable(norm_enable),
//         .relu_enable(relu_enable),
//         .pool_enable(pool_enable),
//         .done(done),
//         .serial_weight_data(serial_weight_data),
//         .serial_weight_valid(serial_weight_valid),
//         .serial_line_data(serial_line_data),
//         .serial_line_valid(serial_line_valid)
//     );

//     wire [7:0] weight_out0, weight_out1, weight_out2, weight_out3, weight_out4;
//     wire [7:0] weight_out5, weight_out6, weight_out7, weight_out8, weight_out9;
//     wire [7:0] weight_out10, weight_out11, weight_out12, weight_out13, weight_out14;
//     wire [7:0] weight_out15, weight_out16, weight_out17, weight_out18, weight_out19;
//     wire [7:0] weight_out20, weight_out21, weight_out22, weight_out23, weight_out24;
//     wire [7:0] weight_out25, weight_out26, weight_out27, weight_out28, weight_out29;
//     wire [7:8] weight_out30, weight_out31;

//     WeightBuffer weight_buffer (
//         .clk(clk),
//         .reset(reset),
//         .weight_data(weight_data),
//         .write_enable(weight_write_enable),
//         .write_addr(weight_write_addr),
//         .weight_out0(weight_out0), .weight_out1(weight_out1), .weight_out2(weight_out2), .weight_out3(weight_out3),
//         .weight_out4(weight_out4), .weight_out5(weight_out5), .weight_out6(weight_out6), .weight_out7(weight_out7),
//         .weight_out8(weight_out8), .weight_out9(weight_out9), .weight_out10(weight_out10), .weight_out11(weight_out11),
//         .weight_out12(weight_out12), .weight_out13(weight_out13), .weight_out14(weight_out14), .weight_out15(weight_out15),
//         .weight_out16(weight_out16), .weight_out17(weight_out17), .weight_out18(weight_out18), .weight_out19(weight_out19),
//         .weight_out20(weight_out20), .weight_out21(weight_out21), .weight_out22(weight_out22), .weight_out23(weight_out23),
//         .weight_out24(weight_out24), .weight_out25(weight_out25), .weight_out26(weight_out26), .weight_out27(weight_out27),
//         .weight_out28(weight_out28), .weight_out29(weight_out29), .weight_out30(weight_out30), .weight_out31(weight_out31)
//     );

//     wire [7:0] line_out0, line_out1, line_out2, line_out3, line_out4;
//     wire [7:0] line_out5, line_out6, line_out7, line_out8, line_out9;
//     wire [7:0] line_out10, line_out11, line_out12, line_out13, line_out14;
//     wire [7:0] line_out15, line_out16, line_out17, line_out18, line_out19;
//     wire [7:0] line_out20, line_out21, line_out22, line_out23, line_out24;
//     wire [7:0] line_out25, line_out26, line_out27, line_out28, line_out29;
//     wire [7:0] line_out30, line_out31;

//     LineBuffer line_buffers [31:0] (
//         .clk(clk),
//         .reset(reset),
//         .data_in(line_data),
//         .write_enable(line_write_enable),
//         .write_addr(line_write_addr),
//         .read_addr(line_write_addr),
//         .data_out({line_out31, line_out30, line_out29, line_out28, line_out27, line_out26, line_out25, line_out24,
//                    line_out23, line_out22, line_out21, line_out20, line_out19, line_out18, line_out17, line_out16,
//                    line_out15, line_out14, line_out13, line_out12, line_out11, line_out10, line_out9, line_out8,
//                    line_out7, line_out6, line_out5, line_out4, line_out3, line_out2, line_out1, line_out0})
//     );

//     wire [15:0] products [0:31];

//     MACArray mac_array(
//         .a0(line_out0), .a1(line_out1), .a2(line_out2), .a3(line_out3), .a4(line_out4),
//         .a5(line_out5), .a6(line_out6), .a7(line_out7), .a8(line_out8), .a9(line_out9),
//         .a10(line_out10), .a11(line_out11), .a12(line_out12), .a13(line_out13), .a14(line_out14),
//         .a15(line_out15), .a16(line_out16), .a17(line_out17), .a18(line_out18), .a19(line_out19),
//         .a20(line_out20), .a21(line_out21), .a22(line_out22), .a23(line_out23), .a24(line_out24),
//         .a25(line_out25), .a26(line_out26), .a27(line_out27), .a28(line_out28), .a29(line_out29),
//         .a30(line_out30), .a31(line_out31),
//         .b0(weight_out0), .b1(weight_out1), .b2(weight_out2), .b3(weight_out3), .b4(weight_out4),
//         .b5(weight_out5), .b6(weight_out6), .b7(weight_out7), .b8(weight_out8), .b9(weight_out9),
//         .b10(weight_out10), .b11(weight_out11), .b12(weight_out12), .b13(weight_out13), .b14(weight_out14),
//         .b15(weight_out15), .b16(weight_out16), .b17(weight_out17), .b18(weight_out18), .b19(weight_out19),
//         .b20(weight_out20), .b21(weight_out21), .b22(weight_out22), .b23(weight_out23), .b24(weight_out24),
//         .b25(weight_out25), .b26(weight_out26), .b27(weight_out27), .b28(weight_out28), .b29(weight_out29),
//         .b30(weight_out30), .b31(weight_out31),
//         .product0(products[0]), .product1(products[1]), .product2(products[2]), .product3(products[3]),
//         .product4(products[4]), .product5(products[5]), .product6(products[6]), .product7(products[7]),
//         .product8(products[8]), .product9(products[9]), .product10(products[10]), .product11(products[11]),
//         .product12(products[12]), .product13(products[13]), .product14(products[14]), .product15(products[15]),
//         .product16(products[16]), .product17(products[17]), .product18(products[18]), .product19(products[19]),
//         .product20(products[20]), .product21(products[21]), .product22(products[22]), .product23(products[23]),
//         .product24(products[24]), .product25(products[25]), .product26(products[26]), .product27(products[27]),
//         .product28(products[28]), .product29(products[29]), .product30(products[30]), .product31(products[31])
//     );

//     wire [15:0] sum [0:31];

//     ConfigurableAdderTree adder_tree(
//         .products0(products[0]), .products1(products[1]), .products2(products[2]), .products3(products[3]),
//         .products4(products[4]), .products5(products[5]), .products6(products[6]), .products7(products[7]),
//         .products8(products[8]), .products9(products[9]), .products10(products[10]), .products11(products[11]),
//         .products12(products[12]), .products13(products[13]), .products14(products[14]), .products15(products[15]),
//         .products16(products[16]), .products17(products[17]), .products18(products[18]), .products19(products[19]),
//         .products20(products[20]), .products21(products[21]), .products22(products[22]), .products23(products[23]),
//         .products24(products[24]), .products25(products[25]), .products26(products[26]), .products27(products[27]),
//         .products28(products[28]), .products29(products[29]), .products30(products[30]), .products31(products[31]),
//         .sum_mode(sum_mode),
//         .final_sum0(sum[0]), .final_sum1(sum[1]), .final_sum2(sum[2]), .final_sum3(sum[3]),
//         .final_sum4(sum[4]), .final_sum5(sum[5]), .final_sum6(sum[6]), .final_sum7(sum[7]),
//         .final_sum8(sum[8]), .final_sum9(sum[9]), .final_sum10(sum[10]), .final_sum11(sum[11]),
//         .final_sum12(sum[12]), .final_sum13(sum[13]), .final_sum14(sum[14]), .final_sum15(sum[15]),
//         .final_sum16(sum[16]), .final_sum17(sum[17]), .final_sum18(sum[18]), .final_sum19(sum[19]),
//         .final_sum20(sum[20]), .final_sum21(sum[21]), .final_sum22(sum[22]), .final_sum23(sum[23]),
//         .final_sum24(sum[24]), .final_sum25(sum[25]), .final_sum26(sum[26]), .final_sum27(sum[27]),
//         .final_sum28(sum[28]), .final_sum29(sum[29]), .final_sum30(sum[30]), .final_sum31(sum[31])
//     );

//     wire [7:0] norm_out [0:31];
//     wire [7:0] relu_out [0:31];
//     wire [7:0] pool_out [0:31];

//     // Example Norm instantiation for demonstration
//     Norm norm [31:0] (
//         .data_in(sum),
//         .norm_param(norm_param[7:0]),  // Only using 8-bit part of norm_param for this example
//         .data_out(norm_out)
//     );

//     ReLU relu [31:0] (
//         .data_in(norm_out),
//         .data_out(relu_out)
//     );

//     Pooling pool [31:0] (
//         .data_in0(relu_out),
//         .data_in1(relu_out),
//         .data_in2(relu_out),
//         .data_in3(relu_out),
//         .data_out(pool_out)
//     );

//     assign result0 = pool_out[0];
//     assign result1 = pool_out[1];
//     assign result2 = pool_out[2];
//     assign result3 = pool_out[3];
//     assign result4 = pool_out[4];
//     assign result5 = pool_out[5];
//     assign result6 = pool_out[6];
//     assign result7 = pool_out[7];
//     assign result8 = pool_out[8];
//     assign result9 = pool_out[9];
//     assign result10 = pool_out[10];
//     assign result11 = pool_out[11];
//     assign result12 = pool_out[12];
//     assign result13 = pool_out[13];
//     assign result14 = pool_out[14];
//     assign result15 = pool_out[15];
//     assign result16 = pool_out[16];
//     assign result17 = pool_out[17];
//     assign result18 = pool_out[18];
//     assign result19 = pool_out[19];
//     assign result20 = pool_out[20];
//     assign result21 = pool_out[21];
//     assign result22 = pool_out[22];
//     assign result23 = pool_out[23];
//     assign result24 = pool_out[24];
//     assign result25 = pool_out[25];
//     assign result26 = pool_out[26];
//     assign result27 = pool_out[27];
//     assign result28 = pool_out[28];
//     assign result29 = pool_out[29];
//     assign result30 = pool_out[30];
//     assign result31 = pool_out[31];

// endmodule

// module WeightBuffer(
//     input clk,
//     input reset,
//     input [7:0] weight_data,
//     input write_enable,
//     input [4:0] write_addr,
//     output [7:0] weight_out0, weight_out1, weight_out2, weight_out3, weight_out4,
//     output [7:0] weight_out5, weight_out6, weight_out7, weight_out8, weight_out9,
//     output [7:0] weight_out10, weight_out11, weight_out12, weight_out13, weight_out14,
//     output [7:0] weight_out15, weight_out16, weight_out17, weight_out18, weight_out19,
//     output [7:0] weight_out20, weight_out21, weight_out22, weight_out23, weight_out24,
//     output [7:0] weight_out25, weight_out26, weight_out27, weight_out28, weight_out29,
//     output [7:0] weight_out30, weight_out31
// );
//     reg [7:0] weights [0:31];
//     integer i;

//     always @(posedge clk or posedge reset) begin
//         if (reset) begin
//             for (i = 0; i < 32; i = i + 1)
//                 weights[i] <= 8'b0;
//         end else if (write_enable) begin
//             weights[write_addr] <= weight_data;
//         end
//     end

//     assign weight_out0 = weights[0];
//     assign weight_out1 = weights[1];
//     assign weight_out2 = weights[2];
//     assign weight_out3 = weights[3];
//     assign weight_out4 = weights[4];
//     assign weight_out5 = weights[5];
//     assign weight_out6 = weights[6];
//     assign weight_out7 = weights[7];
//     assign weight_out8 = weights[8];
//     assign weight_out9 = weights[9];
//     assign weight_out10 = weights[10];
//     assign weight_out11 = weights[11];
//     assign weight_out12 = weights[12];
//     assign weight_out13 = weights[13];
//     assign weight_out14 = weights[14];
//     assign weight_out15 = weights[15];
//     assign weight_out16 = weights[16];
//     assign weight_out17 = weights[17];
//     assign weight_out18 = weights[18];
//     assign weight_out19 = weights[19];
//     assign weight_out20 = weights[20];
//     assign weight_out21 = weights[21];
//     assign weight_out22 = weights[22];
//     assign weight_out23 = weights[23];
//     assign weight_out24 = weights[24];
//     assign weight_out25 = weights[25];
//     assign weight_out26 = weights[26];
//     assign weight_out27 = weights[27];
//     assign weight_out28 = weights[28];
//     assign weight_out29 = weights[29];
//     assign weight_out30 = weights[30];
//     assign weight_out31 = weights[31];
// endmodule

// module LineBuffer(
//     input clk,
//     input reset,
//     input [7:0] data_in,
//     input write_enable,
//     input [4:0] write_addr,
//     input [4:0] read_addr,
//     output [7:0] data_out
// );
//     reg [7:0] line_data [0:31];
//     integer i;

//     always @(posedge clk or posedge reset) begin
//         if (reset) begin
//             for (i = 0; i < 32; i = i + 1)
//                 line_data[i] <= 8'b0;
//         end else if (write_enable) begin
//             line_data[write_addr] <= data_in;
//         end
//     end
    
//     assign data_out = line_data[read_addr];
// endmodule

// module MAC(
//     input [7:0] a,
//     input [7:0] b,
//     output [15:0] y
// );
//     assign y = a * b;
// endmodule

// module MACArray(
//     input [7:0] a0, input [7:0] a1, input [7:0] a2, input [7:0] a3,
//     input [7:0] a4, input [7:0] a5, input [7:0] a6, input [7:0] a7,
//     input [7:0] a8, input [7:0] a9, input [7:0] a10, input [7:0] a11,
//     input [7:0] a12, input [7:0] a13, input [7:0] a14, input [7:0] a15,
//     input [7:0] a16, input [7:0] a17, input [7:0] a18, input [7:0] a19,
//     input [7:0] a20, input [7:0] a21, input [7:0] a22, input [7:0] a23,
//     input [7:0] a24, input [7:0] a25, input [7:0] a26, input [7:0] a27,
//     input [7:0] a28, input [7:0] a29, input [7:0] a30, input [7:0] a31,
//     input [7:0] b0, input [7:0] b1, input [7:0] b2, input [7:0] b3,
//     input [7:0] b4, input [7:0] b5, input [7:0] b6, input [7:0] b7,
//     input [7:0] b8, input [7:0] b9, input [7:0] b10, input [7:0] b11,
//     input [7:0] b12, input [7:0] b13, input [7:0] b14, input [7:0] b15,
//     input [7:0] b16, input [7:0] b17, input [7:0] b18, input [7:0] b19,
//     input [7:0] b20, input [7:0] b21, input [7:0] b22, input [7:0] b23,
//     input [7:0] b24, input [7:0] b25, input [7:0] b26, input [7:0] b27,
//     input [7:0] b28, input [7:0] b29, input [7:0] b30, input [7:0] b31,
//     output [15:0] product0, output [15:0] product1, output [15:0] product2, output [15:0] product3,
//     output [15:0] product4, output [15:0] product5, output [15:0] product6, output [15:0] product7,
//     output [15:0] product8, output [15:0] product9, output [15:0] product10, output [15:0] product11,
//     output [15:0] product12, output [15:0] product13, output [15:0] product14, output [15:0] product15,
//     output [15:0] product16, output [15:0] product17, output [15:0] product18, output [15:0] product19,
//     output [15:0] product20, output [15:0] product21, output [15:0] product22, output [15:0] product23,
//     output [15:0] product24, output [15:0] product25, output [15:0] product26, output [15:0] product27,
//     output [15:0] product28, output [15:0] product29, output [15:0] product30, output [15:0] product31
// );
//     MAC mac0 (.a(a0), .b(b0), .y(product0));
//     MAC mac1 (.a(a1), .b(b1), .y(product1));
//     MAC mac2 (.a(a2), .b(b2), .y(product2));
//     MAC mac3 (.a(a3), .b(b3), .y(product3));
//     MAC mac4 (.a(a4), .b(b4), .y(product4));
//     MAC mac5 (.a(a5), .b(b5), .y(product5));
//     MAC mac6 (.a(a6), .b(b6), .y(product6));
//     MAC mac7 (.a(a7), .b(b7), .y(product7));
//     MAC mac8 (.a(a8), .b(b8), .y(product8));
//     MAC mac9 (.a(a9), .b(b9), .y(product9));
//     MAC mac10 (.a(a10), .b(b10), .y(product10));
//     MAC mac11 (.a(a11), .b(b11), .y(product11));
//     MAC mac12 (.a(a12), .b(b12), .y(product12));
//     MAC mac13 (.a(a13), .b(b13), .y(product13));
//     MAC mac14 (.a(a14), .b(b14), .y(product14));
//     MAC mac15 (.a(a15), .b(b15), .y(product15));
//     MAC mac16 (.a(a16), .b(b16), .y(product16));
//     MAC mac17 (.a(a17), .b(b17), .y(product17));
//     MAC mac18 (.a(a18), .b(b18), .y(product18));
//     MAC mac19 (.a(a19), .b(b19), .y(product19));
//     MAC mac20 (.a(a20), .b(b20), .y(product20));
//     MAC mac21 (.a(a21), .b(b21), .y(product21));
//     MAC mac22 (.a(a22), .b(b22), .y(product22));
//     MAC mac23 (.a(a23), .b(b23), .y(product23));
//     MAC mac24 (.a(a24), .b(b24), .y(product24));
//     MAC mac25 (.a(a25), .b(b25), .y(product25));
//     MAC mac26 (.a(a26), .b(b26), .y(product26));
//     MAC mac27 (.a(a27), .b(b27), .y(product27));
//     MAC mac28 (.a(a28), .b(b28), .y(product28));
//     MAC mac29 (.a(a29), .b(b29), .y(product29));
//     MAC mac30 (.a(a30), .b(b30), .y(product30));
//     MAC mac31 (.a(a31), .b(b31), .y(product31));
// endmodule

// module AdderTree(
//     input [15:0] in0,
//     input [15:0] in1,
//     input [15:0] in2,
//     input [15:0] in3,
//     input [15:0] in4,
//     input [15:0] in5,
//     input [15:0] in6,
//     input [15:0] in7,
//     output [15:0] sum
// );
//     wire [15:0] sum1, sum2, sum3, sum4;

//     assign sum1 = in0 + in1;
//     assign sum2 = in2 + in3;
//     assign sum3 = in4 + in5;
//     assign sum4 = in6 + in7;

//     wire [15:0] sum5, sum6;

//     assign sum5 = sum1 + sum2;
//     assign sum6 = sum3 + sum4;

//     assign sum = sum5 + sum6;
// endmodule

// module ConfigurableAdderTree(
//     input [15:0] products0, input [15:0] products1, input [15:0] products2, input [15:0] products3,
//     input [15:0] products4, input [15:0] products5, input [15:0] products6, input [15:0] products7,
//     input [15:0] products8, input [15:0] products9, input [15:0] products10, input [15:0] products11,
//     input [15:0] products12, input [15:0] products13, input [15:0] products14, input [15:0] products15,
//     input [15:0] products16, input [15:0] products17, input [15:0] products18, input [15:0] products19,
//     input [15:0] products20, input [15:0] products21, input [15:0] products22, input [15:0] products23,
//     input [15:0] products24, input [15:0] products25, input [15:0] products26, input [15:0] products27,
//     input [15:0] products28, input [15:0] products29, input [15:0] products30, input [15:0] products31,
//     input sum_mode,  // 0 for pointwise, 1 for depthwise
//     output [15:0] final_sum0, final_sum1, final_sum2, final_sum3,
//     output [15:0] final_sum4, final_sum5, final_sum6, final_sum7,
//     output [15:0] final_sum8, final_sum9, final_sum10, final_sum11,
//     output [15:0] final_sum12, final_sum13, final_sum14, final_sum15,
//     output [15:0] final_sum16, final_sum17, final_sum18, final_sum19,
//     output [15:0] final_sum20, final_sum21, final_sum22, final_sum23,
//     output [15:0] final_sum24, final_sum25, final_sum26, final_sum27,
//     output [15:0] final_sum28, final_sum29, final_sum30, final_sum31
// );

//     wire [15:0] sum_depthwise [0:31];
//     wire [15:0] sum_pointwise [0:31];

//     // Depthwise summation
//     assign sum_depthwise[0] = products0 + products1 + products2 + products3 + products4 + products5 + products6 + products7;
//     assign sum_depthwise[1] = products8 + products9 + products10 + products11 + products12 + products13 + products14 + products15;
//     assign sum_depthwise[2] = products16 + products17 + products18 + products19 + products20 + products21 + products22 + products23;
//     assign sum_depthwise[3] = products24 + products25 + products26 + products27 + products28 + products29 + products30 + products31;

//     // Pointwise summation
//     AdderTree adder_tree_0 (.in0(products0), .in1(products1), .in2(products2), .in3(products3), .in4(products4), .in5(products5), .in6(products6), .in7(products7), .sum(sum_pointwise[0]));
//     AdderTree adder_tree_1 (.in0(products8), .in1(products9), .in2(products10), .in3(products11), .in4(products12), .in5(products13), .in6(products14), .in7(products15), .sum(sum_pointwise[1]));
//     AdderTree adder_tree_2 (.in0(products16), .in1(products17), .in2(products18), .in3(products19), .in4(products20), .in5(products21), .in6(products22), .in7(products23), .sum(sum_pointwise[2]));
//     AdderTree adder_tree_3 (.in0(products24), .in1(products25), .in2(products26), .in3(products27), .in4(products28), .in5(products29), .in6(products30), .in7(products31), .sum(sum_pointwise[3]));

//     // Multiplex the final sum based on sum_mode
//     assign final_sum0 = sum_mode ? sum_depthwise[0] : sum_pointwise[0];
//     assign final_sum1 = sum_mode ? sum_depthwise[1] : sum_pointwise[1];
//     assign final_sum2 = sum_mode ? sum_depthwise[2] : sum_pointwise[2];
//     assign final_sum3 = sum_mode ? sum_depthwise[3] : sum_pointwise[3];
//     assign final_sum4 = sum_mode ? sum_depthwise[4] : sum_pointwise[4];
//     assign final_sum5 = sum_mode ? sum_depthwise[5] : sum_pointwise[5];
//     assign final_sum6 = sum_mode ? sum_depthwise[6] : sum_pointwise[6];
//     assign final_sum7 = sum_mode ? sum_depthwise[7] : sum_pointwise[7];
//     assign final_sum8 = sum_mode ? sum_depthwise[8] : sum_pointwise[8];
//     assign final_sum9 = sum_mode ? sum_depthwise[9] : sum_pointwise[9];
//     assign final_sum10 = sum_mode ? sum_depthwise[10] : sum_pointwise[10];
//     assign final_sum11 = sum_mode ? sum_depthwise[11] : sum_pointwise[11];
//     assign final_sum12 = sum_mode ? sum_depthwise[12] : sum_pointwise[12];
//     assign final_sum13 = sum_mode ? sum_depthwise[13] : sum_pointwise[13];
//     assign final_sum14 = sum_mode ? sum_depthwise[14] : sum_pointwise[14];
//     assign final_sum15 = sum_mode ? sum_depthwise[15] : sum_pointwise[15];
//     assign final_sum16 = sum_mode ? sum_depthwise[16] : sum_pointwise[16];
//     assign final_sum17 = sum_mode ? sum_depthwise[17] : sum_pointwise[17];
//     assign final_sum18 = sum_mode ? sum_depthwise[18] : sum_pointwise[18];
//     assign final_sum19 = sum_mode ? sum_depthwise[19] : sum_pointwise[19];
//     assign final_sum20 = sum_mode ? sum_depthwise[20] : sum_pointwise[20];
//     assign final_sum21 = sum_mode ? sum_depthwise[21] : sum_pointwise[21];
//     assign final_sum22 = sum_mode ? sum_depthwise[22] : sum_pointwise[22];
//     assign final_sum23 = sum_mode ? sum_depthwise[23] : sum_pointwise[23];
//     assign final_sum24 = sum_mode ? sum_depthwise[24] : sum_pointwise[24];
//     assign final_sum25 = sum_mode ? sum_depthwise[25] : sum_pointwise[25];
//     assign final_sum26 = sum_mode ? sum_depthwise[26] : sum_pointwise[26];
//     assign final_sum27 = sum_mode ? sum_depthwise[27] : sum_pointwise[27];
//     assign final_sum28 = sum_mode ? sum_depthwise[28] : sum_pointwise[28];
//     assign final_sum29 = sum_mode ? sum_depthwise[29] : sum_pointwise[29];
//     assign final_sum30 = sum_mode ? sum_depthwise[30] : sum_pointwise[30];
//     assign final_sum31 = sum_mode ? sum_depthwise[31] : sum_pointwise[31];
// endmodule

// module Norm(
//     input [15:0] data_in,
//     input [7:0] norm_param,
//     output [7:0] data_out
// );
//     assign data_out = (data_in * norm_param) >> 16; // Example normalization
// endmodule

// module ReLU(
//     input [7:0] data_in,
//     output [7:0] data_out
// );
//     assign data_out = (data_in[7] == 1'b1) ? 8'b0 : data_in; // Check if the MSB (sign bit) is 1
// endmodule

// module Pooling(
//     input [7:0] data_in0,
//     input [7:0] data_in1,
//     input [7:0] data_in2,
//     input [7:0] data_in3,
//     output [7:0] data_out
// );
//     assign data_out = (data_in0 + data_in1 + data_in2 + data_in3) >> 2; // Average pooling
// endmodule

// module ControlFSM(
//     input clk,
//     input reset,
//     input start,
//     input [7:0] norm_param,
//     output reg weight_write_enable,
//     output reg line_write_enable,
//     output reg sum_mode,
//     output reg [7:0] weight_data,
//     output reg [4:0] weight_write_addr,
//     output reg [7:0] line_data,
//     output reg [4:0] line_write_addr,
//     output reg norm_enable,
//     output reg relu_enable,
//     output reg pool_enable,
//     output reg done,
//     input [7:0] serial_weight_data,  // Serial weight data from top module
//     input serial_weight_valid,        // Valid signal for serial weight data
//     input [7:0] serial_line_data,    // Serial line data from top module
//     input serial_line_valid           // Valid signal for serial line data
// );
//     reg [3:0] state, next_state;

//     localparam IDLE = 4'b0000,
//                LOAD_WEIGHTS = 4'b0001,
//                LOAD_LINE = 4'b0010,
//                COMPUTE = 4'b0011,
//                NORM = 4'b0100,
//                RELU = 4'b0101,
//                POOL = 4'b0110,
//                DONE = 4'b0111;

//     always @(posedge clk or posedge reset) begin
//         if (reset)
//             state <= IDLE;
//         else
//             state <= next_state;
//     end

//     always @(*) begin
//         next_state = state;
//         weight_write_enable = 0;
//         line_write_enable = 0;
//         sum_mode = 0;
//         weight_data = 0;
//         weight_write_addr = 0;
//         line_data = 0;
//         line_write_addr = 0;
//         norm_enable = 0;
//         relu_enable = 0;
//         pool_enable = 0;
//         done = 0;

//         case (state)
//             IDLE: begin
//                 if (start)
//                     next_state = LOAD_WEIGHTS;
//             end
//             LOAD_WEIGHTS: begin
//                 if (serial_weight_valid) begin
//                     weight_write_enable = 1;
//                     weight_data = serial_weight_data;
//                     weight_write_addr = weight_write_addr + 1;
//                     if (weight_write_addr == 5'b11111)
//                         next_state = LOAD_LINE;
//                 end
//             end
//             LOAD_LINE: begin
//                 if (serial_line_valid) begin
//                     line_write_enable = 1;
//                     line_data = serial_line_data;
//                     line_write_addr = line_write_addr + 1;
//                     if (line_write_addr == 5'b11111)
//                         next_state = COMPUTE;
//                 end
//             end
//             COMPUTE: begin
//                 sum_mode = 1;
//                 next_state = NORM;
//             end
//             NORM: begin
//                 norm_enable = 1;
//                 next_state = RELU;
//             end
//             RELU: begin
//                 relu_enable = 1;
//                 next_state = POOL;
//             end
//             POOL: begin
//                 pool_enable = 1;
//                 next_state = DONE;
//             end
//             DONE: begin
//                 done = 1;
//                 next_state = IDLE;
//             end
//         endcase
//     end
// endmodule

// // module CNN_Accelerator_Top(
// //     input clk,
// //     input reset,
// //     input start,
// //     input [31:0] norm_param,
// //     input [31:0] serial_weight_data,  // Serial weight data from top module
// //     input serial_weight_valid,        // Valid signal for serial weight data
// //     input [31:0] serial_line_data,    // Serial line data from top module
// //     input serial_line_valid,          // Valid signal for serial line data
// //     output [31:0] result0, result1, result2, result3, result4, result5, result6, result7,
// //     output [31:0] result8, result9, result10, result11, result12, result13, result14, result15,
// //     output [31:0] result16, result17, result18, result19, result20, result21, result22, result23,
// //     output [31:0] result24, result25, result26, result27, result28, result29, result30, result31,
// //     output done
// // );
// //     wire weight_write_enable;
// //     wire line_write_enable;
// //     wire sum_mode;
// //     wire [31:0] weight_data;
// //     wire [4:0] weight_write_addr;
// //     wire [31:0] line_data;
// //     wire [4:0] line_write_addr;
// //     wire norm_enable;
// //     wire relu_enable;
// //     wire pool_enable;

// //     ControlFSM control_fsm(
// //         .clk(clk),
// //         .reset(reset),
// //         .start(start),
// //         .norm_param(norm_param),
// //         .weight_write_enable(weight_write_enable),
// //         .line_write_enable(line_write_enable),
// //         .sum_mode(sum_mode),
// //         .weight_data(weight_data),
// //         .weight_write_addr(weight_write_addr),
// //         .line_data(line_data),
// //         .line_write_addr(line_write_addr),
// //         .norm_enable(norm_enable),
// //         .relu_enable(relu_enable),
// //         .pool_enable(pool_enable),
// //         .done(done),
// //         .serial_weight_data(serial_weight_data),
// //         .serial_weight_valid(serial_weight_valid),
// //         .serial_line_data(serial_line_data),
// //         .serial_line_valid(serial_line_valid)
// //     );

// //     WeightBuffer weight_buffer(
// //         .clk(clk),
// //         .reset(reset),
// //         .weight_data(weight_data),
// //         .write_enable(weight_write_enable),
// //         .write_addr(weight_write_addr),
// //         .weight_out0(weight_out[0]), .weight_out1(weight_out[1]), .weight_out2(weight_out[2]), .weight_out3(weight_out[3]),
// //         .weight_out4(weight_out[4]), .weight_out5(weight_out[5]), .weight_out6(weight_out[6]), .weight_out7(weight_out[7]),
// //         .weight_out8(weight_out[8]), .weight_out9(weight_out[9]), .weight_out10(weight_out[10]), .weight_out11(weight_out[11]),
// //         .weight_out12(weight_out[12]), .weight_out13(weight_out[13]), .weight_out14(weight_out[14]), .weight_out15(weight_out[15]),
// //         .weight_out16(weight_out[16]), .weight_out17(weight_out[17]), .weight_out18(weight_out[18]), .weight_out19(weight_out[19]),
// //         .weight_out20(weight_out[20]), .weight_out21(weight_out[21]), .weight_out22(weight_out[22]), .weight_out23(weight_out[23]),
// //         .weight_out24(weight_out[24]), .weight_out25(weight_out[25]), .weight_out26(weight_out[26]), .weight_out27(weight_out[27]),
// //         .weight_out28(weight_out[28]), .weight_out29(weight_out[29]), .weight_out30(weight_out[30]), .weight_out31(weight_out[31])
// //     );

// //     CNN_Accelerator cnn_accel(
// //         .clk(clk),
// //         .reset(reset),
// //         .weight_data0(weight_out[0]), .weight_data1(weight_out[1]), .weight_data2(weight_out[2]), .weight_data3(weight_out[3]),
// //         .weight_data4(weight_out[4]), .weight_data5(weight_out[5]), .weight_data6(weight_out[6]), .weight_data7(weight_out[7]),
// //         .weight_data8(weight_out[8]), .weight_data9(weight_out[9]), .weight_data10(weight_out[10]), .weight_data11(weight_out[11]),
// //         .weight_data12(weight_out[12]), .weight_data13(weight_out[13]), .weight_data14(weight_out[14]), .weight_data15(weight_out[15]),
// //         .weight_data16(weight_out[16]), .weight_data17(weight_out[17]), .weight_data18(weight_out[18]), .weight_data19(weight_out[19]),
// //         .weight_data20(weight_out[20]), .weight_data21(weight_out[21]), .weight_data22(weight_out[22]), .weight_data23(weight_out[23]),
// //         .weight_data24(weight_out[24]), .weight_data25(weight_out[25]), .weight_data26(weight_out[26]), .weight_data27(weight_out[27]),
// //         .weight_data28(weight_out[28]), .weight_data29(weight_out[29]), .weight_data30(weight_out[30]), .weight_data31(weight_out[31]),
// //         .weight_write_enable(weight_write_enable),
// //         .weight_write_addr(weight_write_addr),
// //         .line_data(line_data),
// //         .line_write_enable(line_write_enable),
// //         .line_write_addr(line_write_addr),
// //         .sum_mode(sum_mode),
// //         .norm_param(norm_param),
// //         .norm_enable(norm_enable),
// //         .relu_enable(relu_enable),
// //         .pool_enable(pool_enable),
// //         .result0(result0), .result1(result1), .result2(result2), .result3(result3),
// //         .result4(result4), .result5(result5), .result6(result6), .result7(result7),
// //         .result8(result8), .result9(result9), .result10(result10), .result11(result11),
// //         .result12(result12), .result13(result13), .result14(result14), .result15(result15),
// //         .result16(result16), .result17(result17), .result18(result18), .result19(result19),
// //         .result20(result20), .result21(result21), .result22(result22), .result23(result23),
// //         .result24(result24), .result25(result25), .result26(result26), .result27(result27),
// //         .result28(result28), .result29(result29), .result30(result30), .result31(result31)
// //     );
// // endmodule


// module CNN_Accelerator_Top(
//     input clk,
//     input reset,
//     input start,
//     input [31:0] norm_param,
//     input [7:0] serial_weight_data,  // Serial weight data from off-chip memory
//     input serial_weight_valid,       // Valid signal for serial weight data
//     input [7:0] serial_line_data,    // Serial line data from off-chip memory
//     input serial_line_valid,         // Valid signal for serial line data
//     output reg [7:0] serial_result,  // Serial output for result data
//     output reg serial_result_valid,  // Valid signal for serial result data
//     output done
// );

//     // Registers to store the results
//     reg [7:0] result [0:31];
//     reg [4:0] result_index;
//     reg [7:0] sum [0:31];  // Example sum array for the Norm module

//     // Instantiate the Control FSM
//     ControlFSM control_fsm (
//         .clk(clk),
//         .reset(reset),
//         .start(start),
//         .norm_param(norm_param),
//         .weight_write_enable(weight_write_enable),
//         .line_write_enable(line_write_enable),
//         .sum_mode(sum_mode),
//         .weight_data(weight_data),
//         .weight_write_addr(weight_write_addr),
//         .line_data(line_data),
//         .line_write_addr(line_write_addr),
//         .norm_enable(norm_enable),
//         .relu_enable(relu_enable),
//         .pool_enable(pool_enable),
//         .done(done),
//         .serial_weight_data(serial_weight_data),
//         .serial_weight_valid(serial_weight_valid),
//         .serial_line_data(serial_line_data),
//         .serial_line_valid(serial_line_valid)
//     );

//     // State machine for serializing the results
//     localparam IDLE = 2'b00,
//                SERIALIZE = 2'b01;

//     reg [1:0] state, next_state;

//     always @(posedge clk or posedge reset) begin
//         if (reset) begin
//             state <= IDLE;
//             result_index <= 0;
//             serial_result_valid <= 0;
//         end else begin
//             state <= next_state;
//             if (state == SERIALIZE) begin
//                 serial_result <= result[result_index];
//                 serial_result_valid <= 1;
//                 result_index <= result_index + 1;
//                 if (result_index == 31)
//                     next_state <= IDLE;
//             end else begin
//                 serial_result_valid <= 0;
//                 result_index <= 0;
//             end
//         end
//     end

//     always @(*) begin
//         next_state = state;
//         case (state)
//             IDLE: if (done) next_state = SERIALIZE;
//             SERIALIZE: if (result_index == 31) next_state = IDLE;
//         endcase
//     end

    

// endmodule

// /*
//  *-------------------------------------------------------------
//  *
//  * user_project_wrapper
//  *
//  * This wrapper enumerates all of the pins available to the
//  * user for the user project.
//  *
//  *-------------------------------------------------------------
//  */

// module user_project_wrapper #(
//     parameter BITS = 32
// ) (
// `ifdef USE_POWER_PINS
//     inout vdda1,    // User area 1 3.3V supply
//     inout vdda2,    // User area 2 3.3V supply
//     inout vssa1,    // User area 1 analog ground
//     inout vssa2,    // User area 2 analog ground
//     inout vccd1,    // User area 1 1.8V supply
//     inout vccd2,    // User area 2 1.8v supply
//     inout vssd1,    // User area 1 digital ground
//     inout vssd2,    // User area 2 digital ground
// `endif

//     // Wishbone Slave ports (WB MI A)
//     input wb_clk_i,
//     input wb_rst_i,
//     input wbs_stb_i,
//     input wbs_cyc_i,
//     input wbs_we_i,
//     input [3:0] wbs_sel_i,
//     input [31:0] wbs_dat_i,
//     input [31:0] wbs_adr_i,
//     output wbs_ack_o,
//     output [31:0] wbs_dat_o,

//     // Logic Analyzer Signals
//     input  [127:0] la_data_in,
//     output [127:0] la_data_out,
//     input  [127:0] la_oenb,

//     // IOs
//     input  [`MPRJ_IO_PADS-1:0] io_in,
//     output [`MPRJ_IO_PADS-1:0] io_out,
//     output [`MPRJ_IO_PADS-1:0] io_oeb,

//     // Analog (direct connection to GPIO pad---use with caution)
//     // Note that analog I/O is not available on the 7 lowest-numbered
//     // GPIO pads, and so the analog_io indexing is offset from the
//     // GPIO indexing by 7 (also upper 2 GPIOs do not have analog_io).
//     inout [`MPRJ_IO_PADS-10:0] analog_io,

//     // Independent clock (on independent integer divider)
//     input   user_clock2,

//     // User maskable interrupt signals
//     output [2:0] user_irq
// );

// /*--------------------------------------*/
// /* User project is instantiated here    */
// /*--------------------------------------*/

// // Additional signals for weight and line data
// wire [7:0] serial_weight_data = io_in[7:0];
// wire serial_weight_valid = io_in[8];
// wire [7:0] serial_line_data = io_in[15:8];
// wire serial_line_valid = io_in[16];
// wire [7:0] serial_result;
// wire serial_result_valid;

// CNN_Accelerator_Top mprj (
// `ifdef USE_POWER_PINS
//     .vccd1(vccd1),  // User area 1 1.8V power
//     .vssd1(vssd1),  // User area 1 digital ground
// `endif

//     .clk(wb_clk_i),
//     .reset(wb_rst_i),
//     .start(wbs_stb_i & wbs_cyc_i),  // Example start condition
//     .norm_param(wbs_dat_i),         // Example parameter passing
//     .serial_weight_data(serial_weight_data),
//     .serial_weight_valid(serial_weight_valid),
//     .serial_line_data(serial_line_data),
//     .serial_line_valid(serial_line_valid),
//     .serial_result(serial_result),
//     .serial_result_valid(serial_result_valid),
//     .done(user_irq[0])             // Mapping the done signal to user IRQ
// );

// // Connecting the serial result to output pins
// assign io_out[7:0] = serial_result;
// assign io_out[8] = serial_result_valid;
// assign io_oeb = {`MPRJ_IO_PADS{1'b0}}; // Set all IOs to output

// endmodule  // user_project_wrapper


`default_nettype wire
