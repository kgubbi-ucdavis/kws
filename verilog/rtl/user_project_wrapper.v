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
    .result(io_out),                // Mapping the result to IO pins
    .done(user_irq[0]),             // Mapping the done signal to user IRQ

    // Connecting weight loading signals
    .weight_data(weight_data),
    .weight_write_enable(weight_write_enable),
    .weight_write_addr(weight_write_addr),

    // Assuming other signals as required by the top module
    .line_data(line_data), // Example, connect accordingly
    .line_write_enable(line_write_enable),
    .line_write_addr(line_write_addr),
    .sum_mode(sum_mode),
    .norm_enable(norm_enable),
    .relu_enable(relu_enable),
    .pool_enable(pool_enable)
);

endmodule  // user_project_wrapper

module WeightBuffer(
    input clk,
    input reset,
    input [31:0] weight_data,
    input write_enable,
    input [4:0] write_addr,
    output reg [31:0] weight_out [0:31]
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

    always @(posedge clk) begin
        for (i = 0; i < 32; i = i + 1)
            weight_out[i] <= weights[i];
    end
endmodule

module LineBuffer(
    input clk,
    input reset,
    input [31:0] data_in,
    input write_enable,
    input [4:0] write_addr,
    input [4:0] read_addr,
    output reg [31:0] data_out
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

    always @(posedge clk) begin
        data_out <= line_data[read_addr];
    end
endmodule

module MAC(
    input [31:0] a,
    input [31:0] b,
    output [31:0] y
);
    assign y = a * b;
endmodule

module MACArray(
    input [31:0] a [0:31],
    input [31:0] b [0:31],
    output [31:0] product [0:31]
);
    genvar i;
    generate
        for (i = 0; i < 32; i = i + 1) begin : mac_array
            MAC mac_inst (
                .a(a[i]),
                .b(b[i]),
                .y(product[i])
            );
        end
    endgenerate
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
    wire [31:0] sum5, sum6;

    assign sum1 = in0 + in1;
    assign sum2 = in2 + in3;
    assign sum3 = in4 + in5;
    assign sum4 = in6 + in7;
    assign sum5 = sum1 + sum2;
    assign sum6 = sum3 + sum4;
    assign sum = sum5 + sum6;
endmodule

module ConfigurableAdderTree(
    input [31:0] products [0:31],
    input sum_mode,
    output reg [31:0] final_sum [0:31]
);
    reg [31:0] sum_depthwise [0:31];
    wire [31:0] sum_pointwise;

    AdderTree adder_tree_inst (
        .in0(products[0]),
        .in1(products[1]),
        .in2(products[2]),
        .in3(products[3]),
        .in4(products[4]),
        .in5(products[5]),
        .in6(products[6]),
        .in7(products[7]),
        .sum(sum_pointwise)
    );

    integer i;
    always @(*) begin
        for (i = 0; i < 32; i = i + 1) begin
            sum_depthwise[i] = products[i];
            final_sum[i] = sum_mode ? sum_depthwise[i] : (i < 8 ? sum_pointwise : 32'b0);
        end
    end
endmodule

module Norm(
    input [31:0] data_in,
    input [31:0] norm_param,
    output [31:0] data_out
);
    assign data_out = (data_in * norm_param) >> 16; // Example normalization
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
    assign data_out = (data_in0 + data_in1 + data_in2 + data_in3) >> 2; // Average pooling
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
    typedef enum reg [2:0] {
        IDLE,
        LOAD_WEIGHTS,
        LOAD_INPUTS,
        COMPUTE,
        NORMALIZE,
        ACTIVATE,
        POOL,
        DONE
    } state_t;

    reg [2:0] state, next_state;
    reg [4:0] weight_counter;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= IDLE;
        end else begin
            state <= next_state;
        end
    end

    always @(*) begin
        case (state)
            IDLE: next_state = start ? LOAD_WEIGHTS : IDLE;
            LOAD_WEIGHTS: next_state = LOAD_INPUTS;
            LOAD_INPUTS: next_state = COMPUTE;
            COMPUTE: next_state = NORMALIZE;
            NORMALIZE: next_state = ACTIVATE;
            ACTIVATE: next_state = POOL;
            POOL: next_state = DONE;
            DONE: next_state = IDLE;
            default: next_state = IDLE;
        endcase
    end

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            weight_write_enable <= 0;
            line_write_enable <= 0;
            sum_mode <= 0;
            norm_enable <= 0;
            relu_enable <= 0;
            pool_enable <= 0;
            weight_counter <= 0;
            done <= 0;
        end else begin
            case (state)
                IDLE: begin
                    weight_write_enable <= 0;
                    line_write_enable <= 0;
                    norm_enable <= 0;
                    relu_enable <= 0;
                    pool_enable <= 0;
                    done <= 0;
                    weight_counter <= 0;
                end
                LOAD_WEIGHTS: begin
                    weight_write_enable <= 1;
                    weight_write_addr <= weight_counter;
                    weight_counter <= weight_counter + 1;
                end
                LOAD_INPUTS: begin
                    weight_write_enable <= 0;
                    line_write_enable <= 1;
                end
                COMPUTE: begin
                    line_write_enable <= 0;
                    sum_mode <= 0; // or 1 based on requirement
                end
                NORMALIZE: begin
                    norm_enable <= 1;
                end
                ACTIVATE: begin
                    norm_enable <= 0;
                    relu_enable <= 1;
                end
                POOL: begin
                    relu_enable <= 0;
                    pool_enable <= 1;
                end
                DONE: begin
                    pool_enable <= 0;
                    done <= 1;
                end
            endcase
        end
    end
endmodule

module CNN_Accelerator(
    input clk,
    input reset,
    input [31:0] weight_data [0:31],
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
    output [31:0] result [0:31]
);
    wire [31:0] weight_out [0:31];
    wire [31:0] line_out [0:31];
    wire [31:0] products [0:31];
    wire [31:0] add_tree_out [0:31];
    wire [31:0] norm_out [0:31];
    wire [31:0] relu_out [0:31];
    wire [31:0] pooling_out [0:31];

    LineBuffer lb0 (.clk(clk), .reset(reset), .data_in(line_data), .write_enable(line_write_enable), .write_addr(line_write_addr), .read_addr(line_write_addr), .data_out(line_out[0]));
    LineBuffer lb1 (.clk(clk), .reset(reset), .data_in(line_data), .write_enable(line_write_enable), .write_addr(line_write_addr), .read_addr(line_write_addr), .data_out(line_out[1]));
    LineBuffer lb2 (.clk(clk), .reset(reset), .data_in(line_data), .write_enable(line_write_enable), .write_addr(line_write_addr), .read_addr(line_write_addr), .data_out(line_out[2]));
    LineBuffer lb3 (.clk(clk), .reset(reset), .data_in(line_data), .write_enable(line_write_enable), .write_addr(line_write_addr), .read_addr(line_write_addr), .data_out(line_out[3]));
    LineBuffer lb4 (.clk(clk), .reset(reset), .data_in(line_data), .write_enable(line_write_enable), .write_addr(line_write_addr), .read_addr(line_write_addr), .data_out(line_out[4]));
    LineBuffer lb5 (.clk(clk), .reset(reset), .data_in(line_data), .write_enable(line_write_enable), .write_addr(line_write_addr), .read_addr(line_write_addr), .data_out(line_out[5]));
    LineBuffer lb6 (.clk(clk), .reset(reset), .data_in(line_data), .write_enable(line_write_enable), .write_addr(line_write_addr), .read_addr(line_write_addr), .data_out(line_out[6]));
    LineBuffer lb7 (.clk(clk), .reset(reset), .data_in(line_data), .write_enable(line_write_enable), .write_addr(line_write_addr), .read_addr(line_write_addr), .data_out(line_out[7]));
    LineBuffer lb8 (.clk(clk), .reset(reset), .data_in(line_data), .write_enable(line_write_enable), .write_addr(line_write_addr), .read_addr(line_write_addr), .data_out(line_out[8]));

    MACArray ma_array (
        .a({line_out[0], line_out[1], line_out[2], line_out[3], line_out[4], line_out[5], line_out[6], line_out[7], line_out[8], line_out[8], line_out[8], line_out[8], line_out[8], line_out[8], line_out[8], line_out[8], line_out[8], line_out[8], line_out[8], line_out[8], line_out[8], line_out[8], line_out[8], line_out[8], line_out[8], line_out[8], line_out[8], line_out[8], line_out[8], line_out[8], line_out[8], line_out[8]}),
        .b(weight_out),
        .product(products)
    );

    ConfigurableAdderTree cat (
        .products(products),
        .sum_mode(sum_mode),
        .final_sum(add_tree_out)
    );

    Norm norm0 (.data_in(add_tree_out[0]), .norm_param(norm_param), .data_out(norm_out[0]));
    Norm norm1 (.data_in(add_tree_out[1]), .norm_param(norm_param), .data_out(norm_out[1]));
    // Repeat for all 32 Norm instances

    ReLU relu0 (.data_in(norm_enable ? norm_out[0] : add_tree_out[0]), .data_out(relu_out[0]));
    ReLU relu1 (.data_in(norm_enable ? norm_out[1] : add_tree_out[1]), .data_out(relu_out[1]));
    // Repeat for all 32 ReLU instances

    Pooling pool0 (.data_in0(relu_enable ? relu_out[0] : (norm_enable ? norm_out[0] : add_tree_out[0])), .data_in1(relu_enable ? relu_out[1] : (norm_enable ? norm_out[1] : add_tree_out[1])), .data_in2(relu_enable ? relu_out[2] : (norm_enable ? norm_out[2] : add_tree_out[2])), .data_in3(relu_enable ? relu_out[3] : (norm_enable ? norm_out[3] : add_tree_out[3])), .data_out(pooling_out[0]));
    Pooling pool1 (.data_in0(relu_enable ? relu_out[4] : (norm_enable ? norm_out[4] : add_tree_out[4])), .data_in1(relu_enable ? relu_out[5] : (norm_enable ? norm_out[5] : add_tree_out[5])), .data_in2(relu_enable ? relu_out[6] : (norm_enable ? norm_out[6] : add_tree_out[6])), .data_in3(relu_enable ? relu_out[7] : (norm_enable ? norm_out[7] : add_tree_out[7])), .data_out(pooling_out[1]));
    // Repeat for all 32 Pooling instances

    assign result[0] = pool_enable ? pooling_out[0] : (relu_enable ? relu_out[0] : (norm_enable ? norm_out[0] : add_tree_out[0]));
    assign result[1] = pool_enable ? pooling_out[1] : (relu_enable ? relu_out[1] : (norm_enable ? norm_out[1] : add_tree_out[1]));
    // Repeat for all 32 result assignments
endmodule

module CNN_Accelerator_Top(
    input clk,
    input reset,
    input start,
    input [31:0] norm_param,
    output [31:0] result [0:31],
    output done
);
    wire weight_write_enable;
    wire line_write_enable;
    wire sum_mode;
    wire [31:0] weight_data;
    wire [4:0] weight_write_addr;
    wire [31:0] line_data;
    wire [4:0] line_write_addr;
    wire norm_enable;
    wire relu_enable;
    wire pool_enable;

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
        .done(done)
    );

    wire [31:0] weight_out [0:31];

    WeightBuffer weight_buffer (
        .clk(clk),
        .reset(reset),
        .weight_data(weight_data),
        .write_enable(weight_write_enable),
        .write_addr(weight_write_addr),
        .weight_out(weight_out)
    );

    CNN_Accelerator cnn_accel (
        .clk(clk),
        .reset(reset),
        .weight_data(weight_out),
        .weight_write_enable(weight_write_enable),
        .weight_write_addr(weight_write_addr),
        .line_data(line_data),
        .line_write_enable(line_write_enable),
        .line_write_addr(line_write_addr),
        .sum_mode(sum_mode),
        .norm_param(norm_param),
        .norm_enable(norm_enable),
        .relu_enable(relu_enable),
        .pool_enable(pool_enable),
        .result(result)
    );
endmodule



`default_nettype wire
