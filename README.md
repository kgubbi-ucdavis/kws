# Caravel User Project - Keyword Spotting Accelerator



# CNN Accelerator Project

## Overview

This project implements a Convolutional Neural Network (CNN) accelerator on an FPGA. The design includes modules for managing weights and inputs, performing multiply-accumulate (MAC) operations, summing results with an adder tree, normalizing data, applying ReLU activation, and performing pooling operations. The project is integrated into a top-level wrapper (`user_project_wrapper`) that interfaces with external pins and a Wishbone bus.

## Project Structure

### Modules

1. **WeightBuffer**
    - **Description**: Stores weights for the CNN operations.
    - **Ports**:
        - `clk` (input): Clock signal.
        - `reset` (input): Reset signal.
        - `weight_data` (input): Weight data to be written.
        - `write_enable` (input): Write enable signal.
        - `write_addr` (input): Address to write weight data.
        - `weight_out` (output): Array of stored weights.

2. **LineBuffer**
    - **Description**: Stores input data for the CNN operations.
    - **Ports**:
        - `clk` (input): Clock signal.
        - `reset` (input): Reset signal.
        - `data_in` (input): Input data to be stored.
        - `write_enable` (input): Write enable signal.
        - `write_addr` (input): Address to write input data.
        - `read_addr` (input): Address to read input data.
        - `data_out` (output): Output data.

3. **MAC**
    - **Description**: Performs multiplication of two input values.
    - **Ports**:
        - `a` (input): First operand.
        - `b` (input): Second operand.
        - `y` (output): Product of operands.

4. **MACArray**
    - **Description**: Consists of an array of MAC units.
    - **Ports**:
        - `a` (input): Array of first operands.
        - `b` (input): Array of second operands.
        - `product` (output): Array of products.

5. **AdderTree**
    - **Description**: Sums multiple inputs to produce a single output.
    - **Ports**:
        - `in0` to `in7` (input): Input values.
        - `sum` (output): Sum of inputs.

6. **ConfigurableAdderTree**
    - **Description**: Sums products based on a mode (depthwise or pointwise).
    - **Ports**:
        - `products` (input): Array of product values.
        - `sum_mode` (input): Mode select (0 for pointwise, 1 for depthwise).
        - `final_sum` (output): Array of final sums.

7. **Norm**
    - **Description**: Performs normalization on the input data.
    - **Ports**:
        - `data_in` (input): Input data.
        - `norm_param` (input): Normalization parameter.
        - `data_out` (output): Normalized data.

8. **ReLU**
    - **Description**: Applies the ReLU activation function.
    - **Ports**:
        - `data_in` (input): Input data.
        - `data_out` (output): Activated data.

9. **Pooling**
    - **Description**: Performs average pooling on the input data.
    - **Ports**:
        - `data_in0` to `data_in3` (input): Input data values.
        - `data_out` (output): Pooled data.

10. **ControlFSM**
    - **Description**: Controls the state transitions and operations of the CNN accelerator.
    - **Ports**:
        - `clk` (input): Clock signal.
        - `reset` (input): Reset signal.
        - `start` (input): Start signal.
        - `norm_param` (input): Normalization parameter.
        - `weight_write_enable`, `line_write_enable`, `sum_mode`, `weight_data`, `weight_write_addr`, `line_data`, `line_write_addr`, `norm_enable`, `relu_enable`, `pool_enable` (output): Control signals.
        - `done` (output): Done signal.

11. **CNN_Accelerator**
    - **Description**: Performs the core CNN operations, integrating all the sub-modules.
    - **Ports**:
        - `clk`, `reset`, `weight_data`, `weight_write_enable`, `weight_write_addr`, `line_data`, `line_write_enable`, `line_write_addr`, `sum_mode`, `norm_param`, `norm_enable`, `relu_enable`, `pool_enable` (input): Control signals.
        - `result` (output): Result of the CNN operations.

12. **CNN_Accelerator_Top**
    - **Description**: Top-level module integrating the `ControlFSM`, `WeightBuffer`, and `CNN_Accelerator` modules.
    - **Ports**:
        - `clk`, `reset`, `start`, `norm_param` (input): Control signals.
        - `result` (output): Result of the CNN operations.
        - `done` (output): Done signal.

13. **user_project_wrapper**
    - **Description**: Integrates the `CNN_Accelerator_Top` into a wrapper that interfaces with external pins and a Wishbone bus.
    - **Ports**:
        - Power, Wishbone bus, Logic Analyzer, IO, Analog, and Clock signals.

### File Structure  
  
├── src    
│ ├── WeightBuffer.v  
│ ├── LineBuffer.v  
│ ├── MAC.v  
│ ├── MACArray.v  
│ ├── AdderTree.v  
│ ├── ConfigurableAdderTree.v  
│ ├── Norm.v  
│ ├── ReLU.v  
│ ├── Pooling.v  
│ ├── ControlFSM.v  
│ ├── CNN_Accelerator.v  
│ ├── CNN_Accelerator_Top.v  
│ └── user_project_wrapper.v  
├── tb  
│ └── tb_CNN_Accelerator.sv  
├── README.md  


This README file provides an overview of the project, descriptions of each module, the file structure, and a sample testbench for verification. You can paste this directly into your GitHub repository's README.md file.


### Use of Generative AI (ChatGPT-4) in the CNN Accelerator Project

Generative AI, specifically ChatGPT-4, played a pivotal role in the design, verification, development, and debugging of the CNN Accelerator Project. Leveraging the capabilities of ChatGPT-4 provided several advantages, including accelerating the development process, ensuring accuracy, and enhancing the overall efficiency of the project.

#### How Generative AI Was Used

1. **Design and Development**
    - **Module Creation**: ChatGPT-4 assisted in creating Verilog modules for different components of the CNN accelerator, such as weight buffers, line buffers, MAC units, and more.
    - **Architecture Design**: The AI provided insights and suggestions for optimizing the hardware architecture, ensuring efficient data flow and processing.
    - **Code Generation**: Generated Verilog code snippets for various modules, reducing the manual coding effort and minimizing errors.

2. **Verification**
    - **Testbench Generation**: ChatGPT-4 helped in generating testbenches for verifying the functionality of individual modules and the entire system.
    - **Simulation Guidance**: Provided guidance on running simulations and interpreting results, ensuring that the design met the expected performance and functionality.

3. **Debugging and Fixing Code**
    - **Error Diagnosis**: Assisted in identifying and diagnosing syntax and logical errors in the Verilog code.
    - **Code Correction**: Suggested fixes and improvements to resolve errors and enhance code robustness.

#### Steps to Use Generative AI Effectively

To maximize the benefits of Generative AI in hardware design and development, consider the following steps:

1. **Clear and Specific Prompts**
    - Provide detailed and specific prompts when asking for code generation or debugging help. Clear instructions help the AI understand the requirements better and deliver more accurate results.

2. **Iterative Development**
    - Use an iterative approach to development. Start with simple designs and gradually add complexity, using Generative AI to verify each step and suggest improvements.

3. **Code Review and Validation**
    - Always review and validate the code generated by AI. While AI can significantly speed up development, human oversight ensures that the design meets all requirements and standards.

4. **Combining AI with Traditional Methods**
    - Use Generative AI alongside traditional design and verification methods. AI can handle repetitive and straightforward tasks, freeing up human developers to focus on complex and critical aspects of the project.

5. **Continuous Learning and Adaptation**
    - Stay updated with the latest advancements in Generative AI and adapt your workflows to incorporate new features and capabilities. This ensures that you are leveraging the full potential of AI in your projects.

#### Example Workflow

1. **Initial Design**: Describe the required module and functionality to the AI and request a basic Verilog implementation.
2. **Verification Setup**: Ask the AI to generate a testbench for the module and guide you through running simulations.
3. **Iterative Refinement**: Based on simulation results, request modifications and enhancements to the design from the AI.
4. **Debugging**: When errors are encountered, provide the error messages to the AI and request assistance in diagnosing and fixing the issues.
5. **Final Integration**: Once all modules are verified, ask the AI to help integrate them into the top-level design and generate the final project wrapper.

By following these steps, you can effectively harness the power of Generative AI to streamline the design, verification, development, and debugging processes in hardware projects like the CNN Accelerator.
