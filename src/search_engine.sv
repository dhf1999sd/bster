// copyright damien pretet 2021
// distributed under the mit license
// https://opensource.org/licenses/mit-license.php

`timescale 1 ns / 1 ps `default_nettype none

`include "bster_h.sv"

// Engine managing the user request to operate over the tree. Rely on
// memory driver to access the AXI4 RAM and tree space manager to get and free
// address

module search_engine #(
    // Define the token width
    parameter TOKEN_WIDTH = 8,
    // Define the payload width
    parameter PAYLOAD_WIDTH = 32,
    // Width of data bus in bits
    parameter RAM_DATA_WIDTH = 32,
    // Width of address bus in bits
    parameter RAM_ADDR_WIDTH = 16,
    // Width of wstrb (width of data bus in words)
    parameter RAM_STRB_WIDTH = (RAM_DATA_WIDTH / 8),
    // Width of ID signal
    parameter RAM_ID_WIDTH = 8 
  ) (
    input  wire                      aclk,
    input  wire                      aresetn,
    output wire [    `FSM_WIDTH-1:0] fsm_state,
    input  wire                      tree_ready,  //树中有节点，树就准备好了
    input  wire                      engine_ready, //所有状态机为IDLE时，接受新的命令
    // Command interface
    input  wire                      req_valid,   //状态机接受新的请求有效
    output wire                      req_ready,   //状态机在空闲位置，可以接受新的请求
    input  wire [               7:0] req_cmd,    //请求的命令
    input  wire [   TOKEN_WIDTH-1:0] req_token,  //请求的flow ID，似乎没有发挥作用
    // Completion interface
    output wire                      cpl_valid, //查找完成，并输出
    input  wire                      cpl_ready, //状态机接受新的请求有效
    output reg  [ PAYLOAD_WIDTH-1:0] cpl_data,  //输出的数据/结果
    output reg                       cpl_status, //找到之后，置为0，在此之前，置为1
    // serves a search for other engines
    input  wire                      search_valid,  //
    output wire                      search_ready,  //与请求准备一样
    input  wire [   TOKEN_WIDTH-1:0] search_token,  //查找的flow ID
    output reg  [RAM_ADDR_WIDTH-1:0] search_cpl_addr,//查找到的节点所在地址
    output wire                      search_cpl_valid,//查找完成
    output wire                      search_cpl_status,//找到之后，置为0，在此之前，置为1
    // Memory driver
    output wire                      mem_valid,
    input  wire                      mem_ready,
    output wire                      mem_rd,    //外部存储的，读取的标志位
    output wire                      mem_wr,   //写入的标志位，没用到
    output wire [RAM_ADDR_WIDTH-1:0] mem_addr,
    output wire [RAM_DATA_WIDTH-1:0] mem_wr_data,//需要写入的数据，没用到
    input  wire                      mem_rd_valid,
    output wire                      mem_rd_ready,
    input  wire [RAM_DATA_WIDTH-1:0] mem_rd_data
  );

  // TODO: Get it from CSR or tree space manager
  localparam [RAM_ADDR_WIDTH-1:0] ROOT_ADDR = {RAM_ADDR_WIDTH{1'b0}};//根节点地址全为0

  // Central controller of the engine
  engine_states                      fsm;
  // Store the previous state as stack for branching in a processor
  // to remember last operations. Usefull to avoid numerous "empty"
  // states to handle the FSM transitions and next operations
  engine_states                      fsm_stack;

  logic         [   TOKEN_WIDTH-1:0] token_store;

  logic         [RAM_ADDR_WIDTH-1:0] next_addr;

  logic         [RAM_ADDR_WIDTH-1:0] addr;
  logic         [RAM_DATA_WIDTH-1:0] rddata;

  logic         [RAM_ADDR_WIDTH-1:0] cpl;

  logic                              status;
  logic                              internal_search;

  logic         [ PAYLOAD_WIDTH-1:0] rdnode_payload;
  logic                              rdnode_has_right_child;
  logic                              rdnode_has_left_child;
  logic         [RAM_ADDR_WIDTH-1:0] rdnode_right_child_addr;
  logic         [RAM_ADDR_WIDTH-1:0] rdnode_left_child_addr;
  logic         [RAM_ADDR_WIDTH-1:0] rdnode_parent_addr;
  logic         [   TOKEN_WIDTH-1:0] rdnode_token;
  logic         [             8-1:0] rdnode_info;
  // -------------------------------------------------------------------------
  // AXI4-stream interface issuing the commands and returning the completion
  // -------------------------------------------------------------------------

  // Accept a new command only if IDLE and out of reset
  assign req_ready = (fsm == IDLE && aresetn == 1'b1) ? 1'b1 : 1'b0;
  assign search_ready = req_ready;

  // Store commands' parameter and available address when activated
  always @(posedge aclk or negedge aresetn)
  begin
    if (aresetn == 1'b0)
    begin
      token_store <= {TOKEN_WIDTH{1'b0}};
    end
    else
    begin
      if (req_valid && req_ready)
      begin
        token_store <= req_token;
      end
      else if (search_valid && search_ready)
      begin
        token_store <= search_token;  //需要比较的东西，一直存着
      end
    end
  end

  assign cpl_valid = (fsm == REQ_COMPLETION && ~internal_search);
  assign search_cpl_valid = (fsm == REQ_COMPLETION && internal_search);

  assign cpl_status = status;
  assign search_cpl_status = status;

  // Inform the parent about its state for switching correctly interfaces
  // to memory and tree space manager
  assign fsm_state = fsm;

  // -------------------------------------------------------------------------
  // Data path to memory driver
  // -------------------------------------------------------------------------

  assign mem_valid = (fsm == RD_RAM);
  assign mem_wr = 1'b0;
  assign mem_rd = (fsm == RD_RAM);  //状态机在读取状态RD_RAM时，mem_rd为1
  assign mem_addr = addr;         //    状态机在读取状态RD_RAM时，mem_addr为addr
  assign mem_wr_data = {RAM_DATA_WIDTH{1'b0}};
  assign mem_rd_ready = (fsm == WAIT_RAM_CPL);

  // In charge of data storage coming from the RAM  将从RAM中读取的数据存储到rddata中
  always @(posedge aclk or negedge aresetn)
  begin
    if (~aresetn)
    begin
      rddata <= {RAM_DATA_WIDTH{1'b0}};
    end
    else
    begin
      if (mem_rd_valid && mem_rd_ready)
        rddata <= mem_rd_data;
    end
  end

  // Local split of the different node fields to read
  // easier the code and debug the waveform
  assign {rdnode_payload,
          rdnode_left_child_addr,
          rdnode_right_child_addr,
          rdnode_parent_addr,
          rdnode_token,
          rdnode_info
         } = rddata;

  assign rdnode_has_left_child = rdnode_info[1];//判断有没有左子节点
  assign rdnode_has_right_child = rdnode_info[0];//判断有没有右子节点

  // -------------------------------------------------------------------------
  // Main FSM managing the user requests
  // -------------------------------------------------------------------------

  always @(posedge aclk or negedge aresetn)
  begin

    if (aresetn == 1'b0)
    begin
      addr <= {RAM_ADDR_WIDTH{1'b0}};
      fsm <= IDLE;
      fsm_stack <= IDLE;
      cpl_data <= {PAYLOAD_WIDTH{1'b0}};
      status <= 1'b0;
      internal_search <= 1'b0;
    end

    else
    begin
      case (fsm)
        // IDLE state, waiting for user requests
        default:
        begin
          fsm_stack <= IDLE;
          status <= 1'b0;

          // Serves as search engine for insert or delete
          // engines.
          if (search_valid)
          begin
            addr <= ROOT_ADDR;
            fsm <= RD_RAM;
            fsm_stack <= SEARCH_TOKEN;
            internal_search <= 1'b1; //开始查找标志位
          end  // SEARCH_TOKEN instruction
          else if (req_valid && req_cmd == `SEARCH_TOKEN && engine_ready)
          begin
            // If root node is NULL, return an error
            if (~tree_ready)
            begin
              status <= 1'b1;
              cpl <= {PAYLOAD_WIDTH{1'b0}};
              fsm <= REQ_COMPLETION;
            end
            else
            begin
              addr <= ROOT_ADDR;
              fsm <= RD_RAM;
              fsm_stack <= SEARCH_TOKEN;
            end
          end

        end


        // Search engine for user to get a token information.
        // IDLE state starts to search from root, we reach this state
        // after the first read.
        SEARCH_TOKEN:
        begin
          // Here we found the token, then we return the payload
          if (token_store == rdnode_token)
          begin
            cpl_data <= rdnode_payload;
            search_cpl_addr <= addr;
            status <= 1'b0;
            fsm <= REQ_COMPLETION;
          end  // If not found, dive into the left branch stored
          // into the left child if value is smaller than node
          else if (token_store < rdnode_token)
          begin
            // If no left child exists, return an error
            if (~rdnode_has_left_child)
            begin
              status <= 1'b1;
              cpl <= {PAYLOAD_WIDTH{1'b0}};
              fsm <= REQ_COMPLETION;
            end  // Else read left child
            else
            begin
              addr <= rdnode_left_child_addr;
              fsm <= RD_RAM;
              fsm_stack <= SEARCH_TOKEN;
            end
          end  // If not found, dive into the right branch stored
          // into the right child if value is smaller than node
          else
          begin
            // If no right child exists, return an error
            if (~rdnode_has_right_child)
            begin
              status <= 1'b1;
              cpl <= {PAYLOAD_WIDTH{1'b0}};
              fsm <= REQ_COMPLETION;
            end  // Else read right child
            else
            begin
              addr <= rdnode_right_child_addr;
              fsm <= RD_RAM;
              fsm_stack <= SEARCH_TOKEN;
            end
          end
        end

        // Deliver completion of a search or delete request
        REQ_COMPLETION:
        begin

          if (internal_search)
          begin
            internal_search <= 1'b0;  //结束查找标志位
            fsm <= IDLE;
          end
          else if (cpl_ready)
            fsm <= IDLE;
        end

        // Read stage handling node read
        RD_RAM:
        begin
          if (mem_ready)
            fsm <= WAIT_RAM_CPL;
        end

        // Once read, move to the state defined in the stack
        // by the operation which specified it
        WAIT_RAM_CPL:
        begin
          if (mem_rd_valid)
            fsm <= fsm_stack;
        end

      endcase
    end
  end

endmodule

`resetall
