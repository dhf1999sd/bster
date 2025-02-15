// copyright damien pretet 2021
// distributed under the mit license
// https://opensource.org/licenses/mit-license.php

`timescale 1 ns / 1 ps
`default_nettype none

`include "bster_h.sv"

// Engine handling the user request to operate over the tree. Rely on
// memory driver to access the AXI4 RAM and tree space manager to get and free
// address and the different engines managing the request types

module bst_engine

    #(
        // Define the token width
        parameter TOKEN_WIDTH = 8,
        // Define the payload width
        parameter PAYLOAD_WIDTH = 32,
        // Width of data bus in bits
        parameter RAM_DATA_WIDTH = 32,
        // Width of address bus in bits
        parameter RAM_ADDR_WIDTH = 16,
        // Width of wstrb (width of data bus in words)
        parameter RAM_STRB_WIDTH = (RAM_DATA_WIDTH/8),
        // Width of ID signal
        parameter RAM_ID_WIDTH = 8 
    )(
        input  wire                        aclk,
        input  wire                        aresetn,
        output wire                        tree_ready,
        // Command interface
        input  wire                        req_valid,
        output wire                        req_ready,
        input  wire [                 7:0] req_cmd,
        input  wire [     TOKEN_WIDTH-1:0] req_token,
        input  wire [   PAYLOAD_WIDTH-1:0] req_data,
        // Completion interface
        output wire                        cpl_valid,
        input  wire                        cpl_ready,
        output reg  [   PAYLOAD_WIDTH-1:0] cpl_data,
        output reg                         cpl_status,
        // Tree manager access
        output wire                        tree_mgt_req_valid,
        input  wire                        tree_mgt_req_ready,
        input  wire [  RAM_ADDR_WIDTH-1:0] tree_mgt_req_addr,
        output wire                        tree_mgt_free_valid,
        output wire                        tree_mgt_free_is_root,
        input  wire                        tree_mgt_free_ready,
        output wire [  RAM_ADDR_WIDTH-1:0] tree_mgt_free_addr,
        // Memory driver
        output wire                        mem_valid,
        input  wire                        mem_ready,
        output wire                        mem_rd,
        output wire                        mem_wr,
        output wire [  RAM_ADDR_WIDTH-1:0] mem_addr,
        output wire [  RAM_DATA_WIDTH-1:0] mem_wr_data,
        input  wire                        mem_rd_valid,
        output wire                        mem_rd_ready,
        input  wire [  RAM_DATA_WIDTH-1:0] mem_rd_data,
        output wire [           `BE_W-1:0] csr_mst
    );


    engine_states fsm_insert;
    engine_states fsm_search;
    engine_states fsm_delete;


    logic                      engine_ready;

    logic                      req_ready_insert;
    logic                      req_ready_search;
    logic                      req_ready_delete;

    logic                      cpl_valid_insert;
    logic [ PAYLOAD_WIDTH-1:0] cpl_data_insert;
    logic                      cpl_status_insert;
    logic                      cpl_valid_search;
    logic [ PAYLOAD_WIDTH-1:0] cpl_data_search;
    logic                      cpl_status_search;
    logic                      cpl_valid_delete;
    logic [ PAYLOAD_WIDTH-1:0] cpl_data_delete;
    logic                      cpl_status_delete;

    logic                      mem_valid_insert;
    logic                      mem_rd_insert;
    logic                      mem_wr_insert;
    logic [RAM_ADDR_WIDTH-1:0] mem_addr_insert;
    logic [RAM_DATA_WIDTH-1:0] mem_wr_data_insert;
    logic                      mem_rd_ready_insert;

    logic                      mem_valid_search;
    logic                      mem_rd_search;
    logic                      mem_wr_search;
    logic [RAM_ADDR_WIDTH-1:0] mem_addr_search;
    logic [RAM_DATA_WIDTH-1:0] mem_wr_data_search;
    logic                      mem_rd_ready_search;

    logic                      mem_valid_delete;
    logic                      mem_rd_delete;
    logic                      mem_wr_delete;
    logic [RAM_ADDR_WIDTH-1:0] mem_addr_delete;
    logic [RAM_DATA_WIDTH-1:0] mem_wr_data_delete;
    logic                      mem_rd_ready_delete;

    logic                      search_valid;
    logic                      search_ready;
    logic [             8-1:0] search_cmd;
    logic [   TOKEN_WIDTH-1:0] search_token;
    logic [RAM_ADDR_WIDTH-1:0] search_cpl_addr;
    logic                      search_cpl_valid;
    logic                      search_cpl_status;

    logic                      insert_valid;
    logic                      insert_ready;
    logic [               7:0] insert_cmd;
    logic [RAM_DATA_WIDTH-1:0] insert_node;
    logic [RAM_ADDR_WIDTH-1:0] insert_addr;
    logic                      insert_cpl_valid;
    logic                      insert_cpl_status;
    logic                      tree_emptied;

    assign tree_emptied = tree_mgt_free_valid && tree_mgt_free_ready &&
                          tree_mgt_free_is_root;

    // -------------------------------------------------------------------------
    // AXI4-stream interface issuing the commands and returning the completion
    // -------------------------------------------------------------------------

    // Accept a new command only if all FSM are IDLE
    assign req_ready = (fsm_insert == IDLE && fsm_search == IDLE && fsm_delete == IDLE);//�?有状态机为IDLE时，接受新的命令
    assign engine_ready = req_ready;

    assign cpl_valid = (fsm_insert == REQ_COMPLETION) ? cpl_valid_insert :
                       (fsm_search == REQ_COMPLETION) ? cpl_valid_search :
                       (fsm_delete == REQ_COMPLETION) ? cpl_valid_delete : 1'b0;

    assign cpl_data = (fsm_insert == REQ_COMPLETION) ? cpl_data_insert :
                      (fsm_search == REQ_COMPLETION) ? cpl_data_search :
                      (fsm_delete == REQ_COMPLETION) ? cpl_data_delete : 'b0;

    assign cpl_status = (fsm_insert == REQ_COMPLETION) ? cpl_status_insert :
                        (fsm_search == REQ_COMPLETION) ? cpl_status_search :
                        (fsm_delete == REQ_COMPLETION) ? cpl_status_delete : 1'b0;

    // -------------------------------------------------------------------------
    // Data path to memory driver
    // -------------------------------------------------------------------------

    assign mem_valid = (fsm_insert == WR_RAM ||
                        fsm_insert == RD_RAM ||
                        fsm_search == WR_RAM ||
                        fsm_search == RD_RAM ||
                        fsm_delete == WR_RAM ||
                        fsm_delete == RD_RAM
                       );

    assign mem_wr = (fsm_insert == WR_RAM ||
                     fsm_delete == WR_RAM
                    );

    assign mem_rd = (fsm_insert == RD_RAM ||
                     fsm_search == RD_RAM ||
                     fsm_delete == RD_RAM
                    );

    assign mem_addr = (fsm_insert == WR_RAM || fsm_insert == RD_RAM) ? mem_addr_insert:
                      (fsm_search == WR_RAM || fsm_search == RD_RAM) ? mem_addr_search:
                      (fsm_delete == WR_RAM || fsm_delete == RD_RAM) ? mem_addr_delete:
                                                                       {RAM_ADDR_WIDTH{1'b0}};

    assign mem_wr_data = (fsm_insert == WR_RAM ) ? mem_wr_data_insert:
                         (fsm_search == WR_RAM ) ? mem_wr_data_search:
                         (fsm_delete == WR_RAM ) ? mem_wr_data_delete:
                                                   {RAM_DATA_WIDTH{1'b0}};

    assign mem_rd_ready = (fsm_insert == WAIT_RAM_CPL ||
                           fsm_search == WAIT_RAM_CPL ||
                           fsm_delete == WAIT_RAM_CPL
                          );

    assign csr_mst = {{tree_ready,
                      {{(8-`FSM_WIDTH){1'b0}},fsm_search},
                      {{(8-`FSM_WIDTH){1'b0}},fsm_delete},
                      {{(8-`FSM_WIDTH){1'b0}},fsm_insert}}};

    insert_engine
    #(
    .TOKEN_WIDTH    (TOKEN_WIDTH),
    .PAYLOAD_WIDTH  (PAYLOAD_WIDTH),
    .RAM_DATA_WIDTH (RAM_DATA_WIDTH),
    .RAM_ADDR_WIDTH (RAM_ADDR_WIDTH),
    .RAM_STRB_WIDTH (RAM_STRB_WIDTH),
    .RAM_ID_WIDTH   (RAM_ID_WIDTH)
    )
    insert_engine_inst
    (
    .aclk                (aclk               ),
    .aresetn             (aresetn            ),
    .tree_ready          (tree_ready         ),
    .tree_emptied        (tree_emptied       ),
    .engine_ready        (engine_ready       ),
    .fsm_state           (fsm_insert         ),
    .req_valid           (req_valid          ),
    .req_ready           (req_ready_insert   ),
    .req_cmd             (req_cmd            ),
    .req_token           (req_token          ),
    .req_data            (req_data           ),
    .cpl_valid           (cpl_valid_insert   ),
    .cpl_ready           (cpl_ready          ),
    .cpl_data            (cpl_data_insert    ),
    .cpl_status          (cpl_status_insert  ),
    .insert_valid        (insert_valid       ),
    .insert_ready        (insert_ready       ),
    .insert_node         (insert_node        ),
    .insert_cmd          (insert_cmd         ),
    .insert_addr         (insert_addr        ),
    .insert_cpl_valid    (insert_cpl_valid   ),
    .insert_cpl_status   (insert_cpl_status  ),
    .tree_mgt_req_valid  (tree_mgt_req_valid ),
    .tree_mgt_req_ready  (tree_mgt_req_ready ),
    .tree_mgt_req_addr   (tree_mgt_req_addr  ),
    .mem_valid           (mem_valid_insert   ),
    .mem_ready           (mem_ready          ),
    .mem_rd              (mem_rd_insert      ),
    .mem_wr              (mem_wr_insert      ),
    .mem_addr            (mem_addr_insert    ),
    .mem_wr_data         (mem_wr_data_insert ),
    .mem_rd_valid        (mem_rd_valid       ),
    .mem_rd_ready        (mem_rd_ready_insert),
    .mem_rd_data         (mem_rd_data        )
    );


    search_engine
    #(
    .TOKEN_WIDTH    (TOKEN_WIDTH),
    .PAYLOAD_WIDTH  (PAYLOAD_WIDTH),
    .RAM_DATA_WIDTH (RAM_DATA_WIDTH),
    .RAM_ADDR_WIDTH (RAM_ADDR_WIDTH),
    .RAM_STRB_WIDTH (RAM_STRB_WIDTH),
    .RAM_ID_WIDTH   (RAM_ID_WIDTH)
    )
    search_engine_inst
    (
    .aclk                (aclk               ),
    .aresetn             (aresetn            ),
    .tree_ready          (tree_ready         ),
    .engine_ready        (engine_ready       ),
    .fsm_state           (fsm_search         ),
    .req_valid           (req_valid          ),
    .req_ready           (req_ready_search   ),
    .req_cmd             (req_cmd            ),
    .req_token           (req_token          ),
//    .req_data            (req_data           ),
    .cpl_valid           (cpl_valid_search   ),
    .cpl_ready           (cpl_ready          ),
    .cpl_data            (cpl_data_search    ),
    .cpl_status          (cpl_status_search  ),
    .search_valid        (search_valid       ),
    .search_ready        (search_ready       ),
    .search_token        (search_token       ),
    .search_cpl_addr     (search_cpl_addr    ),
    .search_cpl_valid    (search_cpl_valid   ),
    .search_cpl_status   (search_cpl_status  ),
    .mem_valid           (mem_valid_search   ),
    .mem_ready           (mem_ready          ),
    .mem_rd              (mem_rd_search      ),
    .mem_wr              (mem_wr_search      ),
    .mem_addr            (mem_addr_search    ),
    .mem_wr_data         (mem_wr_data_search ),
    .mem_rd_valid        (mem_rd_valid       ),
    .mem_rd_ready        (mem_rd_ready_search),
    .mem_rd_data         (mem_rd_data        )
    );


    delete_engine
    #(
    .TOKEN_WIDTH    (TOKEN_WIDTH),
    .PAYLOAD_WIDTH  (PAYLOAD_WIDTH),
    .RAM_DATA_WIDTH (RAM_DATA_WIDTH),
    .RAM_ADDR_WIDTH (RAM_ADDR_WIDTH),
    .RAM_STRB_WIDTH (RAM_STRB_WIDTH),
    .RAM_ID_WIDTH   (RAM_ID_WIDTH)
    )
    delete_engine_inst
    (
    .aclk                  (aclk                 ),
    .aresetn               (aresetn              ),
    .tree_ready            (tree_ready           ),
    .engine_ready          (engine_ready         ),
    .fsm_state             (fsm_delete           ),
    .req_valid             (req_valid            ),
    .req_ready             (req_ready_delete     ),
    .req_cmd               (req_cmd              ),
    .req_token             (req_token            ),
    .req_data              (req_data             ),
    .cpl_valid             (cpl_valid_delete     ),
    .cpl_ready             (cpl_ready            ),
    .cpl_data              (cpl_data_delete      ),
    .cpl_status            (cpl_status_delete    ),
    .search_valid          (search_valid         ),
    .search_ready          (search_ready         ),
    .search_token          (search_token         ),
    .search_cmd            (search_cmd           ),
    .search_cpl_addr       (search_cpl_addr      ),
    .search_cpl_valid      (search_cpl_valid     ),
    .search_cpl_status     (search_cpl_status    ),
    .insert_valid          (insert_valid         ),
    .insert_ready          (insert_ready         ),
    .insert_node           (insert_node          ),
    .insert_cmd            (insert_cmd           ),
    .insert_addr           (insert_addr          ),
    .insert_cpl_valid      (insert_cpl_valid     ),
    .insert_cpl_status     (insert_cpl_status    ),
    .tree_mgt_free_valid   (tree_mgt_free_valid  ),
    .tree_mgt_free_ready   (tree_mgt_free_ready  ),
    .tree_mgt_free_is_root (tree_mgt_free_is_root),
    .tree_mgt_free_addr    (tree_mgt_free_addr   ),
    .mem_valid             (mem_valid_delete     ),
    .mem_ready             (mem_ready            ),
    .mem_rd                (mem_rd_delete        ),
    .mem_wr                (mem_wr_delete        ),
    .mem_addr              (mem_addr_delete      ),
    .mem_wr_data           (mem_wr_data_delete   ),
    .mem_rd_valid          (mem_rd_valid         ),
    .mem_rd_ready          (mem_rd_ready_delete  ),
    .mem_rd_data           (mem_rd_data          )
    );

   

endmodule

`resetall
