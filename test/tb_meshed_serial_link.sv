// Author: Pu Deng <piaodeng@stanford.edu>

`include "floo_noc/typedef.svh"

module tb_meshed_serial_link;

    import floo_pkg::*;

    localparam time         Tck = 200ns;
    localparam bit EnDdr = 1'b1;
    localparam int DATA_WIDTH = 256;
    localparam int MEM_SIZE_BYTE                = 1 * 1024; //1KB
    localparam int MEM_SIZE_BIT                 = MEM_SIZE_BYTE * 8;
    localparam int unsigned MEM_NUM_WORD        = MEM_SIZE_BIT / DATA_WIDTH;

    localparam int NumChannels  = serial_link_pkg::NumChannels;
    localparam int NumLanes     = serial_link_pkg::NumLanes;

    `include "axi/assign.svh"
    `include "axi/typedef.svh"

    `include "register_interface/assign.svh"
    `include "register_interface/typedef.svh"

    `include "tcdm_interface/assign.svh"
    `include "tcdm_interface/typedef.svh"

    `include "common_cells/registers.svh"

    import cf_math_pkg::idx_width;

    import meshed_network_ctrl_regs_reg_pkg::*;

    // ==============
    //    Config
    // ==============
    localparam int unsigned NumNodes        = 4;
    localparam int unsigned NumColumns      = 2;
    localparam int unsigned NumRows         = 2;
    localparam int unsigned TestDuration    = 100;
    localparam int unsigned MaxClkDiv       = serial_link_pkg::MaxClkDiv;

    localparam time         TckSys1         = 50ns;
    localparam time         TckReg          = 200ns;
    localparam int unsigned RstClkCyclesSys = 1;

    localparam int unsigned AxiIdWidth      = 5;
    localparam int unsigned AxiAddrWidth    = 32;
    localparam int unsigned AxiDataWidth    = DATA_WIDTH;
    localparam int unsigned AxiStrbWidth    = AxiDataWidth / 8;
    localparam int unsigned AxiUserWidth    = 2;

    localparam int unsigned RegAddrWidth    = 32;
    localparam int unsigned RegDataWidth    = 64;
    localparam int unsigned RegStrbWidth    = RegDataWidth / 8;

    // ==============
    //    DDR Link
    // ==============

    // AXI types for typedefs
    typedef logic [AxiIdWidth-1:0  ]  axi_id_t;
    typedef logic [AxiAddrWidth-1:0]  axi_addr_t;
    typedef logic [AxiDataWidth-1:0]  axi_data_t;
    typedef logic [AxiStrbWidth-1:0]  axi_strb_t;
    typedef logic [AxiUserWidth-1:0]  axi_user_t;

    `AXI_TYPEDEF_ALL(axi, axi_addr_t, axi_id_t, axi_data_t, axi_strb_t, axi_user_t)

    // RegBus types for typedefs
    typedef logic [RegAddrWidth-1:0]  cfg_addr_t;
    typedef logic [RegDataWidth-1:0]  cfg_data_t;
    typedef logic [RegStrbWidth-1:0]  cfg_strb_t;

    `REG_BUS_TYPEDEF_ALL(cfg, cfg_addr_t, cfg_data_t, cfg_strb_t)

    typedef logic [NumLanes*(1+EnDdr)-1:0]  phy_data_t;

    // Model signals
    axi_req_t   axi_req [NumNodes-1:0];
    axi_resp_t  axi_rsp [NumNodes-1:0];
    cfg_req_t   cfg_req [NumNodes-1:0];
    cfg_rsp_t   cfg_rsp [NumNodes-1:0];

    // link
    logic [NumNodes-1:0][3:0][NumChannels-1:0]                ddr_rcv_clk_input;
    logic [NumNodes-1:0][3:0][NumChannels-1:0]                ddr_rcv_clk_output;
    logic [NumNodes-1:0][3:0][NumChannels-1:0][NumLanes-1:0]  ddr_input;
    logic [NumNodes-1:0][3:0][NumChannels-1:0][NumLanes-1:0]  ddr_output;

    // =============
    //    Router
    // =============
    localparam int unsigned NumRoutes = 5;
    localparam int unsigned NumVirtChannels = 6;
    localparam int unsigned IdWidth = $clog2(NumRoutes); //5 ports per router
    localparam int unsigned FlitWidth = AxiDataWidth;
    

    typedef logic [FlitWidth-1:0] payload_t;
    typedef logic [$clog2(NumRows)-1:0] y_t;
    typedef logic [$clog2(NumColumns)-1:0] x_t;
    typedef logic [1:0] port_id_t;
    typedef logic [NumNodes-1:0] mask_t;

    `FLOO_TYPEDEF_XY_NODE_ID_T(id_t, x_t, y_t, port_id_t)
    `FLOO_TYPEDEF_ROM_HDR_T(hdr_t, id_t, id_t, logic, logic, mask_t)
    `FLOO_TYPEDEF_GENERIC_FLIT_T(req, hdr_t, payload_t)

    //                                                  Source Port   Virtual Channel
    floo_req_generic_flit_t  stimuli_queue [NumNodes-1:0][NumRoutes][NumVirtChannels][$];

    //                                                Destination  Virtual Channel  Source Port
    floo_req_generic_flit_t  golden_queue [NumNodes-1:0][NumRoutes][NumVirtChannels][NumRoutes][$];

    // =============
    //    Clock
    // =============

    // clock for each node
    logic clk_reg;
    logic rst_reg_n;
    
    clk_rst_gen #(
      .ClkPeriod    ( TckReg          ),
      .RstClkCycles ( RstClkCyclesSys )
    ) i_clk_rst_gen_reg (
      .clk_o  ( clk_reg   ),
      .rst_no ( rst_reg_n )
    );

    ////////////////////////
    // Generate each node //
    ////////////////////////

    logic [NumNodes-1:0] clk_i;
    logic [NumNodes-1:0] rst_i_n;

    for (genvar node_id = 0; node_id < NumNodes; node_id++) begin : generate_nodes

        clk_rst_gen #(
          .ClkPeriod    ( TckSys1 + node_id / 2 ),
          .RstClkCycles ( RstClkCyclesSys )
        ) i_clk_rst_gen_sys (
          .clk_o  ( clk_i[node_id]   ),
          .rst_no ( rst_i_n[node_id] )
        );

        // NoC/Mem for each node
        // Interface with the memory with the TCDM protocol
        `TCDM_TYPEDEF_ALL(main_mem, logic [AxiAddrWidth-1:0], logic [AxiDataWidth-1:0], logic [AxiDataWidth/8-1:0], logic)
        main_mem_req_t main_mem_req;
        main_mem_rsp_t main_mem_rsp;

        // Mem model
        tc_sram #(
          .DataWidth(AxiDataWidth      ),
          .NumPorts (1                 ),
          .NumWords (MEM_NUM_WORD      )
        ) i_main_memory (
          .clk_i  (clk_i[node_id]                                                                                                            ),
          .rst_ni (rst_i_n[node_id]                                                                                                          ),
          .req_i  (main_mem_req.q_valid                                                                                                      ),
          .addr_i (main_mem_req.q.addr[idx_width(MEM_NUM_WORD)+idx_width(AxiDataWidth)-1:idx_width(AxiDataWidth)]      ),
          .be_i   (main_mem_req.q.strb                                                                                                       ),
          .wdata_i(main_mem_req.q.data                                                                                                       ),
          .we_i   (main_mem_req.q.write                                                                                                      ),
          .rdata_o(main_mem_rsp.p.data                                                                                                       )
        );

        // Always ready
        assign main_mem_rsp.q_ready = 1'b1;
        // One cycle latency
        `FF(main_mem_rsp.p_valid, main_mem_req.q_valid, 1'b0, clk_i[node_id], rst_i_n[node_id])

        // Serial Link
        meshed_serial_link #(
          .NumNodes         ( NumNodes        ),
          .NumRoutes        ( NumRoutes-1     ),
          .NumVirtChannels  ( 1               ),
          .flit_t           ( floo_req_generic_flit_t ),
          .id_t             ( id_t            ),
          .InFifoDepth      ( 2               ),
          .RouteAlgo        ( XYRouting       ),
          .NumChannels      ( NumChannels     ),
          .NumLanes         ( 8               ),
          .EnDdr            ( EnDdr           ),
          .axi_req_t        ( axi_req_t       ),
          .axi_rsp_t        ( axi_resp_t      ),
          .aw_chan_t        ( axi_aw_chan_t   ),
          .w_chan_t         ( axi_w_chan_t    ),
          .b_chan_t         ( axi_b_chan_t    ),
          .ar_chan_t        ( axi_ar_chan_t   ),
          .r_chan_t         ( axi_r_chan_t    ),
          .cfg_req_t        ( cfg_req_t       ),
          .cfg_rsp_t        ( cfg_rsp_t       )
        ) i_meshed_serial_link (
          .clk_i          ( clk_i[node_id]                ),
          .rst_ni         ( rst_i_n[node_id]              ),
          .clk_sl_i       ( clk_i[node_id]                ),
          .rst_sl_ni      ( rst_i_n[node_id]              ),
          .clk_reg_i      ( clk_reg                       ),
          .rst_reg_ni     ( rst_reg_n                     ),
          .testmode_i     ( 1'b0                          ),
          .axi_req_o      ( axi_req[node_id]              ),
          .axi_rsp_i      ( axi_rsp[node_id]              ),
          .cfg_req_i      ( cfg_req[node_id]              ),
          .cfg_rsp_o      ( cfg_rsp[node_id]              ),
          .ddr_rcv_clk_i  ( ddr_rcv_clk_input[node_id]    ),
          .ddr_rcv_clk_o  ( ddr_rcv_clk_output[node_id]   ),
          .ddr_i          ( ddr_input[node_id]            ),
          .ddr_o          ( ddr_output[node_id]           ),
          .clk_ena_o      ( /*unused*/      ),
          .reset_no       ( /*unused*/      )
        );

        axi_to_tcdm #(
          .axi_req_t (axi_req_t       ),
          .axi_rsp_t (axi_resp_t      ),
          .tcdm_req_t(main_mem_req_t  ),
          .tcdm_rsp_t(main_mem_rsp_t  ),
          .AddrWidth (AxiAddrWidth    ),
          .DataWidth (AxiDataWidth    ),
          .IdWidth   (AxiIdWidth      ),
          .BufDepth  (2               )
        ) i_axi_to_main_memory (
          .clk_i     (  clk_i[node_id]    ),
          .rst_ni    (  rst_i_n[node_id]  ),
          .axi_req_i (  axi_req[node_id]  ),
          .axi_rsp_o (  axi_rsp[node_id]  ),
          .tcdm_req_o(  main_mem_req      ),
          .tcdm_rsp_i(  main_mem_rsp      )
        );

        // Init memory
        int mem_row_start_addr  = 0 / AxiStrbWidth;
        initial begin
            for (int mem_row_id = 0; mem_row_id < MEM_NUM_WORD; mem_row_id += 1) begin
                for (int bit_offset = 0; bit_offset < AxiDataWidth; bit_offset += 32) begin
                    i_main_memory.sram[mem_row_id][bit_offset +: 32] = $urandom;
                end
            end
        end
    end

    /////////////////////////
    // Generate Connection //
    /////////////////////////

    //   - 1: upper bits decreasing (South)
    //   - 2: lower bits decreasing (West )
    //   - 3: upper bits increasing (North)
    //   - 4: lower bits increasing (East )

    for (genvar node_id = 0; node_id < NumNodes; node_id++) begin
            
        int row_id = node_id / NumColumns;
        int column_id = node_id % NumColumns;

        int south_node_id = node_id - NumRows;
        int west_node_id = node_id - 1;
        int north_node_id = node_id + NumRows;
        int east_node_id = node_id + 1;

        always_comb begin
            // South connection
            if(row_id != 0) begin
                ddr_rcv_clk_input[node_id][0]  = ddr_rcv_clk_output[south_node_id][2];
                ddr_input[node_id][0]          = ddr_output[south_node_id][2];
            end
          
            // West connection
            if(column_id != 0) begin
                ddr_rcv_clk_input[node_id][1]  = ddr_rcv_clk_output[west_node_id][3];
                ddr_input[node_id][1]          = ddr_output[west_node_id][3];
            end

            // North connection
            if(row_id != NumRows-1) begin
                ddr_rcv_clk_input[node_id][2]  = ddr_rcv_clk_output[north_node_id][0];
                ddr_input[node_id][2]          = ddr_output[north_node_id][0];
            end

            // East Connection
            if(column_id != (NumColumns - 1)) begin
                ddr_rcv_clk_input[node_id][3]  = ddr_rcv_clk_output[east_node_id][1];
                ddr_input[node_id][3]          = ddr_output[east_node_id][1];
            end
        end
    end


    ////////////////
    // Test logic //
    ////////////////

    typedef reg_test::reg_driver #(
      .AW ( RegAddrWidth  ),
      .DW ( RegDataWidth  ),
      .TA ( 100ps         ),
      .TT ( 500ps         )
    ) reg_master_t;

    REG_BUS #(
      .ADDR_WIDTH (RegAddrWidth),
      .DATA_WIDTH (RegDataWidth)
    ) cfg [NumNodes] (clk_reg);

    reg_master_t reg_masters [NumNodes];

    for (genvar test_node_id = 0; test_node_id < NumNodes; test_node_id ++) begin
        `REG_BUS_ASSIGN_TO_REQ(cfg_req[test_node_id], cfg[test_node_id])
        `REG_BUS_ASSIGN_FROM_RSP(cfg[test_node_id], cfg_rsp[test_node_id])

        initial begin
            reg_masters[test_node_id] = new ( cfg[test_node_id]);
        end
    end

    task automatic cfg_write(reg_master_t drv, cfg_addr_t addr, cfg_data_t data, cfg_strb_t strb='1);
      automatic logic resp;
      drv.send_write(addr, data, strb, resp);
      assert (!resp) else $error("Not able to write cfg reg");
    endtask

    task automatic cfg_read(reg_master_t drv, cfg_addr_t addr, output cfg_data_t data);
      automatic logic resp;
      drv.send_read(addr, data, resp);
      assert (!resp) else $error("Not able to write cfg reg");
    endtask

    task automatic configure_network_xy_package (reg_master_t drv, axi_addr_t start_addr, axi_addr_t data_len, logic [3:0] dst_chip_id);
        automatic axi_addr_t meshed_network_ctrl_reg_offset = serial_link_pkg::linkCtrlRegLen * 4;

        cfg_data_t register_val;
        cfg_data_t data;

        // config start addr and len
        register_val = {data_len, start_addr};
        cfg_write(drv, meshed_network_ctrl_reg_offset + MESHED_NETWORK_CTRL_REGS_MESHED_NETWORK_DATA_FETCHER_DATA_OFFSET, register_val);

        // read the original val in the ctrl reg
        cfg_read(drv, meshed_network_ctrl_reg_offset + MESHED_NETWORK_CTRL_REGS_MESHED_NETWORK_CTRL_OFFSET, data);

        // config dst chip id and trigger send, keep the recv setting
        register_val  = {data[63:5], dst_chip_id, 1'b1};
        cfg_write(drv, meshed_network_ctrl_reg_offset + MESHED_NETWORK_CTRL_REGS_MESHED_NETWORK_CTRL_OFFSET, register_val);
    endtask

    task automatic configure_network_ring_package (reg_master_t drv, axi_addr_t start_addr, axi_addr_t data_len, logic [3:0] dst_chip_id, logic [3:0] dst_chip_mask, logic traffic_dir);
        automatic axi_addr_t meshed_network_ctrl_reg_offset = serial_link_pkg::linkCtrlRegLen * 4;

        cfg_data_t register_val;
        cfg_data_t data;

        // config start addr and len
        register_val = {data_len, start_addr};
        cfg_write(drv, meshed_network_ctrl_reg_offset + MESHED_NETWORK_CTRL_REGS_MESHED_NETWORK_DATA_FETCHER_DATA_OFFSET, register_val);

        // enable ROM, set traffic dir and dst mask
        register_val = {57'd0, dst_chip_mask, traffic_dir, 1'b1};
        cfg_write(drv, meshed_network_ctrl_reg_offset + MESHED_NETWORK_CTRL_REGS_MESHED_NETWORK_ROM_CTRL_OFFSET, register_val);

        // read the original val in the ctrl reg
        cfg_read(drv, meshed_network_ctrl_reg_offset + MESHED_NETWORK_CTRL_REGS_MESHED_NETWORK_CTRL_OFFSET, data);

        // trigger send, keep the recv setting
        register_val  = {data[63:5], dst_chip_id, 1'b1};
        cfg_write(drv, meshed_network_ctrl_reg_offset + MESHED_NETWORK_CTRL_REGS_MESHED_NETWORK_CTRL_OFFSET, register_val);
    endtask

    task automatic configure_recv_packege (reg_master_t drv, axi_addr_t start_addr, axi_addr_t max_data_len);
          automatic axi_addr_t meshed_network_ctrl_reg_offset = serial_link_pkg::linkCtrlRegLen * 4;

          cfg_data_t register_val;
          cfg_data_t data;

          // set the recv addr and max len
          register_val = {max_data_len, start_addr};
          cfg_write(drv, meshed_network_ctrl_reg_offset + MESHED_NETWORK_CTRL_REGS_MESHED_NETWORK_DATA_RECV_DATA_OFFSET, register_val);

          // read the original val in the ctrl reg
          cfg_read(drv, meshed_network_ctrl_reg_offset + MESHED_NETWORK_CTRL_REGS_MESHED_NETWORK_CTRL_OFFSET, data);

          // trigger recv, keep the send setting
          register_val  = {57'd0, 1'b0, 1'b1, data[4:0]};
          cfg_write(drv, meshed_network_ctrl_reg_offset + MESHED_NETWORK_CTRL_REGS_MESHED_NETWORK_CTRL_OFFSET, register_val);
    endtask

    task automatic wait_recv_package (reg_master_t drv);
          automatic axi_addr_t meshed_network_ctrl_reg_offset = serial_link_pkg::linkCtrlRegLen * 4;
          cfg_data_t data;

          // read the original val in the ctrl reg
          do begin
              cfg_read(drv, meshed_network_ctrl_reg_offset + MESHED_NETWORK_CTRL_REGS_MESHED_NETWORK_STATUS_OFFSET, data);
          end while(data[6] != 1'b1);
    endtask

    task automatic configure_router_id (reg_master_t drv, logic [3:0] xy_id, logic [3:0] ring_id, logic [2:0] ring_up_port, logic [2:0] ring_down_port);
        automatic axi_addr_t meshed_network_ctrl_reg_offset = serial_link_pkg::linkCtrlRegLen * 4;

        cfg_data_t register_val;

        register_val = {52'd0, ring_down_port, ring_up_port, ring_id, xy_id};
        cfg_write(drv, meshed_network_ctrl_reg_offset + MESHED_NETWORK_CTRL_REGS_MESHED_NETWORK_ID_OFFSET, register_val);
    endtask

    task automatic initialize_serial_link (reg_master_t drv, int serial_link_dir);
        automatic axi_addr_t meshed_network_ctrl_reg_offset = serial_link_pkg::linkCtrlRegLen * serial_link_dir;

        // De-assert reset
        cfg_write(drv, meshed_network_ctrl_reg_offset + serial_link_reg_pkg::SERIAL_LINK_CTRL_OFFSET, 64'h300);
        // Assert res64
        cfg_write(drv, meshed_network_ctrl_reg_offset + serial_link_reg_pkg::SERIAL_LINK_CTRL_OFFSET, 64'h302);
        // Enable clo64
        cfg_write(drv, meshed_network_ctrl_reg_offset + serial_link_reg_pkg::SERIAL_LINK_CTRL_OFFSET, 64'h303);
        // Enable channel allocator bypass mode and
        // auto flush feature but disable sync for RX side
        if (NumChannels > 1) begin
          cfg_write(drv, serial_link_reg_pkg::SERIAL_LINK_CHANNEL_ALLOC_TX_CFG_OFFSET, 64'h3);
          cfg_write(drv, serial_link_reg_pkg::SERIAL_LINK_CHANNEL_ALLOC_RX_CFG_OFFSET, 64'h3);
        end
        // Wait for some clock cycles
        repeat(50) drv.cycle_end();
    endtask

    task automatic wait_for_reset(int node_id);
      @(posedge rst_i_n[node_id]);
    endtask

    initial begin
        void'($urandom(1234)); // set seed

        reg_masters[0].reset_master();
        reg_masters[1].reset_master();
        reg_masters[2].reset_master();
        reg_masters[3].reset_master();

        fork
          wait_for_reset(0);
          wait_for_reset(1);
          wait_for_reset(2);
          wait_for_reset(3);
        join

        configure_router_id(reg_masters[0], 0, 0, 4, 3);
        configure_router_id(reg_masters[1], 1, 1, 3, 2);
        configure_router_id(reg_masters[2], 2, 3, 1, 4);
        configure_router_id(reg_masters[3], 3, 2, 2, 1);
        
        // initialize links
        for (int node_id = 0; node_id < NumNodes; node_id++) begin
            for (int link_dir = 0; link_dir < 4; link_dir++) begin
                initialize_serial_link(reg_masters[node_id], link_dir);
                $info("[Node%0d] Link %0d is ready", node_id, link_dir);
            end
        end
        
        // configure recv at chip 1, 2, 3
        configure_recv_packege(reg_masters[0], 0, 32);
        configure_recv_packege(reg_masters[1], 0, 32);
        configure_recv_packege(reg_masters[2], 0, 32);
        configure_recv_packege(reg_masters[3], 0, 32);

        // configure send
        // send a message from chip 0 to chip 1, 2, 3; with start addr 0 and len 32 bytes
        configure_network_ring_package (reg_masters[0], 0, 32, 4'd3, 4'b1110, 1);

        //wait packaget to recv
        fork
            wait_recv_package(reg_masters[1]);
            wait_recv_package(reg_masters[2]);
            wait_recv_package(reg_masters[3]);
        join
        
    end

endmodule