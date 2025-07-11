// Author: Pu Deng <piaodeng@stanford.edu>


module tb_meshed_serial_link;

    localparam time         Tck = 200ns;
    localparam bit EnDdr = 1'b1;
    localparam int DATA_WIDTH = 256;
    localparam int MEM_SIZE_BYTE                = 4 * 1024 * 1024; //4MB
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

    // ==============
    //    Config
    // ==============
    localparam int unsigned TestDuration    = 100;
    localparam int unsigned MaxClkDiv       = serial_link_pkg::MaxClkDiv;

    localparam time         TckSys1         = 50ns;
    localparam time         TckSys2         = 54ns;
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
    logic [3:0][NumChannels-1:0]  ddr_rcv_clk_1, ddr_rcv_clk_2;
    axi_req_t   axi_req_1, axi_req_2;
    axi_resp_t  axi_rsp_1, axi_rsp_2;
    cfg_req_t   cfg_req_1;
    cfg_rsp_t   cfg_rsp_1;
    cfg_req_t   cfg_req_2;
    cfg_rsp_t   cfg_rsp_2;

    // link
    wire [3:0][NumChannels*NumLanes-1:0] ddr_o_1;
    wire [3:0][NumChannels*NumLanes-1:0] ddr_o_2;

    // clock and reset
    logic clk_1, clk_2, clk_reg;
    logic rst_1_n, rst_2_n, rst_reg_n;

    // system clock and reset
    clk_rst_gen #(
      .ClkPeriod    ( TckReg          ),
      .RstClkCycles ( RstClkCyclesSys )
    ) i_clk_rst_gen_reg (
      .clk_o  ( clk_reg   ),
      .rst_no ( rst_reg_n )
    );

    clk_rst_gen #(
      .ClkPeriod    ( TckSys1         ),
      .RstClkCycles ( RstClkCyclesSys )
    ) i_clk_rst_gen_sys_1 (
      .clk_o  ( clk_1   ),
      .rst_no ( rst_1_n )
    );

    clk_rst_gen #(
      .ClkPeriod    ( TckSys2          ),
      .RstClkCycles ( RstClkCyclesSys  )
    ) i_clk_rst_gen_sys_2 (
      .clk_o  ( clk_2   ),
      .rst_no ( rst_2_n )
    );


  /**************************
   *  Serial Link System 1  *
   **************************/

  // Memory for 1st serial link

  // Interface with the memory with the TCDM protocol
  `TCDM_TYPEDEF_ALL(main_mem, logic [AxiAddrWidth-1:0], logic [AxiDataWidth-1:0], logic [AxiDataWidth/8-1:0], logic)
  main_mem_req_t main_mem_1_req;
  main_mem_rsp_t main_mem_1_rsp;

  meshed_serial_link #(
    .NumChannels      (NumChannels      ),
    .NumLanes         (8                ),
    .EnDdr            (EnDdr            ),
    .axi_req_t        ( axi_req_t       ),
    .axi_rsp_t        ( axi_resp_t      ),
    .aw_chan_t        ( axi_aw_chan_t   ),
    .w_chan_t         ( axi_w_chan_t    ),
    .b_chan_t         ( axi_b_chan_t    ),
    .ar_chan_t        ( axi_ar_chan_t   ),
    .r_chan_t         ( axi_r_chan_t    ),
    .cfg_req_t        ( cfg_req_t       ),
    .cfg_rsp_t        ( cfg_rsp_t       )
  ) i_serial_link_1(
    .clk_i          ( clk_1           ),
    .rst_ni         ( rst_1_n         ),
    .clk_sl_i       ( clk_1           ),
    .rst_sl_ni      ( rst_1_n         ),
    .clk_reg_i      ( clk_reg         ),
    .rst_reg_ni     ( rst_reg_n       ),
    .testmode_i     ( 1'b0            ),
    .axi_req_o      ( axi_req_1       ),
    .axi_rsp_i      ( axi_rsp_1       ),
    .cfg_req_i      ( cfg_req_1       ),
    .cfg_rsp_o      ( cfg_rsp_1       ),
    .ddr_rcv_clk_i  ( ddr_rcv_clk_2   ),
    .ddr_rcv_clk_o  ( ddr_rcv_clk_1   ),
    .ddr_i          ( ddr_o_2         ),
    .ddr_o          ( ddr_o_1         ),
    .isolated_i     ( 8'b0            ), /*unused*/
    .isolate_o      ( /*unused*/      ),
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
  ) i_axi_to_main_memory_1 (
    .clk_i     (  clk_1            ),
    .rst_ni    (  rst_1_n          ),
    .axi_req_i (  axi_req_1        ),
    .axi_rsp_o (  axi_rsp_1        ),
    .tcdm_req_o(  main_mem_1_req   ),
    .tcdm_rsp_i(  main_mem_1_rsp   )
  );

  // Main memory
  localparam int unsigned MainMemDataWidthInBytes = AxiIdWidth / 8;

  tc_sram #(
    .DataWidth(AxiDataWidth      ),
    .NumPorts (1                 ),
    .NumWords (MEM_NUM_WORD      )
  ) i_main_memory_1 (
    .clk_i  (clk_1                                                                                                                       ),
    .rst_ni (rst_1_n                                                                                                                     ),
    .req_i  (main_mem_1_req.q_valid                                                                                                      ),
    .addr_i (main_mem_1_req.q.addr[idx_width(MEM_NUM_WORD)+idx_width(MainMemDataWidthInBytes)-1:idx_width(MainMemDataWidthInBytes)]      ),
    .be_i   (main_mem_1_req.q.strb                                                                                                       ),
    .wdata_i(main_mem_1_req.q.data                                                                                                       ),
    .we_i   (main_mem_1_req.q.write                                                                                                      ),
    .rdata_o(main_mem_1_rsp.p.data                                                                                                       )
  );

  // Always ready
  assign main_mem_1_rsp.q_ready = 1'b1;
  // One cycle latency
  `FF(main_mem_1_rsp.p_valid, main_mem_1_req.q_valid, 1'b0, clk_1, rst_1_n)


  /**************************
   *  Serial Link System 2  *
   **************************/

  // Memory for 2nd serial link

  // Interface with the memory with the TCDM protocol
  main_mem_req_t main_mem_2_req;
  main_mem_rsp_t main_mem_2_rsp;

  meshed_serial_link #(
    .NumChannels      (NumChannels      ),
    .NumLanes         (8                ),
    .EnDdr            (EnDdr            ),
    .axi_req_t        ( axi_req_t       ),
    .axi_rsp_t        ( axi_resp_t      ),
    .aw_chan_t        ( axi_aw_chan_t   ),
    .w_chan_t         ( axi_w_chan_t    ),
    .b_chan_t         ( axi_b_chan_t    ),
    .ar_chan_t        ( axi_ar_chan_t   ),
    .r_chan_t         ( axi_r_chan_t    ),
    .cfg_req_t        ( cfg_req_t       ),
    .cfg_rsp_t        ( cfg_rsp_t       )
  ) i_serial_link_2(
    .clk_i          ( clk_2           ),
    .rst_ni         ( rst_2_n         ),
    .clk_sl_i       ( clk_2           ),
    .rst_sl_ni      ( rst_2_n         ),
    .clk_reg_i      ( clk_reg         ),
    .rst_reg_ni     ( rst_reg_n       ),
    .testmode_i     ( 1'b0            ),
    .axi_req_o      ( axi_req_2       ),
    .axi_rsp_i      ( axi_rsp_2       ),
    .cfg_req_i      ( cfg_req_2       ),
    .cfg_rsp_o      ( cfg_rsp_2       ),
    .ddr_rcv_clk_i  ( ddr_rcv_clk_1   ),
    .ddr_rcv_clk_o  ( ddr_rcv_clk_2   ),
    .ddr_i          ( ddr_o_1         ),
    .ddr_o          ( ddr_o_2         ),
    .isolated_i     ( 8'b0            ), /*unused*/
    .isolate_o      ( /*unused*/      ),
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
  ) i_axi_to_main_memory_2 (
    .clk_i     (  clk_2            ),
    .rst_ni    (  rst_2_n          ),
    .axi_req_i (  axi_req_2        ),
    .axi_rsp_o (  axi_rsp_2        ),
    .tcdm_req_o(  main_mem_2_req   ),
    .tcdm_rsp_i(  main_mem_2_rsp   )
  );

  tc_sram #(
    .DataWidth(AxiDataWidth      ),
    .NumPorts (1                 ),
    .NumWords (MEM_NUM_WORD      )
  ) i_main_memory_2 (
    .clk_i  (clk_2                                                                                                                       ),
    .rst_ni (rst_2_n                                                                                                                     ),
    .req_i  (main_mem_2_req.q_valid                                                                                                      ),
    .addr_i (main_mem_2_req.q.addr[idx_width(MEM_NUM_WORD)+idx_width(MainMemDataWidthInBytes)-1:idx_width(MainMemDataWidthInBytes)]      ),
    .be_i   (main_mem_2_req.q.strb                                                                                                       ),
    .wdata_i(main_mem_2_req.q.data                                                                                                       ),
    .we_i   (main_mem_2_req.q.write                                                                                                      ),
    .rdata_o(main_mem_2_rsp.p.data                                                                                                       )
  );

  // Always ready
  assign main_mem_2_rsp.q_ready = 1'b1;
  // One cycle latency
  `FF(main_mem_2_rsp.p_valid, main_mem_2_req.q_valid, 1'b0, clk_2, rst_2_n)


  ////////////////
  // Test logic //
  ////////////////

  REG_BUS #(
    .ADDR_WIDTH (RegAddrWidth),
    .DATA_WIDTH (RegDataWidth)
  ) cfg_1(clk_reg), cfg_2(clk_reg);

  `REG_BUS_ASSIGN_TO_REQ(cfg_req_1, cfg_1)
  `REG_BUS_ASSIGN_FROM_RSP(cfg_1, cfg_rsp_1)

  `REG_BUS_ASSIGN_TO_REQ(cfg_req_2, cfg_2)
  `REG_BUS_ASSIGN_FROM_RSP(cfg_2, cfg_rsp_2)

  typedef reg_test::reg_driver #(
    .AW ( RegAddrWidth  ),
    .DW ( RegDataWidth  ),
    .TA ( 100ps         ),
    .TT ( 500ps         )
  ) reg_master_t;

  static reg_master_t reg_master_1 = new ( cfg_1 );
  static reg_master_t reg_master_2 = new ( cfg_2 );

  initial begin
      reg_master_1.reset_master();
      
      configure_network_package (reg_master_1, 0, 0, 32, 0, 0);
  end

  task automatic cfg_write(reg_master_t drv, cfg_addr_t addr, cfg_data_t data, cfg_strb_t strb='1);
    automatic logic resp;
    drv.send_write(addr, data, strb, resp);
    assert (!resp) else $error("Not able to write cfg reg");
  endtask

  task automatic configure_network_package (reg_master_t drv, axi_addr_t start_addr, axi_addr_t dst_chip_statr_addr, axi_addr_t data_len, logic [3:0] dst_chip_id, logic [1:0] out_dir);
      automatic axi_addr_t meshed_network_ctrl_reg_offset = serial_link_pkg::linkCtrlRegLen * 4;

      // config start addr
      cfg_write(drv, meshed_network_ctrl_reg_offset + meshed_network_ctrl_regs_reg_pkg::MESHED_NETWORK_CTRL_REGS_MESHED_NETWORK_DATA_FETCHER_START_ADDR_OFFSET, start_addr);

      // config start addr at dst chip
      cfg_write(drv, meshed_network_ctrl_reg_offset + meshed_network_ctrl_regs_reg_pkg::MESHED_NETWORK_CTRL_REGS_MESHED_NETWORK_DATA_FETCHER_DST_CHIP_START_ADDR_OFFSET, dst_chip_statr_addr);

      // config length of package
      cfg_write(drv, meshed_network_ctrl_reg_offset + meshed_network_ctrl_regs_reg_pkg::MESHED_NETWORK_CTRL_REGS_MESHED_NETWORK_DATA_FETCHER_LEN_OFFSET, data_len);

      // config dst chip id
      cfg_write(drv, meshed_network_ctrl_reg_offset + meshed_network_ctrl_regs_reg_pkg::MESHED_NETWORK_CTRL_REGS_MESHED_NETWORK_DATA_DST_CHIP_OFFSET, dst_chip_id);

      // config out dir
      cfg_write(drv, meshed_network_ctrl_reg_offset + meshed_network_ctrl_regs_reg_pkg::MESHED_NETWORK_CTRL_REGS_MESHED_NETWORK_DATA_OUT_DIR_OFFSET, out_dir);

      // trigger
      cfg_write(drv, meshed_network_ctrl_reg_offset + meshed_network_ctrl_regs_reg_pkg::MESHED_NETWORK_CTRL_REGS_MESHED_NETWORK_DATA_FETCHER_TRIGGER_OFFSET, 'd1);

  endtask

endmodule