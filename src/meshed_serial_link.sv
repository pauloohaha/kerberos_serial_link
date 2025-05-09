// Authors:

// - Pu Deng <piaodeng@stanford.edu>

`include "common_cells/registers.svh"
`include "common_cells/assertions.svh"
`include "axis/typedef.svh"

// A meshed chiplet inter-chip network. Consist of a netowrk controller and 4 way serial links

module meshed_serial_link #(
  // The number of physical chnannelsc per direction
  parameter int NumChannels       = 1,
  // The number of lanes per channel
  parameter int NumLanes          = 8,
  // Whether to enable DDR mode
  parameter bit EnDdr             = 1'b1,
  // Number of credits for flow control
  parameter int NumCredits        = 8,
  // The maximum clock division factor
  parameter int MaxClkDiv         = 1024,
  // Whether to use a register CDC for the configuration registers
  parameter bit NoRegCdc          = 1'b0,
  // The depth of the raw mode FIFO
  parameter int RawModeFifoDepth  = 8,
  // width of the AXIS payload data
  parameter int AXISdataWidth = 256,
  parameter type axi_req_t  = logic,
  parameter type axi_rsp_t  = logic,
  parameter type aw_chan_t  = logic,
  parameter type ar_chan_t  = logic,
  parameter type r_chan_t   = logic,
  parameter type w_chan_t   = logic,
  parameter type b_chan_t   = logic,
  parameter type cfg_req_t  = logic,
  parameter type cfg_rsp_t  = logic,
  parameter type hw2reg_t   = logic,
  parameter type reg2hw_t   = logic
) (
    // clock
    input  logic                          clk_i,
    input  logic                          rst_ni,

    // AXI data req
    output axi_req_t                      axi_req_o,
    input  axi_rsp_t                      axi_rsp_i,

    // ctrl reg inf
    input  cfg_req_t                      cfg_req_i,
    output cfg_rsp_t                      cfg_rsp_o,

    // Serial link inf
    input  logic [3:0][NumChannels-1:0]    ddr_rcv_clk_i,
    output logic [3:0][NumChannels-1:0]    ddr_rcv_clk_o,
    input  logic [3:0][NumChannels-1:0][NumLanes-1:0] ddr_i,
    output logic [3:0][NumChannels-1:0][NumLanes-1:0] ddr_o,

    // AXI isolation signals (in/out), if not used tie to 0
    input  logic [1:0]                isolated_i,
    output logic [1:0]                isolate_o,
    // Clock gate register
    output logic                      clk_ena_o,
    // synch-reset register
    output logic                      reset_no
);

  localparam int unsigned NumBitsPerCycle = NumLanes * (1 + EnDdr);

  typedef logic [$clog2(NumCredits):0] credit_t;
  typedef logic [NumBitsPerCycle-1:0] phy_data_t;

  // Payload in message passing protocol:
  // 1) data
  // 2) credit
  typedef struct packed {
    logic [AXISdataWidth-1:0] axi_ch;
    credit_t credit;
  } payload_t;

  localparam int BandWidth = NumChannels * NumBitsPerCycle; // doubled BW if DDR enabled
  localparam int PayloadSplits = ($bits(payload_t) + BandWidth - 1) / BandWidth;
  localparam int RecvFifoDepth = NumCredits * PayloadSplits;

  // Axi stream dimension must be a multiple of 8 bits
  localparam int StreamDataBytes = ($bits(payload_t) + 7) / 8;

  // Typdefs for Axi Stream interface
  // All except tdata_t are unused at the moment
  typedef logic [StreamDataBytes*8-1:0] tdata_t;
  typedef logic [StreamDataBytes-1:0] tstrb_t;
  typedef logic [StreamDataBytes-1:0] tkeep_t;
  typedef logic tlast_t;
  typedef logic tid_t;
  typedef logic tdest_t;
  typedef logic tuser_t;
  typedef logic tready_t;
  `AXIS_TYPEDEF_ALL(axis, tdata_t, tstrb_t, tkeep_t, tlast_t, tid_t, tdest_t, tuser_t, tready_t)

  // AXIs connections for 4 way serial links data link 
  axis_req_t  [3:0] axis_out_req;
  axis_req_t  [3:0] axis_in_req;

  axis_rsp_t  [3:0] axis_out_rsp;
  axis_rsp_t  [3:0] axis_in_rsp;

  cfg_req_t cfg_req;
  cfg_rsp_t cfg_rsp;

  reg2hw_t reg2hw;
  hw2reg_t hw2reg;

  phy_data_t [3:0][NumChannels-1:0]  data_link2alloc_data_out;
  logic [3:0][NumChannels-1:0]       data_link2alloc_data_out_valid;
  logic                         alloc2data_link_data_out_ready;

  phy_data_t [3:0][NumChannels-1:0]  alloc2data_link_data_in;
  logic [3:0][NumChannels-1:0]       alloc2data_link_data_in_valid;
  logic [3:0][NumChannels-1:0]       data_link2alloc_data_in_ready;

  phy_data_t [3:0][NumChannels-1:0]  alloc2phy_data_out;
  logic [3:0][NumChannels-1:0]       alloc2phy_data_out_valid;
  logic [3:0][NumChannels-1:0]       phy2alloc_data_out_ready;

  phy_data_t [3:0][NumChannels-1:0]  phy2alloc_data_in;
  logic [3:0][NumChannels-1:0]       phy2alloc_data_in_valid;
  logic [3:0][NumChannels-1:0]       alloc2phy_data_in_ready;

  ///////////////////////
  //   NETWORK LAYER   //
  ///////////////////////

  // TODO: implemented meshed network 

  meshed_serial_link_network #(

  ) i_meshed_serial_link_network (
    .clk_i                    ( clk_i         ),
    .rst_ni                   ( rst_ni        ),
    .axi_req_o                ( axi_req_o     ),
    .axi_rsp_i                ( axi_rsp_i     ),
    .reg2hw_i                 ( reg2hw        ),
    .hw2reg_o                 ( hw2reg        ),
    .axis_out_req_o           ( axis_out_req  ),
    .axis_out_rsp_i           ( axis_out_rsp  ),
    .axis_in_req_i            ( axis_in_req   ),
    .axis_in_rsp_o            ( axis_in_rsp   )
  );

  // 4 way links
  for (genvar i = 0; i < 4; i++) begin: gen_4_way_data_serial_link

    /////////////////////////
    //   DATA LINK LAYER   //
    /////////////////////////

    //TODO: fix reg connections for each way

    logic cfg_flow_control_fifo_clear;
    logic cfg_raw_mode_out_data_fifo_clear;

    assign cfg_flow_control_fifo_clear = reg2hw.flow_control_fifo_clear.q
      & reg2hw.flow_control_fifo_clear.qe;
    assign cfg_raw_mode_out_data_fifo_clear = reg2hw.raw_mode_out_data_fifo_ctrl.clear.q
      & reg2hw.raw_mode_out_data_fifo_ctrl.clear.qe;

    serial_link_data_link #(
      .axis_req_t       ( axis_req_t        ),
      .axis_rsp_t       ( axis_rsp_t        ),
      .phy_data_t       ( phy_data_t        ),
      .NumChannels      ( NumChannels       ),
      .NumLanes         ( NumLanes          ),
      .RecvFifoDepth    ( RecvFifoDepth     ),
      .RawModeFifoDepth ( RawModeFifoDepth  ),
      .PayloadSplits    ( PayloadSplits     ),
      .EnDdr            ( EnDdr             )
    ) i_serial_link_data_link (
      .clk_i                                   ( clk_sl_i                                         ),
      .rst_ni                                  ( rst_sl_ni                                        ),
      .axis_in_req_i                           ( axis_out_req[i]                                  ),
      .axis_in_rsp_o                           ( axis_out_rsp[i]                                  ),
      .axis_out_req_o                          ( axis_in_req[i]                                   ),
      .axis_out_rsp_i                          ( axis_in_rsp[i]                                   ),
      .data_out_o                              ( data_link2alloc_data_out[i]                         ),
      .data_out_valid_o                        ( data_link2alloc_data_out_valid[i]                   ),
      .data_out_ready_i                        ( alloc2data_link_data_out_ready[i]                   ),
      .data_in_i                               ( alloc2data_link_data_in[i]                          ),
      .data_in_valid_i                         ( alloc2data_link_data_in_valid[i]                    ),
      .data_in_ready_o                         ( data_link2alloc_data_in_ready[i]                    ),
      .cfg_flow_control_fifo_clear_i           ( cfg_flow_control_fifo_clear                      ),
      .cfg_raw_mode_en_i                       ( reg2hw.raw_mode_en                               ),
      .cfg_raw_mode_in_ch_sel_i                ( reg2hw.raw_mode_in_ch_sel                        ),
      .cfg_raw_mode_in_data_o                  ( hw2reg.raw_mode_in_data[NumBitsPerCycle-1:0]     ),
      .cfg_raw_mode_in_data_valid_o            ( hw2reg.raw_mode_in_data_valid                    ),
      .cfg_raw_mode_in_data_ready_i            ( reg2hw.raw_mode_in_data.re                       ),
      .cfg_raw_mode_out_ch_mask_i              ( reg2hw.raw_mode_out_ch_mask                      ),
      .cfg_raw_mode_out_data_i                 ( phy_data_t'(reg2hw.raw_mode_out_data_fifo.q)     ),
      .cfg_raw_mode_out_data_valid_i           ( reg2hw.raw_mode_out_data_fifo.qe                 ),
      .cfg_raw_mode_out_en_i                   ( reg2hw.raw_mode_out_en                           ),
      .cfg_raw_mode_out_data_fifo_clear_i      ( cfg_raw_mode_out_data_fifo_clear                 ),
      .cfg_raw_mode_out_data_fifo_fill_state_o ( hw2reg.raw_mode_out_data_fifo_ctrl.fill_state.d  ),
      .cfg_raw_mode_out_data_fifo_is_full_o    ( hw2reg.raw_mode_out_data_fifo_ctrl.is_full.d     )
    );

    ///////////////////////
    // CHANNEL ALLOCATOR //
    ///////////////////////

    //TODO: fix reg connections for each way

    logic cfg_tx_clear, cfg_rx_clear;
    logic cfg_tx_flush_trigger;

    assign cfg_tx_clear = reg2hw.channel_alloc_tx_ctrl.clear.q
      & reg2hw.channel_alloc_tx_ctrl.clear.qe;
    assign cfg_rx_clear = reg2hw.channel_alloc_rx_ctrl.q
      & reg2hw.channel_alloc_rx_ctrl.qe;
    assign cfg_tx_flush_trigger = reg2hw.channel_alloc_tx_ctrl.flush.q
      & reg2hw.channel_alloc_tx_ctrl.flush.qe;

    serial_link_channel_allocator #(
      .phy_data_t  ( phy_data_t    ),
      .NumChannels ( NumChannels   )
    ) i_channel_allocator(
      .clk_i                     ( clk_sl_i                                       ),
      .rst_ni                    ( rst_sl_ni                                      ),
      .cfg_tx_clear_i            ( cfg_tx_clear                                   ),
      .cfg_tx_channel_en_i       ( reg2hw.channel_alloc_tx_ch_en                  ),
      .cfg_tx_bypass_en_i        ( reg2hw.channel_alloc_tx_cfg.bypass_en.q        ),
      .cfg_tx_auto_flush_en_i    ( reg2hw.channel_alloc_tx_cfg.auto_flush_en.q    ),
      .cfg_tx_auto_flush_count_i ( reg2hw.channel_alloc_tx_cfg.auto_flush_count.q ),
      .cfg_tx_flush_trigger_i    ( cfg_tx_flush_trigger                           ),
      .cfg_rx_clear_i            ( cfg_rx_clear                                   ),
      .cfg_rx_bypass_en_i        ( reg2hw.channel_alloc_rx_cfg.bypass_en.q        ),
      .cfg_rx_channel_en_i       ( reg2hw.channel_alloc_rx_ch_en                  ),
      .cfg_rx_auto_flush_en_i    ( reg2hw.channel_alloc_rx_cfg.auto_flush_en.q    ),
      .cfg_rx_auto_flush_count_i ( reg2hw.channel_alloc_rx_cfg.auto_flush_count.q ),
      .cfg_rx_sync_en_i          ( reg2hw.channel_alloc_rx_cfg.sync_en.q          ),
      // From Data Link Layer
      .data_out_i                ( data_link2alloc_data_out[i]                    ),
      .data_out_valid_i          ( data_link2alloc_data_out_valid[i]              ),
      .data_out_ready_o          ( alloc2data_link_data_out_ready[i]              ),
      // To Phy
      .data_out_o                ( alloc2phy_data_out[i]                          ),
      .data_out_valid_o          ( alloc2phy_data_out_valid[i]                    ),
      .data_out_ready_i          ( phy2alloc_data_out_ready[i]                    ),
      // From Phy
      .data_in_i                 ( phy2alloc_data_in[i]                           ),
      .data_in_valid_i           ( phy2alloc_data_in_valid[i]                     ),
      .data_in_ready_o           ( alloc2phy_data_in_ready[i]                     ),
      // To Data Link Layer
      .data_in_o                 ( alloc2data_link_data_in[i]                     ),
      .data_in_valid_o           ( alloc2data_link_data_in_valid[i]               ),
      .data_in_ready_i           ( data_link2alloc_data_in_ready[i]               )
    );


    ////////////////////////
    //   PHYSICAL LAYER   //
    ////////////////////////

    //TODO: fix reg connections for each way

    for (genvar j = 0; j < NumChannels; j++) begin : gen_phy_channels
      serial_link_physical #(
        .NumLanes         ( NumLanes          ),
        .FifoDepth        ( RawModeFifoDepth  ),
        .MaxClkDiv        ( MaxClkDiv         ),
        .EnDdr            ( EnDdr             ),
        .phy_data_t       ( phy_data_t        )
      ) i_serial_link_physical (
        .clk_i             ( clk_sl_i                        ),
        .rst_ni            ( rst_sl_ni                       ),
        .clk_div_i         ( reg2hw.tx_phy_clk_div[j].q      ),
        .clk_shift_start_i ( reg2hw.tx_phy_clk_start[j].q    ),
        .clk_shift_end_i   ( reg2hw.tx_phy_clk_end[j].q      ),
        .ddr_rcv_clk_i     ( ddr_rcv_clk_i[i][j]             ),
        .ddr_rcv_clk_o     ( ddr_rcv_clk_o[i][j]             ),
        .data_out_i        ( alloc2phy_data_out[i][j]        ),
        .data_out_valid_i  ( alloc2phy_data_out_valid[i][j]  ),
        .data_out_ready_o  ( phy2alloc_data_out_ready[i][j]  ),
        .data_in_o         ( phy2alloc_data_in[i][j]         ),
        .data_in_valid_o   ( phy2alloc_data_in_valid[i][j]   ),
        .data_in_ready_i   ( alloc2phy_data_in_ready[i][j]   ),
        .ddr_i             ( ddr_i[i][j]                     ),
        .ddr_o             ( ddr_o[i][j]                     )
      );
    end

  end

  /////////////////////////////////
  //   CONFIGURATION REGISTERS   //
  /////////////////////////////////

  if (!NoRegCdc) begin : gen_reg_cdc
    reg_cdc #(
      .req_t  ( cfg_req_t ),
      .rsp_t  ( cfg_rsp_t )
    ) i_cdc_cfg (
      .src_clk_i  ( clk_reg_i   ),
      .src_rst_ni ( rst_reg_ni  ),
      .src_req_i  ( cfg_req_i   ),
      .src_rsp_o  ( cfg_rsp_o   ),

      .dst_clk_i  ( clk_i       ),
      .dst_rst_ni ( rst_ni      ),
      .dst_req_o  ( cfg_req     ),
      .dst_rsp_i  ( cfg_rsp     )
    );
  end else begin : gen_no_reg_cdc
    assign cfg_req = cfg_req_i;
    assign cfg_rsp_o = cfg_rsp;
  end

  //TODO: fix reg connections for each way

  meshed_serial_link_reg_top #(
    .reg_req_t (cfg_req_t),
    .reg_rsp_t (cfg_rsp_t)
  ) i_serial_link_reg_top (
    .clk_i      ( clk_i       ),
    .rst_ni     ( rst_ni      ),
    .reg_req_i  ( cfg_req     ),
    .reg_rsp_o  ( cfg_rsp     ),
    .reg2hw     ( reg2hw      ),
    .hw2reg     ( hw2reg      ),
    .devmode_i  ( testmode_i  )
  );

  assign clk_ena_o = reg2hw.ctrl.clk_ena.q;
  assign reset_no = reg2hw.ctrl.reset_n.q;
  assign isolate_o = {reg2hw.ctrl.axi_out_isolate.q, reg2hw.ctrl.axi_in_isolate.q};
  assign hw2reg.isolated.axi_in.d = isolated_i[0];
  assign hw2reg.isolated.axi_out.d = isolated_i[1];

  ////////////////////
  //   ASSERTIONS   //
  ////////////////////

  `ASSERT_INIT(RawModeFifoDim, RecvFifoDepth >= RawModeFifoDepth)

endmodule