// Copyright 2022 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Author: Tim Fischer <fischeti@iis.ee.ethz.ch>
// Authro: Pu Deng <piaodeng@stanford.edu>

/// A wrapper around the Serial Link intended for integration into Kerberis, based on wrapper for Occamy
/// The wrapper additionally includes AXI isolation, reset controller & clock gating
module serial_link_kerberos_wrapper #(
  parameter type axi_req_t  = logic,
  parameter type axi_rsp_t  = logic,
  parameter type aw_chan_t  = logic,
  parameter type ar_chan_t  = logic,
  parameter type r_chan_t   = logic,
  parameter type w_chan_t   = logic,
  parameter type b_chan_t   = logic,
  parameter type cfg_req_t  = logic,
  parameter type cfg_rsp_t  = logic,
  parameter int NumChannels = 1,
  parameter int NumLanes = 4,
  parameter int MaxClkDiv = 32,
  parameter bit EnDdr = 1'b1
) (
  input  logic                      clk_i,
  input  logic                      rst_ni,
  input  logic                      clk_reg_i,
  input  logic                      rst_reg_ni,
  input  logic                      testmode_i,
  input  axi_req_t                  axi_in_req_i,
  output axi_rsp_t                  axi_in_rsp_o,
  output axi_req_t                  axi_out_req_o,
  input  axi_rsp_t                  axi_out_rsp_i,
  input  cfg_req_t                  cfg_req_i,
  output cfg_rsp_t                  cfg_rsp_o,
  input  logic [3:0] [NumChannels-1:0]    ddr_rcv_clk_i,
  output logic [3:0] [NumChannels-1:0]    ddr_rcv_clk_o,
  input  logic [3:0] [NumChannels-1:0][NumLanes-1:0] ddr_i,
  output logic [3:0] [NumChannels-1:0][NumLanes-1:0] ddr_o,

  input  logic [1:0]  direction_select
);

  logic clk_serial_link;
  logic rst_serial_link_n;

  logic clk_ena;
  logic reset_n;
  

  axi_req_t axi_in_req, axi_out_req;
  axi_rsp_t axi_in_rsp, axi_out_rsp;

  // Quadrant clock gate controlled by register
  tc_clk_gating i_tc_clk_gating (
    .clk_i,
    .en_i (clk_ena),
    .test_en_i (testmode_i),
    .clk_o (clk_serial_link)
  );

  // Reset directly from register (i.e. (de)assertion inherently synchronized)
  // Multiplex with glitchless multiplexor, top reset for testing purposes
  tc_clk_mux2 i_tc_reset_mux (
    .clk0_i (reset_n),
    .clk1_i (rst_ni),
    .clk_sel_i (testmode_i),
    .clk_o (rst_serial_link_n)
  );

  logic [1:0] isolated, isolate;

  axi_isolate #(
    .TerminateTransaction(0),
    .AtopSupport(1),
    .AxiIdWidth($bits(axi_in_req_i.aw.id)),
    .AxiAddrWidth($bits(axi_in_req_i.aw.addr)),
    .AxiDataWidth($bits(axi_in_req_i.w.data)),
    .AxiUserWidth($bits(axi_in_req_i.aw.user)),
    .axi_req_t    ( axi_req_t                   ),
    .axi_resp_t   ( axi_rsp_t                   )

  ) i_serial_link_in_isolate  (
    .clk_i        ( clk_i         ),
    .rst_ni       ( rst_ni        ),
    .slv_req_i    ( axi_in_req_i  ),
    .slv_resp_o   ( axi_in_rsp_o  ),
    .mst_req_o    ( axi_in_req    ),
    .mst_resp_i   ( axi_in_rsp    ),
    .isolate_i    ( isolate[0]    ),
    .isolated_o   ( isolated[0]   )
  );

  axi_isolate #(
    .TerminateTransaction(0),
    .AtopSupport(1),
    .AxiIdWidth($bits(axi_in_req_i.aw.id)),
    .AxiAddrWidth($bits(axi_in_req_i.aw.addr)),
    .AxiDataWidth($bits(axi_in_req_i.w.data)),
    .AxiUserWidth($bits(axi_in_req_i.aw.user)),
    .axi_req_t        ( axi_req_t                   ),
    .axi_resp_t       ( axi_rsp_t                   )

  ) i_serial_link_out_isolate (
    .clk_i        ( clk_i         ),
    .rst_ni       ( rst_ni        ),
    .slv_req_i    ( axi_out_req   ),
    .slv_resp_o   ( axi_out_rsp   ),
    .mst_req_o    ( axi_out_req_o ),
    .mst_resp_i   ( axi_out_rsp_i ),
    .isolate_i    ( isolate[1]    ),
    .isolated_o   ( isolated[1]   )
  );


  axi_req_t   [3:0] serial_link_axi_in_req, serial_link_axi_out_req;
  axi_rsp_t   [3:0] serial_link_axi_in_rsp, serial_link_axi_out_rsp;

  always_comb begin : data_serial_link_mux
      serial_link_axi_in_req = 'd0;
      serial_link_axi_out_rsp = 'd0;

      case (direction_select)
        2'b00: begin
          serial_link_axi_in_req[0] = axi_in_req;
          axi_in_rsp = serial_link_axi_in_rsp[0];

          axi_out_req = serial_link_axi_out_req[0];
          serial_link_axi_out_rsp[0] = axi_out_rsp;
        end

        2'b01: begin
          serial_link_axi_in_req[1] = axi_in_req;
          axi_in_rsp = serial_link_axi_in_rsp[1];

          axi_out_req = serial_link_axi_out_req[1];
          serial_link_axi_out_rsp[1] = axi_out_rsp;
        end

        2'b10: begin
          serial_link_axi_in_req[2] = axi_in_req;
          axi_in_rsp = serial_link_axi_in_rsp[2];

          axi_out_req = serial_link_axi_out_req[2];
          serial_link_axi_out_rsp[2] = axi_out_rsp;
        end

        2'b11: begin
          serial_link_axi_in_req[3] = axi_in_req;
          axi_in_rsp = serial_link_axi_in_rsp[3];

          axi_out_req = serial_link_axi_out_req[3];
          serial_link_axi_out_rsp[3] = axi_out_rsp;
        end

        default: begin
          serial_link_axi_in_req[3] = axi_in_req;
          axi_in_rsp = serial_link_axi_in_rsp[3];

          axi_out_req = serial_link_axi_out_req[3];
          serial_link_axi_out_rsp[3] = axi_out_rsp;
        end
      endcase
  end
  for (genvar i = 0; i < 4; i++) begin: gen_data_serial_link

      if (i == 0) begin
        serial_link #(
          .axi_req_t        ( axi_req_t   ),
          .axi_rsp_t        ( axi_rsp_t   ),
          .aw_chan_t        ( aw_chan_t   ),
          .w_chan_t         ( w_chan_t    ),
          .b_chan_t         ( b_chan_t    ),
          .ar_chan_t        ( ar_chan_t   ),
          .r_chan_t         ( r_chan_t    ),
          .cfg_req_t        ( cfg_req_t   ),
          .cfg_rsp_t        ( cfg_rsp_t   ),
          .hw2reg_t         ( serial_link_reg_pkg::serial_link_hw2reg_t ),
          .reg2hw_t         ( serial_link_reg_pkg::serial_link_reg2hw_t ),
          .NumChannels      ( NumChannels ),
          .NumLanes         ( NumLanes    ),
          .MaxClkDiv        ( MaxClkDiv   ),
          .EnDdr            ( EnDdr       )
        ) i_serial_link (
          .clk_i          ( clk_i                       ),
          .rst_ni         ( rst_ni                      ),
          .clk_sl_i       ( clk_serial_link             ),
          .rst_sl_ni      ( rst_serial_link_n           ),
          .clk_reg_i      ( clk_reg_i                   ),
          .rst_reg_ni     ( rst_reg_ni                  ),
          .testmode_i     ( 1'b0                        ),
          .axi_in_req_i   ( serial_link_axi_in_req[i]   ),
          .axi_in_rsp_o   ( serial_link_axi_in_rsp[i]   ),
          .axi_out_req_o  ( serial_link_axi_out_req[i]  ),
          .axi_out_rsp_i  ( serial_link_axi_out_rsp[i]  ),
          .cfg_req_i      ( cfg_req_i                   ),
          .cfg_rsp_o      ( cfg_rsp_o                   ),
          .ddr_rcv_clk_i  ( ddr_rcv_clk_i[i]            ),
          .ddr_rcv_clk_o  ( ddr_rcv_clk_o[i]            ),
          .ddr_i          ( ddr_i[i]                    ),
          .ddr_o          ( ddr_o[i]                    ),
          .isolated_i     ( isolated                    ),
          .isolate_o      ( isolate                     ),
          .clk_ena_o      ( clk_ena                     ),
          .reset_no       ( reset_n                     )
        );
      end else begin
        serial_link #(
          .axi_req_t        ( axi_req_t   ),
          .axi_rsp_t        ( axi_rsp_t   ),
          .aw_chan_t        ( aw_chan_t   ),
          .w_chan_t         ( w_chan_t    ),
          .b_chan_t         ( b_chan_t    ),
          .ar_chan_t        ( ar_chan_t   ),
          .r_chan_t         ( r_chan_t    ),
          .cfg_req_t        ( cfg_req_t   ),
          .cfg_rsp_t        ( cfg_rsp_t   ),
          .hw2reg_t         ( serial_link_reg_pkg::serial_link_hw2reg_t ),
          .reg2hw_t         ( serial_link_reg_pkg::serial_link_reg2hw_t ),
          .NumChannels      ( NumChannels ),
          .NumLanes         ( NumLanes    ),
          .MaxClkDiv        ( MaxClkDiv   ),
          .EnDdr            ( EnDdr       )
        ) i_serial_link (
          .clk_i          ( clk_i                       ),
          .rst_ni         ( rst_ni                      ),
          .clk_sl_i       ( clk_serial_link             ),
          .rst_sl_ni      ( rst_serial_link_n           ),
          .clk_reg_i      ( clk_reg_i                   ),
          .rst_reg_ni     ( rst_reg_ni                  ),
          .testmode_i     ( 1'b0                        ),
          .axi_in_req_i   ( serial_link_axi_in_req[i]   ),
          .axi_in_rsp_o   ( serial_link_axi_in_rsp[i]   ),
          .axi_out_req_o  ( serial_link_axi_out_req[i]  ),
          .axi_out_rsp_i  ( serial_link_axi_out_rsp[i]  ),
          .cfg_req_i      ( cfg_req_i                   ),
          .cfg_rsp_o      ( /*unsed*/                   ),
          .ddr_rcv_clk_i  ( ddr_rcv_clk_i[i]            ),
          .ddr_rcv_clk_o  ( ddr_rcv_clk_o[i]            ),
          .ddr_i          ( ddr_i[i]                    ),
          .ddr_o          ( ddr_o[i]                    ),
          .isolated_i     ( isolated                    ),
          .isolate_o      ( /*unsed*/                   ),
          .clk_ena_o      ( /*unsed*/                   ),
          .reset_no       ( /*unsed*/                   )
        );
      end
      
  end

endmodule
