// Authors:

// - Pu Deng <piaodeng@stanford.edu>

`include "common_cells/registers.svh"
`include "common_cells/assertions.svh"
`include "axis/typedef.svh"

// First implementations: multiplexing 4 way data serial link


module meshed_serial_link_network #(
  parameter type axi_req_t  = logic,
  parameter type axi_rsp_t  = logic,
  parameter type reg2hw_t   = logic,
  parameter type hw2reg_t   = logic,
  parameter type axis_req_t = logic,
  parameter type axis_rsp_t = logic
) (
  input   logic                 clk_i,
  input   logic                 rst_ni,
  output  axi_req_t             axi_req_o,
  input   axi_rsp_t             axi_rsp_i,
  input   reg2hw_t              reg2hw_i,
  output  hw2reg_t              hw2reg_o,
  output  axis_req_t  [3:0]     axis_out_req_o,
  input   axis_rsp_t  [3:0]     axis_out_rsp_i,
  input   axis_req_t  [3:0]     axis_in_req_i,
  output  axis_rsp_t  [3:0]     axis_in_rsp_o 
)

  //////////////////
  // Data Fetcher //
  //////////////////

  localparam int AxiAddrWidth = $bits(axis_in_req_i.ar.addr)
  localparam int AxiLenWidth  = $bits(axis_in_req_i.ar.len)
  localparam int AxiDataWidth  = $bits(axis_in_req_i.w.data)

  logic [AxiAddrWidth:0]  ar_addr;

  logic [AxiAddrWidth:0] start_address = reg2hw_i.meshed_network_data_fetcher_start_addr.q;
  logic [AxiAddrWidth:0] dst_chip_start_address = reg2hw_i.meshed_network_data_fetcher_dst_chip_start_addr.q;
  logic [AxiAddrWidth:0] fetch_len  = reg2hw_i.meshed_network_data_fetcher_fetch_len.q; // in bytes

  logic trigger = reg2hw_i.meshed_network_data_fetcher_trigger.q;

  axis_req_t  fetcher_axis_req;
  axis_rsp_t  fetcher_axis_rsp;

  // TODO: data fetcher FSM and logic, pack the starting address and the len into the first package

  //////////////////////////
  //  Data outgoing deMUX //
  //////////////////////////

  logic [1:0] dst_chip = reg2hw_i.meshed_network_data_dst_chip;

  // TODO: a demux that distribute data strams from fetecher to different chips


  //////////////////////
  // Data ingoing Mux //
  //////////////////////

  axis_req_t  fetcher_axis_req;
  axis_rsp_t  fetcher_axis_rsp;

  logic [1:0] selected_source_data_link;

  // TODO: round robin wise check if there is valid axis req and serve them, forward to data writer


  /////////////////
  // Data writer //
  /////////////////

  logic [AxiAddrWidth:0]  aw_addr;
  logic [AxiDataWidth:0]  w_data;
  logic [AxiAddrWidth:0]  write_len;


  // TODO: get the destination addr and the length of the message from the first pacakage, generate write req



endmodule