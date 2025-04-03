// Copyright 2022 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Author: Tim Fischer <fischeti@iis.ee.ethz.ch>

/// A simple serial link package
package serial_link_pkg;

  // Physical Layer parameters
  // Also modify in serial_link.hjson!
  localparam int NumChannels = 38;
  localparam int NumLanes = 8;

  // Number of outstanding transactions for flow control
  // FIFO depths depend on it -> expensive to increase!
  localparam int NumCredits = 8;

  // Number of words in the pattern sequence that can be sent out at once
  // Must be smaller than RecvFifoDepth
  localparam int RawModeFifoDepth = 8;

  // Maximum Clock division
  localparam int MaxClkDiv = 1024;

  // Also modify the "NumBit", "TX_PHY_CLK_START" and "TX_PHY_CLK_END parameter
  //accordingly in serial_link.hjson or serial_link_single_channel.hjson, respectively
  localparam bit EnDdr = 1'b1; //by default 1, set to 0 for SDR

  typedef logic [NumLanes*(1+EnDdr)-1:0] phy_data_t;
  typedef logic [NumLanes-1:0] phy_ddr_data_t;

  typedef enum logic [1:0] {LinkSendIdle, LinkSendBusy} link_state_e;

  // TODO(zarubaf,fschuiki) ICEBOX: Fix
  // B beats are generated on the write side without the knowledge of the
  // receiver. Hence we also have no means to see whether writes actually
  // succeeded.
  typedef enum logic [3:0]  {
    TagIdle       = 0,
    TagAW         = 1,
    TagW          = 2,
    TagAR         = 3,
    TagR          = 4
  } tag_e;

  typedef logic [$clog2(NumCredits):0] credit_t;

  // PHY FSM
  typedef enum logic [2:0] {
    PHYIdle           = 0,
    PHYBusy           = 1,
    PHYTrain          = 2,
    PHYTrainWaitPeer  = 3,
    PHYTrainWaitIdle  = 4
  } phy_state_e;

  function automatic int find_max_channel(input int channel[5]);
    int max_value = 0;
    for (int i = 0; i < 5; i++) begin
      if (max_value < channel[i]) max_value = channel[i];
    end
    return max_value;
  endfunction

  /*Serial Link ctrl reg define*/

  `include "register_interface/typedef.svh"

  localparam int unsigned SerialLinkRegAddrWidth    = 32;
  localparam int unsigned SerialLinkRegDataWidth    = 32;
  localparam int unsigned SerialLinkRegStrbWidth    = SerialLinkRegDataWidth / 8;

  // RegBus types for typedefs
  typedef logic [SerialLinkRegAddrWidth-1:0]  serial_link_cfg_addr_t;
  typedef logic [SerialLinkRegDataWidth-1:0]  serial_link_cfg_data_t;
  typedef logic [SerialLinkRegStrbWidth-1:0]  serial_link_cfg_strb_t;

  `REG_BUS_TYPEDEF_ALL(serial_link_cfg, serial_link_cfg_addr_t, serial_link_cfg_data_t, serial_link_cfg_strb_t)

endpackage : serial_link_pkg
