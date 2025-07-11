// Authors:

// - Pu Deng <piaodeng@stanford.edu>

`include "common_cells/registers.svh"
`include "common_cells/assertions.svh"
`include "axis/typedef.svh"

// Use the ring on mesh FlooNoc router as the core
// Data fetcher is responsible for fetching the local data
// Data writer is responsible for writing the arriving traffic in to memory
// Credit control responsible for tracking the credit at each port


module meshed_serial_link_network import floo_pkg::*; import serial_link_pkg::*; #(
  parameter type axi_req_t  = logic,
  parameter type axi_rsp_t  = logic,
  parameter type reg2hw_t   = logic,
  parameter type hw2reg_t   = logic,
  parameter type axis_req_t = logic,
  parameter type axis_rsp_t = logic,
  parameter type payload_t  = logic,
  parameter int  max_outstanding_axi_req = 3,

  // ring on mesh router
  parameter int unsigned NumNodes         = 16, // Number of external port
  parameter int unsigned NumRoutes        = 4, // Number of external port
  parameter int unsigned NumVirtChannels  = 0,
  parameter type         flit_t           = logic,
  parameter int unsigned InFifoDepth      = 0,
  parameter route_algo_e RouteAlgo        = IdTable,
  parameter int unsigned IdWidth          = 0,
  parameter int NumCredits                = 8 // Number of credits for flow control
  // Force send out credits belonging to the other side
  // after ForceSendThresh is reached
  localparam int ForceSendThresh  = NumCredits - 4
) (
  input   logic                           clk_i,
  input   logic                           rst_ni,
  output  axi_req_t                       axi_req_o,
  input   axi_rsp_t                       axi_rsp_i,
  input   reg2hw_t                        reg2hw_i,
  output  hw2reg_t                        hw2reg_o,
  output  axis_req_t  [NumRoutes-1:0]     axis_out_req_o,
  input   axis_rsp_t  [NumRoutes-1:0]     axis_out_rsp_i,
  input   axis_req_t  [NumRoutes-1:0]     axis_in_req_i,
  output  axis_rsp_t  [NumRoutes-1:0]     axis_in_rsp_o 
);

  /* Inter component connections */
  logic       [NumRoutes:0][NumVirtChannels-1:0] router_valid_in, router_valid_out;
  logic       [NumRoutes:0][NumVirtChannels-1:0] router_ready_in, router_ready_out;
  flit_t      [NumRoutes:0]                      router_data_in,  router_data_out;

  //////////////////
  // Data Fetcher //
  //////////////////

  localparam int AxiAddrWidth   = $bits(axi_req_o.ar.addr);
  localparam int AxiLenWidth    = $bits(axi_req_o.ar.len);
  localparam int AxiDataWidth   = $bits(axi_req_o.w.data);
  localparam int AxiDataByteLen = AxiDataWidth / 8; //length of each AXI word in byte format

  logic [$clog2(max_outstanding_axi_req) - 1:0] axi_credit_d, axi_credit_q; 

  logic [AxiAddrWidth:0] fetch_start_address;
  logic [AxiAddrWidth:0] fetch_len;

  logic data_fetcher_trigger_d;
  logic data_fetcher_trigger_q, data_fetcher_trigger; 

  logic network_package_done_d;
  logic network_package_done_de;

  flit_t      fetcher_flit;
  logic       fetcher_flit_vld, fetcher_flit_rdy;

  flit_t      injection_flit;
  logic       injection_flit_vld, injection_flit_rdy;
  logic [$clog2(NumVirtChannels)-1:0] injection_vc;

  `FF(data_fetcher_trigger_q, data_fetcher_trigger_d, 'd0, clk_i, rst_ni)
  `FF(axi_credit_q, axi_credit_d, max_outstanding_axi_req, clk_i, rst_ni)

  typedef enum logic [0:0] {
    REQ_IDLE_STATE,
    SEND_REQ_STATE
  } data_requester_fsm_state_t;

  /* Data requester */
  data_requester_fsm_state_t data_requester_fsm_current_state, data_requester_fsm_next_state;
  logic [AxiAddrWidth:0] requested_data_len_d, requested_data_len_q;

  /* sample the configurations at the rising edge of the trigger*/
  logic [AxiAddrWidth:0] current_fetch_address_d, current_fetch_address_q;

  /* reg2hw */
  assign fetch_start_address  = reg2hw_i.meshed_network_data_fetcher_data.fetcher_start_addr.q;
  assign fetch_len            = reg2hw_i.meshed_network_data_fetcher_data.fetcher_len.q; // in bytes

  assign data_fetcher_trigger_d = reg2hw_i.meshed_network_ctrl.send_start.q;

  /* hw2reg */
  assign hw2reg_o.meshed_network_status.send_done.d   = network_package_done_d;
  assign hw2reg_o.meshed_network_status.send_done.de  = network_package_done_de;

  assign  hw2reg_o.meshed_network_status.send_rdy.d  = (data_requester_fsm_current_state == REQ_IDLE_STATE) ? 1'b1 : 1'b0;
  assign  hw2reg_o.meshed_network_status.send_rdy.de = 1'b1;

  assign  hw2reg_o.meshed_network_status.data_requester_state.d   = data_requester_fsm_current_state;
  assign  hw2reg_o.meshed_network_status.data_requester_state.de  = 1'b1;

  //self reseting trigger
  assign hw2reg_o.meshed_network_ctrl.send_start.d   = 1'b0;
  assign hw2reg_o.meshed_network_ctrl.send_start.de  = data_fetcher_trigger_d;

  // data_fetcher_trigger is a one cycle high signal at the rising cycle of the trigger
  assign data_fetcher_trigger = (~data_fetcher_trigger_q) & data_fetcher_trigger_d; 

  `FF(data_requester_fsm_current_state, data_requester_fsm_next_state, REQ_IDLE_STATE, clk_i, rst_ni)
  `FF(requested_data_len_q, requested_data_len_d, 'd0, clk_i, rst_ni)
  `FF(current_fetch_address_q, current_fetch_address_d, 'd0, clk_i, rst_ni)

  /* data requester FSM state ctrl*/
  always_comb begin
    case (data_requester_fsm_current_state)
      REQ_IDLE_STATE:       data_requester_fsm_next_state = (data_fetcher_trigger) ? SEND_REQ_STATE : REQ_IDLE_STATE;
      SEND_REQ_STATE:       data_requester_fsm_next_state = (requested_data_len_q + AxiDataByteLen) > fetch_len ? 
                                                                                (axi_rsp_i.ar_ready ? REQ_IDLE_STATE : SEND_REQ_STATE) : 
                                                                                SEND_REQ_STATE; 
                                                                                // If next fetch exceed the fetch length and current request is accepeted, goto IDLE
      default:              data_requester_fsm_next_state = REQ_IDLE_STATE;
    endcase
  end

  /* data requester logic */
  always_comb begin
      axi_req_o.ar          = 'd0;
      axi_req_o.ar.id       = 'd0;
      axi_req_o.ar.addr     = 'd0;
      axi_req_o.ar.len      = 'd0;
      axi_req_o.ar.size     = 3'b101;
      axi_req_o.ar.burst    = 'd0;
      axi_req_o.ar.lock     = 'd0;
      axi_req_o.ar.cache    = 'd0;
      axi_req_o.ar.prot     = 'd0;
      axi_req_o.ar_valid    = 'd0;

      current_fetch_address_d         = 'd0;
      requested_data_len_d            = 'd0;

      if (data_requester_fsm_current_state == REQ_IDLE_STATE) begin
          current_fetch_address_d = fetch_start_address;
          requested_data_len_d    = 'd0; //reset read counter

      end else if (data_requester_fsm_current_state == SEND_REQ_STATE) begin
          current_fetch_address_d = (axi_rsp_i.ar_ready & axi_req_o.ar_valid) ? current_fetch_address_q + AxiDataByteLen : current_fetch_address_q; // if the request is accepted, add fetch addr
          requested_data_len_d    = (axi_rsp_i.ar_ready) ? requested_data_len_q + AxiDataByteLen : requested_data_len_q;
          
          axi_req_o.ar.addr       = current_fetch_address_q;
          axi_req_o.ar_valid      = (axi_credit_q > 0) ? 1'b1 : 1'b0; // only issue limited amount of outstanfing AXI req to NoC
      end
  end

  /* AXIStream data feeder */

  typedef enum logic [1:0] {
    FEED_IDLE_STATE,
    FEED_DATA_STATE
  } data_feeder_fsm_state_t;

  data_feeder_fsm_state_t data_feeder_fsm_current_state, data_feeder_fsm_next_state;
  logic [AxiAddrWidth:0] feeded_data_len_d, feeded_data_len_q;

  `FF(data_feeder_fsm_current_state, data_feeder_fsm_next_state, FEED_IDLE_STATE, clk_i, rst_ni)
  `FF(feeded_data_len_q, feeded_data_len_d, 'd0, clk_i, rst_ni)

  assign hw2reg_o.meshed_network_status.data_feeder_state.d   = data_feeder_fsm_current_state;
  assign hw2reg_o.meshed_network_status.data_feeder_state.de  = 1'b1;

  /* AXIS feeder FSM ctrl */
  always_comb begin
    case (data_feeder_fsm_current_state)
      FEED_IDLE_STATE:        data_feeder_fsm_next_state = (data_fetcher_trigger) ? FEED_DATA_STATE : FEED_IDLE_STATE;
                                                           
      FEED_DATA_STATE:        data_feeder_fsm_next_state = (feeded_data_len_q + AxiDataByteLen) > fetch_len ?
                                                           ((fetcher_axis_rsp.tready) ? FEED_IDLE_STATE : FEED_DATA_STATE) :
                                                           FEED_DATA_STATE;
                                                           // if feeded all data and the last data is accepeted, finish
    endcase
  end

  /* data feeder logic */
  always_comb begin
      fetcher_flit        = 'd0;
      fetcher_flit_vld    = 1'b0;
      fetcher_flit_rdy    = 1'd0;

      feeded_data_len_d         = 'd0;
      
      axi_req_o.r_ready         = 'd0;

      network_package_done_d    = 1'b0;
      network_package_done_de   = 1'b0;

      if (data_feeder_fsm_current_state == FEED_IDLE_STATE) begin
          feeded_data_len_d = 'd0;
      end else if (data_feeder_fsm_current_state == FEED_DATA_STATE) begin
          fetcher_flit.payload    = axi_rsp_i.r.data;
          fetcher_flit.hdr.src_id = reg2hw_i.meshed_network_id.xy_id.q;
          fetcher_flit.hdr.dst_id = reg2hw_i.meshed_network_ctrl.dst_chip.q;
          fetcher_flit.hdr.last   = (feeded_data_len_q + AxiDataByteLen) > fetch_len ? 1'b1 : 1'b0;
          
          // ring on mesh header ctrl
          fetcher_flit.hdr.ring_on_mesh_mcast     = reg2hw_i.meshed_network_rom_ctrl.rom_en.q;
          fetcher_flit.hdr.up_down_traffic        = reg2hw_i.meshed_network_rom_ctrl.traffic_dir.q;
          fetcher_flit.hdr.ring_on_mesh_dst_mask  = reg2hw_i.meshed_network_rom_ctrl.dst_mask.q;

          fetcher_flit_vld        = axi_rsp_i.r_valid;
          axi_req_o.r_ready       = fetcher_flit_rdy; //accep the AXI returned data when fetcher accept the data
          feeded_data_len_d       = (axi_rsp_i.r_valid & axi_req_o.r_ready) ? (feeded_data_len_q + AxiDataByteLen) : feeded_data_len_q;

          // set done signal at the last data, reset at the 
          network_package_done_d  = (feeded_data_len_q + AxiDataByteLen) > fetch_len ? 1'b1 : 1'b0;
          network_package_done_de = (feeded_data_len_q + AxiDataByteLen) > fetch_len ? 1'b1 : 1'b0;

      end

  end

  // AXI credit logic
  always_comb begin
      axi_credit_d = axi_credit_q;

      if(axi_rsp_i.ar_ready & axi_req_o.ar_valid) begin
          // one axi req sent
          axi_credit_d--;
      end

      if(axi_req_o.r_ready & axi_rsp_i.r_valid) begin
          // one axi rsp comes back
          axi_credit_d++;
      end
  end

  // FIFO for isolating AXI NoC and router
  stream_fifo #(
    .DEPTH  ( 2          ),
    .T      ( flit_t     )
  ) i_fetcher_inject_reg (
    .clk_i      ( clk_i                 ),
    .rst_ni     ( rst_ni                ),
    .flush_i    ( 1'b0                  ),
    .testmode_i ( 1'b0                  ),
    .usage_o    (                       ),
    .valid_i    ( fetcher_flit_vld      ),
    .ready_o    ( fetcher_flit_rdy      ),
    .data_i     ( fetcher_flit          ),
    .valid_o    ( injection_flit_vld    ),
    .ready_i    ( injection_flit_rdy    ),
    .data_o     ( injection_flit        )
  );

  router_data_in[NumRoutes]                 = injection_flit;
  injection_flit_rdy                        = router_ready_in[NumRoutes][injection_vc];
  router_valid_in[NumRoutes][injection_vc]  = injection_flit_vld

  ////////////////////////////
  // Credit & VC controller //
  ////////////////////////////
  
  /* AXIS <=> router_in/out_payload <=> router_in/out_flit <=> router_in/out_data*/
  /*   Out FIFO                 credit ctrl               VC sel                 */

  // generate credit control logic for all external ports
  for (genvar dir_id = 0; dir_id < NumRoutes; dir_id++) begin : gen_credit_ctrl

      payload_t router_out_payload;
      logic     router_out_payload_vld, router_out_payload_rdy;

      payload_t router_in_payload;
      logic     router_in_payload_vld, router_in_payload_rdy;

      flit_t    router_out_flit;
      logic     router_out_flit_vld, router_out_flit_rdy;
      logic     [$clog2(NumVirtChannels)-1:0] router_out_flit_vc;

      flit_t    router_in_flit;
      logic     router_in_flit_vld, router_in_flit_rdy;
      logic     [$clog2(NumVirtChannels)-1:0] router_in_flit_vc;

      credit_t credits_out_q, credits_out_d;
      credit_t credits_to_send_q, credits_to_send_d;
      
      logic credit_to_send_force;

      logic axis_out_reg_valid;


      /* AXIS <=> router_in/out_payload */

      // out going traffic
      // FIFO between router and data layer of serial links
      stream_fifo #(
        .DEPTH  ( 2           ),
        .T      ( payload_t   )
      ) i_dir_axis_out_reg (
        .clk_i      ( clk_i                         ),
        .rst_ni     ( rst_ni                        ),
        .flush_i    ( 1'b0                          ),
        .testmode_i ( 1'b0                          ),
        .usage_o    (                               ),
        .valid_i    ( axis_out_reg_valid            ),
        .ready_o    ( router_out_payload_rdy        ),
        .data_i     ( router_out_payload            ),
        .valid_o    ( axis_out_req_o[dir_id].tvalid ),
        .ready_i    ( axis_out_rsp_i[dir_id].tready ),
        .data_o     ( axis_out_req_o[dir_id].t.data )
      );
      
      // in coming traffic
      assign router_in_payload            = payload_t'(axis_in_req_i[dir_id].t.data;);
      assign router_in_payload_vld        = axis_in_req_i[dir_id].tvalid;
      assign axis_in_rsp_o[dir_id].tready = router_in_payload_rdy

      /* router_in/out_payload <=> router_in/out_flit */

      // out going traffic
      assign router_out_payload.data    = router_out_flit;
      assign router_out_payload.credit  = credits_to_send_q;

      assign router_out_payload_vld     = router_out_flit_vld;
      assign router_out_flit_rdy        = router_out_payload_rdy;

      // in coming traffic
      assign router_in_flit         = router_in_payload.data;
      assign router_in_flit_vld     = router_in_payload_vld; // mask out valid until the router is ready
      // select the correct VC ready signal only when valid flit
      assign router_in_flit_vc      = router_in_payload.virt_channel_id;
      assign router_in_flit_rdy     = (router_in_payload_vld) ? router_ready_in[dir_id][router_in_flit_vc] : 1'b0;

      // credit logic
      always_comb begin
          credits_out_d = credits_out_q;
          credits_to_send_d = credits_to_send_q;
          credit_to_send_force = 1'b0;

          // Send empty packets with credits if there are too many
          // credits to send but no AXI request transaction
          if (credits_to_send_q >= ForceSendThresh) begin
            credit_to_send_force = 1'b1;
          end

          // The order of the two if blocks matter!
          if (axis_out_reg_valid & router_out_payload_rdy) begin
            // a flit send, comsume a credit and reset the pending credit to be sents
            credits_out_d--;
            credits_to_send_d = 0;
          end

          if (router_in_flit_vld & router_in_flit_rdy) begin
            // a flit is accepted by the router, its credit is added to the credit counter
            credits_out_d += router_in_payload.credit;
            credits_to_send_d++;
          end

          //either when there is a valid packet or need to send credit back, send a packet
          axis_out_reg_valid = credit_to_send_force | router_out_payload_vld;  

          if(credits_out_q == 0) begin
              axis_out_reg_valid = 1'b0; //cannot send when no more credit left
          end else if (credits_out_q == 1 && credits_to_send_q == 0) begin
              // dead lock avoidance, reserve the last credit until there is credit to be sent back
              // if consume this credit without any credit gaved, may prevent the other side from sending credit back
              axis_out_reg_valid = 1'b0;
          end
      end

      /* router_in/out_flit <=> router_in/out_data */
      // virtual channel distribution to the router
      always_comb begin
          // in coming traffic
          router_valid_in = 'd0;
          router_valid_in[dir_id] = 'd0

          router_data_in[dir_id] = router_in_flit;

          // mask the vld signal until the router is ready
          router_valid_in[dir_id][router_in_flit_vc] = (router_in_flit_vld) ? router_ready_in[dir_id][router_in_flit_vc] : 1'b0;
          

          // out going traffic
          out_virtual_channe_id = 'd0;
          for (int i = 0; i < NumVirtChannels; i++) begin
              if (router_valid_out[dir_id][i]) begin
                  out_virtual_channe_id = i;
                  break;
              end
          end

          router_out_flit = router_data_out[dir_id];
          router_out_flit_vld = | router_valid_out[dir_id]; // any channel is valid means valid data
          router_ready_out  = {NumVirtChannels{router_out_flit_rdy}};

          router_out_flit_vc = out_virtual_channe_id;
      end
  end

  /////////////////////////
  // Ring on mesh router //
  /////////////////////////

  floo_ring_on_mesh_router #(
    .NumRoutes          ( NumRoutes       ),
    .NumVirtChannels    ( NumVirtChannels ),
    .NumPhysChannels    ( 1               ),
    .flit_t             ( flit_t          ),
    .InFifoDepth        ( InFifoDepth     ),
    .OutFifoDepth       ( OutFifoDepth    ),
    .RouteAlgo          ( RouteAlgo       ),
    .IdWidth            ( IdWidth         )
  ) ring_on_mesh_router (
    .clk_i                    ( clk_i                   ),
    .rst_ni                   ( rst_ni                  ),
    .test_enable_i            ( test_enable_i           ),
    .xy_id_i                  ( xy_id_i                 ),
    .id_route_map_i           ( 'd0                     ),
    .ring_on_mesh_id_i        ( ring_on_mesh_id_i       ),
    .ring_on_mesh_up_port_i   ( ring_on_mesh_up_port_i  ),
    .ring_on_mesh_down_port_i ( ring_on_mesh_down_port_i),
    .valid_i                  ( router_valid_in         ),
    .ready_o                  ( router_ready_in         ),
    .data_i                   ( router_data_in          ),
    .valid_o                  ( router_valid_out        ),
    .ready_i                  ( router_ready_out        ),
    .data_o                   ( router_data_out         )
  );
  

  /////////////////
  // Data writer //
  /////////////////
  
  // router interface
  flit_t ejection_flit;
  logic  [NumVirtChannels-1:0] ejection_rdy, ejection_vld;

  logic [AxiAddrWidth:0] receive_cnt_q, receive_cnt_d;

  // AXI interface
  logic [AxiDataWidth-1:0] axi_write_reg_data;
  logic                    axi_write_reg_vld, axi_write_reg_rdy, axi_write_reg_last;

  // writer ctrl
  logic reset_writer, writer_done, writer_overflow;
  logic [$clog2(NumNodes)-1:0] packet_src_id;

  typedef enum logic [1:0] {
    RECV_IDLE_STATE,
    RECV_RDY_WRITE_STATE,
    RECV_DONE_STATE,
    RECV_LEN_OVERFLOW_STATE
  } data_writer_fsm_state_t;

  data_writer_fsm_state_t data_writer_fsm_current_state, data_writer_fsm_next_state;

  `FF(data_writer_fsm_current_state, data_writer_fsm_next_state, RECV_IDLE_STATE, clk_i, rst_ni)
  `FF(receive_cnt_q, receive_cnt_d, 'd0, clk_i, rst_ni)

  assign ejection_flit                = router_data_out[NumRoutes];
  assign ejection_vld                 = router_valid_out[NumRoutes];
  assign router_ready_out[NumRoutes]  = ejection_rdy

  assign reset_writer = reg2hw_i.meshed_network_ctrl.reset_writer.q;

  // self reseting ready
  assign hw2reg_o.meshed_network_ctrl.recv_rdy.d        = 1'd0;
  assign hw2reg_o.meshed_network_ctrl.recv_rdy.de       = reg2hw_i.meshed_network_ctrl.recv_rdy.q;

  // self reseting reset ctrl
  assign hw2reg_o.meshed_network_ctrl.reset_writer.d    = 1'd0;
  assign hw2reg_o.meshed_network_ctrl.reset_writer.de   = reg2hw_i.meshed_network_ctrl.reset_writer.q;

  assign hw2reg_o.meshed_network_status.recv_src_id.d   = packet_src_id;
  assign hw2reg_o.meshed_network_status.recv_src_id.de  = 1'b1;

  assign hw2reg_o.meshed_network_status.recv_done.d     = writer_done;
  assign hw2reg_o.meshed_network_status.recv_done.de    = 1'b1;

  assign hw2reg_o.meshed_network_status.data_writer_state.d     = data_writer_fsm_current_state;
  assign hw2reg_o.meshed_network_status.data_writer_state.de    = 1'b1;

  assign hw2reg_o.meshed_network_status.recv_overflow.d   = (data_writer_fsm_current_state == RECV_LEN_OVERFLOW_STATE) ? 1'b1: 1'b0;
  assign hw2reg_o.meshed_network_status.recv_overflow.de  = 1'b0;     

  /* state ctrl */
  always_comb begin
      case (data_writer_fsm_current_state) 
          // ready for receving data when the input buffer is setup
          RECV_IDLE_STATE:          data_writer_fsm_next_state  = (reg2hw_i.meshed_network_ctrl.recv_rdy.q) ? RECV_RDY_WRITE_STATE : RECV_IDLE_STATE;
          // have valid input buffer
          RECV_RDY_WRITE_STATE:     data_writer_fsm_next_state  = (reset_writer) ? RECV_IDLE_STATE :
                                                                  (writer_overflow) ? RECV_LEN_OVERFLOW_STATE : 
                                                                  (ejection_flit.hdr.last) ? RECV_DONE_STATE : RECV_RDY_WRITE_STATE;
          // receving done, wait for core to acknowledge
          RECV_DONE_STATE:          data_writer_fsm_next_state  = (reset_writer) ? RECV_IDLE_STATE : RECV_DONE_STATE;
          // overflow error, wait for core to acknowledge
          RECV_LEN_OVERFLOW_STATE:  data_writer_fsm_next_state  = (reset_writer) ? RECV_IDLE_STATE : RECV_LEN_OVERFLOW_STATE;
      endcase
  end

  always_comb begin
      // data writer ctrl
      ejection_rdy = 'd0;
      receive_cnt_d = receive_cnt_q;

      axi_write_reg_data = 'd0;
      axi_write_reg_vld  = 'd0;
      axi_write_reg_last = 'd0;

      // stats
      packet_src_id = 'd0;
      writer_done = 'd0;
      writer_overflow = 'd0;
      
      if (data_writer_fsm_current_state == IDLE_STATE) begin
          //idle state, can't receive anything yet
          ejection_rdy = 'd0;
          receive_cnt_d = 'd0; // reset receving cnt

      end else if (data_writer_fsm_current_state == RECV_RDY_WRITE_STATE) begin
          axi_write_reg_vld = | ejection_vld;
          axi_write_reg_data = ejection_flit.payload;
          axi_write_reg_last = ejection_flit.hdr.last;
          ejection_rdy = {NumVirtChannels-1{axi_write_reg_rdy}};

          // count the number of received flit
          receive_cnt_d = (axi_write_reg_rdy & axi_write_reg_vld) ? receive_cnt_q + AxiDataByteLen : receive_cnt_q;

          packet_src_id = ejection_flit.hdr.src_id;

          // overflow after current write
          writer_overflow = (receive_cnt_d > reg2hw_i.meshed_network_data_recv_data.recv_len.q) ? 1'b1 : 1'b0;

      end else if (data_writer_fsm_current_state == RECV_DONE_STATE) begin
          
          writer_done   = 1'd1;
          packet_src_id = ejection_flit.hdr.src_id;
      end 
  end

  // AXI interface
  logic [AxiDataWidth-1:0] axi_write_out_data;
  logic axi_write_out_vld, axi_write_out_rdy, axi_write_out_last;
  /* register to isolate router ejection and axi */
  stream_fifo #(
    .DEPTH  ( 2          ),
    .T      ( logic [AxiDataWidth:0]  )
  ) i_fetcher_inject_reg (
    .clk_i      ( clk_i                                     ),
    .rst_ni     ( rst_ni                                    ),
    .flush_i    ( 1'b0                                      ),
    .testmode_i ( 1'b0                                      ),
    .usage_o    (                                           ),
    .valid_i    ( axi_write_reg_vld                         ),
    .ready_o    ( axi_write_reg_rdy                         ),
    .data_i     ( {axi_write_reg_data, axi_write_reg_last}  ),
    .valid_o    ( axi_write_out_vld                         ),
    .ready_i    ( axi_write_out_rdy                         ),
    .data_o     ( {axi_write_out_data, axi_write_out_last}  )
  );

  // AXI writier
  typedef enum logic [0:0] {
    WRITE_IDLE_STATE,
    WRITE_ACTIVE_STATE
  } axi_writer_fsm_state_t;

  logic [AxiAddrWidth:0] axi_writer_addr_d, axi_writer_addr_q;

  axi_writer_fsm_state_t axi_writer_fsm_current_state, axi_writer_fsm_next_state;

  `FF(axi_writer_fsm_current_state, axi_writer_fsm_next_state, WRITE_IDLE_STATE, clk_i, rst_ni)
  `FF(axi_writer_addr_q, axi_writer_addr_d, 'd0, clk_i, rst_ni)

  // writer fsm state ctrl
  always_comb begin
      case (axi_writer_fsm_current_state)
          WRITE_IDLE_STATE:       axi_writer_fsm_next_state = (axi_write_reg_vld & axi_write_reg_rdy) ? WRITE_ACTIVE_STATE : WRITE_IDLE_STATE;
          WRITE_ACTIVE_STATE:     axi_writer_fsm_next_state = (axi_write_reg_vld & axi_write_reg_rdy & axi_write_out_last) ? WRITE_IDLE_STATE : WRITE_ACTIVE_STATE;
      endcase
  end

  always_comb begin
      axi_writer_addr_d = 'd0;

      axi_req_o.aw.id     = 'd0;
      axi_req_o.aw.addr   = axi_writer_addr_q;
      axi_req_o.aw.len    = 'd0; // 1 beat per burst
      axi_req_o.aw.size   = 3'b101; // 32 bytes, 256 bits
      axi_req_o.aw.burst  = 2'b01; // increment mode
      axi_req_o.aw.lock   = 'd0;
      axi_req_o.aw.cache  = 'd0;
      axi_req_o.aw.prot   = 'd0;

      axi_req_o.aw_valid  = axi_write_out_vld;

      axi_req_o.w.data    = axi_write_out_data;
      axi_req_o.w.strb    = {AxiDataByteLen{1'b1}};
      axi_req_o.w.last    = 1'b1; // 1 beat per burst

      axi_req_o.w_valid   = 1'b0;

      axi_req_o.b_ready   = 1'b1; // always ready for write rsp

      if (axi_writer_fsm_current_state == WRITE_IDLE_STATE) begin
          axi_writer_addr_d = (axi_write_reg_vld & axi_write_reg_rdy) ? reg2hw_i.meshed_network_data_recv_data.recv_addr.q + AxiDataByteLen : 
                                                                        reg2hw_i.meshed_network_data_recv_data.recv_addr.q;
      end else if (axi_writer_fsm_current_state == WRITE_ACTIVE_STATE) begin
          axi_writer_addr_d = (axi_write_reg_vld & axi_write_reg_rdy) ? axi_writer_addr_q + AxiDataByteLen : 
                                                                        axi_writer_addr_q;
      end
  end


endmodule