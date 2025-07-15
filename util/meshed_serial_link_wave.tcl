# Copyright 2023 ETH Zurich and University of Bologna.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0


set group_name "TB"

add wave -noupdate -expand -group $group_name /tb_meshed_serial_link/cfg_req
add wave -noupdate -expand -group $group_name /tb_meshed_serial_link/cfg_rsp

set group_name "Router"

add wave -noupdate -expand -group $group_name -ports /tb_meshed_serial_link/generate_nodes[0]/i_meshed_serial_link/i_meshed_serial_link_network/ring_on_mesh_router/*

set group_name "Network"
add wave -noupdate -expand -group $group_name  /tb_meshed_serial_link/generate_nodes[0]/i_meshed_serial_link/i_meshed_serial_link_network/*
