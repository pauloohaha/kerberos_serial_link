# Copyright 2023 ETH Zurich and University of Bologna.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0


set group_name "TB"

add wave -noupdate -expand -group $group_name /tb_meshed_serial_link/cfg_req
add wave -noupdate -expand -group $group_name /tb_meshed_serial_link/cfg_rsp