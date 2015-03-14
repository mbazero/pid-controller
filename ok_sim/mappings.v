//------------------------------------------------------------------------
// mappings.v
//
// Description:
//  This file contains ok1 mappings for simulation.
//
//------------------------------------------------------------------------
// Copyright (c) 2005-2010 Opal Kelly Incorporated
// $Rev: 591 $ $Date: 2010-10-05 21:31:38 -0700 (Tue, 05 Oct 2010) $
//------------------------------------------------------------------------
parameter OK_TI_CLK        = 27;
parameter OK_TI_RESET      = 28;
parameter OK_TI_READ       = 9;
parameter OK_TI_WRITE      = 30;
parameter OK_TI_ADDR_START = 0;
parameter OK_TI_ADDR_END   = 7;
parameter OK_TI_DATAIN_START  = 11;
parameter OK_TI_DATAIN_END    = 26;
parameter OK_TI_WIREUPDATE    = 29;
parameter OK_TI_TRIGUPDATE    = 10;
parameter OK_TI_BLOCKSTROBE   = 8;

parameter OK_DATAOUT_START = 1;
parameter OK_DATAOUT_END   = 16;
parameter OK_READY         = 0;

wire        ti_clock       = ok1[OK_TI_CLK];
wire        ti_reset       = ok1[OK_TI_RESET];
wire        ti_read        = ok1[OK_TI_READ];
wire        ti_write       = ok1[OK_TI_WRITE];
wire [7:0]  ti_addr        = ok1[OK_TI_ADDR_END:OK_TI_ADDR_START];
wire [15:0] ti_datain      = ok1[OK_TI_DATAIN_END:OK_TI_DATAIN_START];
wire        ti_wireupdate  = ok1[OK_TI_WIREUPDATE];
wire        ti_trigupdate  = ok1[OK_TI_TRIGUPDATE];
wire        ti_blockstrobe = ok1[OK_TI_BLOCKSTROBE];