/*
 * Copyright (c) 2016, ASUSTek COMPUTER INC.
 * All rights reserved.
 *
 */

UINT32 mBarCode[]=
{0x034,
0x121,
0x061,
0x160,
0x031,
0x130,
0x070,
0x025,
0x124,
0x064,
0x109,
0x049,
0x148,
0x019,
0x118,
0x058,
0x00d,
0x10c,
0x04c,
0x01c,
0x103,
0x043,
0x142,
0x013,
0x112,
0x052,
0x007,
0x106,
0x046,
0x016,
0x181,
0x0c1,
0x1c0,
0x091,
0x190,
0x0d0};

#define BARCODE_BITMAP_WIDTH_SCALE      4
#define BARCODE_BITMAP_HEIGHT_SCALE     4

#define BARCODE_BAR_PIXEL               1 * BARCODE_BITMAP_WIDTH_SCALE
#define BARCODE_WIDTH_PIXEL             13 * BARCODE_BITMAP_WIDTH_SCALE
#define BARCODE_HEIGHT_PIXEL            50 * BARCODE_BITMAP_HEIGHT_SCALE
#define BARCODE_EDGE_WIDTH_PIXEL        50
#define PIXEL_SIZE                      4

#define BARCODE_CODE_SIZE               9
#define BARCODE_START_END               0x094

#define BARCODE_BLACK                   0
#define BARCODE_WHITE                   1
