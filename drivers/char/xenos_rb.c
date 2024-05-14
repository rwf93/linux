/*
 *  Xenos RingBuffer character driver.
 *
 *  Copyright (C) 2018 Justin Moore
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/circ_buf.h>
#include <linux/log2.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/types.h>

#include <asm/cacheflush.h>

#define DRV_NAME	"xenos_rb"
#define DRV_VERSION	"0.1"

#define IOC_MAGIC 0x58524230
#define IOCTL_RESET _IO(IOC_MAGIC, 0)

static int xenosrb_size = 0x8000;
module_param(xenosrb_size, int, 0444);

static uint32_t xenos_pfp_ucode[] = {
	0xC60400, 0x7E424B, 0xA00000, 0x7E828B, 0x800001, 0xC60400, 0xCC4003,
	0x800000, 0xD60003, 0xC60800, 0xC80C1D, 0x98C007, 0xC61000, 0x978003,
	0xCC4003, 0xD60004, 0x800000, 0xCD0003, 0x9783EF, 0xC60400, 0x800000,
	0xC60400, 0xC60800, 0x348C08, 0x98C006, 0xC80C1E, 0x98C000, 0xC80C1E,
	0x80001F, 0xCC8007, 0xCC8008, 0xCC4003, 0x800000, 0xCC8003, 0xC60400,
	0x1AAC07, 0xCA8821, 0x96C015, 0xC8102C, 0x98800A, 0x329418, 0x9A4004,
	0xCC6810, 0x42401,  0xD00143, 0xD00162, 0xCD0002, 0x7D514C, 0xCD4003,
	0x9B8007, 0x6A801,  0x964003, 0xC28000, 0xCF4003, 0x800001, 0xC60400,
	0x800023, 0xC60400, 0x964003, 0x7E424B, 0xD00283, 0xC8102B, 0xC60800,
	0x99000E, 0xC80C29, 0x98C00A, 0x345002, 0xCD0002, 0xCC8002, 0xD001E3,
	0xD00183, 0xCC8003, 0xCC4018, 0x80004D, 0xCC8019, 0xD00203, 0xD00183,
	0x9783B4, 0xC60400, 0xC8102B, 0xC60800, 0x9903AF, 0xC80C2A, 0x98C00A,
	0x345002, 0xCD0002, 0xCC8002, 0xD001E3, 0xD001A3, 0xCC8003, 0xCC401A,
	0x800000, 0xCC801B, 0xD00203, 0xD001A3, 0x800001, 0xC60400, 0xC60800,
	0xC60C00, 0xC8102D, 0x349402, 0x99000B, 0xC8182E, 0xCD4002, 0xCD8002,
	0xD001E3, 0xD001C3, 0xCCC003, 0xCC801C, 0xCD801D, 0x800001, 0xC60400,
	0xD00203, 0x800000, 0xD001C3, 0xC8081F, 0xC60C00, 0xC80C20, 0x988000,
	0xC8081F, 0xCC4003, 0xCCC003, 0xD60003, 0x800000, 0xCCC022, 0xC81C2F,
	0xC60400, 0xC60800, 0xC60C00, 0xC81030, 0x99C000, 0xC81C2F, 0xCC8021,
	0xCC4020, 0x990011, 0xC107FF, 0xD00223, 0xD00243, 0x345402, 0x7CB18B,
	0x7D95CC, 0xCDC002, 0xCCC002, 0xD00263, 0x978005, 0xCCC003, 0xC60800,
	0x80008B, 0xC60C00, 0x800000, 0xD00283, 0x97836A, 0xC60400, 0xD6001F,
	0x800001, 0xC60400, 0xC60800, 0xC60C00, 0xC61000, 0x348802, 0xCC8002,
	0xCC4003, 0xCCC003, 0xCD0002, 0x800000, 0xCD0003, 0xD2000D, 0xCC000D,
	0x800000, 0xCC000D, 0xC60800, 0xC60C00, 0xCA1433, 0xD022A0, 0xCCE000,
	0x994351, 0xCCE005, 0x800000, 0x62001,  0xC60800, 0xC60C00, 0xD022A0,
	0xCCE000, 0xD022AE, 0xCCE029, 0xCCE005, 0x800000, 0x62001,  0x964000,
	0xC82435, 0xCA0838, 0x366401, 0x964340, 0xCA0C3A, 0xCCA000, 0xCCE000,
	0xCCE029, 0xCCE005, 0x800000, 0x62001,  0xC60800, 0xC60C00, 0xD202C3,
	0xCC8003, 0xCCC003, 0xCCE027, 0x800000, 0x62001,  0xCA0831, 0x9883FF,
	0xCA0831, 0xD6001F, 0x800001, 0xC60400, 0xD02360, 0xD02380, 0xD02385,
	0x800000, 0x62001,  0xA2001,  0xCA0436, 0x9843DF, 0xC82435, 0x800001,
	0xC60400, 0xD20009, 0xD2000A, 0xCC001F, 0x800000, 0xCC001F, 0xD2000B,
	0xD2000C, 0xCC001F, 0x800000, 0xCC001F, 0xCC0023, 0xCC4003, 0x800000,
	0xD60003, 0xD00303, 0xCC0024, 0xCC4003, 0x800000, 0xD60003, 0xD00323,
	0xCC0025, 0xCC4003, 0x800000, 0xD60003, 0xD00343, 0xCC0026, 0xCC4003,
	0x800000, 0xD60003, 0x800000, 0xD6001F, 0x100EF,  0x200F4,  0x300F9,
	0x50004,  0x600D6,  0x1000FE, 0x1700DB, 0x220009, 0x230016, 0x250022,
	0x270061, 0x2D0073, 0x2E007D, 0x2F009C, 0x3700C8, 0x3800B3, 0x3B00A6,
	0x3F00AA, 0x4800EB, 0x5000E1, 0x5100E6, 0x5500F0, 0x5600F5, 0x5700FA,
	0x5D00D0, 6,	6,	6,	6,	6,	6,
	6,
};

static uint32_t xenos_me_ucode[] = {
	0,	  0xC0200400, 0,     0,	  0xA0000A,   0,
	0x1F3,      0x204411,   0,     0x1000000,  0x204811,   0,
	0,	  0x400000,   4,     0xFFFF,     0x284621,   0,
	0,	  0xD9004800, 0,     0,	  0x400000,   0,
	0,	  0x34E00000, 0,     0,	  0x600000,   0x24A,
	0xFFFF,     0xC0280A20, 0,     0,	  0x294582,   0,
	0,	  0xD9004800, 0,     0,	  0x400000,   0,
	0,	  0x600000,   0x24A, 0xFFFF,     0xC0284620, 0,
	0,	  0xD9004800, 0,     0,	  0x400000,   0,
	0,	  0x600000,   0x267, 0x21FC,     0x29462C,   0,
	0,	  0xC0204800, 0,     0,	  0x400000,   0,
	0,	  0x600000,   0x267, 0x21FC,     0x29462C,   0,
	0,	  0xC0204800, 0,     0x3FFF,     0x2F022F,   0,
	0,	  0xCE00000,  0,     0xA1FD,     0x29462C,   0,
	0,	  0xD9004800, 0,     0,	  0x400000,   0,
	0x394,      0x204411,   0,     1,	  0xC0404811, 0,
	0,	  0x600000,   0x267, 0x21F9,     0x29462C,   0,
	8,	  0xC0210A20, 0,     0,	  0x14E00000, 0x25,
	7,	  0x404811,   0,     8,	  0x404811,   0,
	0,	  0x600000,   0x267, 0x21FC,     0x29462C,   0,
	0,	  0xC0204800, 0,     0xA1FD,     0x29462C,   0,
	0,	  0xC0200800, 0,     0,	  0x2F0222,   0,
	0,	  0xCE00000,  0,     0,	  0x40204800, 0,
	1,	  0x40304A20, 0,     2,	  0xC0304A20, 0,
	1,	  0x530A22,   0x2B,  0x80000000, 0xC0204411, 0,
	1,	  0x604811,   0x281, 0,	  0x400000,   0,
	0,	  0xC0200000, 0,     0x12B9B0A1, 0xC02F0220, 0,
	0,	  0xCC00000,  0x3A,  0x1033C4D6, 0xC02F0220, 0,
	0,	  0xCC00000,  0x3A,  0,	  0x400000,   0,
	0x1F3,      0x204411,   0,     0x8000000,  0x204811,   0,
	0,	  0x400000,   0x3C,  0x80000000, 0xC0204411, 0,
	0,	  0x604811,   0x281, 0,	  0x400000,   0,
	0x1F,       0x40280A20, 0,     0x1B,       0x2F0222,   0,
	0,	  0xCE00000,  0x57,  2,	  0x2F0222,   0,
	0,	  0xCE00000,  0x5E,  3,	  0x2F0222,   0,
	0,	  0xCE00000,  0x65,  4,	  0x2F0222,   0,
	0,	  0xCE00000,  0x6C,  0x14,       0x2F0222,   0,
	0,	  0xCE00000,  0x6C,  0x1A,       0x2F0222,   0,
	0,	  0xCE00000,  0x74,  0x15,       0x2F0222,   0,
	0,	  0xCE00000,  0x79,  0x21F9,     0x29462C,   0,
	0,	  0xC0404802, 0,     0x1F,       0x40280A20, 0,
	0x1B,       0x2F0222,   0,     0,	  0xCE00000,  0x57,
	2,	  0x2F0222,   0,     0,	  0xCE00000,  0x5E,
	0,	  0x400000,   0x65,  0x1F,       0xC0210E20, 0,
	0x612,      0x204411,   0,     0,	  0x204803,   0,
	0,	  0xC0204800, 0,     0,	  0xC0204800, 0,
	0x21F9,     0x29462C,   0,     0,	  0x404802,   0,
	0x1E,       0xC0210E20, 0,     0x600,      0x204411,   0,
	0,	  0x204803,   0,     0,	  0xC0204800, 0,
	0,	  0xC0204800, 0,     0x21F9,     0x29462C,   0,
	0,	  0x404802,   0,     0x1E,       0xC0210E20, 0,
	0x605,      0x204411,   0,     0,	  0x204803,   0,
	0,	  0xC0204800, 0,     0,	  0xC0204800, 0,
	0x21F9,     0x29462C,   0,     0,	  0x404802,   0,
	0x1F,       0x40280A20, 0,     0x1F,       0xC0210E20, 0,
	0x60A,      0x204411,   0,     0,	  0x204803,   0,
	0,	  0xC0204800, 0,     0,	  0xC0204800, 0,
	0x21F9,     0x29462C,   0,     0,	  0x404802,   0,
	0x1F,       0xC0280A20, 0,     0x611,      0x204411,   0,
	0,	  0xC0204800, 0,     0x21F9,     0x29462C,   0,
	0,	  0x404802,   0,     0x1F,       0xC0280A20, 0,
	0,	  0x600000,   0x267, 0x21F9,     0x29462C,   0,
	0,	  0x404802,   0,     0x81000000, 0x204411,   0,
	1,	  0x204811,   0,     0x1FFF,     0x40280A20, 0,
	0x80000000, 0x40280E20, 0,     0x40000000, 0xC0281220, 0,
	0x40000,    0x294622,   0,     0,	  0x600000,   0x282,
	0,	  0x201410,   0,     0,	  0x2F0223,   0,
	0,	  0xCC00000,  0x88,  0,	  0xC0401800, 0x8C,
	0x1FFF,     0xC0281A20, 0,     0x40000,    0x294626,   0,
	0,	  0x600000,   0x282, 0,	  0x201810,   0,
	0,	  0x2F0224,   0,     0,	  0xCC00000,  0x8F,
	0,	  0xC0401C00, 0x93,  0x1FFF,     0xC0281E20, 0,
	0x40000,    0x294627,   0,     0,	  0x600000,   0x282,
	0,	  0x201C10,   0,     0,	  0x204402,   0,
	0,	  0x2820C5,   0,     0,	  0x4948E8,   0,
	0,	  0x600000,   0x24A, 0x10,       0x40210A20, 0,
	0xFF,       0x280A22,   0,     0x7FF,      0x40280E20, 0,
	2,	  0x221E23,   0,     5,	  0xC0211220, 0,
	0x80000,    0x281224,   0,     0x13,       0x210224,   0,
	0,	  0x14C00000, 0xA1,  0xA1000000, 0x204411,   0,
	0,	  0x204811,   0,     0,	  0x2F0222,   0,
	0,	  0xCC00000,  0xA5,  8,	  0x20162D,   0,
	0x4000,     0x500E23,   0xB4,  1,	  0x2F0222,   0,
	0,	  0xCC00000,  0xA9,  9,	  0x20162D,   0,
	0x4800,     0x500E23,   0xB4,  2,	  0x2F0222,   0,
	0,	  0xCC00000,  0xAD,  0x37,       0x20162D,   0,
	0x4900,     0x500E23,   0xB4,  3,	  0x2F0222,   0,
	0,	  0xCC00000,  0xB1,  0x36,       0x20162D,   0,
	0x4908,     0x500E23,   0xB4,  0x29,       0x20162D,   0,
	0x2000,     0x300E23,   0,     0,	  0x290D83,   0,
	0x94000000, 0x204411,   0,     0,	  0x2948E5,   0,
	0,	  0x294483,   0,     0,	  0x40201800, 0,
	0,	  0xD9004800, 0,     0x13,       0x210224,   0,
	0,	  0x14C00000, 0,     0x94000000, 0x204411,   0,
	0,	  0x2948E5,   0,     0x93000000, 0x204411,   0,
	0,	  0x404806,   0,     0,	  0x600000,   0x24A,
	0,	  0xC0200800, 0,     0,	  0xC0201400, 0,
	0x1F,       0x211A25,   0,     0,	  0x14E00000, 0,
	0x7FF,      0x280E25,   0,     0x10,       0x211225,   0,
	0x83000000, 0x204411,   0,     0,	  0x2F0224,   0,
	0,	  0xAE00000,  0xCB,  8,	  0x203622,   0,
	0x4000,     0x504A23,   0xDA,  1,	  0x2F0224,   0,
	0,	  0xAE00000,  0xCF,  9,	  0x203622,   0,
	0x4800,     0x504A23,   0xDA,  2,	  0x2F0224,   0,
	0,	  0xAE00000,  0xD3,  0x37,       0x203622,   0,
	0x4900,     0x504A23,   0xDA,  3,	  0x2F0224,   0,
	0,	  0xAE00000,  0xD7,  0x36,       0x203622,   0,
	0x4908,     0x504A23,   0xDA,  0x29,       0x203622,   0,
	0,	  0x290D83,   0,     0x2000,     0x304A23,   0,
	0x84000000, 0x204411,   0,     0,	  0xC0204800, 0,
	0,	  0x21000000, 0,     0,	  0x400000,   0xC1,
	0,	  0x600000,   0x24A, 0x83000000, 0x204411,   0,
	0x4000,     0xC0304A20, 0,     0x84000000, 0x204411,   0,
	0,	  0xC0204800, 0,     0,	  0x21000000, 0,
	0,	  0x400000,   0,     0x81000000, 0x204411,   0,
	1,	  0x204811,   0,     0x40578,    0x204411,   0,
	0,	  0x600000,   0x282, 0,	  0xC0400000, 0,
	0,	  0xC0200C00, 0,     0,	  0xC0201000, 0,
	0,	  0xC0201400, 0,     0,	  0xC0201800, 0,
	0x7F00,     0x280A21,   0,     0x4500,     0x2F0222,   0,
	0,	  0xCE00000,  0xF2,  0,	  0xC0201C00, 0,
	0,	  0x17000000, 0,     0x10,       0x280A23,   0,
	0x10,       0x2F0222,   0,     0,	  0xCE00000,  0xFB,
	0x81000000, 0x204411,   0,     1,	  0x204811,   0,
	0x40000,    0x294624,   0,     0,	  0x600000,   0x282,
	0,	  0x400000,   0x103, 0x81000000, 0x204411,   0,
	0,	  0x204811,   0,     0x1EA,      0x204411,   0,
	0,	  0x204804,   0,     0,	  0x1AC00000, 0xFF,
	0x9E000000, 0x204411,   0,     0xDEADBEEF, 0x204811,   0,
	0,	  0x1AE00000, 0x102, 0,	  0x2820D0,   0,
	7,	  0x280A23,   0,     1,	  0x2F0222,   0,
	0,	  0xAE00000,  0x10A, 0,	  0x2F00A8,   0,
	0,	  0x4E00000,  0x123, 0,	  0x400000,   0x12A,
	2,	  0x2F0222,   0,     0,	  0xAE00000,  0x10F,
	0,	  0x2F00A8,   0,     0,	  0x2E00000,  0x123,
	0,	  0x400000,   0x12A, 3,	  0x2F0222,   0,
	0,	  0xAE00000,  0x114, 0,	  0x2F00A8,   0,
	0,	  0xCE00000,  0x123, 0,	  0x400000,   0x12A,
	4,	  0x2F0222,   0,     0,	  0xAE00000,  0x119,
	0,	  0x2F00A8,   0,     0,	  0xAE00000,  0x123,
	0,	  0x400000,   0x12A, 5,	  0x2F0222,   0,
	0,	  0xAE00000,  0x11E, 0,	  0x2F00A8,   0,
	0,	  0x6E00000,  0x123, 0,	  0x400000,   0x12A,
	6,	  0x2F0222,   0,     0,	  0xAE00000,  0x123,
	0,	  0x2F00A8,   0,     0,	  0x8E00000,  0x123,
	0,	  0x400000,   0x12A, 0x7F00,     0x280A21,   0,
	0x4500,     0x2F0222,   0,     0,	  0xAE00000,  0,
	8,	  0x210A23,   0,     0,	  0x14E00000, 0x14A,
	0,	  0xC0204400, 0,     0,	  0xC0404800, 0,
	0x7F00,     0x280A21,   0,     0x4500,     0x2F0222,   0,
	0,	  0xAE00000,  0x12F, 0,	  0xC0200000, 0,
	0,	  0xC0400000, 0,     0,	  0x404C07,   0xF2,
	0,	  0xC0201000, 0,     0,	  0xC0201400, 0,
	0,	  0xC0201800, 0,     0,	  0xC0201C00, 0,
	0,	  0x17000000, 0,     0x81000000, 0x204411,   0,
	1,	  0x204811,   0,     0x40000,    0x294624,   0,
	0,	  0x600000,   0x282, 0,	  0x2820D0,   0,
	0,	  0x2F00A8,   0,     0,	  0xCE00000,  0,
	0,	  0x404C07,   0x134, 0,	  0xC0201000, 0,
	0,	  0xC0201400, 0,     0,	  0xC0201800, 0,
	0,	  0xC0201C00, 0,     0,	  0x17000000, 0,
	0x81000000, 0x204411,   0,     1,	  0x204811,   0,
	0x40000,    0x294624,   0,     0,	  0x600000,   0x282,
	0,	  0x2820D0,   0,     0,	  0x2F00A8,   0,
	0,	  0x6E00000,  0,     0,	  0x404C07,   0x141,
	0x60D,      0x204411,   0,     0,	  0xC0204800, 0,
	0,	  0xC0404800, 0,     0x81000000, 0x204411,   0,
	9,	  0x204811,   0,     0x60D,      0x204411,   0,
	0,	  0xC0204800, 0,     0,	  0x404810,   0,
	0x1FFF,     0xC0280A20, 0,     0x20000,    0x294622,   0,
	0x18,       0xC0424A20, 0,     0x81000000, 0x204411,   0,
	1,	  0x204811,   0,     0x40000,    0xC0294620, 0,
	0,	  0x600000,   0x282, 0x60D,      0x204411,   0,
	0,	  0xC0204800, 0,     0,	  0x404810,   0,
	0x1F3,      0x204411,   0,     0xE0000000, 0xC0484A20, 0,
	0,	  0xD9000000, 0,     0,	  0x400000,   0,
	0x45D,      0x204411,   0,     0x3F,       0xC0484A20, 0,
	0,	  0x600000,   0x24A, 0x81000000, 0x204411,   0,
	2,	  0x204811,   0,     0xFF,       0x280E30,   0,
	0,	  0x2F0223,   0,     0,	  0xCC00000,  0x165,
	0,	  0x200411,   0,     0x1D,       0x203621,   0,
	0x1E,       0x203621,   0,     0,	  0xC0200800, 0,
	9,	  0x210222,   0,     0,	  0x14C00000, 0x171,
	0,	  0x600000,   0x275, 0,	  0x200C11,   0,
	0x38,       0x203623,   0,     0,	  0x210A22,   0,
	0,	  0x14C00000, 0x17A, 0,	  0xC02F0220, 0,
	0,	  0x400000,   0x177, 0,	  0x600000,   0x1D8,
	0,	  0x400000,   0x178, 0,	  0x600000,   0x1DC,
	0xA0000000, 0x204411,   0,     0,	  0x204811,   0,
	1,	  0x210A22,   0,     0,	  0x14C00000, 0x17F,
	0xF1FFFFFF, 0x283A2E,   0,     0x1A,       0xC0220E20, 0,
	0,	  0x29386E,   0,     1,	  0x210A22,   0,
	0,	  0x14C00000, 0x189, 0xE,	0xC0203620, 0,
	0xF,	0xC0203620, 0,     0x10,       0xC0203620, 0,
	0x11,       0xC0203620, 0,     0x12,       0xC0203620, 0,
	0x13,       0xC0203620, 0,     0x14,       0xC0203620, 0,
	0x15,       0xC0203620, 0,     1,	  0x210A22,   0,
	0,	  0x14C00000, 0x1AC, 0,	  0xC0200C00, 0,
	0x8C000000, 0x204411,   0,     0,	  0x204803,   0,
	0xFFF,      0x281223,   0,     0x19,       0x203624,   0,
	3,	  0x381224,   0,     0x5000,     0x301224,   0,
	0x18,       0x203624,   0,     0x87000000, 0x204411,   0,
	0,	  0x204804,   0,     1,	  0x331224,   0,
	0x86000000, 0x204411,   0,     0,	  0x204804,   0,
	0x88000000, 0x204411,   0,     0x7FFF,     0x204811,   0,
	0x10,       0x211623,   0,     0xFFF,      0x281A23,   0,
	0,	  0x331CA6,   0,     0x8F000000, 0x204411,   0,
	3,	  0x384A27,   0,     0x10,       0x211223,   0,
	0x17,       0x203624,   0,     0x8B000000, 0x204411,   0,
	0,	  0x204804,   0,     3,	  0x381224,   0,
	0x5000,     0x301224,   0,     0x16,       0x203624,   0,
	0x85000000, 0x204411,   0,     0,	  0x204804,   0,
	0x1000,     0x331CD1,   0,     0x90000000, 0x204411,   0,
	3,	  0x384A27,   0,     0x300000,   0x293A2E,   0,
	1,	  0x210A22,   0,     0,	  0x14C00000, 0x1B9,
	0xA3000000, 0x204411,   0,     0,	  0x40204800, 0,
	0xA,	0xC0220E20, 0,     0x21,       0x203623,   0,
	0,	  0x600000,   0x1DC, 0xFFFFE000, 0x200411,   0,
	0x2E,       0x203621,   0,     0x2F,       0x203621,   0,
	0x1FFF,     0x200411,   0,     0x30,       0x203621,   0,
	0x31,       0x203621,   0,     1,	  0x210A22,   0,
	0,	  0x14C00000, 0x1BC, 0,	  0xC0200000, 0,
	1,	  0x210A22,   0,     0,	  0x14C00000, 0x1C2,
	0x9C000000, 0x204411,   0,     0x1F,       0x40214A20, 0,
	0x96000000, 0x204411,   0,     0,	  0xC0204800, 0,
	1,	  0x210A22,   0,     0,	  0x14C00000, 0x1CB,
	0x3FFFFFFF, 0x283A2E,   0,     0xC0000000, 0x40280E20, 0,
	0,	  0x29386E,   0,     0x18000000, 0x40280E20, 0,
	0x38,       0x203623,   0,     0xA4000000, 0x204411,   0,
	0,	  0xC0204800, 0,     1,	  0x210A22,   0,
	0,	  0x14C00000, 0x1D7, 0,	  0xC0200C00, 0,
	0x2B,       0x203623,   0,     0x2D,       0x203623,   0,
	2,	  0x40221220, 0,     0,	  0x301083,   0,
	0x2C,       0x203624,   0,     3,	  0xC0210E20, 0,
	0x10000000, 0x280E23,   0,     0xEFFFFFFF, 0x283A2E,   0,
	0,	  0x29386E,   0,     0,	  0x400000,   0,
	0x25F4,     0x204411,   0,     0xA,	0x214A2C,   0,
	0,	  0x600000,   0x273, 0,	  0x800000,   0,
	0x21F4,     0x204411,   0,     0xA,	0x214A2C,   0,
	0,	  0x600000,   0x275, 0,	  0x800000,   0,
	0,	  0x600000,   0x24A, 0,	  0xC0200800, 0,
	0x1F,       0x210E22,   0,     0,	  0x14E00000, 0,
	0x3FF,      0x280E22,   0,     0x18,       0x211222,   0,
	0xE,	0x301224,   0,     0,	  0x20108D,   0,
	0x2000,     0x291224,   0,     0x83000000, 0x204411,   0,
	0,	  0x294984,   0,     0x84000000, 0x204411,   0,
	0,	  0x204803,   0,     0,	  0x21000000, 0,
	0,	  0x400000,   0x1E1, 0x82000000, 0x204411,   0,
	1,	  0x204811,   0,     0,	  0xC0200800, 0,
	0x3FFF,     0x40280E20, 0,     0x10,       0xC0211220, 0,
	0,	  0x2F0222,   0,     0,	  0xAE00000,  0x1FE,
	0,	  0x2AE00000, 0x208, 0x20000080, 0x281E2E,   0,
	0x80,       0x2F0227,   0,     0,	  0xCE00000,  0x1FB,
	0,	  0x401C0C,   0x1FC, 0x20,       0x201E2D,   0,
	0x21F9,     0x294627,   0,     0,	  0x404811,   0x208,
	1,	  0x2F0222,   0,     0,	  0xAE00000,  0x23D,
	0,	  0x28E00000, 0x208, 0x800080,   0x281E2E,   0,
	0x80,       0x2F0227,   0,     0,	  0xCE00000,  0x205,
	0,	  0x401C0C,   0x206, 0x20,       0x201E2D,   0,
	0x21F9,     0x294627,   0,     1,	  0x204811,   0,
	0x81000000, 0x204411,   0,     0,	  0x2F0222,   0,
	0,	  0xAE00000,  0x20F, 3,	  0x204811,   0,
	0x16,       0x20162D,   0,     0x17,       0x201A2D,   0,
	0xFFDFFFFF, 0x483A2E,   0x213, 4,	  0x204811,   0,
	0x18,       0x20162D,   0,     0x19,       0x201A2D,   0,
	0xFFEFFFFF, 0x283A2E,   0,     0,	  0x201C10,   0,
	0,	  0x2F0067,   0,     0,	  0x6C00000,  0x208,
	0x81000000, 0x204411,   0,     6,	  0x204811,   0,
	0x83000000, 0x204411,   0,     0,	  0x204805,   0,
	0x89000000, 0x204411,   0,     0,	  0x204806,   0,
	0x84000000, 0x204411,   0,     0,	  0x204803,   0,
	0,	  0x21000000, 0,     0,	  0x601010,   0x24A,
	0xC,	0x221E24,   0,     0,	  0x2F0222,   0,
	0,	  0xAE00000,  0x230, 0x20000000, 0x293A2E,   0,
	0x21F7,     0x29462C,   0,     0,	  0x2948C7,   0,
	0x81000000, 0x204411,   0,     5,	  0x204811,   0,
	0x16,       0x203630,   0,     7,	  0x204811,   0,
	0x17,       0x203630,   0,     0x91000000, 0x204411,   0,
	0,	  0x204803,   0,     0,	  0x23000000, 0,
	0x8D000000, 0x204411,   0,     0,	  0x404803,   0x243,
	0x800000,   0x293A2E,   0,     0x21F6,     0x29462C,   0,
	0,	  0x2948C7,   0,     0x81000000, 0x204411,   0,
	5,	  0x204811,   0,     0x18,       0x203630,   0,
	7,	  0x204811,   0,     0x19,       0x203630,   0,
	0x92000000, 0x204411,   0,     0,	  0x204803,   0,
	0,	  0x25000000, 0,     0x8E000000, 0x204411,   0,
	0,	  0x404803,   0x243, 0x83000000, 0x204411,   0,
	3,	  0x381224,   0,     0x5000,     0x304A24,   0,
	0x84000000, 0x204411,   0,     0,	  0x204803,   0,
	0,	  0x21000000, 0,     0x82000000, 0x204411,   0,
	0,	  0x404811,   0,     0x1F3,      0x204411,   0,
	0x4000000,  0x204811,   0,     0,	  0x400000,   0x247,
	0,	  0xC0600000, 0x24A, 0,	  0x400000,   0,
	0,	  0xEE00000,  0x281, 0x21F9,     0x29462C,   0,
	5,	  0x204811,   0,     0,	  0x202C0C,   0,
	0x21,       0x20262D,   0,     0,	  0x2F012C,   0,
	0,	  0xCC00000,  0x252, 0,	  0x403011,   0x253,
	0x400,      0x30322C,   0,     0x81000000, 0x204411,   0,
	2,	  0x204811,   0,     0xA,	0x21262C,   0,
	0,	  0x210130,   0,     0,	  0x14C00000, 0x25B,
	0xA5000000, 0x204411,   0,     1,	  0x204811,   0,
	0,	  0x400000,   0x256, 0xA5000000, 0x204411,   0,
	0,	  0x204811,   0,     0,	  0x2F016C,   0,
	0,	  0xCE00000,  0x263, 0x21F4,     0x29462C,   0,
	0xA,	0x214A2B,   0,     0x4940,     0x204411,   0,
	0xDEADBEEF, 0x204811,   0,     0,	  0x600000,   0x26E,
	0xDFFFFFFF, 0x283A2E,   0,     0xFF7FFFFF, 0x283A2E,   0,
	0x20,       0x80362B,   0,     0x97000000, 0x204411,   0,
	0,	  0x20480C,   0,     0xA2000000, 0x204411,   0,
	0,	  0x204811,   0,     0x81000000, 0x204411,   0,
	2,	  0x204811,   0,     0,	  0x810130,   0,
	0xA2000000, 0x204411,   0,     1,	  0x204811,   0,
	0x81000000, 0x204411,   0,     2,	  0x204811,   0,
	0,	  0x810130,   0,     0x400,      0x203011,   0,
	0x20,       0x80362C,   0,     0,	  0x203011,   0,
	0x20,       0x80362C,   0,     0x1F,       0x201E2D,   0,
	4,	  0x291E27,   0,     0x1F,       0x803627,   0,
	0x21F9,     0x29462C,   0,     6,	  0x204811,   0,
	0x5C8,      0x204411,   0,     0x10000,    0x204811,   0,
	0xE00,      0x204411,   0,     1,	  0x804811,   0,
	0,	  0xC0400000, 0,     0,	  0x800000,   0,
	0,	  0x1AC00000, 0x282, 0x9F000000, 0x204411,   0,
	0xDEADBEEF, 0x204811,   0,     0,	  0x1AE00000, 0x285,
	0,	  0x800000,   0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0,	  0,		0,     0,	  0,	  0,
	0x2015E,    0x20002,    0,     0x20002,    0x3D0031,   0,
	0x20002,    0x20002,    0,     0x20002,    0x1E00002,  0,
	0x7D01F1,   0x200012,   0,     0x20002,    0x2001E,    0,
	0x20002,    0x1EF0002,  0,     0x960002,   0xDE0002,   0,
	0x20002,    0x20002,    0,     0x20002,    0x20016,    0,
	0x20002,    0x20026,    0,     0x14A00EA,  0x20155,    0,
	0x2015C,    0x15E0002,  0,     0xEA0002,   0x15E0040,  0,
	0xBF0162,   0x20002,    0,     0x1520002,  0x14D0002,  0,
	0x20002,    0x13D0130,  0,     0x90160,    0xE000E,    0,
	0x6C0051,   0x790074,   0,     0x200E5,    0x20248,    0,
	0x20002,    0x20002,    0,     0x20002,    0x20002,    0,
	0x20002,    0x20002,    0,     0x20002,    0x20002,    0,
	0x20002,    0x20002,    0,     0x20002,    0x20002,    0,
	0x20002,    0x20002,    0,     0x50280,    0x20008,    0,
};

static struct {
    void __iomem *graphics_base;

    struct circ_buf primary_rb;
    int rb_size;

    spinlock_t gfx_lock;
    spinlock_t write_lock;
} xenosrb_info;

/**
 *	xenosrb_reset_ring - Resets ring read/write pointers to 0
 *  Preferably you would call this with interrupts disabled.
 */
static void xenosrb_reset_ring(void)
{
	int rb_cntl;

	rb_cntl = in_be32(xenosrb_info.graphics_base + 0x0704);

	/* CP_RB_CNTL.RB_RPTR_WR_ENA = 1 */
	out_be32(xenosrb_info.graphics_base + 0x0704, rb_cntl | 0x80000000);

	out_be32(xenosrb_info.graphics_base + 0x071C, 0x00000000); /* CP_RB_RPTR_WR */
	out_be32(xenosrb_info.graphics_base + 0x0714, 0x00000000); /* CP_RB_WPTR */
	udelay(1000);

	/* CP_RB_CNTL.RB_RPTR_WR_ENA = 0 */
	out_be32(xenosrb_info.graphics_base + 0x0704, rb_cntl);
}

static void xenosrb_setup_ring(void *buffer_base, size_t buffer_size)
{
	/* CP_RB_CNTL */
	/* RB_NO_UPDATE = 1 */
	/* BUF_SWAP = 2 */
	out_be32(xenosrb_info.graphics_base + 0x0704,
		 0x08020000 | (ilog2(buffer_size >> 3) & 0xFF));

	/* CP_RB_BASE */
	out_be32(xenosrb_info.graphics_base + 0x0700,
		 virt_to_phys(buffer_base));
}

/**
 *	xenosrb_wait_gui_idle - Waits for GUI idle or timeout
 *	@timeout_jiffies: maximum number of jiffies to wait
 *
 *	Returns 0 on success, or %ETIME on timeout.
 */
static int xenosrb_wait_gui_idle(unsigned long timeout_jiffies)
{
	unsigned long start_jiffy = jiffies;

	/* RBBM_STATUS.GUI_ACTIVE */
	while (in_be32(xenosrb_info.graphics_base + 0x1740) & 0x80000000) {
		if (jiffies - start_jiffy > timeout_jiffies) {
			return -ETIME;
		}
	}

	return 0;
}

/**
 *	xenosrb_upload_pfp_ucode - Upload and verify Prefetch Parser microcode
 *	@ucode: pointer to microcode buffer
 *
 *  Preferably you would call this with interrupts disabled.
 *
 *	Returns 0 on success, or %EIO on verification failure.
 */
static int xenosrb_upload_pfp_ucode(const uint32_t *ucode, size_t size)
{
	size_t i;

	out_be32(xenosrb_info.graphics_base + 0x117C, 0); /* CP_PFP_UCODE_ADDR */
	udelay(100);

	/* Write to CP_PFP_UCODE_DATA */
	for (i = 0; i < size; i++)
		out_be32(xenosrb_info.graphics_base + 0x1180, ucode[i]);

	/* Readback */
	out_be32(xenosrb_info.graphics_base + 0x117C, 0); /* CP_PFP_UCODE_ADDR */
	udelay(100);

	for (i = 0; i < size; i++) {
		/* Verification doesn't work, as the GPU appears to ignore
		 * CP_PFP_UCODE_ADDR and returns data at an offset :\ */
		in_be32(xenosrb_info.graphics_base + 0x1180);
	}

	return 0;
}

/**
 *	xenosrb_upload_me_ucode - Upload and verify ME microcode
 *	@ucode: pointer to microcode buffer
 *
 *  Preferably you would call this with interrupts disabled.
 *
 *	Returns 0 on success, or %EIO on verification failure.
 */
static int xenosrb_upload_me_ucode(const uint32_t *ucode, size_t size)
{
	size_t i;

	out_be32(xenosrb_info.graphics_base + 0x07E0, 0); /* CP_ME_RAM_WADDR */
	udelay(100);

	/* Write to CP_ME_RAM_DATA */
	for (i = 0; i < size; i++)
		out_be32(xenosrb_info.graphics_base + 0x07E8, ucode[i]);

	/* Readback and verify from CP_ME_RAM_DATA */
	out_be32(xenosrb_info.graphics_base + 0x07E4, 0); /* CP_ME_RAM_RADDR */
	udelay(100);

	for (i = 0; i < size; i++) {
		if (in_be32(xenosrb_info.graphics_base + 0x07E8) != ucode[i]) {
			printk(KERN_ERR
			       "%s: failed to verify me microcode @ dword %zu\n",
			       __func__, i);
			return -EIO;
		}
	}

	return 0;
}

static int xenosrb_setup(void *buffer_base, size_t buffer_size)
{
	int rc = 0;

	out_be32(xenosrb_info.graphics_base + 0x07D8, 0x1000FFFF); /* CP_ME_CNTL.ME_HALT = 1 */
	udelay(1000);

	xenosrb_reset_ring();
	xenosrb_setup_ring(buffer_base, buffer_size);

	rc = xenosrb_upload_pfp_ucode(xenos_pfp_ucode,
				      ARRAY_SIZE(xenos_pfp_ucode));
	if (rc) {
		return rc;
	}

	rc = xenosrb_upload_me_ucode(xenos_me_ucode,
				     ARRAY_SIZE(xenos_me_ucode));
	if (rc) {
		return rc;
	}

	xenosrb_wait_gui_idle(HZ / 4);

	out_be32(xenosrb_info.graphics_base + 0x07D8, 0x0000FFFF); /* CP_ME_CNTL.ME_HALT = 0 */
	out_be32(xenosrb_info.graphics_base + 0x07D0, 0x0000FFFF); /* CP_INT_ACK.RTS[0..15]ACK = 1 */
	out_be32(xenosrb_info.graphics_base + 0x07F0, 0x00000000); /* CP_DEBUG */
	out_be32(xenosrb_info.graphics_base + 0x07D8, 0x1000FFFF); /* CP_ME_CNTL.ME_HALT = 1 */

	udelay(2000);
	out_be32(xenosrb_info.graphics_base + 0x00F0, 0x00000001); /* RBBM_SOFT_RESET.SOFT_RESET_CP = 1 */
	udelay(1000);
	out_be32(xenosrb_info.graphics_base + 0x00F0, 0x00000000); /* RBBM_SOFT_RESET.SOFT_RESET_CP = 0 */
	udelay(1000);

	rc = xenosrb_wait_gui_idle(HZ / 4);
	if (rc) {
		printk(KERN_ERR "%s: timed out waiting for GUI idle (reset)\n", __func__);
		return rc;
	}

	xenosrb_setup_ring(buffer_base, buffer_size);
	out_be32(xenosrb_info.graphics_base + 0x07D8, 0x0000FFFF); /* CP_ME_CNTL.ME_HALT = 0 */

	rc = xenosrb_wait_gui_idle(HZ / 4);
	if (rc) {
		printk(KERN_ERR "%s: timed out waiting for GUI idle (final)\n", __func__);
		return rc;
	}

	return 0;
}

static void xenosrb_shutdown(void)
{
	out_be32(xenosrb_info.graphics_base + 0x07D8, 0x10000000); /* CP_ME_CNTL.ME_HALT = 1 */
    out_be32(xenosrb_info.graphics_base + 0x0704, 0x00000000); /* CP_RB_CNTL */
    out_be32(xenosrb_info.graphics_base + 0x0700, 0x00000000); /* CP_RB_BASE */
	xenosrb_reset_ring();
}

static irqreturn_t xenos_interrupt(int irq, void* dev)
{
	return IRQ_HANDLED;
}

static ssize_t xenosrb_write(struct file *file, const char __user *buf,
			     size_t count, loff_t *ppos)
{
	struct circ_buf *primary_rb = &xenosrb_info.primary_rb;
	size_t space;
	size_t trail_space;
	int head_ptr;
	unsigned long flags;

	if (*ppos || (count & 0x3) != 0x0)
		return -EINVAL;

	spin_lock(&xenosrb_info.write_lock);
	space = CIRC_SPACE(primary_rb->head, primary_rb->tail,
			   xenosrb_info.rb_size);
	if (space <= count) {
		/* Update the head pointer and check again */
		primary_rb->tail = in_be32(xenosrb_info.graphics_base + 0x0710) / 4;

		space = CIRC_SPACE(primary_rb->head, primary_rb->tail,
				   xenosrb_info.rb_size);

		if (space < count) {
			spin_unlock(&xenosrb_info.write_lock);
			return -EBUSY;
		}
	}

	trail_space = CIRC_SPACE_TO_END(primary_rb->head, primary_rb->tail,
					xenosrb_info.rb_size);

	/* Copy bytes to end of ringbuffer */
	if (copy_from_user(primary_rb->buf + primary_rb->head, buf,
			   min(trail_space, count))) {
		spin_unlock(&xenosrb_info.write_lock);
		return -EFAULT;
	}

	head_ptr = primary_rb->head + min(trail_space, count);

	if (trail_space < count) {
		/* Wrap around, copy bytes to beginning of ring buffer. */
		if (copy_from_user(primary_rb->buf, buf + trail_space,
				   count - trail_space)) {
			spin_unlock(&xenosrb_info.write_lock);
			return -EFAULT;
		}

		head_ptr = count - trail_space;
	}

	/* Update our tail pointer. */
	primary_rb->head = head_ptr & (xenosrb_info.rb_size - 1);

	/* Flush the CPU cache. */
	flush_dcache_range((uintptr_t)primary_rb->buf,
			   (uintptr_t)primary_rb->buf + xenosrb_info.rb_size);

	spin_lock_irqsave(&xenosrb_info.gfx_lock, flags);
	out_be32(xenosrb_info.graphics_base + 0x0714,
		 primary_rb->head / 4); /* CP_RB_WPTR */
	spin_unlock_irqrestore(&xenosrb_info.gfx_lock, flags);

	spin_unlock(&xenosrb_info.write_lock);
	return count;
}

static long xenosrb_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg)
{
	unsigned long flags;

	switch (cmd) {
	case IOCTL_RESET:
		spin_lock_irqsave(&xenosrb_info.gfx_lock, flags);

		out_be32(xenosrb_info.graphics_base + 0x07D8, 0x1000FFFF); /* CP_ME_CNTL.ME_HALT = 1 */
		xenosrb_reset_ring();
		xenosrb_info.primary_rb.head = 0;
		xenosrb_info.primary_rb.tail = 0;
		out_be32(xenosrb_info.graphics_base + 0x07D8, 0x0000FFFF); /* CP_ME_CNTL.ME_HALT = 0 */

		spin_unlock_irqrestore(&xenosrb_info.gfx_lock, flags);
		return 0;
	}

	return -EINVAL;
}

static const struct file_operations xenos_fops = {
	.write = xenosrb_write,
	.open = nonseekable_open,
    .unlocked_ioctl = xenosrb_ioctl,
};

static struct miscdevice xenosrb_dev = {
	.minor =  MISC_DYNAMIC_MINOR,
	"xenosrb",
	&xenos_fops
};

static int xenosrb_probe(struct pci_dev *dev,
			 const struct pci_device_id *dev_id)
{
	unsigned long flags;
	unsigned long mmio_start, mmio_len, mmio_flags;
	int rc = 0;

	if (!dev) {
		return -EINVAL;
	}

	if ((rc = pci_enable_device(dev))) {
		goto err_out;
	}

	mmio_flags = pci_resource_flags(dev, 0);
	if (!(mmio_flags & IORESOURCE_MEM)) {
		dev_err(&dev->dev,
			"%s: region #0 is not a MEM resource, abort!",
			__func__);
		rc = -ENODEV;
		goto err_release_pci_device;
	}

	mmio_start = pci_resource_start(dev, 0);
	mmio_len = pci_resource_len(dev, 0);

	/* We're going to request all regions, even though there's only one. */
	if (!mmio_len || (rc = pci_request_regions(dev, dev->dev.kobj.name))) {
		dev_err(&dev->dev,
			"%s: failed to request I/O regions (code %d)", __func__,
			rc);
		rc = -EBUSY;
		goto err_release_pci_device;
	}

	xenosrb_info.graphics_base = ioremap(mmio_start, mmio_len);
	if (!xenosrb_info.graphics_base) {
		printk(KERN_ERR "%s: failed to ioremap gfx regs\n", __func__);
		rc = -EIO;
		goto err_release_pci_regions;
	}

	rc = misc_register(&xenosrb_dev);
	if (rc) {
		goto err_release_reg;
	}

	xenosrb_info.rb_size = 1 << (ilog2(xenosrb_size) & 0xFF);
	xenosrb_info.primary_rb.buf = kzalloc(xenosrb_info.rb_size, GFP_KERNEL);
	if (!xenosrb_info.primary_rb.buf) {
		rc = -ENOMEM;
		goto err_unregister_dev;
	}

	spin_lock_init(&xenosrb_info.gfx_lock);
	spin_lock_init(&xenosrb_info.write_lock);

	/* Setup the ringbuffer with interrupts disabled */
	spin_lock_irqsave(&xenosrb_info.gfx_lock, flags);

	if (xenosrb_setup(xenosrb_info.primary_rb.buf, xenosrb_info.rb_size)) {
		spin_unlock_irqrestore(&xenosrb_info.gfx_lock, flags);
		rc = -EIO;
		goto err_free_rb;
	}

	xenosrb_info.primary_rb.head = 0;
	xenosrb_info.primary_rb.tail = 0;

	/* CP_RB_WPTR_DELAY */
	out_be32(xenosrb_info.graphics_base + 0x0718, 0x0010);

	spin_unlock_irqrestore(&xenosrb_info.gfx_lock, flags);

	rc = request_irq(dev->irq, xenos_interrupt, IRQF_SHARED, "Xenos", dev);
	if (rc) {
		printk(KERN_ERR "%s: failed to request IRQ 0x%.2X\n", __func__,
		       dev->irq);
	}

	printk("XenosRB Character Driver Initialized, ring size = %d\n",
	       xenosrb_info.rb_size);
	return 0;

	xenosrb_reset_ring();
err_free_rb:
	kfree(xenosrb_info.primary_rb.buf);
err_unregister_dev:
	misc_deregister(&xenosrb_dev);
err_release_reg:
	iounmap(xenosrb_info.graphics_base);
err_release_pci_regions:
	pci_release_regions(dev);
err_release_pci_device:
	pci_disable_device(dev);
err_out:
	return rc;
}

static void xenosrb_remove(struct pci_dev *dev)
{
	unsigned long flags;

	disable_irq(dev->irq);
	free_irq(dev->irq, dev);
	misc_deregister(&xenosrb_dev);

	spin_lock_irqsave(&xenosrb_info.gfx_lock, flags);
	xenosrb_shutdown();

	spin_unlock_irqrestore(&xenosrb_info.gfx_lock, flags);

	kfree(xenosrb_info.primary_rb.buf);
	iounmap(xenosrb_info.graphics_base);
	pci_release_regions(dev);
}

#define XENOSRB_CHR_DEV_NAME "xenos_rb"

static const struct pci_device_id xenos_pci_tbl[] = {
	{PCI_VDEVICE(MICROSOFT, 0x5811), 0}, /* xenon */
	{PCI_VDEVICE(MICROSOFT, 0x5821), 0}, /* zephyr/falcon */
	{PCI_VDEVICE(MICROSOFT, 0x5831), 0}, /* jasper */
	{PCI_VDEVICE(MICROSOFT, 0x5841), 0}, /* slim */

	{} /* terminate list */
};

static struct pci_driver xenosrb_pci = {
	.name = "xenos_rb",
	.id_table = xenos_pci_tbl,
	.probe = xenosrb_probe,
	.remove = xenosrb_remove,
};

static int __init xenosrb_init(void)
{
	int rc;

	if ((rc = pci_register_driver(&xenosrb_pci))) {
		printk(KERN_ERR "%s: pci_register_driver failed with %d",
		       __func__, rc);
		goto err_out;
	}

	return 0;

err_out:
	return rc;
}

static void __exit xenosrb_exit(void)
{
	pci_unregister_driver(&xenosrb_pci);
}

module_init(xenosrb_init);
module_exit(xenosrb_exit);

MODULE_AUTHOR("Justin Moore <arkolbed@gmail.com>");
MODULE_DESCRIPTION("Ring Buffer Driver for Xenos GPU");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);