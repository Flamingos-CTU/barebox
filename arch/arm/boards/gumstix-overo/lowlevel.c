/*
 * (C) Copyright 2016 Michal Vokáč <vokac.m@gmail.com>
 * Based on code from arch/arm/board/beagle/lowlevel.c
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <init.h>
#include <debug_ll.h>
#include <io.h>
#include <linux/sizes.h>
#include <asm/barebox-arm-head.h>
#include <asm/barebox-arm.h>
#include <mach/control.h>
#include <mach/generic.h>
#include <mach/omap3-silicon.h>
#include <mach/omap3-generic.h>
#include <mach/omap3-mux.h>
#include <mach/sdrc.h>
#include <mach/syslib.h>
#include <mach/sys_info.h>
#include <generated/mach-types.h>
#include "overo.h"

/**
 * @brief Do the pin muxing required for Board operation.
 * We enable ONLY the pins we require to set. OMAP provides pins which do not
 * have alternate modes. Such pins done need to be set.
 *
 * See @ref MUX_VAL for description of the muxing mode.
 *
 * @return void
 */
static void mux_config(void)
{
	/* SDRC */
	MUX_VAL(CP(SDRC_D0),		(IEN  | PTD | DIS | M0)); /*SDRC_D0*/
	MUX_VAL(CP(SDRC_D1),		(IEN  | PTD | DIS | M0)); /*SDRC_D1*/
	MUX_VAL(CP(SDRC_D2),		(IEN  | PTD | DIS | M0)); /*SDRC_D2*/
	MUX_VAL(CP(SDRC_D3),		(IEN  | PTD | DIS | M0)); /*SDRC_D3*/
	MUX_VAL(CP(SDRC_D4),		(IEN  | PTD | DIS | M0)); /*SDRC_D4*/
	MUX_VAL(CP(SDRC_D5),		(IEN  | PTD | DIS | M0)); /*SDRC_D5*/
	MUX_VAL(CP(SDRC_D6),		(IEN  | PTD | DIS | M0)); /*SDRC_D6*/
	MUX_VAL(CP(SDRC_D7),		(IEN  | PTD | DIS | M0)); /*SDRC_D7*/
	MUX_VAL(CP(SDRC_D8),		(IEN  | PTD | DIS | M0)); /*SDRC_D8*/
	MUX_VAL(CP(SDRC_D9),		(IEN  | PTD | DIS | M0)); /*SDRC_D9*/
	MUX_VAL(CP(SDRC_D10),		(IEN  | PTD | DIS | M0)); /*SDRC_D10*/
	MUX_VAL(CP(SDRC_D11),		(IEN  | PTD | DIS | M0)); /*SDRC_D11*/
	MUX_VAL(CP(SDRC_D12),		(IEN  | PTD | DIS | M0)); /*SDRC_D12*/
	MUX_VAL(CP(SDRC_D13),		(IEN  | PTD | DIS | M0)); /*SDRC_D13*/
	MUX_VAL(CP(SDRC_D14),		(IEN  | PTD | DIS | M0)); /*SDRC_D14*/
	MUX_VAL(CP(SDRC_D15),		(IEN  | PTD | DIS | M0)); /*SDRC_D15*/
	MUX_VAL(CP(SDRC_D16),		(IEN  | PTD | DIS | M0)); /*SDRC_D16*/
	MUX_VAL(CP(SDRC_D17),		(IEN  | PTD | DIS | M0)); /*SDRC_D17*/
	MUX_VAL(CP(SDRC_D18),		(IEN  | PTD | DIS | M0)); /*SDRC_D18*/
	MUX_VAL(CP(SDRC_D19),		(IEN  | PTD | DIS | M0)); /*SDRC_D19*/
	MUX_VAL(CP(SDRC_D20),		(IEN  | PTD | DIS | M0)); /*SDRC_D20*/
	MUX_VAL(CP(SDRC_D21),		(IEN  | PTD | DIS | M0)); /*SDRC_D21*/
	MUX_VAL(CP(SDRC_D22),		(IEN  | PTD | DIS | M0)); /*SDRC_D22*/
	MUX_VAL(CP(SDRC_D23),		(IEN  | PTD | DIS | M0)); /*SDRC_D23*/
	MUX_VAL(CP(SDRC_D24),		(IEN  | PTD | DIS | M0)); /*SDRC_D24*/
	MUX_VAL(CP(SDRC_D25),		(IEN  | PTD | DIS | M0)); /*SDRC_D25*/
	MUX_VAL(CP(SDRC_D26),		(IEN  | PTD | DIS | M0)); /*SDRC_D26*/
	MUX_VAL(CP(SDRC_D27),		(IEN  | PTD | DIS | M0)); /*SDRC_D27*/
	MUX_VAL(CP(SDRC_D28),		(IEN  | PTD | DIS | M0)); /*SDRC_D28*/
	MUX_VAL(CP(SDRC_D29),		(IEN  | PTD | DIS | M0)); /*SDRC_D29*/
	MUX_VAL(CP(SDRC_D30),		(IEN  | PTD | DIS | M0)); /*SDRC_D30*/
	MUX_VAL(CP(SDRC_D31),		(IEN  | PTD | DIS | M0)); /*SDRC_D31*/
	MUX_VAL(CP(SDRC_CLK),		(IEN  | PTD | DIS | M0)); /*SDRC_CLK*/
	MUX_VAL(CP(SDRC_DQS0),		(IEN  | PTD | DIS | M0)); /*SDRC_DQS0*/
	MUX_VAL(CP(SDRC_DQS1),		(IEN  | PTD | DIS | M0)); /*SDRC_DQS1*/
	MUX_VAL(CP(SDRC_DQS2),		(IEN  | PTD | DIS | M0)); /*SDRC_DQS2*/
	MUX_VAL(CP(SDRC_DQS3),		(IEN  | PTD | DIS | M0)); /*SDRC_DQS3*/

	/* GPMC */
	MUX_VAL(CP(GPMC_A1),		(IDIS | PTU | EN  | M0)); /*GPMC_A1*/
	MUX_VAL(CP(GPMC_A2),		(IDIS | PTU | EN  | M0)); /*GPMC_A2*/
	MUX_VAL(CP(GPMC_A3),		(IDIS | PTU | EN  | M0)); /*GPMC_A3*/
	MUX_VAL(CP(GPMC_A4),		(IDIS | PTU | EN  | M0)); /*GPMC_A4*/
	MUX_VAL(CP(GPMC_A5),		(IDIS | PTU | EN  | M0)); /*GPMC_A5*/
	MUX_VAL(CP(GPMC_A6),		(IDIS | PTU | EN  | M0)); /*GPMC_A6*/
	MUX_VAL(CP(GPMC_A7),		(IDIS | PTU | EN  | M0)); /*GPMC_A7*/
	MUX_VAL(CP(GPMC_A8),		(IDIS | PTU | EN  | M0)); /*GPMC_A8*/
	MUX_VAL(CP(GPMC_A9),		(IDIS | PTU | EN  | M0)); /*GPMC_A9*/
	MUX_VAL(CP(GPMC_D0),		(IEN  | PTU | EN  | M0)); /*GPMC_D0*/
	MUX_VAL(CP(GPMC_D1),		(IEN  | PTU | EN  | M0)); /*GPMC_D1*/
	MUX_VAL(CP(GPMC_D2),		(IEN  | PTU | EN  | M0)); /*GPMC_D2*/
	MUX_VAL(CP(GPMC_D3),		(IEN  | PTU | EN  | M0)); /*GPMC_D3*/
	MUX_VAL(CP(GPMC_D4),		(IEN  | PTU | EN  | M0)); /*GPMC_D4*/
	MUX_VAL(CP(GPMC_D5),		(IEN  | PTU | EN  | M0)); /*GPMC_D5*/
	MUX_VAL(CP(GPMC_D6),		(IEN  | PTU | EN  | M0)); /*GPMC_D6*/
	MUX_VAL(CP(GPMC_D7),		(IEN  | PTU | EN  | M0)); /*GPMC_D7*/
	MUX_VAL(CP(GPMC_D8),		(IEN  | PTU | EN  | M0)); /*GPMC_D8*/
	MUX_VAL(CP(GPMC_D9),		(IEN  | PTU | EN  | M0)); /*GPMC_D9*/
	MUX_VAL(CP(GPMC_D10),		(IEN  | PTU | EN  | M0)); /*GPMC_D10*/
	MUX_VAL(CP(GPMC_D11),		(IEN  | PTU | EN  | M0)); /*GPMC_D11*/
	MUX_VAL(CP(GPMC_D12),		(IEN  | PTU | EN  | M0)); /*GPMC_D12*/
	MUX_VAL(CP(GPMC_D13),		(IEN  | PTU | EN  | M0)); /*GPMC_D13*/
	MUX_VAL(CP(GPMC_D14),		(IEN  | PTU | EN  | M0)); /*GPMC_D14*/
	MUX_VAL(CP(GPMC_D15),		(IEN  | PTU | EN  | M0)); /*GPMC_D15*/
	MUX_VAL(CP(GPMC_NCS0),		(IDIS | PTU | EN  | M0)); /*GPMC_nCS0*/
	MUX_VAL(CP(GPMC_NCS2),		(IDIS | PTU | EN  | M0)); /*GPMC_nCS2*/
	MUX_VAL(CP(GPMC_NCS5),		(IEN  | PTU | EN  | M0)); /*GPMC_nCS5*/
	MUX_VAL(CP(GPMC_NCS7),		(IEN  | PTU | EN  | M0)); /*GPMC_nCS7*/
	MUX_VAL(CP(GPMC_NBE1),		(IEN  | PTD | DIS | M0)); /*GPMC_nBE*/
	MUX_VAL(CP(GPMC_CLK),		(IEN  | PTU | EN  | M0)); /*GPMC_CLK*/
	MUX_VAL(CP(GPMC_NADV_ALE),	(IDIS | PTD | DIS | M0)); /*GPMC_nADV_ALE*/
	MUX_VAL(CP(GPMC_NOE),		(IDIS | PTD | DIS | M0)); /*GPMC_nOE*/
	MUX_VAL(CP(GPMC_NWE),		(IDIS | PTD | DIS | M0)); /*GPMC_nWE*/
	MUX_VAL(CP(GPMC_NBE0_CLE),	(IDIS | PTD | DIS | M0)); /*GPMC_nBE0_CLE*/
	MUX_VAL(CP(GPMC_NWP),		(IEN  | PTD | DIS | M0)); /*GPMC_nWP*/
	MUX_VAL(CP(GPMC_WAIT0),		(IEN  | PTU | EN  | M0)); /*GPMC_WAIT0*/

	/* Camera */
	MUX_VAL(CP(CAM_HS),		(IEN  | PTU | DIS | M0)); /*CAM_HS */
	MUX_VAL(CP(CAM_VS),		(IEN  | PTU | DIS | M0)); /*CAM_VS */
	MUX_VAL(CP(CAM_XCLKA),		(IDIS | PTD | DIS | M0)); /*CAM_XCLKA*/
	MUX_VAL(CP(CAM_PCLK),		(IEN  | PTU | DIS | M0)); /*CAM_PCLK*/
	MUX_VAL(CP(CAM_D0),		(IEN  | PTD | DIS | M0)); /*CAM_D0*/
	MUX_VAL(CP(CAM_D1),		(IEN  | PTD | DIS | M0)); /*CAM_D1*/
	MUX_VAL(CP(CAM_D2),		(IEN  | PTD | DIS | M0)); /*CAM_D2*/
	MUX_VAL(CP(CAM_D3),		(IEN  | PTD | DIS | M0)); /*CAM_D3*/
	MUX_VAL(CP(CAM_D4),		(IEN  | PTD | DIS | M0)); /*CAM_D4*/
	MUX_VAL(CP(CAM_D5),		(IEN  | PTD | DIS | M0)); /*CAM_D5*/
	MUX_VAL(CP(CAM_D6),		(IEN  | PTD | DIS | M0)); /*CAM_D6*/
	MUX_VAL(CP(CAM_D7),		(IEN  | PTD | DIS | M0)); /*CAM_D7*/
	MUX_VAL(CP(CAM_D8),		(IEN  | PTD | DIS | M0)); /*CAM_D8*/
	MUX_VAL(CP(CAM_D9),		(IEN  | PTD | DIS | M0)); /*CAM_D9*/
	MUX_VAL(CP(CAM_D10),		(IEN  | PTD | DIS | M0)); /*CAM_D10*/
	MUX_VAL(CP(CAM_D11),		(IEN  | PTD | DIS | M0)); /*CAM_D11*/

	/* Audio Interface */
	MUX_VAL(CP(MCBSP2_FSX),		(IEN  | PTD | DIS | M0)); /*McBSP2_FSX*/
	MUX_VAL(CP(MCBSP2_CLKX),	(IEN  | PTD | DIS | M0)); /*McBSP2_CLKX*/
	MUX_VAL(CP(MCBSP2_DR),		(IEN  | PTD | DIS | M0)); /*McBSP2_DR*/
	MUX_VAL(CP(MCBSP2_DX),		(IDIS | PTD | DIS | M0)); /*McBSP2_DX*/

	/* Expansion card */
	MUX_VAL(CP(MMC1_CLK),		(IEN  | PTU | EN  | M0)); /*MMC1_CLK*/
	MUX_VAL(CP(MMC1_CMD),		(IEN  | PTU | EN  | M0)); /*MMC1_CMD*/
	MUX_VAL(CP(MMC1_DAT0),		(IEN  | PTU | EN  | M0)); /*MMC1_DAT0*/
	MUX_VAL(CP(MMC1_DAT1),		(IEN  | PTU | EN  | M0)); /*MMC1_DAT1*/
	MUX_VAL(CP(MMC1_DAT2),		(IEN  | PTU | EN  | M0)); /*MMC1_DAT2*/
	MUX_VAL(CP(MMC1_DAT3),		(IEN  | PTU | EN  | M0)); /*MMC1_DAT3*/
	MUX_VAL(CP(MMC1_DAT4),		(IEN  | PTU | EN  | M0)); /*MMC1_DAT4*/
	MUX_VAL(CP(MMC1_DAT5),		(IEN  | PTU | EN  | M0)); /*MMC1_DAT5*/
	MUX_VAL(CP(MMC1_DAT6),		(IEN  | PTU | EN  | M0)); /*MMC1_DAT6*/
	MUX_VAL(CP(MMC1_DAT7),		(IEN  | PTU | EN  | M0)); /*MMC1_DAT7*/
	MUX_VAL(CP(GPMC_NCS3),		(IEN  | PTU | EN  | M4)); /*GPIO_54*/
								 /* MMC1_WP*/

	/* Wireless LAN */
	MUX_VAL(CP(MMC2_CLK),		(IEN  | PTU | EN  | M4)); /*GPIO_130*/
	MUX_VAL(CP(MMC2_CMD),		(IEN  | PTU | EN  | M0)); /*MMC2_CMD*/
	MUX_VAL(CP(MMC2_DAT0),		(IEN  | PTU | EN  | M0)); /*MMC2_DAT0*/
	MUX_VAL(CP(MMC2_DAT1),		(IEN  | PTU | EN  | M0)); /*MMC2_DAT1*/
	MUX_VAL(CP(MMC2_DAT2),		(IEN  | PTU | EN  | M0)); /*MMC2_DAT2*/
	MUX_VAL(CP(MMC2_DAT3),		(IEN  | PTU | EN  | M0)); /*MMC2_DAT3*/
	MUX_VAL(CP(MMC2_DAT4),		(IEN  | PTU | EN  | M1)); /*MMC2_DIR_DAT0*/
	MUX_VAL(CP(MMC2_DAT5),		(IEN  | PTU | EN  | M1)); /*MMC2_DIR_DAT1*/
	MUX_VAL(CP(MMC2_DAT6),		(IEN  | PTU | EN  | M1)); /*MMC2_DIR_CMD*/
	MUX_VAL(CP(MMC2_DAT7),		(IEN  | PTU | EN  | M4)); /*GPIO_139*/

	/* Bluetooth */
	MUX_VAL(CP(MCBSP3_DX),		(IEN  | PTD | DIS | M1)); /*UART2_CTS*/
	MUX_VAL(CP(MCBSP3_DR),		(IDIS | PTD | DIS | M1)); /*UART2_RTS*/
	MUX_VAL(CP(MCBSP3_CLKX),	(IDIS | PTD | DIS | M1)); /*UART2_TX*/
	MUX_VAL(CP(MCBSP3_FSX),		(IEN  | PTD | DIS | M1)); /*UART2_RX*/
	MUX_VAL(CP(UART1_RTS),		(IEN  | PTU | DIS | M4)); /*GPIO_149*/
	MUX_VAL(CP(MCBSP4_CLKX),	(IEN  | PTD | DIS | M0)); /*McBSP4_CLKX*/
	MUX_VAL(CP(MCBSP4_DR),		(IEN  | PTD | DIS | M0)); /*McBSP4_DR*/
	MUX_VAL(CP(MCBSP4_DX),		(IEN  | PTD | DIS | M0)); /*McBSP4_DX*/
	MUX_VAL(CP(MCBSP4_FSX),		(IEN  | PTD | DIS | M0)); /*McBSP4_FSX*/
	MUX_VAL(CP(MCBSP1_CLKR),	(IEN  | PTD | DIS | M0)); /*McBSP1_CLKR*/
	MUX_VAL(CP(MCBSP1_FSR),		(IEN  | PTD | DIS | M0)); /*McBSP1_FSR*/
	MUX_VAL(CP(MCBSP1_DX),		(IEN  | PTD | DIS | M0)); /*McBSP1_DX*/
	MUX_VAL(CP(MCBSP1_DR),		(IEN  | PTD | DIS | M0)); /*McBSP1_DR*/
	MUX_VAL(CP(MCBSP_CLKS),		(IEN  | PTU | DIS | M0)); /*McBSP_CLKS*/
	MUX_VAL(CP(MCBSP1_FSX),		(IEN  | PTD | DIS | M0)); /*McBSP1_FSX*/
	MUX_VAL(CP(MCBSP1_CLKX),	(IEN  | PTD | DIS | M0)); /*McBSP1_CLKX*/

	/* Serial Interface */
	MUX_VAL(CP(UART3_RX_IRRX),	(IEN  | PTU | EN  | M0)); /*UART3_RX_IRRX*/
	MUX_VAL(CP(UART3_TX_IRTX),	(IDIS | PTD | DIS | M0)); /*UART3_TX_IRTX*/

	/* USB host (port 0) */
	MUX_VAL(CP(HSUSB0_CLK),		(IEN  | PTD | DIS | M0)); /*HSUSB0_CLK*/
	MUX_VAL(CP(HSUSB0_STP),		(IDIS | PTU | EN  | M0)); /*HSUSB0_STP*/
	MUX_VAL(CP(HSUSB0_DIR),		(IEN  | PTD | DIS | M0)); /*HSUSB0_DIR*/
	MUX_VAL(CP(HSUSB0_NXT),		(IEN  | PTD | DIS | M0)); /*HSUSB0_NXT*/
	MUX_VAL(CP(HSUSB0_DATA0),	(IEN  | PTD | DIS | M0)); /*HSUSB0_DATA0*/
	MUX_VAL(CP(HSUSB0_DATA1),	(IEN  | PTD | DIS | M0)); /*HSUSB0_DATA1*/
	MUX_VAL(CP(HSUSB0_DATA2),	(IEN  | PTD | DIS | M0)); /*HSUSB0_DATA2*/
	MUX_VAL(CP(HSUSB0_DATA3),	(IEN  | PTD | DIS | M0)); /*HSUSB0_DATA3*/
	MUX_VAL(CP(HSUSB0_DATA4),	(IEN  | PTD | DIS | M0)); /*HSUSB0_DATA4*/
	MUX_VAL(CP(HSUSB0_DATA5),	(IEN  | PTD | DIS | M0)); /*HSUSB0_DATA5*/
	MUX_VAL(CP(HSUSB0_DATA6),	(IEN  | PTD | DIS | M0)); /*HSUSB0_DATA6*/
	MUX_VAL(CP(HSUSB0_DATA7),	(IEN  | PTD | DIS | M0)); /*HSUSB0_DATA7*/
	MUX_VAL(CP(I2C2_SCL),		(IEN  | PTU | EN  | M4)); /*GPIO_168*/
								 /* - USBH_CPEN*/
	MUX_VAL(CP(I2C2_SDA),		(IEN  | PTU | EN  | M4)); /*GPIO_183*/
								 /* - USBH_RESET*/

	/* USB EHCI (port 2) */
	MUX_VAL(CP(ETK_D10_ES2),	(IEN  | PTD | DIS | M3)); /*HSUSB2_CLK*/
	MUX_VAL(CP(ETK_D11_ES2),	(IDIS | PTD | DIS | M3)); /*HSUSB2_STP*/
	MUX_VAL(CP(ETK_D12_ES2),	(IEN  | PTD | DIS | M3)); /*HSUSB2_DIR*/
	MUX_VAL(CP(ETK_D13_ES2),	(IEN  | PTD | DIS | M3)); /*HSUSB2_NXT*/
	MUX_VAL(CP(ETK_D14_ES2),	(IEN  | PTD | DIS | M3)); /*HSUSB2_DATA0*/
	MUX_VAL(CP(ETK_D15_ES2),	(IEN  | PTD | DIS | M3)); /*HSUSB2_DATA1*/
	MUX_VAL(CP(MCSPI1_CS3),		(IEN  | PTD | DIS | M3)); /*HSUSB2_DATA2*/
	MUX_VAL(CP(MCSPI2_CS1),		(IEN  | PTD | DIS | M3)); /*HSUSB2_DATA3*/
	MUX_VAL(CP(MCSPI2_SIMO),	(IEN  | PTD | DIS | M3)); /*HSUSB2_DATA4*/
	MUX_VAL(CP(MCSPI2_SOMI),	(IEN  | PTD | DIS | M3)); /*HSUSB2_DATA5*/
	MUX_VAL(CP(MCSPI2_CS0),		(IEN  | PTD | DIS | M3)); /*HSUSB2_DATA6*/
	MUX_VAL(CP(MCSPI2_CLK),		(IEN  | PTD | DIS | M3)); /*HSUSB2_DATA7*/

	/* I2C1 */
	MUX_VAL(CP(I2C1_SCL),		(IEN  | PTU | EN  | M0)); /*I2C1_SCL*/
	MUX_VAL(CP(I2C1_SDA),		(IEN  | PTU | EN  | M0)); /*I2C1_SDA*/

	/* I2C3 */
	MUX_VAL(CP(I2C3_SCL),		(IEN  | PTU | EN  | M0)); /*I2C3_SCL*/
	MUX_VAL(CP(I2C3_SDA),		(IEN  | PTU | EN  | M0)); /*I2C3_SDA*/

	/* I2C4 */
	MUX_VAL(CP(I2C4_SCL),		(IEN  | PTU | EN  | M0)); /*I2C4_SCL*/
	MUX_VAL(CP(I2C4_SDA),		(IEN  | PTU | EN  | M0)); /*I2C4_SDA*/

	/* Control and debug */
	MUX_VAL(CP(SYS_32K),		(IEN  | PTD | DIS | M0)); /*SYS_32K*/
	MUX_VAL(CP(SYS_CLKREQ),		(IEN  | PTD | DIS | M0)); /*SYS_CLKREQ*/
	MUX_VAL(CP(SYS_NIRQ),		(IEN  | PTU | EN  | M0)); /*SYS_nIRQ*/
	MUX_VAL(CP(SYS_BOOT0),		(IEN  | PTD | DIS | M4)); /*GPIO_2*/
	MUX_VAL(CP(SYS_BOOT1),		(IEN  | PTD | DIS | M4)); /*GPIO_3 */
	MUX_VAL(CP(SYS_BOOT2),		(IEN  | PTD | DIS | M4)); /*GPIO_4 - MMC1_WP*/
	MUX_VAL(CP(SYS_BOOT3),		(IEN  | PTD | DIS | M4)); /*GPIO_5*/
	MUX_VAL(CP(SYS_BOOT4),		(IEN  | PTD | DIS | M4)); /*GPIO_6*/
	MUX_VAL(CP(SYS_BOOT5),		(IEN  | PTD | DIS | M4)); /*GPIO_7*/
	MUX_VAL(CP(SYS_BOOT6),		(IDIS | PTD | DIS | M4)); /*GPIO_8*/
	MUX_VAL(CP(SYS_OFF_MODE),	(IEN  | PTD | DIS | M0)); /*SYS_OFF_MODE*/
	MUX_VAL(CP(ETK_D1_ES2),		(IEN  | PTD | EN  | M4)); /*GPIO_15 - X_GATE*/
	MUX_VAL(CP(ETK_D2_ES2),		(IEN  | PTU | EN  | M4)); /*GPIO_16*/
								 /* - W2W_NRESET*/
	MUX_VAL(CP(UART2_RX),		(IEN  | PTD | DIS | M4)); /*GPIO_147*/;

	/* Board revision signals */
	MUX_VAL(CP(CSI2_DX0),		(IEN  | PTD | EN  | M4)); /*GPIO_112*/
	MUX_VAL(CP(CSI2_DY0),		(IEN  | PTD | EN  | M4)); /*GPIO_113*/
	MUX_VAL(CP(CSI2_DY1),		(IEN  | PTD | EN  | M4)); /*GPIO_115*/

	/* SMSC9221 Ethernet */
	MUX_VAL(CP(GPMC_WAIT2),		(IDIS | PTU | EN  | M4)); /* GPIO_64 - ETH0_NRESET */
	MUX_VAL(CP(MCSPI1_CS2),		(IEN  | PTD | EN  | M4)); /* GPIO_176 - ETH0_IRQ */
	MUX_VAL(CP(GPMC_A10),		(IDIS | PTU | EN  | M0)); /* GPIO_ - ETH0_FIFOSEL */

	/* die to die */
	MUX_VAL(CP(D2D_MCAD1),		(IEN  | PTD | EN  | M0)); /*d2d_mcad1*/
	MUX_VAL(CP(D2D_MCAD2),		(IEN  | PTD | EN  | M0)); /*d2d_mcad2*/
	MUX_VAL(CP(D2D_MCAD3),		(IEN  | PTD | EN  | M0)); /*d2d_mcad3*/
	MUX_VAL(CP(D2D_MCAD4),		(IEN  | PTD | EN  | M0)); /*d2d_mcad4*/
	MUX_VAL(CP(D2D_MCAD5),		(IEN  | PTD | EN  | M0)); /*d2d_mcad5*/
	MUX_VAL(CP(D2D_MCAD6),		(IEN  | PTD | EN  | M0)); /*d2d_mcad6*/
	MUX_VAL(CP(D2D_MCAD7),		(IEN  | PTD | EN  | M0)); /*d2d_mcad7*/
	MUX_VAL(CP(D2D_MCAD8),		(IEN  | PTD | EN  | M0)); /*d2d_mcad8*/
	MUX_VAL(CP(D2D_MCAD9),		(IEN  | PTD | EN  | M0)); /*d2d_mcad9*/
	MUX_VAL(CP(D2D_MCAD10),		(IEN  | PTD | EN  | M0)); /*d2d_mcad10*/
	MUX_VAL(CP(D2D_MCAD11),		(IEN  | PTD | EN  | M0)); /*d2d_mcad11*/
	MUX_VAL(CP(D2D_MCAD12),		(IEN  | PTD | EN  | M0)); /*d2d_mcad12*/
	MUX_VAL(CP(D2D_MCAD13),		(IEN  | PTD | EN  | M0)); /*d2d_mcad13*/
	MUX_VAL(CP(D2D_MCAD14),		(IEN  | PTD | EN  | M0)); /*d2d_mcad14*/
	MUX_VAL(CP(D2D_MCAD15),		(IEN  | PTD | EN  | M0)); /*d2d_mcad15*/
	MUX_VAL(CP(D2D_MCAD16),		(IEN  | PTD | EN  | M0)); /*d2d_mcad16*/
	MUX_VAL(CP(D2D_MCAD17),		(IEN  | PTD | EN  | M0)); /*d2d_mcad17*/
	MUX_VAL(CP(D2D_MCAD18),		(IEN  | PTD | EN  | M0)); /*d2d_mcad18*/
	MUX_VAL(CP(D2D_MCAD19),		(IEN  | PTD | EN  | M0)); /*d2d_mcad19*/
	MUX_VAL(CP(D2D_MCAD20),		(IEN  | PTD | EN  | M0)); /*d2d_mcad20*/
	MUX_VAL(CP(D2D_MCAD21),		(IEN  | PTD | EN  | M0)); /*d2d_mcad21*/
	MUX_VAL(CP(D2D_MCAD22),		(IEN  | PTD | EN  | M0)); /*d2d_mcad22*/
	MUX_VAL(CP(D2D_MCAD23),		(IEN  | PTD | EN  | M0)); /*d2d_mcad23*/
	MUX_VAL(CP(D2D_MCAD24),		(IEN  | PTD | EN  | M0)); /*d2d_mcad24*/
	MUX_VAL(CP(D2D_MCAD25),		(IEN  | PTD | EN  | M0)); /*d2d_mcad25*/
	MUX_VAL(CP(D2D_MCAD26),		(IEN  | PTD | EN  | M0)); /*d2d_mcad26*/
	MUX_VAL(CP(D2D_MCAD27),		(IEN  | PTD | EN  | M0)); /*d2d_mcad27*/
	MUX_VAL(CP(D2D_MCAD28),		(IEN  | PTD | EN  | M0)); /*d2d_mcad28*/
	MUX_VAL(CP(D2D_MCAD29),		(IEN  | PTD | EN  | M0)); /*d2d_mcad29*/
	MUX_VAL(CP(D2D_MCAD30),		(IEN  | PTD | EN  | M0)); /*d2d_mcad30*/
	MUX_VAL(CP(D2D_MCAD31),		(IEN  | PTD | EN  | M0)); /*d2d_mcad31*/
	MUX_VAL(CP(D2D_MCAD32),		(IEN  | PTD | EN  | M0)); /*d2d_mcad32*/
	MUX_VAL(CP(D2D_MCAD33),		(IEN  | PTD | EN  | M0)); /*d2d_mcad33*/
	MUX_VAL(CP(D2D_MCAD34),		(IEN  | PTD | EN  | M0)); /*d2d_mcad34*/
	MUX_VAL(CP(D2D_MCAD35),		(IEN  | PTD | EN  | M0)); /*d2d_mcad35*/
	MUX_VAL(CP(D2D_MCAD36),		(IEN  | PTD | EN  | M0)); /*d2d_mcad36*/
	MUX_VAL(CP(D2D_CLK26MI),	(IEN  | PTD | DIS | M0)); /*d2d_clk26mi*/
	MUX_VAL(CP(D2D_NRESPWRON),	(IEN  | PTD | EN  | M0)); /*d2d_nrespwron*/
	MUX_VAL(CP(D2D_NRESWARM),	(IEN  | PTU | EN  | M0)); /*d2d_nreswarm */
	MUX_VAL(CP(D2D_ARM9NIRQ),	(IEN  | PTD | DIS | M0)); /*d2d_arm9nirq */
	MUX_VAL(CP(D2D_UMA2P6FIQ),	(IEN  | PTD | DIS | M0)); /*d2d_uma2p6fiq*/
	MUX_VAL(CP(D2D_SPINT),		(IEN  | PTD | EN  | M0)); /*d2d_spint*/
	MUX_VAL(CP(D2D_FRINT),		(IEN  | PTD | EN  | M0)); /*d2d_frint*/
	MUX_VAL(CP(D2D_DMAREQ0),	(IEN  | PTD | DIS | M0)); /*d2d_dmareq0*/
	MUX_VAL(CP(D2D_DMAREQ1),	(IEN  | PTD | DIS | M0)); /*d2d_dmareq1*/
	MUX_VAL(CP(D2D_DMAREQ2),	(IEN  | PTD | DIS | M0)); /*d2d_dmareq2*/
	MUX_VAL(CP(D2D_DMAREQ3),	(IEN  | PTD | DIS | M0)); /*d2d_dmareq3*/
	MUX_VAL(CP(D2D_N3GTRST),	(IEN  | PTD | DIS | M0)); /*d2d_n3gtrst*/
	MUX_VAL(CP(D2D_N3GTDI),		(IEN  | PTD | DIS | M0)); /*d2d_n3gtdi*/
	MUX_VAL(CP(D2D_N3GTDO),		(IEN  | PTD | DIS | M0)); /*d2d_n3gtdo*/
	MUX_VAL(CP(D2D_N3GTMS),		(IEN  | PTD | DIS | M0)); /*d2d_n3gtms*/
	MUX_VAL(CP(D2D_N3GTCK),		(IEN  | PTD | DIS | M0)); /*d2d_n3gtck*/
	MUX_VAL(CP(D2D_N3GRTCK),	(IEN  | PTD | DIS | M0)); /*d2d_n3grtck*/
	MUX_VAL(CP(D2D_MSTDBY),		(IEN  | PTU | EN  | M0)); /*d2d_mstdby*/
	MUX_VAL(CP(D2D_SWAKEUP),	(IEN  | PTD | EN  | M0)); /*d2d_swakeup*/
	MUX_VAL(CP(D2D_IDLEREQ),	(IEN  | PTD | DIS | M0)); /*d2d_idlereq*/
	MUX_VAL(CP(D2D_IDLEACK),	(IEN  | PTU | EN  | M0)); /*d2d_idleack*/
	MUX_VAL(CP(D2D_MWRITE),		(IEN  | PTD | DIS | M0)); /*d2d_mwrite*/
	MUX_VAL(CP(D2D_SWRITE),		(IEN  | PTD | DIS | M0)); /*d2d_swrite*/
	MUX_VAL(CP(D2D_MREAD),		(IEN  | PTD | DIS | M0)); /*d2d_mread*/
	MUX_VAL(CP(D2D_SREAD),		(IEN  | PTD | DIS | M0)); /*d2d_sread*/
	MUX_VAL(CP(D2D_MBUSFLAG),	(IEN  | PTD | DIS | M0)); /*d2d_mbusflag*/
	MUX_VAL(CP(D2D_SBUSFLAG),	(IEN  | PTD | DIS | M0)); /*d2d_sbusflag*/
	MUX_VAL(CP(SDRC_CKE0),		(IDIS | PTU | EN  | M0)); /*sdrc_cke0*/
	MUX_VAL(CP(SDRC_CKE1),		(IDIS | PTU | EN  | M0)); /*sdrc_cke1*/
}

/**
 * @brief Do the SDRC initialization for DDR on CS0
 *
 * @return void
 */
static void sdrc_init(void)
{
	/* SDRAM software reset */
	/* No idle ack and RESET enable */
	writel(0x1A, OMAP3_SDRC_REG(SYSCONFIG));
	sdelay(100);
	/* No idle ack and RESET disable */
	writel(0x18, OMAP3_SDRC_REG(SYSCONFIG));

	/* SDRC Sharing register */
	/* 32-bit SDRAM on data lane [31:0] - CS0 */
	/* pin tri-stated = 1 */
	writel(0x00000100, OMAP3_SDRC_REG(SHARING));

	/* ----- SDRC Registers Configuration --------- */
	/* SDRC_MCFG0 register */
	writel(0x02584099, OMAP3_SDRC_REG(MCFG_0));

	/* SDRC_RFR_CTRL0 register */
	writel(0x54601, OMAP3_SDRC_REG(RFR_CTRL_0));

	/* SDRC_ACTIM_CTRLA0 register */
	writel(0xA29DB4C6, OMAP3_SDRC_REG(ACTIM_CTRLA_0));

	/* SDRC_ACTIM_CTRLB0 register */
	writel(0x12214, OMAP3_SDRC_REG(ACTIM_CTRLB_0));

	/* Disble Power Down of CKE due to 1 CKE on combo part */
	writel(0x00000081, OMAP3_SDRC_REG(POWER));

	/* SDRC_MANUAL command register */
	/* NOP command */
	writel(0x00000000, OMAP3_SDRC_REG(MANUAL_0));
	/* Precharge command */
	writel(0x00000001, OMAP3_SDRC_REG(MANUAL_0));
	/* Auto-refresh command */
	writel(0x00000002, OMAP3_SDRC_REG(MANUAL_0));
	/* Auto-refresh command */
	writel(0x00000002, OMAP3_SDRC_REG(MANUAL_0));

	/* SDRC MR0 register Burst length=4 */
	writel(0x00000032, OMAP3_SDRC_REG(MR_0));

	/* SDRC DLLA control register */
	writel(0x0000000A, OMAP3_SDRC_REG(DLLA_CTRL));

	return;
}

static noinline int overo_board_init_sdram(void)
{
	struct barebox_arm_boarddata *bd = (void *)OMAP3_SRAM_SCRATCH_SPACE + 0x10;

	boarddata_create(bd, MACH_TYPE_OVERO);

	barebox_arm_entry(SDRAM_BASE, SDRAM_SIZE, bd);
}

ENTRY_FUNCTION(start_omap3_overo_sdram, bootinfo, r1, r2)
{
	omap3_save_bootinfo((void *)bootinfo);

	overo_board_init_sdram();
}

/**
 * @brief The basic entry point for board initialization.
 *
 * This is called as part of machine init (after arch init).
 * This is again called with stack in SRAM, so not too many
 * constructs possible here.
 *
 * @return void
 */
static noinline int overo_board_init(void)
{
	int in_sdram = omap3_running_in_sdram();
	struct barebox_arm_boarddata bd;

	if (!in_sdram)
		omap3_core_init();

	mux_config();

	omap_uart_lowlevel_init((void *)OMAP3_UART3_BASE);

	/* Dont reconfigure SDRAM while running in SDRAM! */
	if (!in_sdram)
		sdrc_init();

	boarddata_create(&bd, MACH_TYPE_OVERO);

	barebox_arm_entry(SDRAM_BASE, SDRAM_SIZE, &bd);
}

ENTRY_FUNCTION(start_omap3_overo_sram, bootinfo, r1, r2)
{
	omap3_save_bootinfo((void *)bootinfo);

	arm_cpu_lowlevel_init();

	omap3_gp_romcode_call(OMAP3_GP_ROMCODE_API_L2_INVAL, 0);

	relocate_to_current_adr();
	setup_c();

	overo_board_init();
}
