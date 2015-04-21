/*
 * Copyright (C) 2012-2014 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/pwm_generic.h>
#include <linux/fec.h>
#include <linux/memblock.h>
#include <linux/gpio.h>
#include <linux/ion.h>
#include <linux/etherdevice.h>
#include <linux/power/sabresd_battery.h>
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/wm8994/pdata.h>
#include <linux/mfd/wm8994/gpio.h>
#include <sound/wm8962.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/iomux-mx6q.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>
#include <mach/ahci_sata.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_asrc.h>
#include <mach/mipi_dsi.h>
#include <mach/mxc_ir.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"
#include "board-mx6q_uib.h"
#include "board-mx6dl_uib.h"
#include <mach/imx_rfkill.h>


#define UIB_LVDS_EN			IMX_GPIO_NR(7, 12)
#ifdef CONFIG_MX6DL_UIB_REV_1
 #define UIB_LCD_CNTRL_VGH	IMX_GPIO_NR(2, 10)
#else
 #define UIB_LCD_PWR_EN IMX_GPIO_NR(3, 20)
 #define UIB_LCD_STBYB IMX_GPIO_NR(3, 25)
 #define UIB_LCD_RESET IMX_GPIO_NR(3, 27)
#endif

#define UIB_LED0	IMX_GPIO_NR(1, 2)
#define UIB_LED1	IMX_GPIO_NR(1, 5)
#define UIB_LED2	IMX_GPIO_NR(1, 7)
#define UIB_LED3	IMX_GPIO_NR(1, 8)

#define UIB_TOUCH_IRQ    IMX_GPIO_NR(3, 23)
#define UIB_TOUCH_RESET  IMX_GPIO_NR(3, 24)

#define UIB_RFID_CS0    IMX_GPIO_NR(5, 17)
#define UIB_RFID_EN     IMX_GPIO_NR(3, 30)
#define UIB_RFID_EN2    -1
#define UIB_RFID_IRQ    IMX_GPIO_NR(3, 31)
#define UIB_RFID_MOD    IMX_GPIO_NR(4, 29)
#define UIB_RFID_ASKOOK IMX_GPIO_NR(4, 30)

#define SABRESD_POWER_OFF	IMX_GPIO_NR(3, 8)
#define SABRESD_INFO		IMX_GPIO_NR(3, 9)

#define SABRESD_BT_RESET	IMX_GPIO_NR(1, 2)
#define SABRESD_MICROPHONE_DET	IMX_GPIO_NR(1, 9)
#define SABRESD_CSI0_PWN	IMX_GPIO_NR(1, 16)
#define SABRESD_CSI0_RST	IMX_GPIO_NR(1, 17)
#define SABRESD_ACCL_INT	IMX_GPIO_NR(1, 18)
#define SABRESD_MIPICSI_PWN	IMX_GPIO_NR(1, 19)
#define SABRESD_MIPICSI_RST	IMX_GPIO_NR(1, 20)
#define SABRESD_RGMII_RST	IMX_GPIO_NR(1, 25)
#define SABRESD_RGMII_INT	IMX_GPIO_NR(1, 26)
#define SABRESD_CHARGE_UOK_B	IMX_GPIO_NR(1, 27)
#define SABRESD_USBH1_PWR_EN	IMX_GPIO_NR(1, 29)
#define SABRESD_DISP0_PWR_EN	IMX_GPIO_NR(1, 30)

#define SABRESD_CHARGE_DOK_B	IMX_GPIO_NR(2, 24)
#define SABRESD_GPS_RESET	IMX_GPIO_NR(2, 28)
#define SABRESD_SENSOR_EN	IMX_GPIO_NR(2, 31)

#define SABRESD_PCIE_PWR_EN	IMX_GPIO_NR(3, 19)
#define SABRESD_USB_OTG_PWR_N	IMX_GPIO_NR(3, 22)
#define SABRESD_USB_H1_PWR	IMX_GPIO_NR(1, 29)

#define SABRESD_CAN1_STBY	IMX_GPIO_NR(4, 5)
#define SABRESD_CODEC_PWR_EN	IMX_GPIO_NR(4, 10)
#define SABRESD_HDMI_CEC_IN	IMX_GPIO_NR(4, 11)
#define SABRESD_PCIE_DIS_B	IMX_GPIO_NR(4, 14)

#define SABRESD_DI0_D0_CS	IMX_GPIO_NR(5, 0)
#define SABRESD_CHARGE_FLT_1_B	IMX_GPIO_NR(5, 2)
#define SABRESD_PCIE_WAKE_B	IMX_GPIO_NR(5, 20)

#define SABRESD_DISP_RST_B	IMX_GPIO_NR(6, 11)
#define SABRESD_DISP_PWR_EN	IMX_GPIO_NR(6, 14)
#define SABRESD_CABC_EN0	IMX_GPIO_NR(6, 15)
#define SABRESD_CABC_EN1	IMX_GPIO_NR(6, 16)
#define SABRESD_AUX_3V15_EN	IMX_GPIO_NR(6, 9)
#define SABRESD_DISP0_WR_REVB	IMX_GPIO_NR(6, 9)
#define SABRESD_DI1_D0_CS	IMX_GPIO_NR(6, 31)

#define SABRESD_HEADPHONE_DET	IMX_GPIO_NR(7, 8)
#define SABRESD_PCIE_RST_B_REVB	IMX_GPIO_NR(7, 12)
#define SABRESD_PMIC_INT_B	IMX_GPIO_NR(7, 13)
#define SABRESD_PFUZE_INT	IMX_GPIO_NR(7, 13)

#define MX6_ENET_IRQ		IMX_GPIO_NR(1, 6)
#define IOMUX_OBSRV_MUX1_OFFSET	0x3c
#define OBSRV_MUX1_MASK			0x3f
#define OBSRV_MUX1_ENET_IRQ		0x9

extern char *gp_reg_id;
extern char *soc_reg_id;
extern char *pu_reg_id;
extern int epdc_enabled;

/* eMMC */
static const struct esdhc_platform_data mx6q_sabresd_emmc_data __initconst = {
	.always_present = 1,
	.keep_power_at_suspend = 1,
	.support_8bit = 1,
	.delay_line = 0,
	.cd_type = ESDHC_CD_PERMANENT,
};

static const struct esdhc_platform_data mx6q_sabresd_sd1_data __initconst = {
	.always_present = 0,
	.keep_power_at_suspend = 1,
	.delay_line = 0,
	.cd_type = ESDHC_CD_CONTROLLER,
	.runtime_pm = 1,
};

static const struct gpmi_nand_platform_data
mx6q_gpmi_nand_platform_data __initconst = {
	.min_prop_delay_in_ns    = 5,
	.max_prop_delay_in_ns    = 9,
	.max_chip_count          = 2,
};

static const struct anatop_thermal_platform_data
	mx6q_sabresd_anatop_thermal_data __initconst = {
		.name = "anatop_thermal",
};

static inline void mx6q_sabresd_init_uart(void)
{
	imx6q_add_imx_uart(2, NULL);
	imx6q_add_imx_uart(0, NULL);
}

#ifdef NOTYETFEC
static int mx6q_sabresd_fec_phy_init(struct phy_device *phydev)
{
	unsigned short val;

	/* Ar8031 phy SmartEEE feature cause link status generates glitch,
	 * which cause ethernet link down/up issue, so disable SmartEEE
	 */
	phy_write(phydev, 0xd, 0x3);
	phy_write(phydev, 0xe, 0x805d);
	phy_write(phydev, 0xd, 0x4003);
	val = phy_read(phydev, 0xe);
	val &= ~(0x1 << 8);
	phy_write(phydev, 0xe, val);

	/* To enable AR8031 ouput a 125MHz clk from CLK_25M */
	phy_write(phydev, 0xd, 0x7);
	phy_write(phydev, 0xe, 0x8016);
	phy_write(phydev, 0xd, 0x4007);
	val = phy_read(phydev, 0xe);

	val &= 0xffe3;
	val |= 0x18;
	phy_write(phydev, 0xe, val);

	/* Introduce tx clock delay */
	phy_write(phydev, 0x1d, 0x5);
	val = phy_read(phydev, 0x1e);
	val |= 0x0100;
	phy_write(phydev, 0x1e, val);

	/*check phy power*/
	val = phy_read(phydev, 0x0);

	if (val & BMCR_PDOWN)
		phy_write(phydev, 0x0, (val & ~BMCR_PDOWN));

	return 0;
}

static struct fec_platform_data fec_data __initdata = {
	.init = mx6q_sabresd_fec_phy_init,
	.phy = PHY_INTERFACE_MODE_RGMII,
	.gpio_irq = MX6_ENET_IRQ,
};
#endif

static int mx6q_uib_spi_cs[] = {
	UIB_RFID_CS0,
};

static const struct spi_imx_master mx6q_uib_spi_data __initconst = {
	.chipselect     = mx6q_uib_spi_cs,
	.num_chipselect = ARRAY_SIZE(mx6q_uib_spi_cs),
};

static struct spi_board_info imx6_uib_spi_nfc_device[] __initdata = {
	{
		.modalias = "trf7970a",
		.max_speed_hz = 2000000,	/* max spi clock speed (2MHz) */
		.bus_num = 0,
		.chip_select = 0,
		.irq = gpio_to_irq(UIB_RFID_IRQ),
	},
};

static void spi_device_init(void)
{
	/* pull MOD and ASK/OOK high */
	gpio_request(UIB_RFID_MOD, "rfid-mod");
	gpio_direction_output(UIB_RFID_MOD, 1);
	gpio_request(UIB_RFID_ASKOOK, "rfid-askook");
	gpio_direction_output(UIB_RFID_ASKOOK, 1);

	/* enable RFID */
	gpio_request(UIB_RFID_EN, "rfid-en");
	gpio_direction_output(UIB_RFID_EN, 1);

	/* register driver */
	spi_register_board_info(imx6_uib_spi_nfc_device,
				ARRAY_SIZE(imx6_uib_spi_nfc_device));
}

static struct imxi2c_platform_data mx6q_sabresd_i2c_data = {
	.bitrate = 100000,
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
#ifdef SABRE
	{
		I2C_BOARD_INFO("mxc_hdmi_i2c", 0x50),
	},
#endif
};

static unsigned int SSD_platform_data[1] = {UIB_TOUCH_RESET};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("ssd2543", 0x48),
		.irq = gpio_to_irq(UIB_TOUCH_IRQ),
		.platform_data = SSD_platform_data,
	},
};

#ifndef CONFIG_MX6DL_UIB_REV_1
static void imx6q_sabresd_host1_vbus(bool on)
{
	if (on)
		gpio_set_value(SABRESD_USB_H1_PWR, 1);
	else
		gpio_set_value(SABRESD_USB_H1_PWR, 0);
}
#endif

static void __init imx6q_sabresd_init_usb(void)
{
	int ret = 0;

	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);
	/* disable external charger detect,
	 * or it will affect signal quality at dp .
	 */
	ret = gpio_request(SABRESD_USB_OTG_PWR_N, "usb-pwr");
	if (ret) {
		pr_err("failed to get GPIO SABRESD_USB_OTG_PWR: %d\n",
			ret);
		return;
	}
	// force off, no OTG on UIB
	gpio_direction_output(SABRESD_USB_OTG_PWR_N, 1);

#ifndef CONFIG_MX6DL_UIB_REV_1
	/* keep USB host1 VBUS always on */
	ret = gpio_request(SABRESD_USB_H1_PWR, "usb-h1-pwr");
	if (ret) {
		pr_err("failed to get GPIO SABRESD_USB_H1_PWR: %d\n",
			ret);
		return;
	}
	gpio_direction_output(SABRESD_USB_H1_PWR, 0);
	mx6_set_host1_vbus_func(imx6q_sabresd_host1_vbus);
#endif

#ifdef SABRE
	if (board_is_mx6_reva())
		mxc_iomux_set_gpr_register(1, 13, 1, 1);
	else
		mxc_iomux_set_gpr_register(1, 13, 1, 0);

	mx6_set_otghost_vbus_func(imx6q_sabresd_usbotg_vbus);
#endif
}

#ifdef SABRE
static struct clk *sata_clk;

/* HW Initialization, if return 0, initialization is successful. */
static int mx6q_sabresd_sata_init(struct device *dev, void __iomem *addr)
{
	u32 tmpdata;
	int ret = 0;
	struct clk *clk;

	sata_clk = clk_get(dev, "imx_sata_clk");
	if (IS_ERR(sata_clk)) {
		dev_err(dev, "no sata clock.\n");
		return PTR_ERR(sata_clk);
	}
	ret = clk_enable(sata_clk);
	if (ret) {
		dev_err(dev, "can't enable sata clock.\n");
		goto put_sata_clk;
	}

	/* Set PHY Paremeters, two steps to configure the GPR13,
	 * one write for rest of parameters, mask of first write is 0x07FFFFFD,
	 * and the other one write for setting the mpll_clk_off_b
	 *.rx_eq_val_0(iomuxc_gpr13[26:24]),
	 *.los_lvl(iomuxc_gpr13[23:19]),
	 *.rx_dpll_mode_0(iomuxc_gpr13[18:16]),
	 *.sata_speed(iomuxc_gpr13[15]),
	 *.mpll_ss_en(iomuxc_gpr13[14]),
	 *.tx_atten_0(iomuxc_gpr13[13:11]),
	 *.tx_boost_0(iomuxc_gpr13[10:7]),
	 *.tx_lvl(iomuxc_gpr13[6:2]),
	 *.mpll_ck_off(iomuxc_gpr13[1]),
	 *.tx_edgerate_0(iomuxc_gpr13[0]),
	 */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x07FFFFFD) | 0x0593A044), IOMUXC_GPR13);

	/* enable SATA_PHY PLL */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x2) | 0x2), IOMUXC_GPR13);

	/* Get the AHB clock rate, and configure the TIMER1MS reg later */
	clk = clk_get(NULL, "ahb");
	if (IS_ERR(clk)) {
		dev_err(dev, "no ahb clock.\n");
		ret = PTR_ERR(clk);
		goto release_sata_clk;
	}
	tmpdata = clk_get_rate(clk) / 1000;
	clk_put(clk);

#ifdef CONFIG_SATA_AHCI_PLATFORM
	ret = sata_init(addr, tmpdata);
	if (ret == 0)
		return ret;
#else
	usleep_range(1000, 2000);
	/* AHCI PHY enter into PDDQ mode if the AHCI module is not enabled */
	tmpdata = readl(addr + PORT_PHY_CTL);
	writel(tmpdata | PORT_PHY_CTL_PDDQ_LOC, addr + PORT_PHY_CTL);
	pr_info("No AHCI save PWR: PDDQ %s\n", ((readl(addr + PORT_PHY_CTL)
					>> 20) & 1) ? "enabled" : "disabled");
#endif

release_sata_clk:
	/* disable SATA_PHY PLL */
	writel((readl(IOMUXC_GPR13) & ~0x2), IOMUXC_GPR13);
	clk_disable(sata_clk);
put_sata_clk:
	clk_put(sata_clk);

	return ret;
}

#ifdef CONFIG_SATA_AHCI_PLATFORM
static void mx6q_sabresd_sata_exit(struct device *dev)
{
	clk_disable(sata_clk);
	clk_put(sata_clk);
}

static struct ahci_platform_data mx6q_sabresd_sata_data = {
	.init = mx6q_sabresd_sata_init,
	.exit = mx6q_sabresd_sata_exit,
};
#endif

static void mx6q_sabresd_flexcan0_switch(int enable)
{
	if (enable) {
		gpio_set_value(SABRESD_CAN1_STBY, 1);
	} else {
		gpio_set_value(SABRESD_CAN1_STBY, 0);
	}
}

static const struct flexcan_platform_data
	mx6q_sabresd_flexcan0_pdata __initconst = {
	.transceiver_switch = mx6q_sabresd_flexcan0_switch,
};

static struct imx_asrc_platform_data imx_asrc_data = {
	.channel_bits = 4,
	.clk_map_ver = 2,
};

static void mx6_reset_mipi_dsi(void)
{
	gpio_set_value(SABRESD_DISP_PWR_EN, 1);
	gpio_set_value(SABRESD_DISP_RST_B, 1);
	udelay(10);
	gpio_set_value(SABRESD_DISP_RST_B, 0);
	udelay(50);
	gpio_set_value(SABRESD_DISP_RST_B, 1);

	/*
	 * it needs to delay 120ms minimum for reset complete
	 */
	msleep(120);
}

static struct mipi_dsi_platform_data mipi_dsi_pdata = {
	.ipu_id		= 0,
	.disp_id	= 1,
	.lcd_panel	= "TRULY-WVGA",
	.reset		= mx6_reset_mipi_dsi,
};
#endif /* SABRE */

static struct viv_gpu_platform_data imx6q_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_128M + SZ_64M - SZ_16M,
};

static struct ipuv3_fb_platform_data sabresd_fb_data[] = {
	{ /*fb0*/
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-XGA",
	.default_bpp = 16,
	.int_clk = false,
	.late_init = false,
	}, {
	.disp_dev = "hdmi",
	.interface_pix_fmt = IPU_PIX_FMT_RGB24,
	.mode_str = "1920x1080M@60",
	.default_bpp = 32,
	.int_clk = false,
	.late_init = false,
	}, {
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-XGA",
	.default_bpp = 16,
	.int_clk = false,
	.late_init = false,
	},
};

static struct fsl_mxc_lcd_platform_data lcdif_data = {
	.ipu_id = 0,
	.disp_id = 0,
	.default_ifmt = IPU_PIX_FMT_RGB565,
};

static struct fsl_mxc_ldb_platform_data ldb_data = {
	.ipu_id = 0,
	.disp_id = 1,
	.ext_ref = 1,
	.mode = LDB_SEP1,
	.sec_ipu_id = 0,
	.sec_disp_id = 0,
};

#ifdef SABRE
static struct max8903_pdata charger1_data = {
	.dok = SABRESD_CHARGE_DOK_B,
	.uok = SABRESD_CHARGE_UOK_B,
	.chg = SABRESD_CHARGE_CHG_1_B,
	.flt = SABRESD_CHARGE_FLT_1_B,
	.dcm_always_high = true,
	.dc_valid = true,
	.usb_valid = true,
};

static struct platform_device sabresd_max8903_charger_1 = {
	.name	= "max8903-charger",
	.id	= 1,
	.dev	= {
		.platform_data = &charger1_data,
	},
};
#endif

static struct imx_ipuv3_platform_data ipu_data[] = {
	{
	.rev = 4,
	.csi_clk[0] = "clko_clk",
	.bypass_reset = false,
	}, {
	.rev = 4,
	.csi_clk[0] = "clko_clk",
	.bypass_reset = false,
	},
};

static struct ion_platform_data imx_ion_data = {
	.nr = 1,
	.heaps = {
		{
		.id = 0,
		.type = ION_HEAP_TYPE_CARVEOUT,
		.name = "vpu_ion",
		.size = SZ_16M,
		.cacheable = 0,
		},
	},
};

#ifdef SABRE
static struct fsl_mxc_capture_platform_data capture_data[] = {
	{
		.csi = 0,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 0,
	}, {
		.csi = 1,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 1,
	},
};

static void mx6q_sd_bt_reset(void)
{
	printk(KERN_INFO "mx6q_sd_bt_reset");
	gpio_request(SABRESD_BT_RESET, "bt-reset");
	gpio_direction_output(SABRESD_BT_RESET, 0);
	/* pull down reset pin at least >5ms */
	mdelay(6);
	/* pull up after power supply BT */
	gpio_direction_output(SABRESD_BT_RESET, 1);
	gpio_free(SABRESD_BT_RESET);
	msleep(100);
}

static int mx6q_sd_bt_power_change(int status)
{
	if (status)
		mx6q_sd_bt_reset();
	return 0;
}

static struct platform_device mxc_bt_rfkill = {
	.name = "mxc_bt_rfkill",
};

static struct imx_bt_rfkill_platform_data mxc_bt_rfkill_data = {
	.power_change = mx6q_sd_bt_power_change,
};
#endif /* SABRE */

struct imx_vout_mem {
	resource_size_t res_mbase;
	resource_size_t res_msize;
};

static struct imx_vout_mem vout_mem __initdata = {
	.res_msize = 0,
};

static void sabresd_suspend_enter(void)
{
	/* suspend preparation */
	/* disable RFID */
	gpio_set_value(UIB_RFID_EN, 0);

	/* disable LCD */
	gpio_set_value(UIB_LVDS_EN, 0);
	gpio_set_value(UIB_LCD_PWR_EN, 0);
	gpio_set_value(UIB_LCD_STBYB, 0);
}

static void sabresd_suspend_exit(void)
{
	/* resume restore */
	/* enable RFID */
	gpio_set_value(UIB_RFID_EN, 1);

	/* enable LCD */
	gpio_set_value(UIB_LVDS_EN, 1);
	gpio_set_value(UIB_LCD_PWR_EN, 1);
	gpio_set_value(UIB_LCD_STBYB, 1);
	/* reset LCD */
	gpio_set_value(UIB_LCD_RESET, 0);
	mdelay(1);
	gpio_set_value(UIB_LCD_RESET, 1);
}
static const struct pm_platform_data mx6q_sabresd_pm_data __initconst = {
	.name = "imx_pm",
	.suspend_enter = sabresd_suspend_enter,
	.suspend_exit = sabresd_suspend_exit,
};

static struct regulator_consumer_supply sabresd_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.0"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.1"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
};

static struct regulator_init_data sabresd_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(sabresd_vmmc_consumers),
	.consumer_supplies = sabresd_vmmc_consumers,
};

static struct fixed_voltage_config sabresd_vmmc_reg_config = {
	.supply_name		= "vmmc",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &sabresd_vmmc_init,
};

static struct platform_device sabresd_vmmc_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 3,
	.dev	= {
		.platform_data = &sabresd_vmmc_reg_config,
	},
};

#ifdef SABRE
static int __init imx6q_init_audio(void)
{
	if (board_is_mx6_reva()) {
		mxc_register_device(&mx6_sabresd_audio_wm8958_device,
				    &wm8958_data);
		imx6q_add_imx_ssi(1, &mx6_sabresd_ssi_pdata);

		mxc_wm8958_init();
	} else {
		platform_device_register(&sabresd_vwm8962_reg_devices);
		mxc_register_device(&mx6_sabresd_audio_wm8962_device,
				    &wm8962_data);
		imx6q_add_imx_ssi(1, &mx6_sabresd_ssi_pdata);

		mxc_wm8962_init();
	}

	return 0;
}

static void gps_power_on(bool on)
{
	/* Enable/disable aux_3v15 */
	gpio_request(SABRESD_AUX_3V15_EN, "aux_3v15_en");
	gpio_direction_output(SABRESD_AUX_3V15_EN, 1);
	gpio_set_value(SABRESD_AUX_3V15_EN, on);
	gpio_free(SABRESD_AUX_3V15_EN);
	/*Enable/disable gps_en*/
	gpio_request(SABRESD_GPS_EN, "gps_en");
	gpio_direction_output(SABRESD_GPS_EN, 1);
	gpio_set_value(SABRESD_GPS_EN, on);
	gpio_free(SABRESD_GPS_EN);

}
#endif /* SABRE */

#if defined(CONFIG_LEDS_TRIGGER) || defined(CONFIG_LEDS_GPIO)

#define GPIO_LED(gpio_led, name_led, act_low, state_suspend, trigger)	\
{									\
	.gpio			= gpio_led,				\
	.name			= name_led,				\
	.active_low		= act_low,				\
	.retain_state_suspended = state_suspend,			\
	.default_state		= 0,					\
	.default_trigger	= trigger,		\
}

static struct gpio_led imx6q_gpio_leds[] = {
	/* set 4th argument to 1 to keep LED on during suspend */
	GPIO_LED(UIB_LED0, "led0", 0, 0, ""),
	GPIO_LED(UIB_LED1, "led1", 0, 0, ""),
	GPIO_LED(UIB_LED2, "led2", 0, 0, ""),
	GPIO_LED(UIB_LED3, "led3", 0, 0, ""),
};

static struct gpio_led_platform_data imx6q_gpio_leds_data = {
	.leds		= imx6q_gpio_leds,
	.num_leds	= ARRAY_SIZE(imx6q_gpio_leds),
};

static struct platform_device imx6q_gpio_led_device = {
	.name		= "leds-gpio",
	.id		= -1,
	.num_resources  = 0,
	.dev		= {
		.platform_data = &imx6q_gpio_leds_data,
	}
};

static void __init imx6q_add_device_gpio_leds(void)
{
	platform_device_register(&imx6q_gpio_led_device);
}
#else
static void __init imx6q_add_device_gpio_leds(void) {}
#endif

#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
#define GPIO_BUTTON(gpio_num, ev_code, act_low, descr, wake, debounce)	\
{								\
	.gpio		= gpio_num,				\
	.type		= EV_KEY,				\
	.code		= ev_code,				\
	.active_low	= act_low,				\
	.desc		= "btn " descr,				\
	.wakeup		= wake,					\
	.debounce_interval = debounce,				\
}

static struct gpio_keys_button sabresd_buttons[] = {
	GPIO_BUTTON(SABRESD_POWER_OFF, KEY_POWER, 0, "power-key", 1, 1),
	GPIO_BUTTON(SABRESD_INFO, KEY_INFO, 0, "info-key", 1, 1),
};

static struct gpio_keys_platform_data sabresd_button_data = {
	.buttons	= sabresd_buttons,
	.nbuttons	= ARRAY_SIZE(sabresd_buttons),
};

static struct platform_device sabresd_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources  = 0,
	.dev	= {
		.platform_data = &sabresd_button_data,
	},
};

static void __init imx6q_add_device_buttons(void)
{
	platform_device_register(&sabresd_button_device);
}
#else
static void __init imx6q_add_device_buttons(void) {}
#endif

static struct platform_pwm_backlight_data mx6_sabresd_pwm_backlight_data = {
	.pwm_id = 0,
	.max_brightness = 255,
	.dft_brightness = 128,
	.pwm_period_ns = 50000,
};

static struct platform_pwm_generic_data mx6_sabresd_pwm_speaker = {
	.pwm_id = 2,
};

#ifdef SABRE
static struct platform_ir_data mx6_sabresd_ir_data = {
    .pwm_id = 1,
    .epit_id = 0,
    .gpio_id = 0,
};
#endif

static struct mxc_dvfs_platform_data sabresd_dvfscore_data = {
	.reg_id = "VDDCORE",
	.soc_id	= "VDDSOC",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 80,
};

static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
	char *str;
	struct tag *t;
	int i = 0;
	struct ipuv3_fb_platform_data *pdata_fb = sabresd_fb_data;

	for_each_tag(t, tags) {
		if (t->hdr.tag == ATAG_CMDLINE) {
			str = t->u.cmdline.cmdline;
			str = strstr(str, "fbmem=");
			if (str != NULL) {
				str += 6;
				pdata_fb[i++].res_size[0] = memparse(str, &str);
				while (*str == ',' &&
					i < ARRAY_SIZE(sabresd_fb_data)) {
					str++;
					pdata_fb[i++].res_size[0] = memparse(str, &str);
				}
			}
			/* ION reserved memory */
			str = t->u.cmdline.cmdline;
			str = strstr(str, "ionmem=");
			if (str != NULL) {
				str += 7;
				imx_ion_data.heaps[0].size = memparse(str, &str);
			}
			/* Primary framebuffer base address */
			str = t->u.cmdline.cmdline;
			str = strstr(str, "fb0base=");
			if (str != NULL) {
				str += 8;
				pdata_fb[0].res_base[0] =
						simple_strtol(str, &str, 16);
			}
			/* GPU reserved memory */
			str = t->u.cmdline.cmdline;
			str = strstr(str, "gpumem=");
			if (str != NULL) {
				str += 7;
				imx6q_gpu_pdata.reserved_mem_size = memparse(str, &str);
			}
			break;
		}
	}
}

#ifdef SABRE
static struct mipi_csi2_platform_data mipi_csi2_pdata = {
	.ipu_id	 = 0,
	.csi_id = 1,
	.v_channel = 0,
	.lanes = 2,
	.dphy_clk = "mipi_pllref_clk",
	.pixel_clk = "emi_clk",
};

static int __init caam_setup(char *__unused)
{
	caam_enabled = 1;
	return 1;
}
early_param("caam", caam_setup);

static const struct imx_pcie_platform_data mx6_sabresd_pcie_data __initconst = {
	.pcie_pwr_en	= SABRESD_PCIE_PWR_EN,
	.pcie_rst	= SABRESD_PCIE_RST_B_REVB,
	.pcie_wake_up	= SABRESD_PCIE_WAKE_B,
	.pcie_dis	= SABRESD_PCIE_DIS_B,
#ifdef CONFIG_IMX_PCIE_EP_MODE_IN_EP_RC_SYS
	.type_ep	= 1,
#else
	.type_ep	= 0,
#endif
	.pcie_power_always_on = 1,
};
#endif // SABRE

#define SNVS_LPCR 0x38
static void mx6_snvs_poweroff(void)
{

	void __iomem *mx6_snvs_base =  MX6_IO_ADDRESS(MX6Q_SNVS_BASE_ADDR);
	u32 value;
	value = readl(mx6_snvs_base + SNVS_LPCR);
	/*set TOP and DP_EN bit*/
	writel(value | 0x60, mx6_snvs_base + SNVS_LPCR);
}

#ifdef CONFIG_ANDROID_RAM_CONSOLE
static struct resource ram_console_resource = {
	.name = "android ram console",
	.flags = IORESOURCE_MEM,
};

static struct platform_device android_ram_console = {
	.name = "ram_console",
	.num_resources = 1,
	.resource = &ram_console_resource,
};

static int __init imx6x_add_ram_console(void)
{
	return platform_device_register(&android_ram_console);
}
#else
#define imx6x_add_ram_console() do {} while (0)
#endif

/*!
 * Board specific initialization.
 */
static void __init mx6_sabresd_board_init(void)
{
	int i;
	struct clk *clko, *clko2;
	struct clk *new_parent;
	int rate;
	struct platform_device *voutdev;

	gp_reg_id = "vddarmsoc_ext";

	// set up external pads
	if (cpu_is_mx6q()) {
		mxc_iomux_v3_setup_multiple_pads(mx6q_sabresd_pads,
			ARRAY_SIZE(mx6q_sabresd_pads));
	} else if (cpu_is_mx6dl()) {
		mxc_iomux_v3_setup_multiple_pads(mx6dl_sabresd_pads,
			ARRAY_SIZE(mx6dl_sabresd_pads));

#ifdef CONFIG_MX6DL_UIB_REV_1
		// drive PMIC standby pad low
		// @@@ note: this is changing to the iMX6 PMIC_STBY_REQ pin in Rev 2
#define PMIC_STBY IMX_GPIO_NR(3, 16)
		mxc_iomux_v3_setup_pad(MX6DL_PAD_EIM_D16__GPIO_3_16);
		gpio_request(PMIC_STBY, "");
		gpio_direction_output(PMIC_STBY, 0);
		// allow sysfs to modify this gpio for testing
		gpio_export(PMIC_STBY, false);

		// @@@ power down Micrel RGMII phy
#define MICREL_STBY IMX_GPIO_NR(6, 29)
		mxc_iomux_v3_setup_pad(MX6DL_PAD_RGMII_RD3__GPIO_6_29);
		gpio_request(MICREL_STBY, "");
		gpio_direction_output(MICREL_STBY, 0);
#endif
	}

	gpio_request(UIB_LVDS_EN, "LVDS_EN");
	gpio_direction_output(UIB_LVDS_EN, 1);
#ifdef CONFIG_MX6DL_UIB_REV_1
	gpio_request(UIB_LCD_CNTRL_VGH, "LCD_CNTRL_VGH");
	gpio_direction_output(UIB_LCD_CNTRL_VGH, 1);
#else
	gpio_request(UIB_LCD_PWR_EN, "LCD_PWR_EN");
	gpio_direction_output(UIB_LCD_PWR_EN, 1);
	gpio_request(UIB_LCD_STBYB, "LCD_STBYB");
	gpio_direction_output(UIB_LCD_STBYB, 1);
	gpio_request(UIB_LCD_RESET, "LCD_RESET");
	gpio_direction_output(UIB_LCD_RESET, 1);
#endif

#ifdef CONFIG_FEC_1588
	/* Set GPIO_16 input for IEEE-1588 ts_clk and RMII reference clock
	 * For MX6 GPR1 bit21 meaning:
	 * Bit21:       0 - GPIO_16 pad output
	 *              1 - GPIO_16 pad input
	 */
	 mxc_iomux_set_gpr_register(1, 21, 1, 1);
#endif

	mx6q_sabresd_init_uart();
	imx6x_add_ram_console();

	/*
	 * MX6DL/Solo only supports single IPU
	 * The following codes are used to change ipu id
	 * and display id information for MX6DL/Solo. Then
	 * register 1 IPU device and up to 2 displays for
	 * MX6DL/Solo
	 */
	if (cpu_is_mx6dl()) {
		ldb_data.ipu_id = 0;
		ldb_data.disp_id = 0;
		// mipi_dsi_pdata.ipu_id = 0;
		// mipi_dsi_pdata.disp_id = 1;
		ldb_data.sec_ipu_id = 0;
		// UIB
		ldb_data.mode = LDB_SEP0;
		sabresd_fb_data[0].mode_str = "LDB-WSVGA";
	}
	// imx6q_add_mxc_hdmi_core(&hdmi_core_data);

	imx6q_add_ipuv3(0, &ipu_data[0]);
	if (cpu_is_mx6q()) {
		imx6q_add_ipuv3(1, &ipu_data[1]);
		for (i = 0; i < 4 && i < ARRAY_SIZE(sabresd_fb_data); i++)
			imx6q_add_ipuv3fb(i, &sabresd_fb_data[i]);
	} else
		for (i = 0; i < 2 && i < ARRAY_SIZE(sabresd_fb_data); i++)
			imx6q_add_ipuv3fb(i, &sabresd_fb_data[i]);

	imx6q_add_vdoa();
	// imx6q_add_mipi_dsi(&mipi_dsi_pdata);
	imx6q_add_lcdif(&lcdif_data);
	imx6q_add_ldb(&ldb_data);
	voutdev = imx6q_add_v4l2_output(0);
	if (vout_mem.res_msize && voutdev) {
		dma_declare_coherent_memory(&voutdev->dev,
					    vout_mem.res_mbase,
					    vout_mem.res_mbase,
					    vout_mem.res_msize,
					    (DMA_MEMORY_MAP |
					     DMA_MEMORY_EXCLUSIVE));
	}

#ifdef SABRE
	imx6q_add_v4l2_capture(0, &capture_data[0]);
	imx6q_add_v4l2_capture(1, &capture_data[1]);
	imx6q_add_mipi_csi2(&mipi_csi2_pdata);

	if (1 == caam_enabled)
		imx6q_add_imx_caam();

	if (board_is_mx6_reva()) {
		strcpy(mxc_i2c0_board_info[0].type, "wm8958");
		mxc_i2c0_board_info[0].platform_data = &wm8958_config_data;
	} else {
		strcpy(mxc_i2c0_board_info[0].type, "wm8962");
		mxc_i2c0_board_info[0].platform_data = &wm8962_config_data;
	}
#endif
	imx6q_add_imx_snvs_rtc();

	imx6q_add_device_gpio_leds();

	imx6q_add_imx_i2c(0, &mx6q_sabresd_i2c_data);
	imx6q_add_imx_i2c(1, &mx6q_sabresd_i2c_data);
	imx6q_add_imx_i2c(2, &mx6q_sabresd_i2c_data);
	if (cpu_is_mx6dl())
		imx6q_add_imx_i2c(3, &mx6q_sabresd_i2c_data);

	gpio_request(UIB_TOUCH_RESET, "SSD-RESET");
	gpio_request(UIB_TOUCH_IRQ, "SSD-IRQ");

	i2c_register_board_info(0, mxc_i2c0_board_info,
			ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
			ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
			ARRAY_SIZE(mxc_i2c2_board_info));

	/* SPI */
	imx6q_add_ecspi(0, &mx6q_uib_spi_data);
	spi_device_init();

	// imx6q_add_mxc_hdmi(&hdmi_data);

	imx6q_add_anatop_thermal_imx(1, &mx6q_sabresd_anatop_thermal_data);

#ifdef SABRE
	if (enet_to_gpio_6)
		/* Make sure the IOMUX_OBSRV_MUX1 is set to ENET_IRQ. */
		mxc_iomux_set_specialbits_register(
			IOMUX_OBSRV_MUX1_OFFSET,
			OBSRV_MUX1_ENET_IRQ,
			OBSRV_MUX1_MASK);
	else
		fec_data.gpio_irq = -1;
	imx6_init_fec(fec_data);
#endif

	imx6q_add_pm_imx(0, &mx6q_sabresd_pm_data);

	/*
	   Mfgtools wants emmc as mmcblk0 and other sd card as mmcblk1.
	*/
	imx6q_add_sdhci_usdhc_imx(2, &mx6q_sabresd_emmc_data);
	imx6q_add_sdhci_usdhc_imx(1, &mx6q_sabresd_sd1_data);
	imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);
	imx6q_sabresd_init_usb();

#ifdef SABRE
	/* SATA is not supported by MX6DL/Solo */
	if (cpu_is_mx6q()) {
#ifdef CONFIG_SATA_AHCI_PLATFORM
		imx6q_add_ahci(0, &mx6q_sabresd_sata_data);
#else
		mx6q_sabresd_sata_init(NULL,
			(void __iomem *)ioremap(MX6Q_SATA_BASE_ADDR, SZ_4K));
#endif
	}
#endif
	imx6q_add_vpu();
	/* imx6q_init_audio(); */
	platform_device_register(&sabresd_vmmc_reg_devices);
	// imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	// imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	// imx6q_add_asrc(&imx_asrc_data);

#ifdef CONFIG_HAVE_EPIT
	imx6q_add_mxc_epit(0);
	imx6q_add_mxc_epit(1);
#endif

	imx6q_add_mxc_pwm(0);
	imx6q_add_mxc_pwm(1);
	imx6q_add_mxc_pwm(2);
	imx6q_add_mxc_pwm(3);
	imx6q_add_mxc_pwm_backlight(0, &mx6_sabresd_pwm_backlight_data);
	platform_device_register_resndata(NULL, "pwm-generic", 2, NULL, 0,
		&mx6_sabresd_pwm_speaker, sizeof(mx6_sabresd_pwm_speaker));

#ifdef SABRE
	/* add MXC IR device */
	imx6q_add_mxc_ir(0, &mx6_sabresd_ir_data);
#endif

	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();
	// imx6q_add_gpmi(&mx6q_gpmi_nand_platform_data);

	imx6q_add_dvfs_core(&sabresd_dvfscore_data);

	if (imx_ion_data.heaps[0].size)
		imx6q_add_ion(0, &imx_ion_data,
			sizeof(imx_ion_data) + sizeof(struct ion_platform_heap));

	imx6q_add_device_buttons();

#ifdef SABRE
	/* enable sensor 3v3 and 1v8 */
	gpio_request(SABRESD_SENSOR_EN, "sensor-en");
	gpio_direction_output(SABRESD_SENSOR_EN, 1);

	/* enable accel intr */
	gpio_request(SABRESD_ACCL_INT, "accel-int");
	gpio_direction_input(SABRESD_ACCL_INT);
	/* enable ecompass intr */
	gpio_request(SABRESD_eCOMPASS_INT, "ecompass-int");
	gpio_direction_input(SABRESD_eCOMPASS_INT);
	/* enable light sensor intr */
	gpio_request(SABRESD_ALS_INT, "als-int");
	gpio_direction_input(SABRESD_ALS_INT);
#endif

	// imx6q_add_hdmi_soc();
	// imx6q_add_hdmi_soc_dai();

	if (cpu_is_mx6dl()) {
		imx6dl_add_imx_pxp();
		imx6dl_add_imx_pxp_client();
	}

	clko2 = clk_get(NULL, "clko2_clk");
	if (IS_ERR(clko2))
		pr_err("can't get CLKO2 clock.\n");

	new_parent = clk_get(NULL, "osc_clk");
	if (!IS_ERR(new_parent)) {
		clk_set_parent(clko2, new_parent);
		clk_put(new_parent);
	}
	rate = clk_round_rate(clko2, 24000000);
	clk_set_rate(clko2, rate);
	clk_enable(clko2);

	/* Camera and audio use osc clock */
	clko = clk_get(NULL, "clko_clk");
	if (!IS_ERR(clko))
		clk_set_parent(clko, clko2);

#ifdef SABRE
	gps_power_on(true);

	/* Register charger chips */
	platform_device_register(&sabresd_max8903_charger_1);
#endif

	pm_power_off = mx6_snvs_poweroff;
	imx6q_add_busfreq();

#ifdef SABRE
	/* Add PCIe RC interface support
	 */
	imx6q_add_pcie(&mx6_sabresd_pcie_data);
	if (cpu_is_mx6dl()) {
		mxc_iomux_v3_setup_multiple_pads(mx6dl_arm2_elan_pads,
						ARRAY_SIZE(mx6dl_arm2_elan_pads));

		/* ELAN Touchscreen */
		gpio_request(SABRESD_ELAN_INT, "elan-interrupt");
		gpio_direction_input(SABRESD_ELAN_INT);

		gpio_request(SABRESD_ELAN_CE, "elan-cs");
		gpio_direction_output(SABRESD_ELAN_CE, 1);
		gpio_direction_output(SABRESD_ELAN_CE, 0);

		gpio_request(SABRESD_ELAN_RST, "elan-rst");
		gpio_direction_output(SABRESD_ELAN_RST, 1);
		gpio_direction_output(SABRESD_ELAN_RST, 0);
		mdelay(1);
		gpio_direction_output(SABRESD_ELAN_RST, 1);
		gpio_direction_output(SABRESD_ELAN_CE, 1);
	}
#endif

	imx6_add_armpmu();
	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);
}

extern void __iomem *twd_base;
static void __init mx6_sabresd_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.0", NULL);
	early_console_setup(UART1_BASE_ADDR, uart_clk);
}

static struct sys_timer mx6_sabresd_timer = {
	.init   = mx6_sabresd_timer_init,
};

static void __init mx6q_sabresd_reserve(void)
{
	phys_addr_t phys;
	int i, fb0_reserved = 0, fb_array_size;

	/*
	 * Reserve primary framebuffer memory if its base address
	 * is set by kernel command line.
	 */
	fb_array_size = ARRAY_SIZE(sabresd_fb_data);
	if (fb_array_size > 0 && sabresd_fb_data[0].res_base[0] &&
	    sabresd_fb_data[0].res_size[0]) {
		if (sabresd_fb_data[0].res_base[0] > SZ_2G)
			printk(KERN_INFO"UI Performance downgrade with FB phys address %x!\n",
			    sabresd_fb_data[0].res_base[0]);
		memblock_reserve(sabresd_fb_data[0].res_base[0],
				 sabresd_fb_data[0].res_size[0]);
		memblock_remove(sabresd_fb_data[0].res_base[0],
				sabresd_fb_data[0].res_size[0]);
		sabresd_fb_data[0].late_init = true;
		ipu_data[ldb_data.ipu_id].bypass_reset = true;
		fb0_reserved = 1;
	}
	for (i = fb0_reserved; i < fb_array_size; i++)
		if (sabresd_fb_data[i].res_size[0]) {
			/* Reserve for other background buffer. */
			phys = memblock_alloc_base(sabresd_fb_data[i].res_size[0],
						SZ_4K, SZ_2G);
			memblock_remove(phys, sabresd_fb_data[i].res_size[0]);
			sabresd_fb_data[i].res_base[0] = phys;
		}

#ifdef CONFIG_ANDROID_RAM_CONSOLE
	phys = memblock_alloc_base(SZ_1M, SZ_4K, SZ_1G);
	memblock_remove(phys, SZ_1M);
	memblock_free(phys, SZ_1M);
	ram_console_resource.start = phys;
	ram_console_resource.end   = phys + SZ_1M - 1;
#endif

#if defined(CONFIG_MXC_GPU_VIV) || defined(CONFIG_MXC_GPU_VIV_MODULE)
	if (imx6q_gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(imx6q_gpu_pdata.reserved_mem_size,
					   SZ_4K, SZ_2G);
		memblock_remove(phys, imx6q_gpu_pdata.reserved_mem_size);
		imx6q_gpu_pdata.reserved_mem_base = phys;
	}
#endif

#if defined(CONFIG_ION)
	if (imx_ion_data.heaps[0].size) {
		phys = memblock_alloc(imx_ion_data.heaps[0].size, SZ_4K);
		memblock_remove(phys, imx_ion_data.heaps[0].size);
		imx_ion_data.heaps[0].base = phys;
	}
#endif

	if (vout_mem.res_msize) {
		phys = memblock_alloc_base(vout_mem.res_msize,
					   SZ_4K, SZ_1G);
		memblock_remove(phys, vout_mem.res_msize);
		vout_mem.res_mbase = phys;
	}
}

/*
 * initialize __mach_desc_MX6Q_SABRESD data structure.
 */
MACHINE_START(MX6Q_SABRESD, "Freescale i.MX 6Quad/DualLite/Solo Sabre-SD Board")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.boot_params = MX6_PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx6_map_io,
	.init_irq = mx6_init_irq,
	.init_machine = mx6_sabresd_board_init,
	.timer = &mx6_sabresd_timer,
	.reserve = mx6q_sabresd_reserve,
MACHINE_END
