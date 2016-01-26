/*
 * (C) Copyright 2004-2011
 * Texas Instruments, <www.ti.com>
 *
 * Author :
 *	Sunil Kumar <sunilsaini05@gmail.com>
 *	Shashi Ranjan <shashiranjanmca05@gmail.com>
 *
 * Derived from Beagle Board and 3430 SDP code by
 *	Richard Woodruff <r-woodruff2@ti.com>
 *	Syed Mohammed Khasim <khasim@ti.com>
 *
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <dm.h>
#include <ns16550.h>
#ifdef CONFIG_STATUS_LED
#include <status_led.h>
#endif
#include <twl4030.h>
#include <linux/mtd/nand.h>
#include <asm/io.h>
#include <asm/arch/mmc_host_def.h>
#include <asm/arch/mux.h>
#include <asm/arch/mem.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/mach-types.h>
#include <asm/omap_musb.h>
#include <asm/errno.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/musb.h>
#include "beagle.h"
#include <command.h>

#ifdef CONFIG_USB_EHCI
#include <usb.h>
#include <asm/ehci-omap.h>
#endif

#define TWL4030_I2C_BUS			0
#define EXPANSION_EEPROM_I2C_BUS	1
#define EXPANSION_EEPROM_I2C_ADDRESS	0x50

#define TINCANTOOLS_ZIPPY		0x01000100
#define TINCANTOOLS_ZIPPY2		0x02000100
#define TINCANTOOLS_TRAINER		0x04000100
#define TINCANTOOLS_SHOWDOG		0x03000100
#define KBADC_BEAGLEFPGA		0x01000600
#define LW_BEAGLETOUCH			0x01000700
#define BRAINMUX_LCDOG			0x01000800
#define BRAINMUX_LCDOGTOUCH		0x02000800
#define BBTOYS_WIFI			0x01000B00
#define BBTOYS_VGA			0x02000B00
#define BBTOYS_LCD			0x03000B00
#define BCT_BRETTL3			0x01000F00
#define BCT_BRETTL4			0x02000F00
#define LSR_COM6L_ADPT			0x01001300
#define BEAGLE_NO_EEPROM		0xffffffff

DECLARE_GLOBAL_DATA_PTR;

static struct {
	unsigned int device_vendor;
	unsigned char revision;
	unsigned char content;
	char fab_revision[8];
	char env_var[16];
	char env_setting[64];
} expansion_config;

static const struct ns16550_platdata beagle_serial = {
	OMAP34XX_UART3,
	2,
	V_NS16550_CLK
};

U_BOOT_DEVICE(beagle_uart) = {
	"ns16550_serial",
	&beagle_serial
};

/*
 * Routine: board_init
 * Description: Early hardware init.
 */
int board_init(void)
{
	gpmc_init(); /* in SRAM or SDRAM, finish GPMC */
	/* board id for Linux */
	gd->bd->bi_arch_number = MACH_TYPE_OMAP3_BEAGLE;
	/* boot param addr */
	gd->bd->bi_boot_params = (OMAP34XX_SDRC_CS0 + 0x100);

#if defined(CONFIG_STATUS_LED) && defined(STATUS_LED_BOOT)
	status_led_set (STATUS_LED_BOOT, STATUS_LED_ON);
#endif

	return 0;
}

/*
 * Routine: get_board_revision
 * Description: Detect if we are running on a Beagle revision Ax/Bx,
 *		C1/2/3, C4, xM Ax/Bx or xM Cx. This can be done by reading
 *		the level of GPIO173, GPIO172 and GPIO171. This should
 *		result in
 *		GPIO173, GPIO172, GPIO171: 1 1 1 => Ax/Bx
 *		GPIO173, GPIO172, GPIO171: 1 1 0 => C1/2/3
 *		GPIO173, GPIO172, GPIO171: 1 0 1 => C4
 *		GPIO173, GPIO172, GPIO171: 0 1 0 => xM Cx
 *		GPIO173, GPIO172, GPIO171: 0 0 0 => xM Ax/Bx
 */
static int get_board_revision(void)
{
	static int revision = -1;

	if (revision == -1) {
		if (!gpio_request(171, "rev0") &&
		    !gpio_request(172, "rev1") &&
		    !gpio_request(173, "rev2")) {
			gpio_direction_input(171);
			gpio_direction_input(172);
			gpio_direction_input(173);	
			revision = gpio_get_value(173) << 2 | gpio_get_value(172) << 1 | gpio_get_value(171);
		} else {
			printf("Error: unable to acquire board revision GPIOs\n");
		}
	}

	return revision;
}

#ifdef CONFIG_SPL_BUILD
/*
 * Routine: get_board_mem_timings
 * Description: If we use SPL then there is no x-loader nor config header
 * so we have to setup the DDR timings ourself on both banks.
 */
void get_board_mem_timings(struct board_sdrc_timings *timings)
{
	int pop_mfr, pop_id;

	/*
	 * We need to identify what PoP memory is on the board so that
	 * we know what timings to use.  If we can't identify it then
	 * we know it's an xM.  To map the ID values please see nand_ids.c
	 */
	identify_nand_chip(&pop_mfr, &pop_id);

	timings->mr = MICRON_V_MR_165;
	switch (get_board_revision()) {
	case REVISION_C4:
		if (pop_mfr == NAND_MFR_STMICRO && pop_id == 0xba) {
			/* 512MB DDR */
			timings->mcfg = NUMONYX_V_MCFG_165(512 << 20);
			timings->ctrla = NUMONYX_V_ACTIMA_165;
			timings->ctrlb = NUMONYX_V_ACTIMB_165;
			timings->rfr_ctrl = SDP_3430_SDRC_RFR_CTRL_165MHz;
			break;
		} else if (pop_mfr == NAND_MFR_MICRON && pop_id == 0xba) {
			/* Beagleboard Rev C4, 512MB Nand/256MB DDR*/
			timings->mcfg = MICRON_V_MCFG_165(128 << 20);
			timings->ctrla = MICRON_V_ACTIMA_165;
			timings->ctrlb = MICRON_V_ACTIMB_165;
			timings->rfr_ctrl = SDP_3430_SDRC_RFR_CTRL_165MHz;
			break;
		} else if (pop_mfr == NAND_MFR_MICRON && pop_id == 0xbc) {
			/* Beagleboard Rev C5, 256MB DDR */
			timings->mcfg = MICRON_V_MCFG_200(256 << 20);
			timings->ctrla = MICRON_V_ACTIMA_200;
			timings->ctrlb = MICRON_V_ACTIMB_200;
			timings->rfr_ctrl = SDP_3430_SDRC_RFR_CTRL_200MHz;
			break;
		}
	case REVISION_XM_AB:
	case REVISION_XM_C:
		if (pop_mfr == 0) {
			/* 256MB DDR */
			timings->mcfg = MICRON_V_MCFG_200(256 << 20);
			timings->ctrla = MICRON_V_ACTIMA_200;
			timings->ctrlb = MICRON_V_ACTIMB_200;
			timings->rfr_ctrl = SDP_3430_SDRC_RFR_CTRL_200MHz;
		} else {
			/* 512MB DDR */
			timings->mcfg = NUMONYX_V_MCFG_165(512 << 20);
			timings->ctrla = NUMONYX_V_ACTIMA_165;
			timings->ctrlb = NUMONYX_V_ACTIMB_165;
			timings->rfr_ctrl = SDP_3430_SDRC_RFR_CTRL_165MHz;
		}
		break;
	default:
		/* Assume 128MB and Micron/165MHz timings to be safe */
		timings->mcfg = MICRON_V_MCFG_165(128 << 20);
		timings->ctrla = MICRON_V_ACTIMA_165;
		timings->ctrlb = MICRON_V_ACTIMB_165;
		timings->rfr_ctrl = SDP_3430_SDRC_RFR_CTRL_165MHz;
	}
}
#endif

/*
 * Routine: get_expansion_id
 * Description: This function checks for expansion board by checking I2C
 *		bus 1 for the availability of an AT24C01B serial EEPROM.
 *		returns the device_vendor field from the EEPROM
 */
static unsigned int get_expansion_id(void)
{
	i2c_set_bus_num(EXPANSION_EEPROM_I2C_BUS);

	/* return BEAGLE_NO_EEPROM if eeprom doesn't respond */
	if (i2c_probe(EXPANSION_EEPROM_I2C_ADDRESS) == 1) {
		i2c_set_bus_num(TWL4030_I2C_BUS);
		return BEAGLE_NO_EEPROM;
	}

	/* read configuration data */
	i2c_read(EXPANSION_EEPROM_I2C_ADDRESS, 0, 1, (u8 *)&expansion_config,
		 sizeof(expansion_config));

	/* retry reading configuration data with 16bit addressing */
	if ((expansion_config.device_vendor == 0xFFFFFF00) ||
	    (expansion_config.device_vendor == 0xFFFFFFFF)) {
		printf("EEPROM is blank or 8bit addressing failed: retrying with 16bit:\n");
		i2c_read(EXPANSION_EEPROM_I2C_ADDRESS, 0, 2, (u8 *)&expansion_config,
			 sizeof(expansion_config));
	}

	i2c_set_bus_num(TWL4030_I2C_BUS);

	return expansion_config.device_vendor;
}

#ifdef CONFIG_VIDEO_OMAP3
/*
 * Configure DSS to display background color on DVID
 * Configure VENC to display color bar on S-Video
 */
static void beagle_display_init(void)
{
	omap3_dss_venc_config(&venc_config_std_tv, VENC_HEIGHT, VENC_WIDTH);
	switch (get_board_revision()) {
	case REVISION_AXBX:
	case REVISION_CX:
	case REVISION_C4:
		omap3_dss_panel_config(&dvid_cfg);
		break;
	case REVISION_XM_AB:
	case REVISION_XM_C:
	default:
		omap3_dss_panel_config(&dvid_cfg_xm);
		break;
	}
}

/*
 * Enable DVI power
 */
static void beagle_dvi_pup(void)
{
	uchar val;

	switch (get_board_revision()) {
	case REVISION_AXBX:
	case REVISION_CX:
	case REVISION_C4:
		gpio_request(170, "dvi");
		gpio_direction_output(170, 0);
		gpio_set_value(170, 1);
		break;
	case REVISION_XM_AB:
	case REVISION_XM_C:
	default:
		#define GPIODATADIR1 (TWL4030_BASEADD_GPIO+3)
		#define GPIODATAOUT1 (TWL4030_BASEADD_GPIO+6)

		i2c_read(TWL4030_CHIP_GPIO, GPIODATADIR1, 1, &val, 1);
		val |= 4;
		i2c_write(TWL4030_CHIP_GPIO, GPIODATADIR1, 1, &val, 1);

		i2c_read(TWL4030_CHIP_GPIO, GPIODATAOUT1, 1, &val, 1);
		val |= 4;
		i2c_write(TWL4030_CHIP_GPIO, GPIODATAOUT1, 1, &val, 1);
		break;
	}
}
#endif

#ifdef CONFIG_USB_MUSB_OMAP2PLUS
static struct musb_hdrc_config musb_config = {
	.multipoint     = 1,
	.dyn_fifo       = 1,
	.num_eps        = 16,
	.ram_bits       = 12,
};

static struct omap_musb_board_data musb_board_data = {
	.interface_type	= MUSB_INTERFACE_ULPI,
};

static struct musb_hdrc_platform_data musb_plat = {
#if defined(CONFIG_USB_MUSB_HOST)
	.mode           = MUSB_HOST,
#elif defined(CONFIG_USB_MUSB_GADGET)
	.mode		= MUSB_PERIPHERAL,
#else
#error "Please define either CONFIG_USB_MUSB_HOST or CONFIG_USB_MUSB_GADGET"
#endif
	.config         = &musb_config,
	.power          = 100,
	.platform_ops	= &omap2430_ops,
	.board_data	= &musb_board_data,
};
#endif

/*
 * Routine: misc_init_r
 * Description: Configure board specific parts
 */
int misc_init_r(void)
{
	struct gpio *gpio5_base = (struct gpio *)OMAP34XX_GPIO5_BASE;
	struct gpio *gpio6_base = (struct gpio *)OMAP34XX_GPIO6_BASE;
	struct control_prog_io *prog_io_base = (struct control_prog_io *)OMAP34XX_CTRL_BASE;
	bool generate_fake_mac = false;
	u32 value;

	/* Enable i2c2 pullup resisters */
	value = readl(&prog_io_base->io1);
	value &= ~(PRG_I2C2_PULLUPRESX);
	writel(value, &prog_io_base->io1);

	switch (get_board_revision()) {
	case REVISION_AXBX:
		printf("Beagle Rev Ax/Bx\n");
		setenv("beaglerev", "AxBx");
		break;
	case REVISION_CX:
		printf("Beagle Rev C1/C2/C3\n");
		setenv("beaglerev", "Cx");
		MUX_BEAGLE_C();
		break;
	case REVISION_C4:
		printf("Beagle Rev C4\n");
		setenv("beaglerev", "C4");
		MUX_BEAGLE_C();
		/* Set VAUX2 to 1.8V for EHCI PHY */
		twl4030_pmrecv_vsel_cfg(TWL4030_PM_RECEIVER_VAUX2_DEDICATED,
					TWL4030_PM_RECEIVER_VAUX2_VSEL_18,
					TWL4030_PM_RECEIVER_VAUX2_DEV_GRP,
					TWL4030_PM_RECEIVER_DEV_GRP_P1);
		break;
	case REVISION_XM_AB:
		printf("Beagle xM Rev A/B\n");
		setenv("beaglerev", "xMAB");
		MUX_BEAGLE_XM();
		/* Set VAUX2 to 1.8V for EHCI PHY */
		twl4030_pmrecv_vsel_cfg(TWL4030_PM_RECEIVER_VAUX2_DEDICATED,
					TWL4030_PM_RECEIVER_VAUX2_VSEL_18,
					TWL4030_PM_RECEIVER_VAUX2_DEV_GRP,
					TWL4030_PM_RECEIVER_DEV_GRP_P1);
		generate_fake_mac = true;
		break;
	case REVISION_XM_C:
		printf("Beagle xM Rev C\n");
		setenv("beaglerev", "xMC");
		MUX_BEAGLE_XM();
		/* Set VAUX2 to 1.8V for EHCI PHY */
		twl4030_pmrecv_vsel_cfg(TWL4030_PM_RECEIVER_VAUX2_DEDICATED,
					TWL4030_PM_RECEIVER_VAUX2_VSEL_18,
					TWL4030_PM_RECEIVER_VAUX2_DEV_GRP,
					TWL4030_PM_RECEIVER_DEV_GRP_P1);
		generate_fake_mac = true;
		break;
	default:
		printf("Beagle unknown 0x%02x\n", get_board_revision());
		MUX_BEAGLE_XM();
		/* Set VAUX2 to 1.8V for EHCI PHY */
		twl4030_pmrecv_vsel_cfg(TWL4030_PM_RECEIVER_VAUX2_DEDICATED,
					TWL4030_PM_RECEIVER_VAUX2_VSEL_18,
					TWL4030_PM_RECEIVER_VAUX2_DEV_GRP,
					TWL4030_PM_RECEIVER_DEV_GRP_P1);
		generate_fake_mac = true;
	}

	switch (get_expansion_id()) {
	case TINCANTOOLS_ZIPPY:
		printf("Recognized Tincantools Zippy board (rev %d %s)\n",
			expansion_config.revision,
			expansion_config.fab_revision);
		MUX_TINCANTOOLS_ZIPPY();
		setenv("buddy", "zippy");
		break;
	case TINCANTOOLS_ZIPPY2:
		printf("Recognized Tincantools Zippy2 board (rev %d %s)\n",
			expansion_config.revision,
			expansion_config.fab_revision);
		MUX_TINCANTOOLS_ZIPPY();
		setenv("buddy", "zippy2");
		break;
	case TINCANTOOLS_TRAINER:
		printf("Recognized Tincantools Trainer board (rev %d %s)\n",
			expansion_config.revision,
			expansion_config.fab_revision);
		
		MUX_TINCANTOOLS_ZIPPY();
		MUX_TINCANTOOLS_TRAINER();
		setenv("buddy", "trainer");
		break;
	case TINCANTOOLS_SHOWDOG:
		printf("Recognized Tincantools Showdow board (rev %d %s)\n",
			expansion_config.revision,
			expansion_config.fab_revision);
		/* Place holder for DSS2 definition for showdog lcd */
		setenv("defaultdisplay", "showdoglcd");
		setenv("buddy", "showdog");
		break;
	case KBADC_BEAGLEFPGA:
		printf("Recognized KBADC Beagle FPGA board\n");
		MUX_KBADC_BEAGLEFPGA();
		setenv("buddy", "beaglefpga");
		break;
	case LW_BEAGLETOUCH:
		printf("Recognized Liquidware BeagleTouch board\n");
		setenv("buddy", "beagletouch");
		break;
	case BRAINMUX_LCDOG:
		printf("Recognized Brainmux LCDog board\n");
		setenv("buddy", "lcdog");
		break;
	case BRAINMUX_LCDOGTOUCH:
		printf("Recognized Brainmux LCDog Touch board\n");
		setenv("buddy", "lcdogtouch");
		break;
	case BBTOYS_WIFI:
		printf("Recognized BeagleBoardToys WiFi board\n");
		MUX_BBTOYS_WIFI()
		setenv("buddy", "bbtoys-wifi");
		break;;
	case BBTOYS_VGA:
		printf("Recognized BeagleBoardToys VGA board\n");
		break;;
	case BBTOYS_LCD:
		printf("Recognized BeagleBoardToys LCD board\n");
		break;;
	case BCT_BRETTL3:
		printf("Recognized bct electronic GmbH brettl3 board\n");
		break;
	case BCT_BRETTL4:
		printf("Recognized bct electronic GmbH brettl4 board\n");
		break;
	case LSR_COM6L_ADPT:
		printf("Recognized LSR COM6L Adapter Board\n");
		MUX_BBTOYS_WIFI()
		setenv("buddy", "lsr-com6l-adpt");
		break;
	case BEAGLE_NO_EEPROM:
		printf("No EEPROM on expansion board\n");
		setenv("buddy", "none");
		break;
	default:
		printf("Unrecognized expansion board: %x\n",
			expansion_config.device_vendor);
		setenv("buddy", "unknown");
	}

	if (expansion_config.content == 1)
		setenv(expansion_config.env_var, expansion_config.env_setting);

	twl4030_power_init();
	switch (get_board_revision()) {
	case REVISION_XM_AB:
		twl4030_led_init(TWL4030_LED_LEDEN_LEDBON);
		break;
	default:
		twl4030_led_init(TWL4030_LED_LEDEN_LEDAON | TWL4030_LED_LEDEN_LEDBON);
		break;
	}

	/* Set GPIO states before they are made outputs */
	writel(GPIO23 | GPIO10 | GPIO8 | GPIO2 | GPIO1,
		&gpio6_base->setdataout);
	writel(GPIO31 | GPIO30 | GPIO29 | GPIO28 | GPIO22 | GPIO21 |
		GPIO15 | GPIO14 | GPIO13 | GPIO12, &gpio5_base->setdataout);

	/* Configure GPIOs to output */
	writel(~(GPIO23 | GPIO10 | GPIO8 | GPIO2 | GPIO1), &gpio6_base->oe);
	writel(~(GPIO31 | GPIO30 | GPIO29 | GPIO28 | GPIO22 | GPIO21 |
		GPIO15 | GPIO14 | GPIO13 | GPIO12), &gpio5_base->oe);

	omap_die_id_display();

#ifdef CONFIG_VIDEO_OMAP3
	beagle_dvi_pup();
	beagle_display_init();
	omap3_dss_enable();
#endif

#ifdef CONFIG_USB_MUSB_OMAP2PLUS
	musb_register(&musb_plat, &musb_board_data, (void *)MUSB_BASE);
#endif

	if (generate_fake_mac)
		omap_die_id_usbethaddr();

	return 0;
}

/*
 * Routine: set_muxconf_regs
 * Description: Setting up the configuration Mux registers specific to the
 *		hardware. Many pins need to be moved from protect to primary
 *		mode.
 */
#define ___SKIP_set_muxconf_regs_spl_FUNC
#define ___SKIP_set_muxconf_regs_main_FUNC

void set_muxconf_regs(void)
{
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
  ; /*GPMC*/								
  MUX_VAL(CP(GPMC_A1),		(IDIS | PTD | DIS | M0)); /*GPMC_A1*/	
  MUX_VAL(CP(GPMC_A2),		(IDIS | PTD | DIS | M0)); /*GPMC_A2*/	
  MUX_VAL(CP(GPMC_A3),		(IDIS | PTD | DIS | M0)); /*GPMC_A3*/	
  MUX_VAL(CP(GPMC_A4),		(IDIS | PTD | DIS | M0)); /*GPMC_A4*/	
  MUX_VAL(CP(GPMC_A5),		(IDIS | PTD | DIS | M0)); /*GPMC_A5*/	
  MUX_VAL(CP(GPMC_A6),		(IDIS | PTD | DIS | M0)); /*GPMC_A6*/	
  MUX_VAL(CP(GPMC_A7),		(IDIS | PTD | DIS | M0)); /*GPMC_A7*/	
  MUX_VAL(CP(GPMC_A8),		(IDIS | PTD | DIS | M0)); /*GPMC_A8*/	
  MUX_VAL(CP(GPMC_A9),		(IDIS | PTD | DIS | M0)); /*GPMC_A9*/	
  MUX_VAL(CP(GPMC_A10),		(IDIS | PTD | DIS | M0)); /*GPMC_A10*/	
  MUX_VAL(CP(GPMC_D0),		(IEN  | PTD | DIS | M0)); /*GPMC_D0*/	
  MUX_VAL(CP(GPMC_D1),		(IEN  | PTD | DIS | M0)); /*GPMC_D1*/	
  MUX_VAL(CP(GPMC_D2),		(IEN  | PTD | DIS | M0)); /*GPMC_D2*/	
  MUX_VAL(CP(GPMC_D3),		(IEN  | PTD | DIS | M0)); /*GPMC_D3*/	
  MUX_VAL(CP(GPMC_D4),		(IEN  | PTD | DIS | M0)); /*GPMC_D4*/	
  MUX_VAL(CP(GPMC_D5),		(IEN  | PTD | DIS | M0)); /*GPMC_D5*/	
  MUX_VAL(CP(GPMC_D6),		(IEN  | PTD | DIS | M0)); /*GPMC_D6*/	
  MUX_VAL(CP(GPMC_D7),		(IEN  | PTD | DIS | M0)); /*GPMC_D7*/	
  MUX_VAL(CP(GPMC_D8),		(IEN  | PTD | DIS | M0)); /*GPMC_D8*/	
  MUX_VAL(CP(GPMC_D9),		(IEN  | PTD | DIS | M0)); /*GPMC_D9*/	
  MUX_VAL(CP(GPMC_D10),		(IEN  | PTD | DIS | M0)); /*GPMC_D10*/	
  MUX_VAL(CP(GPMC_D11),		(IEN  | PTD | DIS | M0)); /*GPMC_D11*/	
  MUX_VAL(CP(GPMC_D12),		(IEN  | PTD | DIS | M0)); /*GPMC_D12*/	
  MUX_VAL(CP(GPMC_D13),		(IEN  | PTD | DIS | M0)); /*GPMC_D13*/	
  MUX_VAL(CP(GPMC_D14),		(IEN  | PTD | DIS | M0)); /*GPMC_D14*/	
  MUX_VAL(CP(GPMC_D15),		(IEN  | PTD | DIS | M0)); /*GPMC_D15*/	
  MUX_VAL(CP(GPMC_NCS0),		(IDIS | PTU | EN  | M0)); /*GPMC_nCS0*/	
  MUX_VAL(CP(GPMC_NCS1),		(IDIS | PTU | EN  | M0)); /*GPMC_nCS1*/	
  MUX_VAL(CP(GPMC_NCS2),		(IDIS | PTU | EN  | M0)); /*GPMC_nCS2*/	
  MUX_VAL(CP(GPMC_NCS3),		(IDIS | PTU | EN  | M0)); /*GPMC_nCS3*/	
  MUX_VAL(CP(GPMC_NCS4),		(IDIS | PTU | EN  | M0)); /*GPMC_nCS4*/	
  MUX_VAL(CP(GPMC_NCS5),		(IDIS | PTD | DIS | M0)); /*GPMC_nCS5*/	
  MUX_VAL(CP(GPMC_NCS6),		(IEN  | PTD | DIS | M1)); /*SYS_nDMA_REQ2*/ 
  MUX_VAL(CP(GPMC_NCS7),		(IEN  | PTU | EN  | M1)); /*SYS_nDMA_REQ3*/ 
  MUX_VAL(CP(GPMC_NBE1),		(IEN  | PTD | DIS | M0)); /*GPMC_nBE1*/	
  MUX_VAL(CP(GPMC_WAIT2),		(IEN  | PTU | EN  | M0)); /*GPMC_WAIT2*/ 
  MUX_VAL(CP(GPMC_WAIT3),		(IEN  | PTU | EN  | M0)); /*GPMC_WAIT3*/ 
  MUX_VAL(CP(GPMC_CLK),		(IDIS | PTD | DIS | M0)); /*GPMC_CLK*/	
  MUX_VAL(CP(GPMC_NADV_ALE),	(IDIS | PTD | DIS | M0)); /*GPMC_nADV_ALE*/ 
  MUX_VAL(CP(GPMC_NOE),		(IDIS | PTD | DIS | M0)); /*GPMC_nOE*/	
  MUX_VAL(CP(GPMC_NWE),		(IDIS | PTD | DIS | M0)); /*GPMC_nWE*/	
  MUX_VAL(CP(GPMC_NBE0_CLE),	(IDIS | PTD | DIS | M0)); /*GPMC_nBE0_CLE*/ 
  MUX_VAL(CP(GPMC_NWP),		(IEN  | PTD | DIS | M0)); /*GPMC_nWP*/	
  MUX_VAL(CP(GPMC_WAIT0),		(IEN  | PTU | EN  | M0)); /*GPMC_WAIT0*/ 
  MUX_VAL(CP(GPMC_WAIT1),		(IEN  | PTU | EN  | M0)); /*GPMC_WAIT1*/ 
  ; /*DSS*/								
  MUX_VAL(CP(DSS_PCLK),		(IDIS | PTD | DIS | M0)); /*DSS_PCLK*/	
  MUX_VAL(CP(DSS_HSYNC),		(IDIS | PTD | DIS | M0)); /*DSS_HSYNC*/	
  MUX_VAL(CP(DSS_VSYNC),		(IDIS | PTD | DIS | M0)); /*DSS_VSYNC*/	
  MUX_VAL(CP(DSS_ACBIAS),		(IDIS | PTD | DIS | M0)); /*DSS_ACBIAS*/ 
  MUX_VAL(CP(DSS_DATA0),		(IDIS | PTD | DIS | M0)); /*DSS_DATA0*/	
  MUX_VAL(CP(DSS_DATA1),		(IDIS | PTD | DIS | M0)); /*DSS_DATA1*/	
  MUX_VAL(CP(DSS_DATA2),		(IDIS | PTD | DIS | M0)); /*DSS_DATA2*/	
  MUX_VAL(CP(DSS_DATA3),		(IDIS | PTD | DIS | M0)); /*DSS_DATA3*/	
  MUX_VAL(CP(DSS_DATA4),		(IDIS | PTD | DIS | M0)); /*DSS_DATA4*/	
  MUX_VAL(CP(DSS_DATA5),		(IDIS | PTD | DIS | M0)); /*DSS_DATA5*/	
  MUX_VAL(CP(DSS_DATA6),		(IDIS | PTD | DIS | M0)); /*DSS_DATA6*/	
  MUX_VAL(CP(DSS_DATA7),		(IDIS | PTD | DIS | M0)); /*DSS_DATA7*/	
  MUX_VAL(CP(DSS_DATA8),		(IDIS | PTD | DIS | M0)); /*DSS_DATA8*/	
  MUX_VAL(CP(DSS_DATA9),		(IDIS | PTD | DIS | M0)); /*DSS_DATA9*/	
  MUX_VAL(CP(DSS_DATA10),		(IDIS | PTD | DIS | M0)); /*DSS_DATA10*/ 
  MUX_VAL(CP(DSS_DATA11),		(IDIS | PTD | DIS | M0)); /*DSS_DATA11*/ 
  MUX_VAL(CP(DSS_DATA12),		(IDIS | PTD | DIS | M0)); /*DSS_DATA12*/ 
  MUX_VAL(CP(DSS_DATA13),		(IDIS | PTD | DIS | M0)); /*DSS_DATA13*/ 
  MUX_VAL(CP(DSS_DATA14),		(IDIS | PTD | DIS | M0)); /*DSS_DATA14*/ 
  MUX_VAL(CP(DSS_DATA15),		(IDIS | PTD | DIS | M0)); /*DSS_DATA15*/ 
  MUX_VAL(CP(DSS_DATA16),		(IDIS | PTD | DIS | M0)); /*DSS_DATA16*/ 
  MUX_VAL(CP(DSS_DATA17),		(IDIS | PTD | DIS | M0)); /*DSS_DATA17*/ 
  MUX_VAL(CP(DSS_DATA18),		(IDIS | PTD | DIS | M0)); /*DSS_DATA18*/ 
  MUX_VAL(CP(DSS_DATA19),		(IDIS | PTD | DIS | M0)); /*DSS_DATA19*/ 
  MUX_VAL(CP(DSS_DATA20),		(IDIS | PTD | DIS | M0)); /*DSS_DATA20*/ 
  MUX_VAL(CP(DSS_DATA21),		(IDIS | PTD | DIS | M0)); /*DSS_DATA21*/ 
  MUX_VAL(CP(DSS_DATA22),		(IDIS | PTD | DIS | M0)); /*DSS_DATA22*/ 
  MUX_VAL(CP(DSS_DATA23),		(IDIS | PTD | DIS | M0)); /*DSS_DATA23*/ 
  ; /*CAMERA*/								
  MUX_VAL(CP(CAM_HS),		(IEN  | PTU | EN  | M0)); /*CAM_HS */	
  MUX_VAL(CP(CAM_VS),		(IEN  | PTU | EN  | M0)); /*CAM_VS */	
  MUX_VAL(CP(CAM_XCLKA),		(IDIS | PTD | DIS | M0)); /*CAM_XCLKA*/	
  MUX_VAL(CP(CAM_PCLK),		(IEN  | PTU | EN  | M0)); /*CAM_PCLK*/	
  MUX_VAL(CP(CAM_FLD),		(IDIS | PTD | DIS | M4)); /*GPIO_98*/	
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
  MUX_VAL(CP(CAM_XCLKB),		(IDIS | PTD | DIS | M0)); /*CAM_XCLKB*/	
  MUX_VAL(CP(CAM_WEN),		(IEN  | PTD | DIS | M4)); /*GPIO_167*/	
  MUX_VAL(CP(CAM_STROBE),		(IDIS | PTD | DIS | M0)); /*CAM_STROBE*/ 
  MUX_VAL(CP(CSI2_DX0),		(IEN  | PTD | DIS | M0)); /*CSI2_DX0*/	
  MUX_VAL(CP(CSI2_DY0),		(IEN  | PTD | DIS | M0)); /*CSI2_DY0*/	
  MUX_VAL(CP(CSI2_DX1),		(IEN  | PTD | DIS | M0)); /*CSI2_DX1*/	
  MUX_VAL(CP(CSI2_DY1),		(IEN  | PTD | DIS | M0)); /*CSI2_DY1*/	
  ; /*Audio Interface */						
  MUX_VAL(CP(MCBSP2_FSX),		(IEN  | PTD | DIS | M0)); /*McBSP2_FSX*/ 
  MUX_VAL(CP(MCBSP2_CLKX),	(IEN  | PTD | DIS | M0)); /*McBSP2_CLKX*/ 
  MUX_VAL(CP(MCBSP2_DR),		(IEN  | PTD | DIS | M0)); /*McBSP2_DR*/	
  MUX_VAL(CP(MCBSP2_DX),		(IDIS | PTD | DIS | M0)); /*McBSP2_DX*/	
  ; /*Expansion card */							
  MUX_VAL(CP(MMC1_CLK),		(IDIS | PTU | EN  | M0)); /*MMC1_CLK*/	
  MUX_VAL(CP(MMC1_CMD),		(IEN  | PTU | EN  | M0)); /*MMC1_CMD*/	
  MUX_VAL(CP(MMC1_DAT0),		(IEN  | PTU | EN  | M0)); /*MMC1_DAT0*/	
  MUX_VAL(CP(MMC1_DAT1),		(IEN  | PTU | EN  | M0)); /*MMC1_DAT1*/	
  MUX_VAL(CP(MMC1_DAT2),		(IEN  | PTU | EN  | M0)); /*MMC1_DAT2*/	
  MUX_VAL(CP(MMC1_DAT3),		(IEN  | PTU | EN  | M0)); /*MMC1_DAT3*/	
  MUX_VAL(CP(MMC1_DAT4),		(IEN  | PTU | EN  | M0)); /*MMC1_DAT4*/	
  MUX_VAL(CP(MMC1_DAT5),		(IEN  | PTU | EN  | M0)); /*MMC1_DAT5*/	
  MUX_VAL(CP(MMC1_DAT6),		(IEN  | PTU | EN  | M0)); /*MMC1_DAT6*/	
  MUX_VAL(CP(MMC1_DAT7),		(IEN  | PTU | EN  | M0)); /*MMC1_DAT7*/	
  ; /*Wireless LAN */							
  MUX_VAL(CP(MMC2_CLK),		(IEN  | PTU | EN  | M4)); /*GPIO_130*/	
  MUX_VAL(CP(MMC2_CMD),		(IEN  | PTU | EN  | M4)); /*GPIO_131*/	
  MUX_VAL(CP(MMC2_DAT0),		(IEN  | PTU | EN  | M4)); /*GPIO_132*/ 
  MUX_VAL(CP(MMC2_DAT1),		(IEN  | PTU | EN  | M4)); /*GPIO_133*/ 
  MUX_VAL(CP(MMC2_DAT2),		(IEN  | PTU | EN  | M4)); /*GPIO_134*/ 
  MUX_VAL(CP(MMC2_DAT3),		(IEN  | PTU | EN  | M4)); /*GPIO_135*/ 
  MUX_VAL(CP(MMC2_DAT4),		(IEN  | PTU | EN  | M4)); /*GPIO_136*/ 
  MUX_VAL(CP(MMC2_DAT5),		(IEN  | PTU | EN  | M4)); /*GPIO_137*/ 
  MUX_VAL(CP(MMC2_DAT6),		(IEN  | PTU | EN  | M4)); /*GPIO_138*/ 
  MUX_VAL(CP(MMC2_DAT7),		(IEN  | PTU | EN  | M4)); /*GPIO_139*/ 
  ; /*Bluetooth*/							
  MUX_VAL(CP(MCBSP3_DX),		(IEN  | PTD | DIS | M1)); /*UART2_CTS*/	
  MUX_VAL(CP(MCBSP3_DR),		(IDIS | PTD | DIS | M1)); /*UART2_RTS*/	
  MUX_VAL(CP(MCBSP3_CLKX),	(IDIS | PTD | DIS | M1)); /*UART2_TX*/	
  MUX_VAL(CP(MCBSP3_FSX),		(IEN  | PTD | DIS | M1)); /*UART2_RX*/ 
  MUX_VAL(CP(UART2_CTS),		(IEN  | PTD | DIS | M4)); /*GPIO_144*/ 
  MUX_VAL(CP(UART2_RTS),		(IEN  | PTD | DIS | M4)); /*GPIO_145*/ 
  MUX_VAL(CP(UART2_TX),		(IEN  | PTD | DIS | M4)); /*GPIO_146*/	
  MUX_VAL(CP(UART2_RX),		(IEN  | PTD | DIS | M4)); /*GPIO_147*/	
  ; /*Modem Interface */						
  MUX_VAL(CP(UART1_TX),		(IDIS | PTD | DIS | M0)); /*UART1_TX*/	
  MUX_VAL(CP(UART1_RTS),		(IDIS | PTD | DIS | M4)); /*GPIO_149*/ 
  MUX_VAL(CP(UART1_CTS),		(IDIS | PTD | DIS | M4)); /*GPIO_150*/ 
  MUX_VAL(CP(UART1_RX),		(IEN  | PTD | DIS | M0)); /*UART1_RX*/	
  MUX_VAL(CP(MCBSP4_CLKX),	(IEN  | PTD | DIS | M1)); /*SSI1_DAT_RX*/ 
  MUX_VAL(CP(MCBSP4_DR),		(IEN  | PTD | DIS | M1)); /*SSI1_FLAG_RX*/ 
  MUX_VAL(CP(MCBSP4_DX),		(IEN  | PTD | DIS | M1)); /*SSI1_RDY_RX*/ 
  MUX_VAL(CP(MCBSP4_FSX),		(IEN  | PTD | DIS | M1)); /*SSI1_WAKE*/	
  MUX_VAL(CP(MCBSP1_CLKR),	(IDIS | PTD | DIS | M4)); /*GPIO_156*/	
  MUX_VAL(CP(MCBSP1_FSR),		(IDIS | PTU | EN  | M4)); /*GPIO_157*/ 
  MUX_VAL(CP(MCBSP1_DX),		(IDIS | PTD | DIS | M4)); /*GPIO_158*/ 
  MUX_VAL(CP(MCBSP1_DR),		(IDIS | PTD | DIS | M4)); /*GPIO_159*/ 
  MUX_VAL(CP(MCBSP_CLKS),		(IEN  | PTU | DIS | M0)); /*McBSP_CLKS*/ 
  MUX_VAL(CP(MCBSP1_FSX),		(IDIS | PTD | DIS | M4)); /*GPIO_161*/ 
  MUX_VAL(CP(MCBSP1_CLKX),	(IDIS | PTD | DIS | M4)); /*GPIO_162*/	
  ; /*Serial Interface*/						
  MUX_VAL(CP(UART3_CTS_RCTX),	(IEN  | PTD | EN  | M0)); /*UART3_CTS_RCTX*/ 
  MUX_VAL(CP(UART3_RTS_SD),	(IDIS | PTD | DIS | M0)); /*UART3_RTS_SD */ 
  MUX_VAL(CP(UART3_RX_IRRX),	(IEN  | PTD | DIS | M0)); /*UART3_RX_IRRX*/ 
  MUX_VAL(CP(UART3_TX_IRTX),	(IDIS | PTD | DIS | M0)); /*UART3_TX_IRTX*/ 
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
  MUX_VAL(CP(I2C1_SCL),		(IEN  | PTU | EN  | M0)); /*I2C1_SCL*/	
  MUX_VAL(CP(I2C1_SDA),		(IEN  | PTU | EN  | M0)); /*I2C1_SDA*/	
  MUX_VAL(CP(I2C2_SCL),		(IEN  | PTU | EN  | M0)); /*I2C2_SCL*/	
  MUX_VAL(CP(I2C2_SDA),		(IEN  | PTU | EN  | M0)); /*I2C2_SDA*/	
  MUX_VAL(CP(I2C3_SCL),		(IEN  | PTU | EN  | M0)); /*I2C3_SCL*/	
  MUX_VAL(CP(I2C3_SDA),		(IEN  | PTU | EN  | M0)); /*I2C3_SDA*/	
  MUX_VAL(CP(I2C4_SCL),		(IEN  | PTU | EN  | M0)); /*I2C4_SCL*/	
  MUX_VAL(CP(I2C4_SDA),		(IEN  | PTU | EN  | M0)); /*I2C4_SDA*/	
  MUX_VAL(CP(HDQ_SIO),		(IDIS | PTU | EN  | M4)); /*GPIO_170*/	
  MUX_VAL(CP(MCSPI1_CLK),		(IEN  | PTU | EN  | M4)); /*GPIO_171*/ 
  MUX_VAL(CP(MCSPI1_SIMO),	(IEN  | PTU | EN  | M4)); /*GPIO_172*/	
  MUX_VAL(CP(MCSPI1_SOMI),	(IEN  | PTU | EN  | M4)); /*GPIO_173*/	
  MUX_VAL(CP(MCSPI1_CS0),		(IEN  | PTD | EN  | M0)); /*McSPI1_CS0*/ 
  MUX_VAL(CP(MCSPI1_CS1),		(IDIS | PTD | EN  | M0)); /*McSPI1_CS1*/ 
  MUX_VAL(CP(MCSPI1_CS2),		(IDIS | PTD | DIS | M4)); /*GPIO_176*/ 
  ; /* USB EHCI (port 2) */						
  MUX_VAL(CP(MCSPI1_CS3),		(IEN  | PTD | EN  | M3)); /*HSUSB2_DATA2*/ 
  MUX_VAL(CP(MCSPI2_CLK),		(IEN  | PTD | EN  | M3)); /*HSUSB2_DATA7*/ 
  MUX_VAL(CP(MCSPI2_SIMO),	(IEN  | PTD | EN  | M3)); /*HSUSB2_DATA4*/ 
  MUX_VAL(CP(MCSPI2_SOMI),	(IEN  | PTD | EN  | M3)); /*HSUSB2_DATA5*/ 
  MUX_VAL(CP(MCSPI2_CS0),		(IEN  | PTD | EN  | M3)); /*HSUSB2_DATA6*/ 
  MUX_VAL(CP(MCSPI2_CS1),		(IEN  | PTD | EN  | M3)); /*HSUSB2_DATA3*/ 
  MUX_VAL(CP(ETK_D10_ES2),	(IDIS | PTD | DIS | M3)); /*HSUSB2_CLK*/ 
  MUX_VAL(CP(ETK_D11_ES2),	(IDIS | PTD | DIS | M3)); /*HSUSB2_STP*/ 
  MUX_VAL(CP(ETK_D12_ES2),	(IEN  | PTD | EN  | M3)); /*HSUSB2_DIR*/ 
  MUX_VAL(CP(ETK_D13_ES2),	(IEN  | PTD | EN  | M3)); /*HSUSB2_NXT*/ 
  MUX_VAL(CP(ETK_D14_ES2),	(IEN  | PTD | EN  | M3)); /*HSUSB2_DATA0*/ 
  MUX_VAL(CP(ETK_D15_ES2),	(IEN  | PTD | EN  | M3)); /*HSUSB2_DATA1*/ 
  ; /*Control and debug */						
  MUX_VAL(CP(SYS_32K),		(IEN  | PTD | DIS | M0)); /*SYS_32K*/	
  MUX_VAL(CP(SYS_CLKREQ),		(IEN  | PTD | DIS | M0)); /*SYS_CLKREQ*/ 
  MUX_VAL(CP(SYS_NIRQ),		(IEN  | PTU | EN  | M0)); /*SYS_nIRQ*/	
  MUX_VAL(CP(SYS_BOOT0),		(IEN  | PTD | DIS | M4)); /*GPIO_2*/ 
  MUX_VAL(CP(SYS_BOOT1),		(IEN  | PTD | DIS | M4)); /*GPIO_3*/ 
  MUX_VAL(CP(SYS_BOOT2),		(IEN  | PTD | DIS | M4)); /*GPIO_4 - MMC1_WP*/ 
  MUX_VAL(CP(SYS_BOOT3),		(IEN  | PTD | DIS | M4)); /*GPIO_5*/ 
  MUX_VAL(CP(SYS_BOOT4),		(IEN  | PTD | DIS | M4)); /*GPIO_6*/ 
  MUX_VAL(CP(SYS_BOOT5),		(IEN  | PTD | DIS | M4)); /*GPIO_7*/ 
  MUX_VAL(CP(SYS_BOOT6),		(IDIS | PTD | DIS | M4)); /*GPIO_8*/ 
  MUX_VAL(CP(SYS_OFF_MODE),	(IEN  | PTD | DIS | M0)); /*SYS_OFF_MODE*/ 
  MUX_VAL(CP(SYS_CLKOUT1),	(IEN  | PTD | DIS | M0)); /*SYS_CLKOUT1*/ 
  MUX_VAL(CP(SYS_CLKOUT2),	(IEN  | PTU | EN  | M4)); /*GPIO_186*/	
  MUX_VAL(CP(ETK_CLK_ES2),	(IDIS | PTU | EN  | M3)); /*HSUSB1_STP*/ 
  MUX_VAL(CP(ETK_CTL_ES2),	(IDIS | PTU | DIS | M3)); /*HSUSB1_CLK*/ 
  MUX_VAL(CP(ETK_D0_ES2),		(IEN  | PTU | DIS | M3)); /*HSUSB1_DATA0*/ 
  MUX_VAL(CP(ETK_D1_ES2),		(IEN  | PTU | DIS | M3)); /*HSUSB1_DATA1*/ 
  MUX_VAL(CP(ETK_D2_ES2),		(IEN  | PTU | DIS | M3)); /*HSUSB1_DATA2*/ 
  MUX_VAL(CP(ETK_D3_ES2),		(IEN  | PTU | DIS | M3)); /*HSUSB1_DATA7*/ 
  MUX_VAL(CP(ETK_D4_ES2),		(IEN  | PTU | DIS | M3)); /*HSUSB1_DATA4*/ 
  MUX_VAL(CP(ETK_D5_ES2),		(IEN  | PTU | DIS | M3)); /*HSUSB1_DATA5*/ 
  MUX_VAL(CP(ETK_D6_ES2),		(IEN  | PTU | DIS | M3)); /*HSUSB1_DATA6*/ 
  MUX_VAL(CP(ETK_D7_ES2),		(IEN  | PTU | DIS | M3)); /*HSUSB1_DATA3*/ 
  MUX_VAL(CP(ETK_D8_ES2),		(IEN  | PTU | DIS | M3)); /*HSUSB1_DIR*/ 
  MUX_VAL(CP(ETK_D9_ES2),		(IEN  | PTU | DIS | M3)); /*HSUSB1_NXT*/ 
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

#if defined(CONFIG_GENERIC_MMC) && !defined(CONFIG_SPL_BUILD)
int board_mmc_init(bd_t *bis)
{
	return omap_mmc_init(0, 0, 0, -1, -1);
}
#endif

#if defined(CONFIG_GENERIC_MMC)
void board_mmc_power_init(void)
{
	twl4030_power_mmc_init(0);
}
#endif

#if defined(CONFIG_USB_EHCI) && !defined(CONFIG_SPL_BUILD)
/* Call usb_stop() before starting the kernel */
void show_boot_progress(int val)
{
	if (val == BOOTSTAGE_ID_RUN_OS)
		usb_stop();
}

static struct omap_usbhs_board_data usbhs_bdata = {
	.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[1] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED
};

int ehci_hcd_init(int index, enum usb_init_type init,
		struct ehci_hccr **hccr, struct ehci_hcor **hcor)
{
	return omap_ehci_hcd_init(index, &usbhs_bdata, hccr, hcor);
}

int ehci_hcd_stop(int index)
{
	return omap_ehci_hcd_stop();
}

#endif /* CONFIG_USB_EHCI */

#if defined(CONFIG_USB_ETHER) && defined(CONFIG_USB_MUSB_GADGET)
int board_eth_init(bd_t *bis)
{
	return usb_eth_initialize(bis);
}
#endif
