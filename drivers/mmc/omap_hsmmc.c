/*
 * (C) Copyright 2008
 * Texas Instruments, <www.ti.com>
 * Sukumar Ghorai <s-ghorai@ti.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation's version 2 of
 * the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */



#include <config.h>
#include <common.h>
#include <malloc.h>
#include <mmc.h>
#include <part.h>
#include <i2c.h>
#include <twl4030.h>
#include <twl6030.h>
#include <palmas.h>
#include <asm/io.h>
#include <asm/arch/mmc_host_def.h>
#if !defined(CONFIG_SOC_KEYSTONE)
#include <asm/gpio.h>
#include <asm/arch/sys_proto.h>
#endif
#include <dm.h>

DECLARE_GLOBAL_DATA_PTR;


/* typedef struct image_header { */
/*  __be32 ih_magic; */ //0x27051956
/*  __be32 ih_hcrc; */
/*  __be32 ih_time; */
/*  __be32 ih_size; */
/*  __be32 ih_load; */ 
/*  __be32 ih_ep; */
/*  __be32 ih_dcrc; */
/*  uint8_t ih_os; */
/*  uint8_t ih_arch; */
/*  uint8_t ih_type; */
/*  uint8_t ih_comp; */
/*  uint8_t ih_name[32]; */
/* } image_header_t; */


// ghost  char *header = {0x56, 0x19, 0x05, 0x27};
// ghost unsigned int collector_mmc_cmd = 0;
// ghost unsigned int fat_mmc_read_size = 0;
// ghost unsigned int collector_mmc_arg = 0;
// ghost unsigned int collector_mmc_stat = 0;
// ghost unsigned int image_header_copied = 0;
// ghost unsigned int collector_mmc_rsp[4];
// ghost unsigned int collector_mmc_data_count = 0;

/* ghost //@ requires (p == &mmc_base->data);
   unsigned int read_mmc_data (volatile unsigned int *p) {
   if ((collector_mmc_cmd >> 24 & 51) == 51) && (collector_mmc_data_count == 0){
      collector_mmc_data_count = 1;
      return 0x02; //02 00 00 00
   } elif ((fat_mmc_read_size == sizeof(struct image_header) && (! image_header_copied))){
     memcpy(header, p, 4);
     image_header_copied = 1;
     return *p;
   } else
     return *p;
   }
  }*/
/* ghost //@ requires (p == &mmc_base->rsp10) || (p == &mmc_base->rsp32) || (p == &mmc_base->rsp54) || (p == &mmc_base->rsp76);
   unsigned int reads_mmcrsp( volatile unsigned int *p) {
  
   if (p == &mmc_base->rsp10) {
      if ((collector_mmc_cmd >> 24 & 9) == 9) {
         return 0 | (4 << 26);
      } elif ((collector_mmc_cmd >> 24 & 8) == 0) {
         return 0xaa;
      }
     return *mmc_base->rsp10;
   } elif (p == &mmc_base->rsp32) {
     return *mmc_base->rsp32;
   } elif (p == &mmc_base->rsp54) {
     return *mmc_base->rsp54;
   } elif (p == &mmc_base->rsp76) {
     if ((collector_mmc_cmd >> 24 & 58) == 58)
         return 0x40000000;
     else
         return *mmc_base->rsp76;
   } else
    return 0; // should not happen
  }
  */


/* ghost //@ requires p == &mmc_base->stat;
   unsigned int mmc_stat_write( volatile unsigned int *p, unsigned int i) {
   if (p == &mmc_base->stat) {
       collector_mmc_stat = i;
       return i;
    } else
    return 0; // should not happen
   }
  */

/* ghost //@ requires p == &mmc_base->stat;
   unsigned int mmc_stat_read( volatile unsigned int *p) {
   unsigned int flags, mmc_stat;
     if (collector_mmc_stat == 0xFFFFFFFF) {
        collector_mmc_stat = 0;
     }
     return 1;
   }
  */
/* ghost //@ requires p == &mmc_base->stat;
   unsigned int mmc_stat_read_data ( volatile unsigned int *p) {
   unsigned int flags, mmc_stat;
   if (p == &mmc_base->stat) {
     return 0 | (0x1 << 0) | (0x1 << 4);
   } else
    return 0; // should not happen
   }
  */


/* ghost //@ requires p == &mmc_base->stat;
   unsigned int mmc_stat_write_data ( volatile unsigned int *p) {
   unsigned int flags, mmc_stat;
   if (p == &mmc_base->stat) {
     return 0 | (0x1 << 0) | (0x1 << 5);
   } else
    return 0; // should not happen
   }
  */


/* ghost //@ requires p == &mmc_base->cmd;
   unsigned int mmc_cmd_write( volatile unsigned int *p, unsigned int i) {
   if (p == &mmc_base->cmd) {
     if ((i >> 24 & 51) == 51) {
       collector_mmc_data_count = 0;
     }
     collector_mmc_cmd = i;
     return i;
   } else
    return 0; // should not happen
   }
  */


/* ghost //@ requires p == &mmc_base->arg;
   unsigned int mmc_arg_write( volatile unsigned int *p, unsigned int i) {
   if (p == &mmc_base->arg) {
       collector_mmc_arg = i;
       return i;
    } else
    return 0; // should not happen
   }
  */



/* simplify defines to OMAP_HSMMC_USE_GPIO */
#if (defined(CONFIG_OMAP_GPIO) && !defined(CONFIG_SPL_BUILD)) || \
	(defined(CONFIG_SPL_BUILD) && defined(CONFIG_SPL_GPIO_SUPPORT))
#define OMAP_HSMMC_USE_GPIO
#else
#undef OMAP_HSMMC_USE_GPIO
#endif

/* common definitions for all OMAPs */
#define SYSCTL_SRC	(1 << 25)
#define SYSCTL_SRD	(1 << 26)

struct omap_hsmmc_data {
	struct hsmmc *base_addr;
	struct mmc_config cfg;
#ifdef OMAP_HSMMC_USE_GPIO
#ifdef CONFIG_DM_MMC
	struct gpio_desc cd_gpio;	/* Change Detect GPIO */
	struct gpio_desc wp_gpio;	/* Write Protect GPIO */
	bool cd_inverted;
#else
	int cd_gpio;
	int wp_gpio;
#endif
#endif
};

/* If we fail after 1 second wait, something is really bad */
#define MAX_RETRY_MS	1000

static int mmc_read_data(struct hsmmc *mmc_base, char *buf, unsigned int size);
static int mmc_write_data(struct hsmmc *mmc_base, const char *buf,
			unsigned int siz);

#if defined(OMAP_HSMMC_USE_GPIO) && !defined(CONFIG_DM_MMC)
static int omap_mmc_setup_gpio_in(int gpio, const char *label)
{
	int ret;

#ifndef CONFIG_DM_GPIO
	if (!gpio_is_valid(gpio))
		return -1;
#endif
	ret = gpio_request(gpio, label);
	if (ret)
		return ret;

	ret = gpio_direction_input(gpio);
	if (ret)
		return ret;

	return gpio;
}
#endif

#if defined(CONFIG_OMAP44XX) && defined(CONFIG_TWL6030_POWER)
static void omap4_vmmc_pbias_config(struct mmc *mmc)
{
	u32 value = 0;

	value = readl((*ctrl)->control_pbiaslite);
	value &= ~(MMC1_PBIASLITE_PWRDNZ | MMC1_PWRDNZ);
	writel(value, (*ctrl)->control_pbiaslite);
	/* set VMMC to 3V */
	twl6030_power_mmc_init();
	value = readl((*ctrl)->control_pbiaslite);
	value |= MMC1_PBIASLITE_VMODE | MMC1_PBIASLITE_PWRDNZ | MMC1_PWRDNZ;
	writel(value, (*ctrl)->control_pbiaslite);
}
#endif

#if defined(CONFIG_OMAP54XX) && defined(CONFIG_PALMAS_POWER)
static void omap5_pbias_config(struct mmc *mmc)
{
	u32 value = 0;

	value = readl((*ctrl)->control_pbias);
	value &= ~SDCARD_PWRDNZ;
	writel(value, (*ctrl)->control_pbias);
	udelay(10); /* wait 10 us */
	value &= ~SDCARD_BIAS_PWRDNZ;
	writel(value, (*ctrl)->control_pbias);

	palmas_mmc1_poweron_ldo();

	value = readl((*ctrl)->control_pbias);
	value |= SDCARD_BIAS_PWRDNZ;
	writel(value, (*ctrl)->control_pbias);
	udelay(150); /* wait 150 us */
	value |= SDCARD_PWRDNZ;
	writel(value, (*ctrl)->control_pbias);
	udelay(150); /* wait 150 us */
}
#endif

#define ___SKIP_mmc_board_init_spl_FUNC
#define ___SKIP_mmc_board_init_main_FUNC
static unsigned char mmc_board_init(struct mmc *mmc)
{
#if defined(CONFIG_OMAP34XX)
	t2_t *t2_base = (t2_t *)T2_BASE;
	struct prcm *prcm_base = (struct prcm *)PRCM_BASE;
	u32 pbias_lite;

	pbias_lite = readl(&t2_base->pbias_lite);
	pbias_lite &= ~(PBIASLITEPWRDNZ1 | PBIASLITEPWRDNZ0);
#ifdef CONFIG_TARGET_OMAP3_CAIRO
	/* for cairo board, we need to set up 1.8 Volt bias level on MMC1 */
	pbias_lite &= ~PBIASLITEVMODE0;
#endif
	writel(pbias_lite, &t2_base->pbias_lite);

	writel(pbias_lite | PBIASLITEPWRDNZ1 |
		PBIASSPEEDCTRL0 | PBIASLITEPWRDNZ0,
		&t2_base->pbias_lite);

	writel(readl(&t2_base->devconf0) | MMCSDIO1ADPCLKISEL,
		&t2_base->devconf0);

	writel(readl(&t2_base->devconf1) | MMCSDIO2ADPCLKISEL,
		&t2_base->devconf1);

	/* Change from default of 52MHz to 26MHz if necessary */
	if (!(mmc->cfg->host_caps & MMC_MODE_HS_52MHz))
		writel(readl(&t2_base->ctl_prog_io1) & ~CTLPROGIO1SPEEDCTRL,
			&t2_base->ctl_prog_io1);

	writel(readl(&prcm_base->fclken1_core) |
		EN_MMC1 | EN_MMC2 | EN_MMC3,
		&prcm_base->fclken1_core);

	writel(readl(&prcm_base->iclken1_core) |
		EN_MMC1 | EN_MMC2 | EN_MMC3,
		&prcm_base->iclken1_core);
#endif

#if defined(CONFIG_OMAP44XX) && defined(CONFIG_TWL6030_POWER)
	/* PBIAS config needed for MMC1 only */
	if (mmc->block_dev.dev == 0)
		omap4_vmmc_pbias_config(mmc);
#endif
#if defined(CONFIG_OMAP54XX) && defined(CONFIG_PALMAS_POWER)
	if (mmc->block_dev.dev == 0)
		omap5_pbias_config(mmc);
#endif

	return 0;
}

/* assigns *(mmc_base->stat) =;
  
  */
#define ___SKIP_mmc_init_stream_spl_FUNC
#define ___SKIP_mmc_init_stream_main_FUNC
void mmc_init_stream(struct hsmmc *mmc_base)
{
#define ___SKIP_mmc_init_stream0_spl_START
#define ___SKIP_mmc_init_stream0_main_START  
	ulong start;
// volatile &mmc_base->stat writes mmc_stat_write reads mmc_stat_read;
// volatile &mmc_base->cmd writes mmc_cmd_write;
// volatile &mmc_base->arg writes mmc_arg_write;	
	writel(readl(&mmc_base->con) | INIT_INITSTREAM, &mmc_base->con);

	writel(MMC_CMD0, &mmc_base->cmd);
	#define ___SKIP_mmc_init_stream0_spl_END
	#define ___SKIP_mmc_init_stream0_main_END
	start = get_timer(0);
	//@ loop pragma UNROLL 0;
	while (!(readl(&mmc_base->stat) & CC_MASK)) {
		if (get_timer(0) - start > MAX_RETRY_MS) {
			printf("%s: timedout waiting for cc!\n", __func__);
			return;
		}
	}
#define ___SKIP_mmc_init_stream1_spl_START
#define ___SKIP_mmc_init_stream1_main_START 
	writel(CC_MASK, &mmc_base->stat)
		;
	writel(MMC_CMD0, &mmc_base->cmd)
		;
#define ___SKIP_mmc_init_stream1_spl_END
#define ___SKIP_mmc_init_stream1_main_END
	start = get_timer(0);
	//@ loop pragma UNROLL 0;	
	while (!(readl(&mmc_base->stat) & CC_MASK)) {
		if (get_timer(0) - start > MAX_RETRY_MS) {
		  printf("%s: timedout waiting for cc2!\n", __func__);
			return;
		}
	}
#define ___SKIP_mmc_init_stream2_spl_NEXT
#define ___SKIP_mmc_init_stream2_main_NEXT	
	writel(readl(&mmc_base->con) & ~INIT_INITSTREAM, &mmc_base->con);
}


static int omap_hsmmc_init_setup(struct mmc *mmc)
{
	struct hsmmc *mmc_base;
	unsigned int reg_val;
	unsigned int dsor;
	ulong start;
#define ___SKIP_mmc_init_setup0_spl_NEXT
#define ___SKIP_mmc_init_setup0_main_NEXT
	//@ assert \valid((struct omap_hsmmc_data *)mmc->priv);
	mmc_base = ((struct omap_hsmmc_data *)mmc->priv)->base_addr;
	//@ assert \valid(mmc_base);
	mmc_board_init(mmc);
#define ___SKIP_mmc_init_setup1_spl_START
#define ___SKIP_mmc_init_setup1_main_START	
	writel(readl(&mmc_base->sysconfig) | MMC_SOFTRESET,
		&mmc_base->sysconfig);
#define ___SKIP_mmc_init_setup1_spl_END
#define ___SKIP_mmc_init_setup1_main_END
	start = get_timer(0);
	while ((readl(&mmc_base->sysstatus) & RESETDONE) == 0) {
		if (get_timer(0) - start > MAX_RETRY_MS) {
		  printf("%s: timedout waiting for cc2!\n", __func__);
			return TIMEOUT;
		}
	}
#define ___SKIP_mmc_init_setup2_spl_NEXT
#define ___SKIP_mmc_init_setup2_main_NEXT	
	writel(readl(&mmc_base->sysctl) | SOFTRESETALL, &mmc_base->sysctl);
	start = get_timer(0);
	//@ loop pragma UNROLL 0;	
	while ((readl(&mmc_base->sysctl) & SOFTRESETALL) != 0x0) {
	  if (get_timer(0) - start > MAX_RETRY_MS) {
	    printf("%s: timedout waiting for softresetall!\n", __func__);
			return TIMEOUT;
		}
	}
#define ___SKIP_mmc_init_setup4_spl_START
#define ___SKIP_mmc_init_setup4_main_START	
	
	writel(DTW_1_BITMODE | SDBP_PWROFF | SDVS_3V0, &mmc_base->hctl);
	writel(readl(&mmc_base->capa) | VS30_3V0SUP | VS18_1V8SUP,
		&mmc_base->capa);

	reg_val = readl(&mmc_base->con) & RESERVED_MASK;

	writel(CTPL_MMC_SD | reg_val | WPP_ACTIVEHIGH | CDP_ACTIVEHIGH |
		MIT_CTO | DW8_1_4BITMODE | MODE_FUNC | STR_BLOCK |
		HR_NOHOSTRESP | INIT_NOINIT | NOOPENDRAIN, &mmc_base->con);

	dsor = 240;
	mmc_reg_out(&mmc_base->sysctl, (ICE_MASK | DTO_MASK | CEN_MASK),
		(ICE_STOP | DTO_15THDTO | CEN_DISABLE));
	mmc_reg_out(&mmc_base->sysctl, ICE_MASK | CLKD_MASK,
		(dsor << CLKD_OFFSET) | ICE_OSCILLATE);
#define ___SKIP_mmc_init_setup4_spl_END
#define ___SKIP_mmc_init_setup4_main_END
	start = get_timer(0);
	while ((readl(&mmc_base->sysctl) & ICS_MASK) == ICS_NOTREADY) {
		if (get_timer(0) - start > MAX_RETRY_MS) {
			printf("%s: timedout waiting for ics!\n", __func__);
			return TIMEOUT;
		}
	}
#define ___SKIP_mmc_init_setup3_spl_START
#define ___SKIP_mmc_init_setup3_main_START
	writel(readl(&mmc_base->sysctl) | CEN_ENABLE, &mmc_base->sysctl);

	writel(readl(&mmc_base->hctl) | SDBP_PWRON, &mmc_base->hctl);

	writel(IE_BADA | IE_CERR | IE_DEB | IE_DCRC | IE_DTO | IE_CIE |
		IE_CEB | IE_CCRC | IE_CTO | IE_BRR | IE_BWR | IE_TC | IE_CC,
		&mmc_base->ie);
#define ___SKIP_mmc_init_setup3_spl_END
#define ___SKIP_mmc_init_setup3_main_END

	mmc_init_stream(mmc_base);

	return 0;
}

/*
 * MMC controller internal finite state machine reset
 *
 * Used to reset command or data internal state machines, using respectively
 * SRC or SRD bit of SYSCTL register
 */
static void mmc_reset_controller_fsm(struct hsmmc *mmc_base, u32 bit)
{
	ulong start;
#define ___SKIP_mmc_reset_controller_fsm_spl_NEXT
#define ___SKIP_mmc_reset_controller_fsm_main_NEXT
	mmc_reg_out(&mmc_base->sysctl, bit, bit);

	/*
	 * CMD(DAT) lines reset procedures are slightly different
	 * for OMAP3 and OMAP4(AM335x,OMAP5,DRA7xx).
	 * According to OMAP3 TRM:
	 * Set SRC(SRD) bit in MMCHS_SYSCTL register to 0x1 and wait until it
	 * returns to 0x0.
	 * According to OMAP4(AM335x,OMAP5,DRA7xx) TRMs, CMD(DATA) lines reset
	 * procedure steps must be as follows:
	 * 1. Initiate CMD(DAT) line reset by writing 0x1 to SRC(SRD) bit in
	 *    MMCHS_SYSCTL register (SD_SYSCTL for AM335x).
	 * 2. Poll the SRC(SRD) bit until it is set to 0x1.
	 * 3. Wait until the SRC (SRD) bit returns to 0x0
	 *    (reset procedure is completed).
	 */
#if defined(CONFIG_OMAP44XX) || defined(CONFIG_OMAP54XX) || \
	defined(CONFIG_AM33XX) || defined(CONFIG_AM43XX)
	if (!(readl(&mmc_base->sysctl) & bit)) {
		start = get_timer(0);
		//@ loop pragma UNROLL 0;
		while (!(readl(&mmc_base->sysctl) & bit)) {
			if (get_timer(0) - start > MAX_RETRY_MS)
				return;
		}
	}
#endif
	start = get_timer(0);
	//@ loop pragma UNROLL 0;
	while ((readl(&mmc_base->sysctl) & bit) != 0) {
	  if (get_timer(0) - start > MAX_RETRY_MS) {
#define ___FRAMAC_delete_line_spl_PATCH			  
		  printf("%s: timedout waiting for sysctl %x to clear\n",
#define ___FRAMAC_delete_line_spl_PATCH
			 __func__, bit);
			return;
		}
	}
}

static int omap_hsmmc_send_cmd(struct mmc *mmc, struct mmc_cmd *cmd,
			struct mmc_data *data)
{
	struct hsmmc *mmc_base;
	unsigned int flags, mmc_stat;
	ulong start;

	mmc_base = ((struct omap_hsmmc_data *)mmc->priv)->base_addr;
// volatile &mmc_base->stat writes mmc_stat_write reads mmc_stat_read;
// volatile &mmc_base->rsp10 reads reads_mmcrsp;
// volatile &mmc_base->rsp32 reads reads_mmcrsp;
// volatile &mmc_base->rsp54 reads reads_mmcrsp;
// volatile &mmc_base->rsp72 reads reads_mmcrsp;
// volatile &mmc_base->cmd writes mmc_cmd_write;


	start = get_timer(0);

	//@ loop pragma UNROLL 0;
	while ((readl(&mmc_base->pstate) & (DATI_MASK | CMDI_MASK)) != 0) { if (get_timer(0) - start > MAX_RETRY_MS) {
#define ___FRAMAC_delete_line_spl_PATCH		    
	    printf("%s: timedout waiting on cmd inhibit to clear\n", __func__);
	    return TIMEOUT; }}

	writel(0xFFFFFFFF, &mmc_base->stat);
	
	start = get_timer(0);
	//@ loop pragma UNROLL 0;
	while (readl(&mmc_base->stat)) {
	  if (get_timer(0) - start > MAX_RETRY_MS) {
#define ___FRAMAC_delete_line_spl_PATCH
	    printf("%s: timedout waiting for STAT (%x) to clear\n", __func__, readl(&mmc_base->stat));
	    return TIMEOUT;
	  }
	}	
	/*
	 * CMDREG
	 * CMDIDX[13:8]	: Command index
	 * DATAPRNT[5]	: Data Present Select
	 * ENCMDIDX[4]	: Command Index Check Enable
	 * ENCMDCRC[3]	: Command CRC Check Enable
	 * RSPTYP[1:0]
	 *	00 = No Response
	 *	01 = Length 136
	 *	10 = Length 48
	 *	11 = Length 48 Check busy after response
	 */
	/* Delay added before checking the status of frq change
	 * retry not supported by mmc.c(core file)
	 */
	
	if (cmd->cmdidx == SD_CMD_APP_SEND_SCR) udelay(50000); /* wait 50 ms */

	if (!(cmd->resp_type & MMC_RSP_PRESENT)) flags = 0;
	else if (cmd->resp_type & MMC_RSP_136) flags = RSP_TYPE_LGHT136 | CICE_NOCHECK;
	else if (cmd->resp_type & MMC_RSP_BUSY) flags = RSP_TYPE_LGHT48B;
	else flags = RSP_TYPE_LGHT48;

	/* enable default flags */
	flags =	flags | (CMD_TYPE_NORMAL | CICE_NOCHECK | CCCE_NOCHECK | MSBS_SGLEBLK | ACEN_DISABLE | BCE_DISABLE | DE_DISABLE);	
	if (cmd->resp_type & MMC_RSP_CRC) flags |= CCCE_CHECK;
	if (cmd->resp_type & MMC_RSP_OPCODE) flags |= CICE_CHECK;

	if (data) {
		if ((cmd->cmdidx == MMC_CMD_READ_MULTIPLE_BLOCK) ||
			 (cmd->cmdidx == MMC_CMD_WRITE_MULTIPLE_BLOCK)) {
			flags |= (MSBS_MULTIBLK | BCE_ENABLE);
			data->blocksize = 512;
			writel(data->blocksize | (data->blocks << 16),
							&mmc_base->blk);
		} else
			writel(data->blocksize | NBLK_STPCNT, &mmc_base->blk);

		if (data->flags & MMC_DATA_READ)
			flags |= (DP_DATA | DDIR_READ);
		else
			flags |= (DP_DATA | DDIR_WRITE);
	}
#define ___SKIP_omap_hsmmc_send_cmd_spl_START
#define ___SKIP_omap_hsmmc_send_cmd_main_START
	writel(cmd->cmdarg, &mmc_base->arg);
	udelay(20);		/* To fix "No status update" error on eMMC */
	writel((cmd->cmdidx << 24) | flags, &mmc_base->cmd);
	start = get_timer(0);
	//@ loop pragma UNROLL 1;
	do {
		mmc_stat = readl(&mmc_base->stat);
		if (get_timer(0) - start > MAX_RETRY_MS) {		  
#define ___SKIP_omap_hsmmc_send_cmd_spl_END
#define ___SKIP_omap_hsmmc_send_cmd_main_END
#define ___FRAMAC_delete_line_spl_PATCH		  
             	        printf("%s : timeout: No status update\n", __func__);
			return TIMEOUT;
		}
	} while (!mmc_stat);
	if ((mmc_stat & IE_CTO) != 0) {
		mmc_reset_controller_fsm(mmc_base, SYSCTL_SRC);
		return TIMEOUT;
	} else if ((mmc_stat & ERRI_MASK) != 0)
		return -1;

	if (mmc_stat & CC_MASK) {
		writel(CC_MASK, &mmc_base->stat);
		if (cmd->resp_type & MMC_RSP_PRESENT) {
			if (cmd->resp_type & MMC_RSP_136) {
				/* response type 2 */
			  cmd->response[3] = readl(&mmc_base->rsp10);
			  cmd->response[2] = readl(&mmc_base->rsp32);
			  cmd->response[1] = readl(&mmc_base->rsp54);
			  cmd->response[0] = readl(&mmc_base->rsp76);
			  //@assert \valid(&(cmd->response[0]));
			  //@assert \valid(&(cmd->response[1]));
			  //@assert \valid(&(cmd->response[2]));
			  //@assert \valid(&(cmd->response[3]));			  
			  //@assert \initialized(&(cmd->response[0]));
			  //@assert \initialized(&(cmd->response[1]));
			  //@assert \initialized(&(cmd->response[2]));
			  //@assert \initialized(&(cmd->response[3]));			  
			} else
			  /* response types 1, 1b, 3, 4, 5, 6 */
			  cmd->response[0] = readl(&mmc_base->rsp10);
			//@assert \valid(&cmd->response[0]);
			//@assert \initialized(&cmd->response[0]);			
		}

	}

	if (data && (data->flags & MMC_DATA_READ)) {
		mmc_read_data(mmc_base,	data->dest,
				data->blocksize * data->blocks);
	} else if (data && (data->flags & MMC_DATA_WRITE)) {
		mmc_write_data(mmc_base, data->src,
				data->blocksize * data->blocks);
	}
	return 0;
}

static int mmc_read_data(struct hsmmc *mmc_base, char *buf, unsigned int size)
{
	unsigned int *output_buf = (unsigned int *)buf;
	unsigned int mmc_stat;
	unsigned int count;
	// assert \valid(output_buf+(0..size));	
	// ghost fat_mmc_read_size = size;
	// ghost image_header_copied = 0;	
	// volatile &mmc_base->stat writes mmc_stat_write reads mmc_stat_read_data;
	// volatile &mmc_base->data reads read_mmc_data;
	/*
	 * Start Polled Read
	 */
#define ___SKIP_mmc_read_data0_spl_START
#define ___SKIP_mmc_read_data0_main_START	
	count = (size > MMCSD_SECTOR_SIZE) ? MMCSD_SECTOR_SIZE : size;
	count /= 4;
#define ___SKIP_mmc_read_data0_spl_END
#define ___SKIP_mmc_read_data0_main_END
	//@ loop pragma UNROLL count;
	while (size) {
	  ulong start = get_timer(0);
#define ___SKIP_mmc_read_data1_spl_START
#define ___SKIP_mmc_read_data1_main_START
	  //@ loop pragma UNROLL 1;
		do {
			mmc_stat = readl(&mmc_base->stat);
#define ___SKIP_mmc_read_data1_spl_END
#define ___SKIP_mmc_read_data1_main_END
			if (get_timer(0) - start > MAX_RETRY_MS) {
#define ___SKIP_mmc_read_data2_spl_NEXT
#define ___SKIP_mmc_read_data2_main_NEXT
#define ___FRAMAC_delete_line_spl_PATCH			  
			  printf("%s: timedout waiting for status!\n",
#define ___FRAMAC_delete_line_spl_PATCH				       
						__func__);
				return TIMEOUT;
			}
		} while (mmc_stat == 0);
		if ((mmc_stat & (IE_DTO | IE_DCRC | IE_DEB)) != 0)
			mmc_reset_controller_fsm(mmc_base, SYSCTL_SRD);

		if ((mmc_stat & ERRI_MASK) != 0)
			return 1;

		if (mmc_stat & BRR_MASK) {
			unsigned int k;

			writel(readl(&mmc_base->stat) | BRR_MASK,
				&mmc_base->stat);
			//@ loop pragma UNROLL count;			
			for (k = 0; k < count; k++) {	
#define ___LONGWRITE_mmc_read_data_spl_BREAK
#define ___LONGWRITE_mmc_read_data_main_BREAK
			  //@ assert \valid(output_buf+(0..1));
			  *output_buf = readl(&mmc_base->data);
				output_buf++;
			}
			size -= (count*4);
		}

		if (mmc_stat & BWR_MASK)
			writel(readl(&mmc_base->stat) | BWR_MASK,
				&mmc_base->stat);

		if (mmc_stat & TC_MASK) {
			writel(readl(&mmc_base->stat) | TC_MASK,
				&mmc_base->stat);
			break;
		}
				       }
	return 0;
}

static int mmc_write_data(struct hsmmc *mmc_base, const char *buf,
				unsigned int size)
{
	unsigned int *input_buf = (unsigned int *)buf;
	unsigned int mmc_stat;
	unsigned int count;
// volatile &mmc_base->stat writes mmc_stat_write reads mmc_stat_write_data;
	/*
	 * Start Polled Write
	 */
#define ___SKIP_mmc_write_data0_spl_START
#define ___SKIP_mmc_write_data0_main_START	
	count = (size > MMCSD_SECTOR_SIZE) ? MMCSD_SECTOR_SIZE : size;
	count /= 4;
#define ___SKIP_mmc_write_data0_spl_END
#define ___SKIP_mmc_write_data0_main_END

	while (size) {
		ulong start = get_timer(0);
		//@ loop pragma UNROLL 1;
		do {
			mmc_stat = readl(&mmc_base->stat);
			if (get_timer(0) - start > MAX_RETRY_MS) {
#define ___FRAMAC_delete_line_spl_PATCH			  
			  printf("%s: timedout waiting for status!\n",
#define ___FRAMAC_delete_line_spl_PATCH				 
						__func__);
				return TIMEOUT;
			}
		} while (mmc_stat == 0);
#define ___SKIP_mmc_write_data2_spl_NEXT
#define ___SKIP_mmc_write_data2_main_NEXT
		if ((mmc_stat & (IE_DTO | IE_DCRC | IE_DEB)) != 0)
			mmc_reset_controller_fsm(mmc_base, SYSCTL_SRD);

		if ((mmc_stat & ERRI_MASK) != 0)
			return 1;

		if (mmc_stat & BWR_MASK) {
			unsigned int k;

			writel(readl(&mmc_base->stat) | BWR_MASK,
					&mmc_base->stat);
			for (k = 0; k < count; k++) {
				writel(*input_buf, &mmc_base->data);
				input_buf++;
			}
			size -= (count*4);
		}
		if (mmc_stat & BRR_MASK)
#define ___SKIP_mmc_write_data1_spl_NEXT
#define ___SKIP_mmc_write_data1_main_NEXT
			writel(readl(&mmc_base->stat) | BRR_MASK,
				&mmc_base->stat);

		if (mmc_stat & TC_MASK) {
#define ___SKIP_mmc_write_data3_spl_NEXT
#define ___SKIP_mmc_write_data3_main_NEXT		       	       
			writel(readl(&mmc_base->stat) | TC_MASK,
				&mmc_base->stat);
			break;
		}
	}
	return 0;
}

static void omap_hsmmc_set_ios(struct mmc *mmc)
{
#define ___SKIP_omap_hsmmc_set_ios_spl_START
#define ___SKIP_omap_hsmmc_set_ios_main_START  
	struct hsmmc *mmc_base;
	unsigned int dsor = 0;
	ulong start;

	mmc_base = ((struct omap_hsmmc_data *)mmc->priv)->base_addr;
	/* configue bus width */
	switch (mmc->bus_width) {
	case 8:
		writel(readl(&mmc_base->con) | DTW_8_BITMODE,
			&mmc_base->con);
		break;

	case 4:
		writel(readl(&mmc_base->con) & ~DTW_8_BITMODE,
			&mmc_base->con);
		writel(readl(&mmc_base->hctl) | DTW_4_BITMODE,
			&mmc_base->hctl);
		break;

	case 1:
	default:
		writel(readl(&mmc_base->con) & ~DTW_8_BITMODE,
			&mmc_base->con);
		writel(readl(&mmc_base->hctl) & ~DTW_4_BITMODE,
			&mmc_base->hctl);
		break;
	}

	/* configure clock with 96Mhz system clock.
	 */	
	if (mmc->clock != 0) {
	  //@ assert mmc->clock != 0;
		dsor = (MMC_CLOCK_REFERENCE * 1000000 / mmc->clock);
		if ((MMC_CLOCK_REFERENCE * 1000000) / dsor > mmc->clock)
			dsor++;
	}

	mmc_reg_out(&mmc_base->sysctl, (ICE_MASK | DTO_MASK | CEN_MASK),
				(ICE_STOP | DTO_15THDTO | CEN_DISABLE));

	mmc_reg_out(&mmc_base->sysctl, ICE_MASK | CLKD_MASK,
				(dsor << CLKD_OFFSET) | ICE_OSCILLATE);
#define ___SKIP_omap_hsmmc_set_ios_spl_END
#define ___SKIP_omap_hsmmc_set_ios_main_END
	start = get_timer(0);
	//@ loop pragma UNROLL 0;
	while ((readl(&mmc_base->sysctl) & ICS_MASK) == ICS_NOTREADY) {
		if (get_timer(0) - start > MAX_RETRY_MS) {
#define ___FRAMAC_delete_line_spl_PATCH		  
			printf("%s: timedout waiting for ics!\n", __func__);
			return;	
		}
	}
	writel(readl(&mmc_base->sysctl) | CEN_ENABLE, &mmc_base->sysctl);
}

#ifdef OMAP_HSMMC_USE_GPIO
#ifdef CONFIG_DM_MMC
static int omap_hsmmc_getcd(struct mmc *mmc)
{
	struct omap_hsmmc_data *priv = mmc->priv;
	int value;

	value = dm_gpio_get_value(&priv->cd_gpio);
	/* if no CD return as 1 */
	if (value < 0)
		return 1;

	if (priv->cd_inverted)
		return !value;
	return value;
}

static int omap_hsmmc_getwp(struct mmc *mmc)
{
	struct omap_hsmmc_data *priv = mmc->priv;
	int value;

	value = dm_gpio_get_value(&priv->wp_gpio);
	/* if no WP return as 0 */
	if (value < 0)
		return 0;
	return value;
}
#else
//@ ghost int i;

/*@ assigns \result \from i;
  @ ensures \result == 1; 
  @*/
static int omap_hsmmc_getcd(struct mmc *mmc)
{
	struct omap_hsmmc_data *priv_data = mmc->priv;
	int cd_gpio;

	/* if no CD return as 1 */
	cd_gpio = priv_data->cd_gpio;
	if (cd_gpio < 0)
		return 1;

	/* NOTE: assumes card detect signal is active-low */
	return !gpio_get_value(cd_gpio);
}

static int omap_hsmmc_getwp(struct mmc *mmc)
{
  struct omap_hsmmc_data *priv_data = mmc->priv;
	//@ assert \valid(priv_data);	
	int wp_gpio;

	/* if no WP return as 0 */
	wp_gpio = priv_data->wp_gpio;
	if (wp_gpio < 0)
		return 0;

	/* NOTE: assumes write protect signal is active-high */
	return gpio_get_value(wp_gpio);
}
#endif
#endif

static const struct mmc_ops omap_hsmmc_ops = {
	.send_cmd	= omap_hsmmc_send_cmd,
	.set_ios	= omap_hsmmc_set_ios,
	.init		= omap_hsmmc_init_setup,
#ifdef OMAP_HSMMC_USE_GPIO
	.getcd		= omap_hsmmc_getcd,
	.getwp		= omap_hsmmc_getwp,
#endif
};

#ifndef CONFIG_DM_MMC
int omap_mmc_init(int dev_index, uint host_caps_mask, uint f_max, int cd_gpio,
		int wp_gpio)
{
	struct mmc *mmc;
	struct omap_hsmmc_data *priv_data;
	struct mmc_config *cfg;
	uint host_caps_val;

	priv_data = malloc(sizeof(*priv_data));
	if (priv_data == NULL)
		return -1;
	// @ assert \valid{priv_data}
	// @ assert \valid{priv_data->base_addr}
	// @ assert \valid{priv_data->cd_gpio}		
	host_caps_val = MMC_MODE_4BIT | MMC_MODE_HS_52MHz | MMC_MODE_HS;

	switch (dev_index) {
	case 0:
		priv_data->base_addr = (struct hsmmc *)OMAP_HSMMC1_BASE;
		break;
#ifdef OMAP_HSMMC2_BASE
	case 1:
		priv_data->base_addr = (struct hsmmc *)OMAP_HSMMC2_BASE;
#if (defined(CONFIG_OMAP44XX) || defined(CONFIG_OMAP54XX) || \
	defined(CONFIG_DRA7XX) || defined(CONFIG_AM57XX) || \
	defined(CONFIG_AM43XX) || defined(CONFIG_SOC_KEYSTONE)) && \
		defined(CONFIG_HSMMC2_8BIT)
		/* Enable 8-bit interface for eMMC on OMAP4/5 or DRA7XX */
		host_caps_val |= MMC_MODE_8BIT;
#endif
		break;
#endif
#ifdef OMAP_HSMMC3_BASE
	case 2:
		priv_data->base_addr = (struct hsmmc *)OMAP_HSMMC3_BASE;
#if (defined(CONFIG_DRA7XX) || defined(CONFIG_AM57XX)) && defined(CONFIG_HSMMC3_8BIT)
		/* Enable 8-bit interface for eMMC on DRA7XX */
		host_caps_val |= MMC_MODE_8BIT;
#endif
		break;
#endif
	default:
		priv_data->base_addr = (struct hsmmc *)OMAP_HSMMC1_BASE;
		return 1;
	}


#ifdef OMAP_HSMMC_USE_GPIO
	/* on error gpio values are set to -1, which is what we want */
	priv_data->cd_gpio = omap_mmc_setup_gpio_in(cd_gpio, "mmc_cd");
	priv_data->wp_gpio = omap_mmc_setup_gpio_in(wp_gpio, "mmc_wp");
#endif

	cfg = &priv_data->cfg;

	cfg->name = "OMAP SD/MMC";
	cfg->ops = &omap_hsmmc_ops;

	cfg->voltages = MMC_VDD_32_33 | MMC_VDD_33_34 | MMC_VDD_165_195;
	cfg->host_caps = host_caps_val & ~host_caps_mask;

	cfg->f_min = 400000;

	if (f_max != 0)
		cfg->f_max = f_max;
	else {
		if (cfg->host_caps & MMC_MODE_HS) {
			if (cfg->host_caps & MMC_MODE_HS_52MHz)
				cfg->f_max = 52000000;
			else
				cfg->f_max = 26000000;
		} else
			cfg->f_max = 20000000;
	}

	cfg->b_max = CONFIG_SYS_MMC_MAX_BLK_COUNT;

#if defined(CONFIG_OMAP34XX)
	/*
	 * Silicon revs 2.1 and older do not support multiblock transfers.
	 */
	if ((get_cpu_family() == CPU_OMAP34XX) && (get_cpu_rev() <= CPU_3XX_ES21))
		cfg->b_max = 1;
#endif
	mmc = mmc_create(cfg, priv_data);
	if (mmc == NULL)
		return -1;

	return 0;
}
#else
static int omap_hsmmc_ofdata_to_platdata(struct udevice *dev)
{
	struct omap_hsmmc_data *priv = dev_get_priv(dev);
	const void *fdt = gd->fdt_blob;
	int node = dev->of_offset;
	struct mmc_config *cfg;
	int val;

	priv->base_addr = (struct hsmmc *)dev_get_addr(dev);
	cfg = &priv->cfg;

	cfg->host_caps = MMC_MODE_HS_52MHz | MMC_MODE_HS;
	val = fdtdec_get_int(fdt, node, "bus-width", -1);
	if (val < 0) {
		printf("error: bus-width property missing\n");
		return -ENOENT;
	}

	switch (val) {
	case 0x8:
		cfg->host_caps |= MMC_MODE_8BIT;
	case 0x4:
		cfg->host_caps |= MMC_MODE_4BIT;
		break;
	default:
		printf("error: invalid bus-width property\n");
		return -ENOENT;
	}

	cfg->f_min = 400000;
	cfg->f_max = fdtdec_get_int(fdt, node, "max-frequency", 52000000);
	cfg->voltages = MMC_VDD_32_33 | MMC_VDD_33_34 | MMC_VDD_165_195;
	cfg->b_max = CONFIG_SYS_MMC_MAX_BLK_COUNT;

	priv->cd_inverted = fdtdec_get_bool(fdt, node, "cd-inverted");

	return 0;
}

static int omap_hsmmc_probe(struct udevice *dev)
{
	struct mmc_uclass_priv *upriv = dev_get_uclass_priv(dev);
	struct omap_hsmmc_data *priv = dev_get_priv(dev);
	struct mmc_config *cfg;
	struct mmc *mmc;

	cfg = &priv->cfg;
	cfg->name = "OMAP SD/MMC";
	cfg->ops = &omap_hsmmc_ops;

	mmc = mmc_create(cfg, priv);
	if (mmc == NULL)
		return -1;

	upriv->mmc = mmc;

	return 0;
}

static const struct udevice_id omap_hsmmc_ids[] = {
	{ .compatible = "ti,omap3-hsmmc" },
	{ .compatible = "ti,omap4-hsmmc" },
	{ .compatible = "ti,am33xx-hsmmc" },
	{ }
};

U_BOOT_DRIVER(omap_hsmmc) = {
	.name	= "omap_hsmmc",
	.id	= UCLASS_MMC,
	.of_match = omap_hsmmc_ids,
	.ofdata_to_platdata = omap_hsmmc_ofdata_to_platdata,
	.probe	= omap_hsmmc_probe,
	.priv_auto_alloc_size = sizeof(struct omap_hsmmc_data),
};
#endif
