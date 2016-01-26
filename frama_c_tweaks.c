/*
 * Copyright (c) 2015 Andreas Bießmann <andreas.devel@googlemail.com>
 *
 * Copyright (c) 2011 The Chromium OS Authors.
 * (C) Copyright 2002-2006
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Marius Groeger <mgroeger@sysgo.de>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>

DECLARE_GLOBAL_DATA_PTR;

#include <configs/ti_armv7_common.h>
#include <spl.h> // arch/arm/include/spl.h
#include <asm/arch-omap3/clock.h>

u32 *sp;
u32 *r1;
u32 *r0;
#define ___FRAMAC_go_to_speed_spl_ADDR_PATCH
u32 *framac_go_to_speed = 0;
#define ___FRAMAC_lowlevel_init_spl_ADDR_PATCH  
u32 *framac_end = lowlevel_init;
#define ___FRAMAC___bss_end_spl_ADDR_PATCH  
u32 *framac__bss_end = 0;
#define ___FRAMAC___bss_start_spl_ADDR_PATCH    
u32 *framac__bss_start = 0;  
u32 *framac_ret;
void cpy_clk_code(u32 *ptr);
void lowlevel_init_finish();
ulong board_init_f_mem(ulong top);
int _main();
int _main_finish();
void s_init();
u32 spl_relocate_stack_gd();
void frama_go();
int lowlevel_init();
struct omap_boot_parameters bxparams = {0, 8, 0, 0, 2, 0};
struct omap_boot_parameters **bxparamptr = 0x4020E024;

/*@ terminates \false;
  @ ensures \false;
*/
#define ___FRAMAC_artifical_spl_ENTRYPOINT
void frama_go()
{
#line 23 "arch/arm/cpu/armv7/omap-common/lowlevel_init.S"
  *bxparamptr =  (u32 *) &bxparams;
#line 50 "arch/arm/cpu/armv7/omap3/lowlevel_init.S"    
  lowlevel_init();
#line 57 "frama_c_tweaks.c"  
}
/*@ terminates \false;
  @ ensures \false;
*/
int lowlevel_init() {  
  sp = LOW_LEVEL_SRAM_STACK;
#line 185 "arch/arm/cpu/armv7/omap3/lowlevel_init.S"  
  *sp = &frama_go;  
#line 194 "arch/arm/cpu/armv7/omap3/lowlevel_init.S"   
  cpy_clk_code(SRAM_CLK_CODE);
#line 68 "frama_c_tweaks.c"  
}
/*@ terminates \false;
  @ ensures \false;
*/
void lowlevel_init_finish()
{
#line 202 "arch/arm/cpu/armv7/omap3/lowlevel_init.S"  
  s_init(); // not defined in any header
#line 77 "frama_c_tweaks.c"  
}

/*@ terminates \false;
  @ ensures \false;
*/
void cpy_clk_code(u32 *ptr)
{
  r0 = framac_go_to_speed;
  while (ptr < framac_end) {
#line 53 "arch/arm/cpu/armv7/omap3/lowlevel_init.S"    
    *ptr = *r0;
#line 52 "arch/arm/cpu/armv7/omap3/lowlevel_init.S"        
    ptr++;
#line 53 "arch/arm/cpu/armv7/omap3/lowlevel_init.S"    
    r0++;
#line 92 "frama_c_tweaks.c"    
  }
#line 59 "arch/arm/cpu/armv7/omap3/lowlevel_init.S"   
  lowlevel_init_finish();
#line 97 "frama_c_tweaks.c"  
}
/*@ terminates \false;
  ensures \false;
*/
int _main()
{
  //  u32 *r1;
  //  u32 *r0;

  //#define ___FRAMAC___bss_end_spl_ADDR_PATCH  
  //u32 *__bss_end = 0;
  //#define ___FRAMAC___bss_start_spl_ADDR_PATCH    
  //u32 *__bss_start = 0;

  sp = CONFIG_SYS_INIT_SP_ADDR; // ti_armv7_commoh.h
  //  bic	sp, sp, #7	/* 8-byte alignment for ABI compliance */
#line 86 "arch/arm/lib/crt0.S"    
  board_init_f_mem(sp); //common.h
#line 116 "frama_c_tweaks.c"  
  return -1; // shouldn't happen
}
ulong board_init_f_mem_finish(ulong framac_ret)
{
#line 121 "frama_c_tweaks.c"
  sp = framac_ret;
#line 94 "arch/arm/lib/crt0.S"  
  board_init_f(0);  //common.h
#line 125 "frama_c_tweaks.c"
  return -1;
}
/*@ terminates \false;
  ensures \false;
*/
int _main_finish() {
  //u32 *r1 = 0;
  //  u32 *r0 = 0;  
#line 140 "arch/arm/lib/crt0.S"  
  framac_ret = (u32) spl_relocate_stack_gd(); // not in any header but defined in spl/spl.c
#line 136 "frama_c_tweaks.c"  
  if (framac_ret != 0) {
    sp = framac_ret;
  }
  clear_bss();
}
/*@ terminates \false;
  @ ensures \false;
*/
int clear_bss(){
  // clear out bss, spl.h
  r1 = framac__bss_end; //@ assert \valid(r1);
  r0 = framac__bss_start; //@ assert \valid(r0);
  //@ loop pragma WIDEN_HINTS r0, 0x80000000, 0x80030144;
  while (r0 != r1) { //@assert \pointer_comparable(r0, r1);
#line 167 "arch/arm/lib/crt0.S"
    //@ assert \valid(r0+(0..4));
    *r0 = 0;
    r0++;
#line 155 "frama_c_tweaks.c"    
  }
  // dest_addr = gd->#gd_relocaddr
#line 184 "arch/arm/lib/crt0.S"  
  board_init_r(gd, gd->relocaddr); //common.h
#line 160 "frama_c_tweaks.c"  
}


dpll_param *get_36x_core_dpll_param()
{
#define ___FRAMAC_core_36x_dpll_param_spl_ADDR_PATCH
  dpll_param *core_36x_dpll_param;
  return core_36x_dpll_param;
}

dpll_param *get_36x_per_dpll_param()
{
#define ___FRAMAC_per_36x_dpll_param_spl_ADDR_PATCH
  dpll_param *per_36x_dpll_param;
  return per_36x_dpll_param;
}

dpll_param *get_36x_per2_dpll_param()
{
#define ___FRAMAC_per2_36x_dpll_param_spl_ADDR_PATCH
  dpll_param *per2_36x_dpll_param;
  return per2_36x_dpll_param;
}

dpll_param *get_36x_iva_dpll_param()
{
#define ___FRAMAC_iva_36x_dpll_param_spl_ADDR_PATCH
  dpll_param *iva_36x_dpll_param;
  return iva_36x_dpll_param;
}

dpll_param *get_36x_mpu_dpll_param()
{
#define ___FRAMAC_mpu_36x_dpll_param_spl_ADDR_PATCH
  dpll_param *mpu_36x_dpll_param;
  return mpu_36x_dpll_param;
}


dpll_param *get_core_dpll_param()
{
#define ___FRAMAC_core_dpll_param_spl_ADDR_PATCH
  dpll_param *core_dpll_param;
  return core_dpll_param;
}


dpll_param *get_per_dpll_param()
{
#define ___FRAMAC_per_dpll_param_spl_ADDR_PATCH
  dpll_param *per_dpll_param;
  return per_dpll_param;
}

dpll_param *get_per2_dpll_param()
{
#define ___FRAMAC_per2_dpll_param_spl_ADDR_PATCH
  dpll_param *per2_dpll_param;
  return per2_dpll_param;
}

dpll_param *get_iva_dpll_param()
{
#define ___FRAMAC_iva_dpll_param_spl_ADDR_PATCH
  dpll_param *iva_dpll_param;
  return iva_dpll_param;
}

dpll_param *get_mpu_dpll_param()
{
#define ___FRAMAC_mpu_dpll_param_spl_ADDR_PATCH
  dpll_param *mpu_dpll_param;
  return mpu_dpll_param;
}
