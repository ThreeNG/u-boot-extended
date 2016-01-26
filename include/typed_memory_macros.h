#ifndef _TYPED_MEMORY_MACROS_H
#define _TYPED_MEMORY_MACROS_H

#ifdef CONFIG_VALIDATE
#define CLEAR_BSS ldr	r0, =__bss_start	/* this is auto-relocated! */ \
\
#ifdef CONFIG_USE_ARCH_MEMSET \
	ldr	r3, =__bss_end		/* this is auto-relocated! */ \
	mov	r1, #0x00000000		/* prepare zero to clear BSS */ \
\
	subs	r2, r3, r0		/* r2 = memset len */ \
	bl	memset \
#else \
	ldr	r1, =__bss_end		/* this is auto-relocated! */ \
	mov	r2, #0x00000000		/* prepare zero to clear BSS */ \
\
clbss_l:cmp	r0, r1			/* while not at end of BSS */ \
#if defined(CONFIG_CPU_V7M) \
	itt	lo \
#endif \
	strlo	r2, [r0]		/* clear 32-bit BSS word */ \
	addlo	r0, r0, #4		/* move to next */ \
	blo	clbss_l \
#endif \

#define EVENT_CLEAR_BSS

#endif // CONFIG_VALIDATE
#endif // _TYPED_MEMORY_MACROS_H
