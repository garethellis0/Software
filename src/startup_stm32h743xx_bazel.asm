
bazel-bin/firmware_new/boards/frankie_v1/_objs/frankie_v1_main_lib/startup_stm32h743xx.o:     file format elf32-littlearm


Disassembly of section .text:

00000000 <.text>:
	...

Disassembly of section .text.Reset_Handler:

00000000 <Reset_Handler>:
   0:	f8df d034 	ldr.w	sp, [pc, #52]	; 38 <LoopFillZerobss+0x14>
   4:	2100      	movs	r1, #0
   6:	e003      	b.n	10 <LoopCopyDataInit>

00000008 <CopyDataInit>:
   8:	4b0c      	ldr	r3, [pc, #48]	; (3c <LoopFillZerobss+0x18>)
   a:	585b      	ldr	r3, [r3, r1]
   c:	5043      	str	r3, [r0, r1]
   e:	3104      	adds	r1, #4

00000010 <LoopCopyDataInit>:
  10:	480b      	ldr	r0, [pc, #44]	; (40 <LoopFillZerobss+0x1c>)
  12:	4b0c      	ldr	r3, [pc, #48]	; (44 <LoopFillZerobss+0x20>)
  14:	1842      	adds	r2, r0, r1
  16:	429a      	cmp	r2, r3
  18:	d3f6      	bcc.n	8 <CopyDataInit>
  1a:	4a0b      	ldr	r2, [pc, #44]	; (48 <LoopFillZerobss+0x24>)
  1c:	e002      	b.n	24 <LoopFillZerobss>

0000001e <FillZerobss>:
  1e:	2300      	movs	r3, #0
  20:	f842 3b04 	str.w	r3, [r2], #4

00000024 <LoopFillZerobss>:
  24:	4b09      	ldr	r3, [pc, #36]	; (4c <LoopFillZerobss+0x28>)
  26:	429a      	cmp	r2, r3
  28:	d3f9      	bcc.n	1e <FillZerobss>
  2a:	f7ff fffe 	bl	0 <SystemInit>
  2e:	f7ff fffe 	bl	0 <__libc_init_array>
  32:	f7ff fffe 	bl	0 <main>
  36:	4770      	bx	lr
	...

Disassembly of section .text.Default_Handler:

00000000 <ADC3_IRQHandler>:
   0:	e7fe      	b.n	0 <ADC3_IRQHandler>
