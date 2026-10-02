@ Code cave: blink LED PC5 on successful SPI RX. Timeout 50 ms stays in spi_isr.
@ Overwrites unused newlib abort (never called). Load at 0x08007FBC.
@
@ cave_spi replaces bl HAL_SPI_TransmitReceive.
@ r0-r3 and [sp] are the original HAL args; r3 must stay Size.

	.syntax unified
	.cpu cortex-m4
	.thumb

	.global	cave_spi

cave_spi:
	push	{lr}
	sub	sp, #4
	ldr	ip, [sp, #8]		@ Timeout; do not clobber r3=Size
	str	ip, [sp]
	bl	0x08004082		@ HAL_SPI_TransmitReceive
	add	sp, #4
	cmp	r0, #0
	bne	1f

	@ HAL_OK: ~8 Hz blink from tick/64
	bl	0x08002428		@ HAL_GetTick
	lsrs	r0, r0, #6
	lsls	r0, r0, #31
	lsrs	r0, r0, #26		@ 0 or PC5 set
	movs	r2, #0x20
	lsls	r2, r2, #16		@ PC5 reset
	orrs	r2, r0			@ BSx wins if both
	ldr	r3, 2f			@ GPIOC
	str	r2, [r3, #24]		@ BSRR
	pop	{pc}

1:
	@ timeout/error: LED off, skip CAN garbage
	ldr	r3, 2f
	movs	r2, #0x20
	lsls	r2, r2, #16
	str	r2, [r3, #24]
	pop	{lr}
	b	0x08001A72		@ spi_isr epilogue

	.align	2
2:
	.word	0x40020800
