@ Code cave: LED PC5 always on after boot. SPI timeout 50ms stays in spi_isr.

	.syntax unified
	.cpu cortex-m4
	.thumb

	.global	cave_boot
	.global	cave_spi

@ Replaces bl MX_GPIO_Init in main.
cave_boot:
	push	{lr}
	bl	0x08001548		@ MX_GPIO_Init
	ldr	r3, =0x40020800		@ GPIOC
	movs	r2, #0x20		@ pin 5 set
	str	r2, [r3, #24]		@ BSRR
	pop	{pc}

@ Replaces bl HAL_SPI_TransmitReceive in spi_isr.
@ LED stays on. Timeout/error skips CAN garbage.
cave_spi:
	push	{lr}
	sub	sp, #4
	ldr	r3, [sp, #8]
	str	r3, [sp]
	bl	0x08004082		@ HAL_SPI_TransmitReceive
	add	sp, #4
	cmp	r0, #0
	bne	1f
	pop	{pc}
1:
	pop	{lr}
	b	0x08001A72		@ spi_isr epilogue
