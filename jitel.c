#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/timer.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#if SEMIHOSTING
# include <dcc_stdio.h>
void hard_fault_handler(void) { while (1); }
void mem_manage_handler(void) { while (1); }
void bus_fault_handler(void) { while (1); }
void usage_fault_handler(void) { while (1); }
#else
# define dbg_printf(...) (void)0;
#endif
#include <assert.h>

#define MIN(a, b) ({ typeof(a) __a = a, __b = b; __a < __b ? __a : __b; })

volatile unsigned int ticks;

struct jive {
	uint16_t magic1;
	uint16_t magic2;
	uint16_t v1;
	uint16_t ubec; /* ubec_v = ubec / 10 + 0.1 */
	uint16_t ibec;
	uint16_t v4;
	uint16_t v5;
	uint16_t tick;
	uint16_t v7;
	uint16_t ubat; /* ubat_v = ubat / 10 + 0.1 */
	uint16_t imot; /* real_imot = (imot / 10 + 0.1) * pwm */
	uint16_t rpm; /* rpm = value * 2 (poles ?) */
	uint16_t v11;
	uint16_t v12;
	uint16_t throt; /* [0, 4096) */
	uint16_t pwmmot; /* [0, 4096) */
	uint16_t v15_24[9];
};
volatile static struct jive jive;
char jive_magic[] = {0x00, 0xff, 0xff, 0xff};
static char jive_sync_lost, jive_lock;
volatile uint16_t dma_ring_buf[25 * 2]; /* DMA ring buffer with space for
					   two jive rows */
volatile static unsigned int jive_row_count;
const int jive_row_per_second = 10;

static unsigned char sbus_slot_data[31][3];
static char slot_base;
static volatile char slot_idx;
const char slot_id[] = { 0x03, 0x83, 0x43, 0xc3, 0x23, 0xa3, 0x63, 0xe3,
			 0x13, 0x93, 0x53, 0xd3, 0x33, 0xb3, 0x73, 0xf3,
			 0x0b, 0x8b, 0x4b, 0xcb, 0x2b, 0xab, 0x6b, 0xeb,
			 0x1b, 0x9b, 0x5b, 0xdb, 0x3b, 0xbb, 0x7b, 0xfb };

static void update_slot_data();

/*
  Jive:
  USART1 9600, RX pin PA10
  DMA1 RX CHANNEL 5
*/

struct jive j;
static void jive_copy_from_dma(volatile uint16_t *ptr)
{
	uint16_t *jptr = (uint16_t *)&jive;
	if (!jive_lock) {
		for (int i = 0; i < 25; i++) {
			uint16_t w = ptr[i];
			jptr[i] = (w << 8) | (w >> 8); /* fix endian */
		}
	}
}

void dma1_channel5_isr(void)
{
	volatile uint16_t *ptr = dma_ring_buf;

	if (DMA1_ISR & DMA_ISR_TCIF(5)) /* select low or high half */
		ptr += 25;
	dma_clear_interrupt_flags(DMA1, 5, DMA_HTIF | DMA_TCIF);

	jive_sync_lost = memcmp((void *)ptr, jive_magic, 4); // FIXME: too much code
	if (!jive_sync_lost) {
		jive_row_count++;
		jive_copy_from_dma(ptr);
		gpio_toggle(GPIOC, GPIO13);
	}
}

static void jive_sync()
{
	USART1_CR3 &= ~USART_CR3_DMAR;
	DMA1_CCR(5) &= ~DMA_CCR_EN;
	DMA1_CMAR(5) = (uint32_t)&dma_ring_buf;
	DMA1_CNDTR(5) = sizeof(dma_ring_buf);

	char buf[4];
	for (int i = 0; i < 4; i++)
		buf[i] = usart_recv_blocking(USART1);
	while (memcmp(buf, jive_magic, 4)) {
		memmove(buf, buf + 1, 3);
		buf[3] = usart_recv_blocking(USART1);
	}
	for (int i = 4; i < 50; i++) /* skip till the end of line */
		usart_recv_blocking(USART1);

	jive_sync_lost = 0;
	USART1_CR3 |= USART_CR3_DMAR;
	DMA1_CCR(5) |= DMA_CCR_EN;
}

static void jive_uart_init()
{
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_DMA1);
	rcc_periph_clock_enable(RCC_USART1);

	nvic_set_priority(NVIC_DMA1_CHANNEL5_IRQ, 3 << 4);
	nvic_enable_irq(NVIC_DMA1_CHANNEL5_IRQ);
	DMA1_CPAR(5) = (uint32_t)&USART1_DR;
	DMA1_CCR(5) |= DMA_CCR_MINC | DMA_CCR_CIRC |
		       DMA_CCR_HTIE | DMA_CCR_TCIE;

	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX); /* PA10 -> PB8 */
	usart_set_baudrate(USART1, 9600);
	USART_CR1(USART1) |= USART_CR1_RE | USART_CR1_UE;

	jive_sync();
}

static void sbus_slot(int n, unsigned short val)
{
	sbus_slot_data[n][0] = slot_id[n];
	sbus_slot_data[n][2] = val & 0xff;
	sbus_slot_data[n][1] = val >> 8;
}

static void sbus_uart_init()
{
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_USART2);
	rcc_periph_clock_enable(RCC_AFIO);

	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX); /* PA2 */
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT, 	      GPIO_USART2_RX); /* PA3 */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL,       GPIO4); /* PA4 -> PA3, in */

	gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_PULL_UPDOWN,     GPIO9); /* PB9 */
	gpio_clear(GPIOB, GPIO9); /* weak pull down */

	gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT,           GPIO5); /* PB5 */


	exti_select_source(EXTI9, GPIOB);
	exti_set_trigger(EXTI9, EXTI_TRIGGER_BOTH);
	nvic_enable_irq(NVIC_EXTI9_5_IRQ);
	exti_enable_request(EXTI9);

	exti_select_source(EXTI2, GPIOA);
	exti_set_trigger(EXTI2, EXTI_TRIGGER_BOTH);
	nvic_enable_irq(NVIC_EXTI2_IRQ);
	exti_enable_request(EXTI2);

	usart_set_baudrate(USART2, 100000);
	USART_CR2(USART2) |= USART_CR2_STOPBITS_2;
	USART_CR1(USART2) |= USART_CR1_RE | USART_CR1_TE | /* rx,tx */
			     USART_CR1_M | USART_CR1_PCE | /* 8E */
			     USART_CR1_UE;
}

void exti9_5_isr(void)
{
	unsigned pin = GPIO_IDR(GPIOB) & GPIO9;
	GPIO_BSRR(GPIOA) = pin ? GPIO4 << 16 : GPIO4;
	EXTI_PR = EXTI9;
}

void exti2_isr(void)
{
	unsigned pin = GPIO_IDR(GPIOA) & GPIO2;
	GPIO_BSRR(GPIOB) = pin ? GPIO5 << 16 : GPIO5;
	EXTI_PR = EXTI2;
}

static void sbus_send_slot(int n)
{
	if (sbus_slot_data[n][0] == 0)
		return;

	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO5); /* PB5 */

	for (int i = 0; i < 3; i++) {
		while ((USART2_SR & USART_SR_TXE) == 0);
		USART2_DR = sbus_slot_data[n][i];
	}
	while ((USART2_SR & USART_SR_TC) == 0);

	gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT, GPIO5);
}

void tim2_us(unsigned delay)
{
	TIM2_CR1 &= ~TIM_CR1_CEN;
	TIM2_SR &= ~TIM_SR_UIF;

	TIM2_CNT = 0;
	TIM2_ARR = delay;
	TIM2_CR1 |= TIM_CR1_CEN;
}

static void sbus_tick()
{
	unsigned char buf[25] = {0};

	do buf[0] = usart_recv_blocking(USART2);
	while (buf[0] != 0x0f);

	/* channel data is 25 bytes long,
	   1 start bit + 8 data bit + parity bit + 2 stop bit
	   gives 120 us per byte and 3000us per 25 bytes */
	tim2_us(3000 - 120 + 50); /* 24 bytes + jitter */

	for (int i = 1; i < 25; i++) {
		while ((USART2_SR & (USART_SR_RXNE | USART_SR_ORE |
				     USART_SR_NE | USART_SR_FE |
				     USART_SR_PE)) == 0 &&
		       (TIM2_SR & TIM_SR_UIF) == 0);

		if ((USART2_SR & (USART_SR_ORE | USART_SR_NE |
				  USART_SR_FE | USART_SR_PE)) ||
		    (TIM2_SR & TIM_SR_UIF))
			return;
		buf[i] = USART2_DR;
	}

	switch (buf[24]) {
	case 0x04: slot_base = 0; break;
	case 0x14: slot_base = 8; break;
	case 0x24: slot_base = 16; break;
	case 0x34: slot_base = 24; break;
	default:
		return; /* framing error ? */
	}

	tim2_us(2000);  /* telemety is 2ms after end of channel data */
	update_slot_data(); /* calulate slot values while waiting */
	while ((TIM2_SR & TIM_SR_UIF) == 0);

	slot_idx = 0;
	TIM3_CNT = 0;
	TIM3_CR1 |= TIM_CR1_CEN; /* start slot timer */

	sbus_send_slot(slot_base + slot_idx);
	while (slot_idx != 7); /* wait till all slots sent */
}


void tim3_isr()
{
	if (++slot_idx == 7)
		TIM3_CR1 &= ~TIM_CR1_CEN;
	sbus_send_slot(slot_base + slot_idx);
	TIM3_SR &= ~TIM_SR_UIF;
}

static void timer_init()
{
	rcc_periph_clock_enable(RCC_TIM2);
	rcc_periph_clock_enable(RCC_TIM3);
	nvic_set_priority(NVIC_TIM3_IRQ, 1 << 4);
	nvic_enable_irq(NVIC_TIM3_IRQ);

	TIM2_PSC = rcc_apb1_frequency / 500000; /* 1usec per count */
	TIM3_PSC = rcc_apb1_frequency / 500000; /* 1usec per count */

	TIM3_ARR = 660; /* 3 bytes * 120us per byte  = 360 us for data
			   an 300us gap between slots */
	TIM3_DIER |= TIM_DIER_UIE;

	TIM2_CR1 |= TIM_CR1_URS; /* setting EGR_UG won't trigger ISR */
	TIM3_CR1 |= TIM_CR1_URS;
	TIM2_EGR |= TIM_EGR_UG; /* force load PSC */
	TIM3_EGR |= TIM_EGR_UG;
}

int main()
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();
	timer_init();

	rcc_periph_clock_enable(RCC_GPIOC);
	gpio_set(GPIOC, GPIO13);
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, /* LED on PC13 */
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO11);

	jive_uart_init();
	sbus_uart_init();

	for (;;) {
		sbus_tick();
		if (jive_sync_lost)
			jive_sync();
	}
}

static uint16_t swap(uint16_t a)
{
	return (a << 8) | (a >> 8);
}


static void update_slot_data()
{
	static unsigned int r;
	if (r == jive_row_count || jive_sync_lost) /* no new data, nothing to update */
		return;
	r = jive_row_count;

	unsigned ubat_mv = jive.ubat ? (jive.ubat + 1) * 100 : 0;
	unsigned imot_ma = jive.imot && jive.pwmmot ? (jive.imot + 1) * 100 * jive.pwmmot / 4096 : 0;
	unsigned mot_rpm = jive.rpm * 2; /* pyro-xxx */

	ubat_mv = ubat_mv * 1.012; /* adjust */

	static unsigned sum_ma;
	sum_ma += imot_ma;


	static enum { INIT, SPOOLUP, DONE } state;
	static unsigned tick;
	static unsigned full_mv, ubat_mv_filter, load_ma;
	static unsigned prev_rpm;
	static unsigned ri;

	switch (state) {
	case INIT:
		if (ubat_mv > 0) {
			if (tick++ > 3) {
				full_mv = ubat_mv_filter = ubat_mv;
				tick = 0;
				state = SPOOLUP;
			}
		}
		break;
	case SPOOLUP:
		if (mot_rpm == 0)
			break;
		int diff = mot_rpm - prev_rpm;
		prev_rpm = mot_rpm;
		if (abs(diff) > mot_rpm / 100) {
			tick = 0;
			load_ma = 0;
			break;
		}
		tick++;
		load_ma += imot_ma;
		if (tick > 20) {
			load_ma /= 20; /* mean */

			ri = 100000 * (full_mv - ubat_mv) / load_ma;
			ri /= 2; /* magic */
			state = DONE;
		}
		break;
	case DONE:
		break;
	}


	if (imot_ma > 0) {
		int adj_mv = ubat_mv + ri * imot_ma / 100000;
		ubat_mv_filter = (ubat_mv_filter * 99 + adj_mv) / 100;
	}

	/* Curr-1678 */
	sbus_slot(1, 0x4000 + MIN(0x3fff, imot_ma / 10));
	sbus_slot(2, ubat_mv / 10);
	sbus_slot(3, sum_ma / 3600 / jive_row_per_second); /* mAh */

	/* SBS-01RM */
	sbus_slot(4, swap(mot_rpm / 6));

	/* SBS-01V */
	if (imot_ma > 0 && ubat_mv_filter > 0) {
		sbus_slot(5, 0x8000 + 0 / 100);
		sbus_slot(6, (ubat_mv_filter + 50) / 100);
	} else {
		sbus_slot(5, 0x8000);
		sbus_slot(6, 0);
	}
}
