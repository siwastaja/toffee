#include <stdint.h>
#include <string.h>
#include "ext_include/stm32f7xx.h"
#include "stm32_cmsis_extension.h"
#include "own_std.h"

#define RLED_ON()  do{GPIOF->BSRR = 1UL<<2;}while(0)
#define RLED_OFF() do{GPIOF->BSRR = 1UL<<(2+16);}while(0)
#define GLED_ON()  do{GPIOF->BSRR = 1UL<<3;}while(0)
#define GLED_OFF() do{GPIOF->BSRR = 1UL<<(3+16);}while(0)

#define EPC_RSTN_HIGH() do{GPIOB->BSRR = 1UL<<4;}while(0)
#define EPC_RSTN_LOW() do{GPIOB->BSRR = 1UL<<(4+16);}while(0)


#define EPCLEDV_ON()  do{GPIOE->BSRR = 1UL<<0;}while(0)
#define EPCLEDV_OFF() do{GPIOE->BSRR = 1UL<<(0+16);}while(0)
#define EPC5V_ON()  do{GPIOE->BSRR = 1UL<<1;}while(0)
#define EPC5V_OFF() do{GPIOE->BSRR = 1UL<<(1+16);}while(0)
#define EPC10V_ON()  do{GPIOE->BSRR = 1UL<<2;}while(0)
#define EPC10V_OFF() do{GPIOE->BSRR = 1UL<<(2+16);}while(0)
#define EPCNEG10V_ON()  do{GPIOE->BSRR = 1UL<<3;}while(0)
#define EPCNEG10V_OFF() do{GPIOE->BSRR = 1UL<<(3+16);}while(0)

#define DBG_BUT() (GPIOC->IDR & (1UL<<13))

#define EPC_ADDR 0b0100000

void delay_us(uint32_t i)
{
	if(i==0) return;
	i *= 90;
	i -= 7;
	while(i--)
		__asm__ __volatile__ ("nop");
}

void delay_ms(uint32_t i)
{
	while(i--)
	{
		delay_us(1000);
	}
}

void error()
{
	__disable_irq();
	EPC_RSTN_LOW();
	EPCNEG10V_OFF();
	EPC10V_OFF();
	EPC5V_OFF();
	EPCLEDV_OFF();
	while(1)
	{
		RLED_ON();
		delay_ms(40);
		RLED_OFF();
		delay_ms(40);
	}
}

void uart_print_string_blocking(const char *buf)
{
	while(buf[0] != 0)
	{
		while((USART3->ISR & (1UL<<7)) == 0) ;
		USART3->TDR = buf[0];
		buf++;
	}
}

void uart_send_blocking(const uint8_t *buf, int len)
{
	while(len--)
	{
		while((USART3->ISR & (1UL<<7)) == 0) ;
		USART3->TDR = buf[0];
		buf++;
	}
}


#define CRC_INITIAL_REMAINDER 0x00
#define CRC_POLYNOMIAL 0x07 // As per CRC-8-CCITT

#define CALC_CRC(remainder) \
	for(int crc__bit = 8; crc__bit > 0; --crc__bit) \
	{ \
		if((remainder) & 0b10000000) \
		{ \
			(remainder) = ((remainder) << 1) ^ CRC_POLYNOMIAL; \
		} \
		else \
		{ \
			(remainder) = ((remainder) << 1); \
		} \
	}

void uart_send_blocking_crc(const uint8_t *buf, uint8_t id, int len)
{
	uint8_t chk = CRC_INITIAL_REMAINDER;

	while((USART3->ISR & (1UL<<7)) == 0) ;
	USART3->TDR = id;
	while((USART3->ISR & (1UL<<7)) == 0) ;
	USART3->TDR = len & 0xff;
	while((USART3->ISR & (1UL<<7)) == 0) ;
	USART3->TDR = (len & 0xff00)>>8;

	while(len--)
	{
		chk ^= buf[0];
		CALC_CRC(chk);
		while((USART3->ISR & (1UL<<7)) == 0) ;
		USART3->TDR = buf[0];
		buf++;
	}
	while((USART3->ISR & (1UL<<7)) == 0) ;
	USART3->TDR = chk;
}


// Things written to I2C CR1 every time:
#define I2C_CR1_BASICS (0UL<<8 /*Digital filter len 0 to 15*/)
#define I2C_CR1_BASICS_ON (I2C_CR1_BASICS | 1UL /*keep it on*/)

volatile int epc_i2c_write_busy, epc_i2c_read_busy;

volatile int epc_i2c_read_state;
uint8_t epc_i2c_read_slave_addr;
volatile uint8_t *epc_i2c_read_buf;
uint8_t epc_i2c_read_len;

uint8_t dbgbuf1[300];
uint8_t dbgbuf2[300];
int dbgi = 0;

void epc_i2c_write_dma(uint8_t slave_addr_7b, volatile uint8_t *buf, uint8_t len)
{
	/*
		Was chasing for a really strange data coherency issue for two days: sometimes, buf[0] is randomly replaced
		by a former value. Input buf is in non-cached section (core-coupled ram), but it still happens!

		I don't know what the hell is going on, but for some reason I can't understand, invalidation of data cache here
		seemed to fix the issue.
	*/
	SCB_InvalidateDCache();

	if(epc_i2c_write_busy || epc_i2c_read_busy)
		error(11);

	if(DMA1_Stream0->CR & 1UL)
		error(12);

	epc_i2c_write_busy = 1;
	I2C3->ICR = 1UL<<5; // Clear any pending STOPF interrupt 

	if(len > 1) // Actually use DMA
	{		
		DMA1_Stream0->CR = 8UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 
				   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
			           1UL<<10 /*mem increment*/ | 0b01UL<<6 /*mem-to-periph*/;

		DMA1_Stream0->NDTR = len;
		DMA1_Stream0->M0AR = (uint32_t)buf;

		DMA_CLEAR_INTFLAGS(DMA1, 0);
		DMA1_Stream0->CR |= 1; // Enable DMA // OPT_TODO: try combining with previous CR write.

		I2C3->CR1 = I2C_CR1_BASICS_ON |  1UL<<5 /*STOPF interrupt - the only specified way to detect finished transfer when AUTOEND=1 and RELOAD=0*/ | 1UL<<14 /*TX DMA*/;

	}
	else	// Just do the single write right now.
	{
		I2C3->CR1 = I2C_CR1_BASICS_ON |  1UL<<5 /*STOPF interrupt - the only specified way to detect finished transfer when AUTOEND=1 and RELOAD=0*/; // no DMA
		I2C3->TXDR = buf[0];
	}

	I2C3->CR2 = 1UL<<25 /*AUTOEND: stop is generated after len bytes*/ | ((uint32_t)len)<<16 | 0UL<<10 /*write*/ | slave_addr_7b<<1;
	I2C3->CR2 |= 1UL<<13; // START. OPT_TODO: try combining with previous CR2 write.
#if 0

	I2C3->CR1 = I2C_CR1_BASICS_ON |  1UL<<5 /*STOPF interrupt - the only specified way to detect finished transfer when AUTOEND=1 and RELOAD=0*/; // no DMA
	I2C3->CR2 = 1UL<<25 /*AUTOEND: stop is generated after len bytes*/ | ((uint32_t)len)<<16 | 0UL<<10 /*write*/ | slave_addr_7b<<1;
	I2C3->CR2 |= 1UL<<13; // START. OPT_TODO: try combining with previous CR2 write.
	for(int i=0; i<len; i++)
	{
		while(!(I2C3->ISR & (1UL<<1)));
		I2C3->TXDR = buf[i];
	}
#endif


}

#define epc_i2c_write epc_i2c_write_dma

void epc_i2c_read_dma(uint8_t slave_addr_7b, volatile uint8_t *buf, uint8_t len)
{
	DMA1_Stream2->CR = 3UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 
			   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/ | 1UL<<4 /*transfer complete interrupt*/;

	DMA1_Stream2->NDTR = len;
	DMA1_Stream2->M0AR = (uint32_t)buf;

	I2C3->CR1 = I2C_CR1_BASICS_ON | 0UL<<5 /*OFF FOR READ: STOPF interrupt - the only specified way to detect finished transfer when AUTOEND=1 and RELOAD=0*/ | 1UL<<15 /*RX DMA*/ ;

	I2C3->CR2 = 1UL<<25 /*AUTOEND: stop is generated after len bytes*/ | ((uint32_t)len)<<16 | 1UL<<10 /*read*/ | slave_addr_7b<<1;

	DMA_CLEAR_INTFLAGS(DMA1, 2);
	DMA1_Stream2->CR |= 1; // Enable DMA // OPT_TODO: try combining with previous CR write.

	I2C3->CR2 |= 1UL<<13; // START. OPT_TODO: try combining with previous CR2 write.
}

/*
	In I2C, a million things can go wrong. STM32 I2C implementation is notoriously buggy, device implementations often are, too.
	It makes little sense to try to detect every possible error condition, since they most often end up in total bus lockup
	anyway, and can only be reliably solved by device&bus reset. Since this is time-consuming anyway, and always leads to loss
	of data and hangup in execution time, we can as well solve all error checking using a simple watchdog that does the same,
	with much less code footprint (good for reliability, execution speed, flash usage, and code maintainability) compared to trying
	to catch all unexpected things in actual interrupt services or API functions.
*/

void epc_i2c_read(uint8_t slave_addr_7b, uint8_t reg_addr, volatile uint8_t *buf, uint8_t len)
{
	if(epc_i2c_write_busy || epc_i2c_read_busy)
		error(12);
	/*
		Full I2C read cycle consists of a write cycle (payload = reg_addr), without stop condition,
		then the actual read cycle starting with the repeated start condition.

		Since the write is always one byte of payload, we run it without DMA.

		After the write is complete, there is no AUTOEND generated, instead we get an interrupt, allowing
		us to go on with the read.
	*/
	epc_i2c_read_busy = 1;

	I2C3->CR1 = I2C_CR1_BASICS_ON | 1UL<<6 /*Transfer Complete interrupt - happens because AUTOEND=0 and RELOAD=0*/; // no DMA
	I2C3->CR2 = 0UL<<25 /*AUTOEND off*/ | 1UL<<16 /*len=1*/ | 0UL<<10 /*write*/ | slave_addr_7b<<1;
	I2C3->TXDR = reg_addr;
	I2C3->CR2 |= 1UL<<13; // START. OPT_TODO: try combining with previous CR2 write.
	epc_i2c_read_state = 1;
	epc_i2c_read_slave_addr = slave_addr_7b;
	epc_i2c_read_buf = buf;
	epc_i2c_read_len = len;

}

// Returns true whenever read or write operation is going on.
int epc_i2c_is_busy()
{
	return epc_i2c_write_busy || epc_i2c_read_busy;
}


void epc_rx_dma_inthandler() // read complete.
{
	DMA_CLEAR_INTFLAGS(DMA1, 2);
	epc_i2c_read_busy = 0;
}

/*
	For some reason, I can't get repeated start (Sr) condition out of the new STM32 I2C - it generates a stop-start sequence even when only asked for a START.
	Luckily, the EPC chip still works correctly even though this violates the spec.
*/
void epc_i2c_inthandler()
{
	I2C3->ICR = 1UL<<5; // Clear the STOPF flag.
	if(epc_i2c_read_state)
	{
		// Writing START to 1 (in epc_i2c_read_dma()) clears the interrupt flag
		epc_i2c_read_dma(epc_i2c_read_slave_addr, epc_i2c_read_buf, epc_i2c_read_len);
		epc_i2c_read_state=0;

	}
	else // STOPF interrupt - this was a write.
	{
		if(I2C3->ISR & (1UL<<15)) // busy shouldn't be high - at least fail instead of doing random shit
		{
			char printbuf[64];
			__disable_irq();
			EPC_RSTN_LOW();
			EPCNEG10V_OFF();
			EPC10V_OFF();
			EPC5V_OFF();
			EPCLEDV_OFF();
			while(1)
			{
				uart_print_string_blocking("ISR="); o_utoa32(I2C3->ISR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking(" ");
				uart_print_string_blocking("NDTR="); o_utoa32(DMA2_Stream0->NDTR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking("\r\n");
			}
		}

		// Write is now finished.
		epc_i2c_write_busy = 0;
		I2C3->CR1 = I2C_CR1_BASICS_ON;
	}
}


void epc_i2c_init()
{
	// DMA1 Stream0 = tx
	// DMA1 Stream2 = rx
	// For now, init in Fast mode 400 kHz
	// Use the datasheet example of 48MHz input clock, with prescaler 1.125 times bigger for 54 MHz. Sync delay calculations are near enough.
	// Errata: tSU;DAT (do they mean SDADEL register??) must be at least one I2CCLK period
	// Errata: bus errors are spuriously generated for no reason, the bug is not said to affect the transfer: just ignore BERR.
 
	DMA1_Stream0->PAR = (uint32_t)&(I2C3->TXDR);
	DMA1_Stream2->PAR = (uint32_t)&(I2C3->RXDR);

	// open drain:
	GPIOC->OTYPER  |= 1UL<<9;
	GPIOA->OTYPER  |= 1UL<<8;

	IO_TO_ALTFUNC(GPIOC, 9);
	IO_TO_ALTFUNC(GPIOA, 8);
	IO_SET_ALTFUNC(GPIOC, 9, 4);
	IO_SET_ALTFUNC(GPIOA, 8, 4);

//	I2C3->TIMINGR = 6UL<<28/*PRESCALER*/  | 9UL<<0/*SCLL*/  | 3UL<<8/*SCLH*/  | 3UL<<16 /*SDADEL*/  | 3UL<<20 /*SCLDEL*/;
	I2C3->TIMINGR = 14UL<<28/*PRESCALER*/  | 9UL<<0/*SCLL*/  | 3UL<<8/*SCLH*/  | 3UL<<16 /*SDADEL*/  | 3UL<<20 /*SCLDEL*/;
	I2C3->CR1 = I2C_CR1_BASICS;

	I2C3->CR1 |= 1UL; // Enable

	NVIC_SetPriority(I2C3_EV_IRQn, 0b0101);
	NVIC_EnableIRQ(I2C3_EV_IRQn);
	NVIC_SetPriority(DMA1_Stream2_IRQn, 0b0101);
	NVIC_EnableIRQ(DMA1_Stream2_IRQn);

}

#define EPC_XS 160
#define EPC_YS 60

struct __attribute__((packed))
{
	uint16_t start_pad[2];
	uint16_t img[EPC_XS*EPC_YS];
} img_mono;

struct __attribute__((packed))
{
	uint16_t start_pad[2];
	uint16_t img[EPC_XS*EPC_YS];
} img_compmono;

struct __attribute__((packed))
{
	uint16_t start_pad[2];
	uint16_t img[EPC_XS*EPC_YS];
} img_dcs[4];

volatile int epc_capture_finished = 0;
void epc_dcmi_dma_inthandler()
{
	// DMA finished
	DMA_CLEAR_INTFLAGS(DMA2, 7);
	epc_capture_finished = 1;
}

void epc_shutdown_inthandler()
{
	/*
		Proper power-down sequence to prevent damaging the sensor.
		This is quickly triggered from the 3V3 regulator POWER_GOOD falling down, which happens
		instantly when its UVLO is triggered.
	*/
	EPC_RSTN_LOW();
	EPCNEG10V_OFF();
	EPC10V_OFF();
	EPC5V_OFF();
	EPCLEDV_OFF();
	RLED_ON();
	error(0);
}

void epc_dcmi_init()
{
	RCC->AHB2ENR |= 1UL;
	IO_TO_ALTFUNC(GPIOA,  6);
	IO_TO_ALTFUNC(GPIOA,  9);
	IO_TO_ALTFUNC(GPIOA, 10);
	IO_TO_ALTFUNC(GPIOG, 10);
	IO_TO_ALTFUNC(GPIOG, 11);
	IO_TO_ALTFUNC(GPIOC, 11);
	IO_TO_ALTFUNC(GPIOB,  6);
	IO_TO_ALTFUNC(GPIOB,  8);
	IO_TO_ALTFUNC(GPIOB,  9);

	IO_SET_ALTFUNC(GPIOA,  6, 13);
	IO_SET_ALTFUNC(GPIOA,  9, 13);
	IO_SET_ALTFUNC(GPIOA, 10, 13);
	IO_SET_ALTFUNC(GPIOG, 10, 13);
	IO_SET_ALTFUNC(GPIOG, 11, 13);
	IO_SET_ALTFUNC(GPIOC, 11, 13);
	IO_SET_ALTFUNC(GPIOB,  6, 13);
	IO_SET_ALTFUNC(GPIOB,  8, 13);
	IO_SET_ALTFUNC(GPIOB,  9, 13);

	DCMI->CR = 0b00UL<<10 /*8-bit data*/ | 0UL<<5 /* CLK falling edge*/ | 1UL<<4 /*Embedded sync*/ | 0UL<<1 /*continuous grab*/;

	// Program the default synchronization codes used in the epc635
	DCMI->ESCR = 0x1eUL<<0 /*frame start*/ | 0xffUL<<24 /*frame end*/ | 0xaaUL<<8 /*line start*/ | 0x55<<16 /*line end*/;
	DCMI->ESUR = 0xffffffffUL; // mask for the previous: all bits compared

	DCMI->CR |= 1UL<<14; // Enable

	DMA2_Stream7->PAR = (uint32_t)&(DCMI->DR);
	DMA2_Stream7->CR = 1UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0UL<<18 /*double buffer OFF*/ | 0UL<<8 /*circular OFF*/ |
			   0b10UL<<13 /*32-bit mem*/ | 0b10UL<<11 /*32-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/ | 1UL<<4 /* Transfer complete interrupt*/;

	//DMA2_Stream7->NDTR = sizeof(epc_frame_t)/4;

	//DMA_CLEAR_INTFLAGS(DMA2, 7);
	//DMA2_Stream7->CR |= 1; // Enable DMA

	NVIC_SetPriority(DMA2_Stream7_IRQn, 0b1111);
	NVIC_EnableIRQ(DMA2_Stream7_IRQn);

	DCMI->CR |= 1UL<<0; // Start CAPTURE


}

void dcmi_start_dma(void *data, int size)
{
	// Disable the stream first
	DMA2_Stream7->CR = 1UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ |
			   0b10UL<<13 /*32-bit mem*/ | 0b10UL<<11 /*32-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/;

	while(DMA2_Stream7->CR & 1UL) ;

	DMA_CLEAR_INTFLAGS(DMA2, 7);

	DMA2_Stream7->M0AR = (uint32_t)data;

	DMA2_Stream7->NDTR = size;
	DMA2_Stream7->CR = 1UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 
			   0b10UL<<13 /*32-bit mem*/ | 0b10UL<<11 /*32-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/ | 1UL<<4 /* Transfer complete interrupt*/;

	DMA_CLEAR_INTFLAGS(DMA2, 7);
	DMA2_Stream7->CR |= 1; // Enable DMA

}


typedef struct __attribute__((packed))
{
	// *4, then -1 to form the register values:
	uint16_t bw_int_len; 
	uint16_t dist_int_len;

	uint8_t clk_div; // 1 = 20MHz LED, 2 = 10 MHz...
} config_t;

config_t config =  // active configuration
{
	.bw_int_len = 1000,
	.dist_int_len = 1000,
	.clk_div = 1
};

config_t rx_config = // modified config
{
	.bw_int_len = 1000,
	.dist_int_len = 1000,
	.clk_div = 1
};

void apply_config(config_t *conf)
{
	if(conf->clk_div < 1 || conf->clk_div > 6 || conf->clk_div == 5)
	{
		error(55);
	}

	if(conf->bw_int_len*4-1 > 65535 || conf->dist_int_len*4-1 > 65535)
	{
		error(55);
	} 

	memcpy(&config, conf, sizeof(config_t));
}


void trig()
{
	static volatile uint8_t b[2] = {0xa4, 1};
	epc_i2c_write(EPC_ADDR, b, 2);
	while(epc_i2c_is_busy());
}

volatile uint8_t epc_wrbuf[16] __attribute__((section(".data2"))); // to skip cache
volatile uint8_t epc_rdbuf[16] __attribute__((section(".data2"))); // to skip cache

void epc_test()
{
	char printbuf[64];

	delay_ms(300);
	EPC10V_ON();
	EPC5V_ON();
	delay_ms(100);
	EPCNEG10V_ON();
	delay_ms(100);
	EPC_RSTN_HIGH();
	delay_ms(300);


	uart_print_string_blocking("epc_i2c_init...");
	epc_i2c_init();
	uart_print_string_blocking("ok\r\n");

/*
	uart_print_string_blocking("read IC type & version...");
	epc_i2c_read(EPC_ADDR, 0x00, epc_rdbuf, 2);

	while(epc_i2c_is_busy());
	uart_print_string_blocking("type=");
	o_utoa8_fixed(epc_rdbuf[0], printbuf); uart_print_string_blocking(printbuf);
	uart_print_string_blocking(" version=");
	o_utoa8_fixed(epc_rdbuf[1], printbuf); uart_print_string_blocking(printbuf);
	uart_print_string_blocking("\r\n");

	uart_print_string_blocking("read embedded sync data labels... ");
	epc_i2c_read(EPC_ADDR, 0x1c, epc_rdbuf, 4);
	while(epc_i2c_is_busy());
	for(int i=0; i<4; i++)
	{
		o_utoa8_fixed(epc_rdbuf[i], printbuf); uart_print_string_blocking(printbuf);
		uart_print_string_blocking(" ");
	}

	uart_print_string_blocking("\r\n");
*/
	//while(DBG_BUT()) ;


	{
		epc_wrbuf[0] = 0xcb;
		epc_wrbuf[1] = 0b00101111;
		uart_print_string_blocking("write i2c&tcmi control reg...");
		epc_i2c_write(EPC_ADDR, epc_wrbuf, 2);
		while(epc_i2c_is_busy());
		uart_print_string_blocking("ok\r\n");
	}

	{
		epc_wrbuf[0] = 0x90;
		epc_wrbuf[1] = 0b11001000;
		uart_print_string_blocking("write led driver control...");
		epc_i2c_write(EPC_ADDR, epc_wrbuf, 2);
		while(epc_i2c_is_busy());
		uart_print_string_blocking("ok\r\n");
	}

	{
		epc_wrbuf[0] = 0x92;
		epc_wrbuf[1] = 0b11000100;
		uart_print_string_blocking("write modulation select (greyscale)...");
		epc_i2c_write(EPC_ADDR, epc_wrbuf, 2);
		while(epc_i2c_is_busy());
		uart_print_string_blocking("ok\r\n");
	}

	EPCLEDV_ON();

	delay_ms(100);


	uart_print_string_blocking("epc_dcmi_init...");
	epc_dcmi_init();
	uart_print_string_blocking("ok\r\n");

	{
		uart_print_string_blocking("write shutter (for dummy frame)...");
		dcmi_start_dma(&img_mono, (EPC_XS*EPC_YS*2)/4+1);
		trig();
		delay_ms(100);
		uart_print_string_blocking("ok\r\n");
	}


//	uart_print_string_blocking("Press button S1 to start (with binary communication)");
	uart_print_string_blocking("\r\n");

	RLED_OFF();
	GLED_OFF();

//	delay_ms(300);

	while(1)
	{

		// fetch config:

		static int conf_errors = 0;
		if(DMA1_Stream1->NDTR == sizeof(config_t))
		{
			config_t pending;
			memcpy(&pending, &rx_config, sizeof(config_t));
			if(DMA1_Stream1->NDTR == sizeof(config_t))
			{
				// config was unaccessed before and after copy - not being modified during copy.
				apply_config(&pending);
				conf_errors=0;
			}
			else
			{
				conf_errors++;
				RLED_ON();
				delay_ms(50);
				RLED_OFF();
			}
		}
		else
		{
			conf_errors++;
			RLED_ON();
			delay_ms(50);
			RLED_OFF();
		}
		epc_wrbuf[0] = 0x99;

		if(conf_errors > 2)
		{
			conf_errors = 0;
			// Very improbable that conf rx happens during the short unallowed period three times in a row - it's just stuck midway transfer (due to missed or excess bytes) - reset DMA.

			DMA1_Stream1->CR = 4UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0UL<<18 /*double buffer OFF*/ | 1UL<<8 /*circular ON*/ |
					   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
					   1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/ | 0UL<<4 /* Transfer complete interrupt*/;
			while(DMA1_Stream1->CR & 1UL) ;
			DMA1_Stream1->NDTR = sizeof(config_t);
			DMA_CLEAR_INTFLAGS(DMA1, 1);
			DMA1_Stream1->CR |= 1; // Enable DMA
		}



		//uart_print_string_blocking("RISR="); o_utoa32(DCMI->RISR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking(" ");
		//uart_print_string_blocking("NDTR="); o_utoa32(DMA2_Stream7->NDTR, printbuf); uart_print_string_blocking(printbuf); uart_print_string_blocking(" ");

		int time1 = 0, time2 = 0, time3 = 0;
		{
			epc_wrbuf[0] = 0x90;
			epc_wrbuf[1] = 0b11001000; // leds disabled
			epc_i2c_write(EPC_ADDR, epc_wrbuf, 2);
			while(epc_i2c_is_busy());
		}

		{
			epc_wrbuf[0] = 0x92;
			epc_wrbuf[1] = 0b11000100; // greyscale modulation
			epc_i2c_write(EPC_ADDR, epc_wrbuf, 2);
			while(epc_i2c_is_busy());
		}

		{
			int intlen = ((int)config.bw_int_len<<2)-1;
			epc_wrbuf[0] = 0xA1;
			epc_wrbuf[1] = 120/config.clk_div;
			epc_wrbuf[2] = (intlen&0xff00)>>8;
			epc_wrbuf[3] = intlen&0xff;

			epc_i2c_write(EPC_ADDR, epc_wrbuf, 4);
			while(epc_i2c_is_busy());
		}



		dcmi_start_dma(&img_mono, (EPC_XS*EPC_YS*2)/4+1);

		trig();

		GLED_ON();
		while(!epc_capture_finished) ;
		epc_capture_finished = 0;
		GLED_OFF();


		{
			int intlen = ((int)config.dist_int_len<<2)-1;
			epc_wrbuf[0] = 0xA1;
			epc_wrbuf[1] = 24/config.clk_div;
			epc_wrbuf[2] = (intlen&0xff00)>>8;
			epc_wrbuf[3] = intlen&0xff;
			epc_i2c_write(EPC_ADDR, epc_wrbuf, 4);
			while(epc_i2c_is_busy());
		}


		dcmi_start_dma(&img_compmono, (EPC_XS*EPC_YS*2)/4+1);

		trig();

		GLED_ON();
		while(!epc_capture_finished) ;
		epc_capture_finished = 0;
		GLED_OFF();


		{
			epc_wrbuf[0] = 0x90;
			epc_wrbuf[1] = 0b11101000; // LED2 output on
			epc_i2c_write(EPC_ADDR, epc_wrbuf, 2);
			while(epc_i2c_is_busy());
		}

		{
			epc_wrbuf[0] = 0x92;
			epc_wrbuf[1] = 0b00110100; // Sinusoidal 4DCS modulation
			epc_i2c_write(EPC_ADDR, epc_wrbuf, 2);
			while(epc_i2c_is_busy());
		}

		{
			int intlen = ((int)config.dist_int_len<<2)-1;
			epc_wrbuf[0] = 0xA1;
			epc_wrbuf[1] = 24/config.clk_div;
			epc_wrbuf[2] = (intlen&0xff00)>>8;
			epc_wrbuf[3] = intlen&0xff;
			epc_i2c_write(EPC_ADDR, epc_wrbuf, 4);
			while(epc_i2c_is_busy());
		}

		dcmi_start_dma(&img_dcs[0], 4*((EPC_XS*EPC_YS*2)/4+1));

		trig();

		GLED_ON();
		while(!epc_capture_finished) ;
		epc_capture_finished = 0;
		GLED_OFF();

		static uint8_t sync_packet[8] = {0xff,0xff,0xff,0xff,  0xff,0xff,0x12,0xab};
		uart_send_blocking_crc(sync_packet, 0xaa, 8);
		delay_ms(5);

		// image data DMA is done at this point, data is not going to change anymore, so dcache is okay to use, just invalidate it first:
		SCB_InvalidateDCache();

		static uint8_t txbuf[EPC_XS*EPC_YS*3/2];

		{
			int i = 0;
			for(int yy=0; yy < 60; yy++)
			{
				for(int xx=0; xx < 160; xx+=2)
				{
					uint16_t val1 = img_mono.img[yy*EPC_XS+xx];
					uint16_t lum1 = ((val1&0b0011111111111100)>>2);
					uint16_t val2 = img_mono.img[yy*EPC_XS+xx+1];
					uint16_t lum2 = ((val2&0b0011111111111100)>>2);
					txbuf[i++] = (lum1&0xff0)>>4;
					txbuf[i++] = (lum1&0xf)<<4 | (lum2&0xf00)>>8;
					txbuf[i++] = (lum2&0xff);
				}
			}
			uart_send_blocking_crc(txbuf, 0x01, i);
			delay_ms(5);
		}


		for(int dcs=0; dcs<4; dcs++)
		{
			int i = 0;
			for(int yy=0; yy < 60; yy++)
			{
				for(int xx=0; xx < 160; xx+=2)
				{
					uint16_t val1 = img_dcs[dcs].img[yy*EPC_XS+xx];
					uint16_t lum1 = ((val1&0b0011111111111100)>>2);
					uint16_t val2 = img_dcs[dcs].img[yy*EPC_XS+xx+1];
					uint16_t lum2 = ((val2&0b0011111111111100)>>2);
					txbuf[i++] = (lum1&0xff0)>>4;
					txbuf[i++] = (lum1&0xf)<<4 | (lum2&0xf00)>>8;
					txbuf[i++] = (lum2&0xff);
				}
			}
			uart_send_blocking_crc(txbuf, 0x10+dcs, i);
			delay_ms(5);
		}


	}


}

#define NUM_ADC_DATA 2

typedef struct
{
	uint16_t vbat;
	uint16_t tcpu;
} adc_datum_t;

volatile adc_datum_t adc_data[NUM_ADC_DATA];

void uart_dbg()
{
	static uint8_t test[10000];
	uint8_t c = 0;
	for(int i=0; i<10000; i++)
	{
		test[i] = c++;
	}

	while(1)
	{
		static uint8_t sync_packet[8] = {0xff,0xff,0xff,0xff,  0xff,0xff,0x12,0xab};
		uart_send_blocking_crc(sync_packet, 0xaa, 8);
		uart_send_blocking_crc(test, 0x30, 10000);

	}
}

void main()
{

	/*
	For 216MHz operation, "overdrive" must be on, device must have -6 suffix, and max Ta is 80 degC.
	Entering the overdrive requires a specific sequence, explained in the refman on page 123.

	XTAL = HSE = 8 MHz
	PLLCLK = SYSCLK = 216 MHz (max)
	AHB = HCLK = 216 MHz (max) --> AHB prescaler = 1
	APB2 = high-speed APB = 108 MHz (108 MHz max) --> APB2 prescaler = 2
	APB1 = PCLK1 = low-speed APB  = 54 MHz (54 MHz max) --> APB1 prescaler = 4
	APB2 timers x2 = 216 MHz
	APB1 timers x2 = 108 MHz

	PLL CONFIG:
	Input: 8 MHz
	M (for PLL input)   =  4 -> 2 MHz (must be between 1..2MHz)
	N (PLL multiplier)  = 216 -> 432 MHz
	P (for main system) = 2  -> 216 MHz
	Q (for USB) = 9 -> 48 MHZ
	*/

	RCC->AHB1ENR = 0xff /*GPIOA to H*/ | 1UL<<21 /*DMA1*/ | 1UL<<22 /*DMA2*/;
	IO_TO_GPO(GPIOF, 2);
	IO_TO_GPO(GPIOF, 3);
	IO_TO_GPO(GPIOE, 2);
	IO_TO_GPO(GPIOE, 3);
	IO_TO_GPO(GPIOE, 1);
	IO_TO_GPO(GPIOE, 0);
	IO_TO_GPO(GPIOB, 4);

	RCC->APB1ENR |= 1UL<<28; // Power interface clock enable - needed to configure PWR registers
	RCC->PLLCFGR = 9UL<<24 /*Q*/ | 1UL<<22 /*HSE as source*/ | 0b00UL<<16 /*P=2*/ | 216UL<<6 /*N*/ | 4UL /*M*/;

	RCC->CR |= 1UL<<16; // HSE clock on
	RCC->CR |= 1UL<<24; // PLL on

	PWR->CR1 |= 1UL<<16; // Overdrive on
	while(!(PWR->CSR1 & (1UL<<16))) ; // Wait for overdrive ready
	PWR->CR1 |= 1UL<<17; // Switch to overdrive
	while(!(PWR->CSR1 & (1UL<<17))) ; // Wait for overdrive switching ready



	FLASH->ACR = 1UL<<9 /* ART accelerator enable (caches) */ | 1UL<<8 /*prefetch enable*/ | 7UL /*7 wait states*/;
	RCC->CFGR = 0b100UL<<13 /*APB2 div 2*/ | 0b101UL<<10 /*APB1 div 4*/;
	RCC->DCKCFGR2 = 0b01UL<<4 /*USART3 = sysclk*/ | 0b00UL<<20 /*I2C3 = APB1clk*/;

	while(!(RCC->CR & 1UL<<25)) ; // Wait for PLL
	RCC->CFGR |= 0b10; // Change PLL to system clock
	while((RCC->CFGR & (0b11UL<<2)) != (0b10UL<<2)) ; // Wait for switchover to PLL.

	RCC->AHB2ENR = 1UL<<0 /*DCMI*/;
	RCC->APB1ENR |= 1UL<<18 /*USART3*/ | 1UL<<23 /*I2C3*/;
	RCC->APB2ENR = 1UL<<14 /*SYSCFG*/ | 1UL<<8 /*ADC1*/;


	// PIN4 = PE5 = 3V3 open-drain nPWRGOOD
	SYSCFG->EXTICR[1] = 0b0100UL<<4; // PORTE used for EXTI5.
	IO_PULLUP_ON(GPIOE, 5);
	EXTI->IMR |= 1UL<<5;
	EXTI->FTSR |= 1UL<<5;

	/*
		USART3 = the raspi USART @ APB1 = 54 MHz
		fck=216MHz
	*/

	IO_TO_ALTFUNC(GPIOB, 10);
	IO_TO_ALTFUNC(GPIOB, 11);
	IO_SET_ALTFUNC(GPIOB, 10, 7);
	IO_SET_ALTFUNC(GPIOB, 11, 7);
//	USART3->BRR = 216; // 1.0 Mbaud/s
	USART3->BRR = 234; // 921600
//	USART3->BRR = 469; // 460800
//	USART3->BRR = 1875; // 115200
//	USART3->CR2 = 0b10UL<<12 /* 2 stop bits*/;
	USART3->CR3 = 1UL<<6 /*RX DMA*/;

	DMA1_Stream1->PAR = (uint32_t)&(USART3->RDR);
	DMA1_Stream1->M0AR = (uint32_t)&rx_config;

	DMA1_Stream1->NDTR = sizeof(config_t);
	DMA1_Stream1->CR = 4UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0UL<<18 /*double buffer OFF*/ | 1UL<<8 /*circular ON*/ |
			   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/;

	DMA_CLEAR_INTFLAGS(DMA1, 1);
	DMA1_Stream1->CR |= 1; // Enable DMA

	USART3->CR1 = 0UL<<15 /*Oversamp:16*/ | 1UL<<5 /*RX interrupt*/ | 1UL<<3 /*TX ena*/ | 1UL<<2 /*RX ena*/ |  1UL /*USART ENA*/;


	/*	
		ADC  @ APB2 (108 MHz)
		ADC clock max 36MHz ("typ: 30 MHz")
		prescaler = 4 -> 27MHz

		Batt voltage = pin 26 = ADC1_IN10

	*/
//	ADC->CCR = 1UL<<23 /*temp sensor & Vref ena*/ | 0b01UL<<16 /*prescaler=4*/ | 

//	ADC1->CR1 = 1UL<<23 /*AWD ena*/ | 1UL<<9 /*AWD only on a single channel*/ | 1UL<<8 /*scan mode*/ | 1UL<<6 /*AWD interrupt*/ | 10UL /*AWD channel*/;
//	ADC1->CR2 = 1UL<<8 /*DMA mode*/ | 1UL<<1 /*continuous conversion*/;
//	DMA2_Stream4->PAR = (uint32_t)&(ADC1->DR);
//	DMA2_Stream4->M0AR = (uint32_t)&(adc_data[0]);
//	DMA2_Stream4->CR = 0UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 1UL<<8 /*circular*/ |
//			   0b01UL<<13 /*16-bit mem*/ | 0b01UL<<11 /*16-bit periph*/ |
//	                   1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/;
//	DMA2_Stream4->NDTR = sizeof(adc_datum_t)*NUM_ADC_DATA;
//	DMA_CLEAR_INTFLAGS(DMA2, 4);
//	DMA2_Stream4->CR |= 1; // Enable DMA
//	ADC1->CR2 |= 1UL; // Enable ADC // OPT_TODO: try combining with the previous
//	ADC1->CR2 |= 1UL<<30; // Start conversion   OPT_TODO: try removing this


	/*
		Interrupts will have 4 levels of pre-emptive priority, and 4 levels of sub-priority.

		Interrupt with more urgent (lower number) pre-emptive priority will interrupt another interrupt running.

		If interrupts with similar or lower urgency happen during another ISR, the subpriority level will decide
		the order they will be run after finishing the more urgent ISR first.
	*/
	NVIC_SetPriorityGrouping(2);
	NVIC_SetPriority(EXTI9_5_IRQn, 0b0000);
	NVIC_EnableIRQ(EXTI9_5_IRQn);

	IO_PULLUP_ON(GPIOC, 13);


	__enable_irq();

	RLED_ON();
	GLED_ON();

	epc_test();
	//uart_dbg();
}
