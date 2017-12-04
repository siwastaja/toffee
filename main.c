#include <stdint.h>
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

// Things written to I2C CR1 every time:
#define I2C_CR1_BASICS (0UL<<8 /*Digital filter len 0 to 15*/)
#define I2C_CR1_BASICS_ON (I2C_CR1_BASICS | 1UL /*keep it on*/)

volatile int epc_i2c_busy;

volatile int epc_i2c_read_state;
uint8_t epc_i2c_read_slave_addr;
uint8_t *epc_i2c_read_buf;
uint8_t epc_i2c_read_len;

void epc_i2c_write_dma(uint8_t slave_addr_7b, uint8_t *buf, uint8_t len)
{
	I2C3->ICR = 1UL<<5; // Clear any pending STOPF interrupt 

	if(len > 1) // Actually use DMA
	{
		I2C3->CR1 = I2C_CR1_BASICS_ON |  1UL<<5 /*STOPF interrupt - the only specified way to detect finished transfer when AUTOEND=1 and RELOAD=0*/ | 1UL<<14 /*TX DMA*/;

		DMA1_Stream0->CR = 8UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 
				   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
			           1UL<<10 /*mem increment*/ | 0b01UL<<6 /*mem-to-periph*/;

		DMA1_Stream0->NDTR = len;
		DMA1_Stream0->M0AR = (uint32_t)buf;

		DMA_CLEAR_INTFLAGS(DMA1, 0);
		DMA1_Stream0->CR |= 1; // Enable DMA // OPT_TODO: try combining with previous CR write.
	}
	else	// Just do the single write right now.
	{
		I2C3->CR1 = I2C_CR1_BASICS_ON |  1UL<<5 /*STOPF interrupt - the only specified way to detect finished transfer when AUTOEND=1 and RELOAD=0*/; // no DMA
		I2C3->TXDR = buf[0];
	}

	I2C3->CR2 = 1UL<<25 /*AUTOEND: stop is generated after len bytes*/ | ((uint32_t)len)<<16 | 0UL<<10 /*write*/ | slave_addr_7b<<1;
	I2C3->CR2 |= 1UL<<13; // START. OPT_TODO: try combining with previous CR2 write.
}

#define epc_i2c_write epc_i2c_write_dma

void epc_i2c_read_dma(uint8_t slave_addr_7b, uint8_t *buf, uint8_t len)
{
	DMA1_Stream1->CR = 1UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 
			   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/ | 1UL<<4 /*transfer complete interrupt*/;

	DMA1_Stream1->NDTR = len;
	DMA1_Stream1->M0AR = (uint32_t)buf;

	I2C3->CR1 = I2C_CR1_BASICS_ON | 0UL<<5 /*OFF FOR READ: STOPF interrupt - the only specified way to detect finished transfer when AUTOEND=1 and RELOAD=0*/ | 1UL<<15 /*RX DMA*/ ;

	I2C3->CR2 = 1UL<<25 /*AUTOEND: stop is generated after len bytes*/ | ((uint32_t)len)<<16 | 1UL<<10 /*read*/ | slave_addr_7b<<1;

	DMA_CLEAR_INTFLAGS(DMA1, 1);
	DMA1_Stream1->CR |= 1; // Enable DMA // OPT_TODO: try combining with previous CR write.

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

void epc_i2c_read(uint8_t slave_addr_7b, uint8_t reg_addr, uint8_t *buf, uint8_t len)
{
	/*
		Full I2C read cycle consists of a write cycle (payload = reg_addr), without stop condition,
		then the actual read cycle starting with the repeated start condition.

		Since the write is always one byte of payload, we run it without DMA.

		After the write is complete, there is no AUTOEND generated, instead we get an interrupt, allowing
		us to go on with the read.
	*/
	epc_i2c_busy = 1;

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
	return epc_i2c_busy;
}


void epc_rx_dma_inthandler() // read complete.
{
	DMA_CLEAR_INTFLAGS(DMA1, 1);
	epc_i2c_busy = 0;
}

/*
	For some reason, I can't get repeated start (Sr) condition out of the new STM32 I2C - it generates a stop-start sequence even when only asked for a START.
	Luckily, the EPC chip still works correctly even though this violates the spec.
*/
void epc_i2c_inthandler()
{
	if(epc_i2c_read_state)
	{
		// Writing START to 1 (in epc_i2c_read_dma()) clears the interrupt flag
		epc_i2c_read_dma(epc_i2c_read_slave_addr, epc_i2c_read_buf, epc_i2c_read_len);
		epc_i2c_read_state=0;

	}
	else // STOPF interrupt - this was a write.
	{
/*
			delay_us(100);
			char printbuf[64];
			GLED_OFF();
			uint32_t tmp = I2C3->ISR;
			uint32_t tmp2 = I2C3->CR1;
			uint32_t tmp3 = I2C3->CR2;
			uint32_t tmp4 = DMA1_Stream1->CR;
			uint32_t tmp5 = DMA1_Stream1->NDTR;
			uint32_t tmp6 = DMA1->LIFCR;
			I2C3->CR1 = 0;
			IO_TO_GPO(GPIOC, 9);
			IO_TO_GPO(GPIOA, 8);

			uart_print_string_blocking("INTERRUPT: I2C->ISR = ");
			o_utoa32_fixed(tmp, printbuf); uart_print_string_blocking(printbuf);
			uart_print_string_blocking("\r\n");

			uart_print_string_blocking("I2C->CR1 = ");
			o_utoa32_fixed(tmp2, printbuf); uart_print_string_blocking(printbuf);
			uart_print_string_blocking("\r\n");

			uart_print_string_blocking("I2C->CR2 = ");
			o_utoa32_fixed(tmp3, printbuf); uart_print_string_blocking(printbuf);
			uart_print_string_blocking("\r\n");

			uart_print_string_blocking("DMA1_Stream1->CR = ");
			o_utoa32_fixed(tmp4, printbuf); uart_print_string_blocking(printbuf);
			uart_print_string_blocking("\r\n");

			uart_print_string_blocking("DMA1_Stream1->NDTR = ");
			o_utoa32_fixed(tmp5, printbuf); uart_print_string_blocking(printbuf);
			uart_print_string_blocking("\r\n");
	
			uart_print_string_blocking("DMA1->LIFCR = ");
			o_utoa32_fixed(tmp6, printbuf); uart_print_string_blocking(printbuf);
			uart_print_string_blocking("\r\n");

			while(1);
*/

		I2C3->ICR = 1UL<<5; // Clear the STOPF flag.
		// Write is now finished.
		epc_i2c_busy = 0;
		I2C3->CR1 = I2C_CR1_BASICS_ON;
	}
}


void epc_i2c_init()
{
	// DMA1 Stream0 = tx
	// DMA1 Stream1 = rx
	// For now, init in Fast mode 400 kHz
	// Use the datasheet example of 48MHz input clock, with prescaler 1.125 times bigger for 54 MHz. Sync delay calculations are near enough.
	// Errata: tSU;DAT (do they mean SDADEL register??) must be at least one I2CCLK period
	// Errata: bus errors are spuriously generated for no reason, the bug is not said to affect the transfer: just ignore BERR.
 
	DMA1_Stream0->PAR = (uint32_t)&(I2C3->TXDR);
	DMA1_Stream1->PAR = (uint32_t)&(I2C3->RXDR);

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
	NVIC_SetPriority(DMA1_Stream1_IRQn, 0b0101);
	NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

#define EPC_XS 160
#define EPC_YS 60

typedef struct __attribute__((packed))
{
	uint16_t img_mono[EPC_XS*EPC_YS];
} epc_frame_t;

// Double buffer: one frame being acquired, another processed
epc_frame_t epc_frames[2];
epc_frame_t *epc_frame_acq;
epc_frame_t *epc_frame_proc;

volatile int epc_capture_finished = 0;
void epc_dcmi_dma_inthandler()
{
	// DMA finished
	DMA_CLEAR_INTFLAGS(DMA2, 7);

/*
	if(DMA2_Stream7->CR & 1UL<<19) // Current (new) DMA target = M1AR = epc_frames[1]
	{
		epc_frame_proc = &epc_frames[0];
		epc_frame_acq  = &epc_frames[1];
	}
	else
	{
		epc_frame_proc = &epc_frames[1];
		epc_frame_acq  = &epc_frames[0];
	}
*/
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
	RLED_ON();
	error(0);
}

void epc_dcmi_init()
{
	epc_frame_acq = &epc_frames[0];
	epc_frame_proc = &epc_frames[0]; // !!!!!!!!!!!!!

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
	DMA2_Stream7->M0AR = (uint32_t)&(epc_frames[0]);
//	DMA2_Stream7->M1AR = (uint32_t)&(epc_frames[1]);
	DMA2_Stream7->CR = 1UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0UL<<18 /*double buffer OFF*/ | 0UL<<8 /*circular OFF*/ |
			   0b10UL<<13 /*32-bit mem*/ | 0b10UL<<11 /*32-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/ | 1UL<<4 /* Transfer complete interrupt*/;

	//DMA2_Stream7->NDTR = sizeof(epc_frame_t)/4;

	//DMA_CLEAR_INTFLAGS(DMA2, 7);
	//DMA2_Stream7->CR |= 1; // Enable DMA

	NVIC_SetPriority(DMA2_Stream7_IRQn, 0b1111);
	NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

void dcmi_start_dma()
{
	// Disable the stream first
	DMA2_Stream7->CR = 1UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0UL<<18 /*double buffer OFF*/ | 0UL<<8 /*circular OFF*/ |
			   0b10UL<<13 /*32-bit mem*/ | 0b10UL<<11 /*32-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/;

	while(DMA2_Stream7->CR & 1UL) ;

	DMA_CLEAR_INTFLAGS(DMA2, 7);

	DMA2_Stream7->NDTR = sizeof(epc_frame_t)/4+1;
	DMA2_Stream7->CR = 1UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0UL<<18 /*double buffer OFF*/ | 0UL<<8 /*circular OFF*/ |
			   0b10UL<<13 /*32-bit mem*/ | 0b10UL<<11 /*32-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-mem*/ | 1UL<<4 /* Transfer complete interrupt*/;

	DMA_CLEAR_INTFLAGS(DMA2, 7);
	DMA2_Stream7->CR |= 1; // Enable DMA

	DCMI->CR |= 1UL<<0; // Start CAPTURE

}

#define EPC_ADDR 0b0100000
void epc_test()
{
	uint8_t i2c_buf[16];
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


	uart_print_string_blocking("read IC type & version...");
	epc_i2c_read(EPC_ADDR, 0x00, i2c_buf, 2);

	while(epc_i2c_is_busy());
	uart_print_string_blocking("type=");
	o_utoa8_fixed(i2c_buf[0], printbuf); uart_print_string_blocking(printbuf);
	uart_print_string_blocking(" version=");
	o_utoa8_fixed(i2c_buf[1], printbuf); uart_print_string_blocking(printbuf);
	uart_print_string_blocking("\r\n");

	uart_print_string_blocking("read embedded sync data labels... ");
	epc_i2c_read(EPC_ADDR, 0x1c, i2c_buf, 4);
/*	uint32_t tmp[30];
	for(int i=0; i<30; i++)
	{
		tmp[i] = DMA1_Stream1->NDTR;
		delay_us(50);
	}

	for(int i=0; i<30; i++)
	{
		o_utoa32(tmp[i], printbuf); uart_print_string_blocking(printbuf);
		uart_print_string_blocking(" ");
	}

*/
	while(epc_i2c_is_busy());
	for(int i=0; i<4; i++)
	{
		o_utoa8_fixed(i2c_buf[i], printbuf); uart_print_string_blocking(printbuf);
		uart_print_string_blocking(" ");
	}

/*
	uart_print_string_blocking("\r\n");

	uart_print_string_blocking("I2C->CR1 = ");
	o_utoa32_fixed(I2C3->CR1, printbuf); uart_print_string_blocking(printbuf);
	uart_print_string_blocking("\r\n");

	uart_print_string_blocking("I2C->CR2 = ");
	o_utoa32_fixed(I2C3->CR2, printbuf); uart_print_string_blocking(printbuf);
	uart_print_string_blocking("\r\n");

	uart_print_string_blocking("I2C->ISR = ");
	o_utoa32_fixed(I2C3->ISR, printbuf); uart_print_string_blocking(printbuf);
	uart_print_string_blocking("\r\n");

	uart_print_string_blocking("DMA1_Stream1->CR = ");
	o_utoa32_fixed(DMA1_Stream1->CR, printbuf); uart_print_string_blocking(printbuf);
	uart_print_string_blocking("\r\n");

	uart_print_string_blocking("DMA1_Stream1->NDTR = ");
	o_utoa32_fixed(DMA1_Stream1->NDTR, printbuf); uart_print_string_blocking(printbuf);
	uart_print_string_blocking("\r\n");
	
	uart_print_string_blocking("DMA1->LIFCR = ");
	o_utoa32_fixed(DMA1->LIFCR, printbuf); uart_print_string_blocking(printbuf);
	uart_print_string_blocking("\r\n");

*/

	uart_print_string_blocking("\r\n");


	{
		uint8_t b[2] = {0xcb, 0b00101111};
		uart_print_string_blocking("write i2c&tcmi control reg...");
		epc_i2c_write(EPC_ADDR, b, 2);
		while(epc_i2c_is_busy());
		uart_print_string_blocking("ok\r\n");
	}


	{
		uint8_t b[2] ={0x90, 0b11001000};
		uart_print_string_blocking("write led driver control...");
		epc_i2c_write(EPC_ADDR, b, 2);
		while(epc_i2c_is_busy());
		uart_print_string_blocking("ok\r\n");
	}

	{
		uint8_t b[2] = {0x92, 0b11000100};
		uart_print_string_blocking("write modulation select (greyscale)...");
		epc_i2c_write(EPC_ADDR, b, 2);
		while(epc_i2c_is_busy());
		uart_print_string_blocking("ok\r\n");
	}

	{
		uint8_t b[3] = {0xA2, 0x80, 0xFF};
		uart_print_string_blocking("write integration length...");
		epc_i2c_write(EPC_ADDR, b, 3);
		while(epc_i2c_is_busy());
		uart_print_string_blocking("ok\r\n");
	}

	delay_ms(100);

	while(DBG_BUT()) ;

	uart_print_string_blocking("epc_dcmi_init...");
	epc_dcmi_init();
	uart_print_string_blocking("ok\r\n");

	{
		uint8_t b[2] = {0xa4, 1};
		uart_print_string_blocking("write shutter (for dummy frame)...");
		dcmi_start_dma();
		epc_i2c_write(EPC_ADDR, b, 2);
		while(epc_i2c_is_busy());
		delay_ms(100);
		uart_print_string_blocking("ok\r\n");
	}


	while(1)
	{
		{
			uint8_t b[2] = {0xa4, 1};
			uart_print_string_blocking("write shutter (acq single frame)...");

			dcmi_start_dma();
			epc_i2c_write(EPC_ADDR, b, 2);
			while(epc_i2c_is_busy());
			uart_print_string_blocking("ok\r\n");
		}


		uart_print_string_blocking("wait frame...");
		while(!epc_capture_finished) ;
 		epc_capture_finished = 0;
		uart_print_string_blocking("ok\r\n");

		for(int yy=10; yy < 60; yy+=20)
		{
			for(int xx=20; xx < 160; xx+=40)
			{
				uint16_t val = epc_frame_proc->img_mono[yy*EPC_XS+xx];
				for(int b=15; b>=0; b--)
				{
					if(val&(1<<b))
						uart_print_string_blocking("1");
					else
						uart_print_string_blocking("0");

				}

				uart_print_string_blocking(" ");
				int16_t lum = ((val&0b0011111111111100)>>2)-2048;
				o_itoa16_fixed(lum, printbuf); uart_print_string_blocking(printbuf);
				uart_print_string_blocking("    ");
			}
			uart_print_string_blocking("\r\n");
		}
		uart_print_string_blocking("\r\n");

		delay_ms(1000);

	}


}

#define NUM_ADC_DATA 2

typedef struct
{
	uint16_t vbat;
	uint16_t tcpu;
} adc_datum_t;

volatile adc_datum_t adc_data[NUM_ADC_DATA];


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
		16x oversampling
	*/

	IO_TO_ALTFUNC(GPIOB, 10);
	IO_TO_ALTFUNC(GPIOB, 11);
	IO_SET_ALTFUNC(GPIOB, 10, 7);
	IO_SET_ALTFUNC(GPIOB, 11, 7);
	USART3->BRR = 0x3a9; //230400
	USART3->CR1 = 1UL<<5 /*RX interrupt*/ | 1UL<<3 /*TX ena*/ | 1UL<<2 /*RX ena*/ | 1UL /*USART ENA*/;

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
}
