#include <stdint.h>

#include "ext_include/stm32f7xx.h"

void stm32init();
void nmi_handler();
void invalid_handler();
void hardfault_handler();

extern void error(int code);
extern void main();

extern void epc_i2c_inthandler();
extern void epc_dcmi_dma_inthandler();

extern unsigned int _STACKTOP;

// Vector table on page 311 on the Reference Manual RM0410
unsigned int * the_nvic_vector[126] __attribute__ ((section(".nvic_vector"))) =
{
/* 0x0000                    */ (unsigned int *) &_STACKTOP,
/* 0x0004 RESET              */ (unsigned int *) stm32init,
/* 0x0008 NMI                */ (unsigned int *) nmi_handler,
/* 0x000C HARDFAULT          */ (unsigned int *) hardfault_handler,
/* 0x0010                    */ (unsigned int *) invalid_handler,
/* 0x0014                    */ (unsigned int *) invalid_handler,
/* 0x0018                    */ (unsigned int *) invalid_handler,
/* 0x001C                    */ (unsigned int *) invalid_handler,
/* 0x0020                    */ (unsigned int *) invalid_handler,
/* 0x0024                    */ (unsigned int *) invalid_handler,
/* 0x0028                    */ (unsigned int *) invalid_handler,
/* 0x002C                    */ (unsigned int *) invalid_handler,
/* 0x0030                    */ (unsigned int *) invalid_handler,
/* 0x0034                    */ (unsigned int *) invalid_handler,
/* 0x0038                    */ (unsigned int *) invalid_handler,
/* 0x003C                    */ (unsigned int *) invalid_handler,
/* 0x0040                    */ (unsigned int *) invalid_handler,
/* 0x0044                    */ (unsigned int *) invalid_handler,
/* 0x0048                    */ (unsigned int *) invalid_handler,
/* 0x004C                    */ (unsigned int *) invalid_handler,
/* 0x0050                    */ (unsigned int *) invalid_handler,
/* 0x0054                    */ (unsigned int *) invalid_handler,
/* 0x0058                    */ (unsigned int *) invalid_handler,
/* 0x005C                    */ (unsigned int *) invalid_handler,
/* 0x0060                    */ (unsigned int *) invalid_handler,
/* 0x0064                    */ (unsigned int *) invalid_handler,
/* 0x0068                    */ (unsigned int *) invalid_handler,
/* 0x006C                    */ (unsigned int *) invalid_handler,
/* 0x0070                    */ (unsigned int *) invalid_handler,
/* 0x0074                    */ (unsigned int *) invalid_handler,
/* 0x0078                    */ (unsigned int *) invalid_handler,
/* 0x007C                    */ (unsigned int *) invalid_handler,
/* 0x0080                    */ (unsigned int *) invalid_handler,
/* 0x0084                    */ (unsigned int *) invalid_handler,
/* 0x0088 ADC                */ (unsigned int *) invalid_handler,
/* 0x008C                    */ (unsigned int *) invalid_handler,
/* 0x0090                    */ (unsigned int *) invalid_handler,
/* 0x0094                    */ (unsigned int *) invalid_handler,
/* 0x0098                    */ (unsigned int *) invalid_handler,
/* 0x009C                    */ (unsigned int *) invalid_handler,
/* 0x00A0                    */ (unsigned int *) invalid_handler,
/* 0x00A4                    */ (unsigned int *) invalid_handler,
/* 0x00A8                    */ (unsigned int *) invalid_handler,
/* 0x00AC                    */ (unsigned int *) invalid_handler,
/* 0x00B0                    */ (unsigned int *) invalid_handler,
/* 0x00B4                    */ (unsigned int *) invalid_handler,
/* 0x00B8                    */ (unsigned int *) invalid_handler,
/* 0x00BC I2C1 event         */ (unsigned int *) invalid_handler,
/* 0x00C0 I2C1 error         */ (unsigned int *) invalid_handler,
/* 0x00C4 I2C2 event         */ (unsigned int *) invalid_handler,
/* 0x00C8 I2C2 error         */ (unsigned int *) invalid_handler,
/* 0x00CC                    */ (unsigned int *) invalid_handler,
/* 0x00D0                    */ (unsigned int *) invalid_handler,
/* 0x00D4                    */ (unsigned int *) invalid_handler,
/* 0x00D8                    */ (unsigned int *) invalid_handler,
/* 0x00DC                    */ (unsigned int *) invalid_handler,
/* 0x00E0                    */ (unsigned int *) invalid_handler,
/* 0x00E4                    */ (unsigned int *) invalid_handler,
/* 0x00E8                    */ (unsigned int *) invalid_handler,
/* 0x00EC                    */ (unsigned int *) invalid_handler,
/* 0x00F0                    */ (unsigned int *) invalid_handler,
/* 0x00F4                    */ (unsigned int *) invalid_handler,
/* 0x00F8                    */ (unsigned int *) invalid_handler,
/* 0x00FC                    */ (unsigned int *) invalid_handler,
/* 0x0100                    */ (unsigned int *) invalid_handler,
/* 0x0104                    */ (unsigned int *) invalid_handler,
/* 0x0108                    */ (unsigned int *) invalid_handler,
/* 0x010C                    */ (unsigned int *) invalid_handler,
/* 0x0110                    */ (unsigned int *) invalid_handler,
/* 0x0114                    */ (unsigned int *) invalid_handler,
/* 0x0118                    */ (unsigned int *) invalid_handler,
/* 0x011C                    */ (unsigned int *) invalid_handler,
/* 0x0120 DMA2_Stream0       */ (unsigned int *) invalid_handler,
/* 0x0124 DMA2_Stream1       */ (unsigned int *) invalid_handler,
/* 0x0128 DMA2_Stream2       */ (unsigned int *) invalid_handler,
/* 0x012C DMA2_Stream3       */ (unsigned int *) invalid_handler,
/* 0x0130 DMA2_Stream4       */ (unsigned int *) invalid_handler,
/* 0x0134                    */ (unsigned int *) invalid_handler,
/* 0x0138                    */ (unsigned int *) invalid_handler,
/* 0x013C                    */ (unsigned int *) invalid_handler,
/* 0x0140                    */ (unsigned int *) invalid_handler,
/* 0x0144                    */ (unsigned int *) invalid_handler,
/* 0x0148                    */ (unsigned int *) invalid_handler,
/* 0x014C                    */ (unsigned int *) invalid_handler,
/* 0x0150 DMA2_Stream5       */ (unsigned int *) invalid_handler,
/* 0x0154 DMA2_Stream6       */ (unsigned int *) invalid_handler,
/* 0x0158 DMA2_Stream7       */ (unsigned int *) epc_dcmi_dma_inthandler,
/* 0x015C                    */ (unsigned int *) invalid_handler,
/* 0x0160 I2C3 event         */ (unsigned int *) epc_i2c_inthandler,
/* 0x0164 I2C3 error         */ (unsigned int *) invalid_handler,
/* 0x0168                    */ (unsigned int *) invalid_handler,
/* 0x016C                    */ (unsigned int *) invalid_handler,
/* 0x0170                    */ (unsigned int *) invalid_handler,
/* 0x0174                    */ (unsigned int *) invalid_handler,
/* 0x0178                    */ (unsigned int *) invalid_handler,
/* 0x017C                    */ (unsigned int *) invalid_handler,
/* 0x0180                    */ (unsigned int *) invalid_handler,
/* 0x0184                    */ (unsigned int *) invalid_handler,
/* 0x0188                    */ (unsigned int *) invalid_handler,
/* 0x018C                    */ (unsigned int *) invalid_handler,
/* 0x0190                    */ (unsigned int *) invalid_handler,
/* 0x0194                    */ (unsigned int *) invalid_handler,
/* 0x0198                    */ (unsigned int *) invalid_handler,
/* 0x019C                    */ (unsigned int *) invalid_handler,
/* 0x01A0                    */ (unsigned int *) invalid_handler,
/* 0x01A4                    */ (unsigned int *) invalid_handler,
/* 0x01A8                    */ (unsigned int *) invalid_handler,
/* 0x01AC                    */ (unsigned int *) invalid_handler,
/* 0x01B0                    */ (unsigned int *) invalid_handler,
/* 0x01B4                    */ (unsigned int *) invalid_handler,
/* 0x01B8                    */ (unsigned int *) invalid_handler,
/* 0x01BC                    */ (unsigned int *) invalid_handler,
/* 0x01C0                    */ (unsigned int *) invalid_handler,
/* 0x01C4                    */ (unsigned int *) invalid_handler,
/* 0x01C8                    */ (unsigned int *) invalid_handler,
/* 0x01CC                    */ (unsigned int *) invalid_handler,
/* 0x01D0                    */ (unsigned int *) invalid_handler,
/* 0x01D4                    */ (unsigned int *) invalid_handler,
/* 0x01D8                    */ (unsigned int *) invalid_handler,
/* 0x01DC                    */ (unsigned int *) invalid_handler,
/* 0x01E0                    */ (unsigned int *) invalid_handler,
/* 0x01E4                    */ (unsigned int *) invalid_handler,
/* 0x01E8                    */ (unsigned int *) invalid_handler,
/* 0x01EC                    */ (unsigned int *) invalid_handler,
/* 0x01F0                    */ (unsigned int *) invalid_handler,
/* 0x01F4                    */ (unsigned int *) invalid_handler
};

extern unsigned int _BSS_BEGIN;
extern unsigned int _BSS_END;

extern unsigned int _DATA_BEGIN;
extern unsigned int _DATA_END;
extern unsigned int _DATAI_BEGIN;

extern unsigned int _BOOST_BEGIN;
extern unsigned int _BOOST_END;
extern unsigned int _BOOSTI_BEGIN;

extern void hwtest_main();

void stm32init(void)
{
	uint32_t* bss_begin = (uint32_t*)&_BSS_BEGIN;
	uint32_t* bss_end   = (uint32_t*)&_BSS_END;
	while(bss_begin < bss_end)
	{
		*bss_begin = 0;
		bss_begin++;
	}

	uint32_t* data_begin  = (uint32_t*)&_DATA_BEGIN;
	uint32_t* data_end    = (uint32_t*)&_DATA_END;
	uint32_t* datai_begin = (uint32_t*)&_DATAI_BEGIN;

	while(data_begin < data_end)
	{
		*data_begin = *datai_begin;
		data_begin++;
		datai_begin++;
	}

	main();
}


void nmi_handler(void)
{
	error(1);
}

void hardfault_handler(void)
{
	error(2);
}

void invalid_handler(void)
{
	error(3);
}
