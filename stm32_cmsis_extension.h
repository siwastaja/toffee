/*
	Extensions to make STM32 CMSIS headers more useful by offering a few simple macros as a midway between accessing each register
	completely "raw", or using the complex and slow configuration functions.
*/

#ifndef _STM32_CMSIS_EXTENSION
#define STM32_CMSIS_EXTENSION


#define IO_TO_GPI(port, idx) do{ uint32_t _tmp_ = (port)->MODER; _tmp_ &= ~(0b11UL<<((idx)*2)); (port)->MODER = _tmp_; }while(0)
#define IO_TO_GPO(port, idx) do{ uint32_t _tmp_ = (port)->MODER; _tmp_ &= ~(1UL<<((idx)*2+1)); _tmp_ |= 1UL<<((idx)*2); (port)->MODER = _tmp_; }while(0)
#define IO_TO_ALTFUNC(port, idx) do{ uint32_t _tmp_ = (port)->MODER; _tmp_ &= ~(1UL<<((idx)*2)); _tmp_ |= 1UL<<((idx)*2+1); (port)->MODER = _tmp_; }while(0)
#define IO_TO_ANALOG(port, idx) do{ uint32_t _tmp_ = (port)->MODER; _tmp_ |= 0b11<<((idx)*2); (port)->MODER = _tmp_; }while(0)

/*
	GCC seems to be stupid enough so that it doesn't understand to "optimize" away overflow/negative shift warnings when these operations happen inside
	the if/else branch that guarantees the "illegal" branch is always optimized away. So, we need to selectively disable warnings.
*/

#define IO_SET_ALTFUNC(port, pin, af) do{ _Pragma("GCC diagnostic push") _Pragma("GCC diagnostic ignored \"-Wshift-count-negative\"")  _Pragma("GCC diagnostic ignored \"-Wshift-count-overflow\"") \
	if((pin)<8) {uint32_t _tmp_ = (port)->AFR[0]; _tmp_ &= ~(0b1111UL<<((pin)*4));     _tmp_ |= af<<((pin)*4);     (port)->AFR[0] = _tmp_;} \
        else {uint32_t _tmp_ = (port)->AFR[1]; _tmp_ &= ~(0b1111UL<<(((pin)-8)*4)); _tmp_ |= af<<(((pin)-8)*4); (port)->AFR[1] = _tmp_;} _Pragma("GCC diagnostic pop") }while(0)


#define DMA_CLEAR_INTFLAGS(_dma_, _stream_) do{                   \
	if     ((_stream_) == 0) (_dma_)->LIFCR = 0b111101UL<<0;  \
	else if((_stream_) == 1) (_dma_)->LIFCR = 0b111101UL<<6;  \
	else if((_stream_) == 2) (_dma_)->LIFCR = 0b111101UL<<16; \
	else if((_stream_) == 3) (_dma_)->LIFCR = 0b111101UL<<22; \
	else if((_stream_) == 4) (_dma_)->HIFCR = 0b111101UL<<0;  \
	else if((_stream_) == 5) (_dma_)->HIFCR = 0b111101UL<<6;  \
	else if((_stream_) == 6) (_dma_)->HIFCR = 0b111101UL<<16; \
	else if((_stream_) == 7) (_dma_)->HIFCR = 0b111101UL<<22; \
	}while(0)


#endif
