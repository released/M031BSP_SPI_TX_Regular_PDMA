/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    A project template for M031 MCU.
 *
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#define _debug_SPI_GET_VER_						(0)
#define _debug_SPI_REGULAR_TX_					(0)
#define _debug_SPI_SEND_RCV_CMD				(0)

#define SPI_FREQ 								(200000ul)

#define SPI_MASTER_TX_DMA_CH   				(3)
#define SPI_MASTER_RX_DMA_CH 					(4)
#define SPI_MASTER_OPENED_CH_TX   				(1 << SPI_MASTER_TX_DMA_CH)
#define SPI_MASTER_OPENED_CH_RX 				(1 << SPI_MASTER_RX_DMA_CH)

#define SPI_DATA_LEN 							(32)
uint8_t SpiTxBuffer[SPI_DATA_LEN] = {0};
uint8_t SpiRxBuffer[SPI_DATA_LEN] = {0};

//#define ENABLE_SPI_RX


#define SPI_SET_CS_LOW							(PA3 = 0)
#define SPI_SET_CS_HIGH							(PA3 = 1)

enum
{
	_state_DEFAULT_ = 0 , 
	_state_RECEIVE_ADDRESS_ = 1 ,
	_state_CHECK_RX_OR_TX_ = 2 ,
	_state_RECEIVE_RX_ = 3 ,	
	_state_TRANSMIT_TX_ = 4 ,	
};

enum
{
	ENABLE_SPI_REGULAR = 0 ,
	ENABLE_SPI_PDMA_POLLING = 1 , 
	ENABLE_SPI_PDMA_IRQ = 2 ,	
};

enum
{
	ENABLE_SPI_NO_AUTO_SS = 0 , 
	ENABLE_SPI_AUTO_SS = 1 ,	
};

typedef enum{
	flag_Data_Ready = 0 ,

	flag_transmit_normal ,
	flag_transmit_getver ,
	flag_transmit_sendReceive ,
	flag_transmit_end ,

	flag_enable_SPI_auto_SS ,
	flag_enable_SPI_regular ,	
	flag_enable_SPI_pdma_polling ,
	flag_enable_SPI_pdma_irq ,	
	flag_error ,	
	flag_DEFAULT	
}flag_Index;

#define HIBYTE(v1)              					((uint8_t)((v1)>>8))                      //v1 is UINT16
#define LOBYTE(v1)              					((uint8_t)((v1)&0xFF))

volatile uint32_t BitFlag = 0;
#define BitFlag_ON(flag)							(BitFlag|=flag)
#define BitFlag_OFF(flag)							(BitFlag&=~flag)
#define BitFlag_READ(flag)							((BitFlag&flag)?1:0)
#define ReadBit(bit)								(uint32_t)(1<<bit)

#define is_flag_set(idx)							(BitFlag_READ(ReadBit(idx)))
#define set_flag(idx,en)							( (en == 1) ? (BitFlag_ON(ReadBit(idx))) : (BitFlag_OFF(ReadBit(idx))))

uint32_t conter_tick = 0;

/*****************************************************************************/

void tick_counter(void)
{
	conter_tick++;
}

uint32_t get_tick(void)
{
	return (conter_tick);
}

void set_tick(uint32_t t)
{
	conter_tick = t;
}

void compare_buffer(uint8_t *src, uint8_t *des, int nBytes)
{
    uint16_t i = 0;	
	
    for (i = 0; i < nBytes; i++)
    {
        if (src[i] != des[i])
        {
            printf("error idx : %4d : 0x%2X , 0x%2X\r\n", i , src[i],des[i]);
			set_flag(flag_error , ENABLE);
        }
    }

	if (!is_flag_set(flag_error))
	{
    	printf("%s finish \r\n" , __FUNCTION__);	
		set_flag(flag_error , DISABLE);
	}

}

void reset_buffer(uint8_t *pucBuff, int nBytes)
{
	#if 1
    uint16_t i = 0;	
    for ( i = 0; i < nBytes; i++)
    {
        pucBuff[i] = 0x00;
    }	
	#else	//extra 20 bytes , with <string.h>
	memset(pucBuff, 0, nBytes * (sizeof(pucBuff[0]) ));
	#endif
}

void dump_buffer(uint8_t *pucBuff, int nBytes)
{
    uint16_t i = 0;
    
    printf("dump_buffer : %2d\r\n" , nBytes);    
    for (i = 0 ; i < nBytes ; i++)
    {
        printf("0x%2X," , pucBuff[i]);
        if ((i+1)%8 ==0)
        {
            printf("\r\n");
        }            
    }
    printf("\r\n\r\n");
}

void  dump_buffer_hex(uint8_t *pucBuff, int nBytes)
{
    int     nIdx, i;

    nIdx = 0;
    while (nBytes > 0)
    {
        printf("0x%04X  ", nIdx);
        for (i = 0; i < 16; i++)
            printf("%02X ", pucBuff[nIdx + i]);
        printf("  ");
        for (i = 0; i < 16; i++)
        {
            if ((pucBuff[nIdx + i] >= 0x20) && (pucBuff[nIdx + i] < 127))
                printf("%c", pucBuff[nIdx + i]);
            else
                printf(".");
            nBytes--;
        }
        nIdx += 16;
        printf("\n");
    }
    printf("\n");
}

void delay(uint16_t dly)
{
/*
	delay(100) : 14.84 us
	delay(200) : 29.37 us
	delay(300) : 43.97 us
	delay(400) : 58.5 us	
	delay(500) : 73.13 us	
	
	delay(1500) : 0.218 ms (218 us)
	delay(2000) : 0.291 ms (291 us)	
*/

	while( dly--);
}


void delay_ms(uint16_t ms)
{
	TIMER_Delay(TIMER0, 1000*ms);
}

void PDMA_Polling_SPI(void)
{
	uint32_t u32RegValue = 0;
	uint32_t u32Abort = 0;	

    while(1)
    {
        /* Get interrupt status */
        u32RegValue = PDMA_GET_INT_STATUS(PDMA);
        /* Check the DMA transfer done interrupt flag */
        if(u32RegValue & PDMA_INTSTS_TDIF_Msk)
        {
            /* Check the PDMA transfer done interrupt flags */
            if((PDMA_GET_TD_STS(PDMA) & SPI_MASTER_OPENED_CH_TX) == SPI_MASTER_OPENED_CH_TX)
            {
                /* Clear the DMA transfer done flags */
                PDMA_CLR_TD_FLAG(PDMA, SPI_MASTER_OPENED_CH_TX);
				
                /* Disable SPI PDMA TX function */
                SPI_DISABLE_TX_PDMA(SPI0);

				set_flag(flag_transmit_end,ENABLE);
				
                break;
            }

			#if defined (ENABLE_SPI_RX)
            if((PDMA_GET_TD_STS(PDMA) & SPI_MASTER_OPENED_CH_RX) == SPI_MASTER_OPENED_CH_RX)
            {
                /* Clear the DMA transfer done flags */
                PDMA_CLR_TD_FLAG(PDMA, SPI_MASTER_OPENED_CH_RX);
				
                /* Disable SPI PDMA TX function */
                SPI_DISABLE_TX_PDMA(SPI0);

				set_flag(flag_transmit_end,ENABLE);
				
                break;
            }
			#endif

            /* Check the DMA transfer abort interrupt flag */
            if(u32RegValue & PDMA_INTSTS_ABTIF_Msk)
            {
                /* Get the target abort flag */
                u32Abort = PDMA_GET_ABORT_STS(PDMA);
                /* Clear the target abort flag */
                PDMA_CLR_ABORT_FLAG(PDMA,u32Abort);
                break;
            }
        }
    }

}

void PDMA_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS(PDMA);

    if (status & PDMA_INTSTS_ABTIF_Msk)   /* abort */
    {
        printf("target abort interrupt !!\n");
        PDMA_CLR_ABORT_FLAG(PDMA, PDMA_GET_ABORT_STS(PDMA));
    }
    else if (status & PDMA_INTSTS_TDIF_Msk)     /* done */
    {
        if((PDMA_GET_TD_STS(PDMA) & SPI_MASTER_OPENED_CH_TX) == SPI_MASTER_OPENED_CH_TX)
        {
            /* Clear PDMA transfer done interrupt flag */
            PDMA_CLR_TD_FLAG(PDMA, SPI_MASTER_OPENED_CH_TX);

			//insert process
			SPI_DISABLE_TX_PDMA(SPI0);

			set_flag(flag_transmit_end,ENABLE);
        } 

		#if defined (ENABLE_SPI_RX)
        if((PDMA_GET_TD_STS(PDMA) & SPI_MASTER_OPENED_CH_RX) == SPI_MASTER_OPENED_CH_RX)
        {
            /* Clear PDMA transfer done interrupt flag */
            PDMA_CLR_TD_FLAG(PDMA, SPI_MASTER_OPENED_CH_RX);

			//insert process
			SPI_DISABLE_RX_PDMA(SPI0);

			set_flag(flag_transmit_end,ENABLE);
        } 
		#endif
    }
    else if (status & (PDMA_INTSTS_REQTOF0_Msk))     /* channel timeout */
    {
        printf("timeout interrupt !!\n");
        PDMA_CLR_TMOUT_FLAG(PDMA, SPI_MASTER_TX_DMA_CH);
    }
    else
    {
//        printf("unknown interrupt !!\n");
    }
}

void SPI_transmit_finish(void)
{
	if (!is_flag_set(flag_enable_SPI_auto_SS))
	{
		while(!is_flag_set(flag_transmit_end));		
		while (SPI_IS_BUSY(SPI0));
		SPI_SET_CS_HIGH;
	}	
}

void SPI_transmit_RxStart(void)
{
    uint32_t i = 0;
	
	if (!is_flag_set(flag_enable_SPI_auto_SS))
	{
		SPI_SET_CS_LOW;
	}
	set_flag(flag_transmit_end,DISABLE);

	if (is_flag_set(flag_enable_SPI_pdma_polling) | is_flag_set(flag_enable_SPI_pdma_irq))
	{
		//RX	
		PDMA_SetTransferCnt(PDMA,SPI_MASTER_RX_DMA_CH, PDMA_WIDTH_8, SPI_DATA_LEN);
		PDMA_SetTransferAddr(PDMA,SPI_MASTER_RX_DMA_CH, (uint32_t)&SPI0->RX, PDMA_SAR_FIX, (uint32_t)SpiRxBuffer, PDMA_DAR_INC);		
		/* Set request source; set basic mode. */
		PDMA_SetTransferMode(PDMA,SPI_MASTER_RX_DMA_CH, PDMA_SPI0_RX, FALSE, 0);
		SPI_TRIGGER_RX_PDMA(SPI0);

		if (is_flag_set(flag_enable_SPI_pdma_irq))
		{
			PDMA_EnableInt(PDMA, SPI_MASTER_RX_DMA_CH, PDMA_INT_TRANS_DONE);	
		}
		if (is_flag_set(flag_enable_SPI_pdma_polling))
		{
			PDMA_Polling_SPI();
		}

		SPI_transmit_finish();
	}
	
	if (is_flag_set(flag_enable_SPI_regular))
	{
	    for(i = 0 ; i < SPI_DATA_LEN ; i++)
	    {
	        SPI_WRITE_TX(SPI0, 0x00);
	        while(SPI_IS_BUSY(SPI0));
	        SpiRxBuffer[i] = SPI_READ_RX(SPI0);		
	    }

		if (!is_flag_set(flag_enable_SPI_auto_SS))
		{
		
			while (SPI_IS_BUSY(SPI0));
			SPI_SET_CS_HIGH;
		}	
	}	
}

void SPI_transmit_TxStart(void)
{
    uint32_t i = 0;

	if (!is_flag_set(flag_enable_SPI_auto_SS))
	{
		SPI_SET_CS_LOW;
	}
	set_flag(flag_transmit_end,DISABLE);
	
	if (is_flag_set(flag_enable_SPI_pdma_polling) | is_flag_set(flag_enable_SPI_pdma_irq))
	{
		//TX
		PDMA_SetTransferCnt(PDMA,SPI_MASTER_TX_DMA_CH, PDMA_WIDTH_8, SPI_DATA_LEN);
		PDMA_SetTransferAddr(PDMA,SPI_MASTER_TX_DMA_CH, (uint32_t)SpiTxBuffer, PDMA_SAR_INC, (uint32_t)&SPI0->TX, PDMA_DAR_FIX);		
		/* Set request source; set basic mode. */
		PDMA_SetTransferMode(PDMA,SPI_MASTER_TX_DMA_CH, PDMA_SPI0_TX, FALSE, 0);
		
	    //Select Single Request
	//    PDMA_SetBurstType(PDMA, SPI_MASTER_TX_DMA_CH, PDMA_REQ_SINGLE, 0);
	    /* Disable table interrupt */
	//    PDMA->DSCT[SPI_MASTER_TX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;	
	//	PDMA_DisableInt(PDMA, SPI_MASTER_TX_DMA_CH,PDMA_INT_TEMPTY);
			
		SPI_TRIGGER_TX_PDMA(SPI0);

		if (is_flag_set(flag_enable_SPI_pdma_irq))
		{
	    	PDMA_EnableInt(PDMA, SPI_MASTER_TX_DMA_CH, PDMA_INT_TRANS_DONE);	
		}
		if (is_flag_set(flag_enable_SPI_pdma_polling))
		{
			PDMA_Polling_SPI();
		}
		
		SPI_transmit_finish();
	}

	if (is_flag_set(flag_enable_SPI_regular))
	{
	    for(i = 0 ; i < SPI_DATA_LEN ; i++)
	    {
	        SPI_WRITE_TX(SPI0, SpiTxBuffer[i]);
	        while(SPI_IS_BUSY(SPI0));
	    }	

		if (!is_flag_set(flag_enable_SPI_auto_SS))
		{		
			while (SPI_IS_BUSY(SPI0));
			SPI_SET_CS_HIGH;
		}
	}	
}

void SPI_PDMA_Init(void)
{
	if (!is_flag_set(flag_enable_SPI_auto_SS))
	{
		SPI_SET_CS_LOW;
	}
	set_flag(flag_transmit_end,DISABLE);

    /* Open PDMA Channel */
    PDMA_Open(PDMA, SPI_MASTER_OPENED_CH_TX | SPI_MASTER_OPENED_CH_RX);

	//TX
    PDMA_SetTransferCnt(PDMA,SPI_MASTER_TX_DMA_CH, PDMA_WIDTH_8, SPI_DATA_LEN);
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA,SPI_MASTER_TX_DMA_CH, (uint32_t)SpiTxBuffer, PDMA_SAR_INC, (uint32_t)&SPI0->TX, PDMA_DAR_FIX);
    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA,SPI_MASTER_TX_DMA_CH, PDMA_SPI0_TX, FALSE, 0);
    /* Single request type. SPI only support PDMA single request type. */
    PDMA_SetBurstType(PDMA,SPI_MASTER_TX_DMA_CH, PDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
//    PDMA->DSCT[SPI_MASTER_TX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;
	PDMA_DisableInt(PDMA, SPI_MASTER_TX_DMA_CH,PDMA_INT_TEMPTY);
    SPI_TRIGGER_TX_PDMA(SPI0);	
	
	if (is_flag_set(flag_enable_SPI_pdma_irq))
	{	
    	PDMA_EnableInt(PDMA, SPI_MASTER_TX_DMA_CH, PDMA_INT_TRANS_DONE);
	}

	if (is_flag_set(flag_enable_SPI_pdma_polling))
	{
		PDMA_Polling_SPI();
	}
	
	#if defined (ENABLE_SPI_RX)	
	//RX	
    PDMA_SetTransferCnt(PDMA,SPI_MASTER_RX_DMA_CH, PDMA_WIDTH_8, SPI_DATA_LEN);
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA,SPI_MASTER_RX_DMA_CH, (uint32_t)&SPI0->RX, PDMA_SAR_FIX, (uint32_t)SpiRxBuffer, PDMA_DAR_INC);
    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA,SPI_MASTER_RX_DMA_CH, PDMA_SPI0_RX, FALSE, 0);
    /* Single request type. SPI only support PDMA single request type. */
    PDMA_SetBurstType(PDMA,SPI_MASTER_RX_DMA_CH, PDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
//    PDMA->DSCT[SPI_MASTER_RX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;
	PDMA_DisableInt(PDMA, SPI_MASTER_RX_DMA_CH,PDMA_INT_TEMPTY);	
    SPI_TRIGGER_RX_PDMA(SPI0);	

	if (is_flag_set(flag_enable_SPI_pdma_irq))
	{
    	PDMA_EnableInt(PDMA, SPI_MASTER_RX_DMA_CH, PDMA_INT_TRANS_DONE);	
	}	

	if (is_flag_set(flag_enable_SPI_pdma_polling))
	{
		PDMA_Polling_SPI();
	}
	
	#endif

	if (is_flag_set(flag_enable_SPI_pdma_irq))
	{
    	NVIC_EnableIRQ(PDMA_IRQn);
	}

	SPI_transmit_finish();
	
}

void SPI_Initial(uint8_t en_spi_pdma , uint8_t en_auto_ss)
{      
    uint16_t i = 0;
	
	//prepare data
    for (i=0; i < SPI_DATA_LEN; i++)
    {
        SpiTxBuffer[i] = 0xFF;
        SpiRxBuffer[i] = 0xFF;
    }

	SPI_Open(SPI0, SPI_MASTER, SPI_MODE_0, 8, SPI_FREQ);

	if (en_auto_ss)
	{
		set_flag(flag_enable_SPI_auto_SS ,ENABLE);

		SPI_EnableAutoSS(SPI0, SPI_SS, SPI_SS_ACTIVE_LOW);		
	}
	else
	{
		set_flag(flag_enable_SPI_auto_SS ,DISABLE);

   		SYS_UnlockReg();
	    SYS->GPA_MFPL &= ~( SYS_GPA_MFPL_PA3MFP_Msk);	
	    SYS->GPA_MFPL |=  SYS_GPA_MFPL_PA3MFP_GPIO;	
		GPIO_SetMode(PA,BIT3,GPIO_MODE_OUTPUT);	
    	SYS_LockReg();

	    SPI_DisableAutoSS(SPI0);
	}

	switch(en_spi_pdma)
	{
		case ENABLE_SPI_REGULAR:
			set_flag(flag_enable_SPI_regular ,ENABLE);				
			break;
		case ENABLE_SPI_PDMA_POLLING:
			set_flag(flag_enable_SPI_pdma_polling ,ENABLE);		
			//	SPI_SET_SUSPEND_CYCLE(SPI0,15);	
			SPI_PDMA_Init();			
			break;
		case ENABLE_SPI_PDMA_IRQ:
			set_flag(flag_enable_SPI_pdma_irq ,ENABLE);	
			//	SPI_SET_SUSPEND_CYCLE(SPI0,15);	
			SPI_PDMA_Init();			
			break;		
	}
	
	set_flag(flag_transmit_normal , DISABLE);

}

void SPI_process(void)
{
	uint16_t i = 0;	
	static uint8_t data_cnt = 1;

	if (is_flag_set(flag_transmit_sendReceive))
	{
		set_flag(flag_transmit_sendReceive , DISABLE);

		for (i = 0 ; i < SPI_DATA_LEN ; i++ )
		{
			SpiTxBuffer[i] = 0x81 + i ;
		}
		SpiTxBuffer[0] = 0x80;	
		SpiTxBuffer[1] = 0x84;	
		SpiTxBuffer[2] = 0x81;		
		SpiTxBuffer[3] = 0x04;	
		
		SpiTxBuffer[4] = 0x40 + data_cnt;	
		SpiTxBuffer[5] = 0x51 + data_cnt;	
		SpiTxBuffer[6] = 0x62 + data_cnt;	
		
		SpiTxBuffer[SPI_DATA_LEN-3] = 0x90 + data_cnt;	
		SpiTxBuffer[SPI_DATA_LEN-1] = 0x92 + data_cnt;
		
		SPI_transmit_TxStart();

		data_cnt = (data_cnt > 0x9) ? (1) : (data_cnt+1) ; 

		#if defined (ENABLE_SPI_RX)
		SPI_transmit_RxStart();
		#endif
		
		#if (_debug_SPI_SEND_RCV_CMD == 1)	//debug
		printf("RX=======\r\n");

		dump_buffer_hex(SpiRxBuffer,SPI_DATA_LEN);

		printf("\r\n");
		#endif		
	}
	

	if (is_flag_set(flag_transmit_getver))
	{
		set_flag(flag_transmit_getver , DISABLE);

		for (i = 0 ; i < SPI_DATA_LEN ; i++ )
		{
			SpiTxBuffer[i] = 0x81 + i ;
		}
		SpiTxBuffer[0] = 0x80;	
		SpiTxBuffer[1] = 0x10 + data_cnt;	
		SpiTxBuffer[2] = 0x21 + data_cnt;	
		SpiTxBuffer[3] = 0x32 + data_cnt;			

		SpiTxBuffer[SPI_DATA_LEN-4] = 'g';	
		SpiTxBuffer[SPI_DATA_LEN-3] = 'e';	
		SpiTxBuffer[SPI_DATA_LEN-2] = 't';	
		SpiTxBuffer[SPI_DATA_LEN-1] = 0x41;	
		
		SPI_transmit_TxStart();

		data_cnt = (data_cnt > 0x9) ? (1) : (data_cnt+1) ; 

		#if defined (ENABLE_SPI_RX)
		SPI_transmit_RxStart();
		#endif

		
		#if (_debug_SPI_GET_VER_ == 1)		//debug
		printf("RX=======\r\n");
		dump_buffer_hex(SpiRxBuffer,SPI_DATA_LEN);
		printf("\r\n");
		#endif
		
	}
	

	if (is_flag_set(flag_transmit_normal))
	{
		set_flag(flag_transmit_normal , DISABLE);

		for (i = 0 ; i < SPI_DATA_LEN ; i++ )
		{
			SpiTxBuffer[i] = 0x80 + i ;
		}

		SpiTxBuffer[SPI_DATA_LEN-2] = 0x90 + data_cnt;	
		SpiTxBuffer[SPI_DATA_LEN-1] = 0x92 + data_cnt;
		
		SPI_transmit_TxStart();

		data_cnt = (data_cnt > 0x9) ? (1) : (data_cnt+1) ; 

		#if defined (ENABLE_SPI_RX)
		SPI_transmit_RxStart();
		#endif
		
		#if (_debug_SPI_REGULAR_TX_ == 1)	//debug
		printf("RX=======\r\n");
		dump_buffer_hex(SpiRxBuffer,SPI_DATA_LEN);
		printf("\r\n");
		#endif
		
	}
}

void GPIO_Init (void)
{
    GPIO_SetMode(PB, BIT14, GPIO_MODE_OUTPUT);

    GPIO_SetMode(PB, BIT15, GPIO_MODE_OUTPUT);	
}


void TMR3_IRQHandler(void)
{
//	static uint32_t LOG = 0;
//	static uint16_t CNT = 0;
//	static uint16_t CNT_SPI = 0;
	static uint8_t state = 0;
	
    if(TIMER_GetIntFlag(TIMER3) == 1)
    {
        TIMER_ClearIntFlag(TIMER3);
		tick_counter();

		if ((get_tick() % 1000) == 0)
		{
//        	printf("%s : %4d\r\n",__FUNCTION__,LOG++);
			PB14 ^= 1;
		}

		if ((get_tick() % 50) == 0)
		{
			switch(state++)
			{
//				case 2:
//					set_flag(flag_transmit_getver , ENABLE);				
//					break;
				case 3:
					set_flag(flag_transmit_sendReceive , ENABLE);
					state = 0;
					break;				
				default:
					set_flag(flag_transmit_normal , ENABLE);					
					break;
				
			}

//			if (state++ >= 2)
//			{
//				set_flag(flag_transmit_getver , ENABLE);
//				state = 0;
//			}
//			else
//			{
//				set_flag(flag_transmit_normal , ENABLE);		
//			}
		}
	
    }
}


void TIMER3_Init(void)
{
    TIMER_Open(TIMER3, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER3);
    NVIC_EnableIRQ(TMR3_IRQn);	
    TIMER_Start(TIMER3);
}

void UARTx_Process(void)
{
	uint8_t res = 0;
	res = UART_READ(UART0);

	if (res == 'x' || res == 'X')
	{
		NVIC_SystemReset();
	}

	if (res > 0x7F)
	{
		printf("invalid command\r\n");
	}
	else
	{
		switch(res)
		{
			case '1':
				break;


			case 'X':
			case 'x':
			case 'Z':
			case 'z':
				NVIC_SystemReset();		
				break;
		}
	}
}

void UART02_IRQHandler(void)
{

    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))     /* UART receive data available flag */
    {
        while(UART_GET_RX_EMPTY(UART0) == 0)
        {
            UARTx_Process();
        }
    }

    if(UART0->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART_ClearIntFlag(UART0, (UART_INTSTS_RLSINT_Msk| UART_INTSTS_BUFERRINT_Msk));
    }	
}

void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
    UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
    NVIC_EnableIRQ(UART02_IRQn);
	
	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());	

}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    CLK_EnableModuleClock(TMR3_MODULE);
  	CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3SEL_HIRC, 0);

    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PCLK1, MODULE_NoMsk);
    CLK_EnableModuleClock(SPI0_MODULE);

    CLK_EnableModuleClock(PDMA_MODULE);

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* PA.3 is SPI0_SS,   PA.2 is SPI0_CLK,
       PA.1 is SPI0_MISO, PA.0 is SPI0_MOSI*/

    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA3MFP_Msk|SYS_GPA_MFPL_PA2MFP_Msk|SYS_GPA_MFPL_PA1MFP_Msk|SYS_GPA_MFPL_PA0MFP_Msk)) |
                    (SYS_GPA_MFPL_PA3MFP_SPI0_SS|SYS_GPA_MFPL_PA2MFP_SPI0_CLK|SYS_GPA_MFPL_PA1MFP_SPI0_MISO|SYS_GPA_MFPL_PA0MFP_SPI0_MOSI);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M031 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{
    SYS_Init();

	UART0_Init();
	GPIO_Init();
	TIMER3_Init();


//	SPI_Initial(ENABLE_SPI_REGULAR,ENABLE_SPI_AUTO_SS);
//	SPI_Initial(ENABLE_SPI_REGULAR,ENABLE_SPI_NO_AUTO_SS);

//	SPI_Initial(ENABLE_SPI_PDMA_IRQ,ENABLE_SPI_AUTO_SS);
	SPI_Initial(ENABLE_SPI_PDMA_IRQ,ENABLE_SPI_NO_AUTO_SS);

//	SPI_Initial(ENABLE_SPI_PDMA_POLLING,ENABLE_SPI_AUTO_SS);
//	SPI_Initial(ENABLE_SPI_PDMA_POLLING,ENABLE_SPI_NO_AUTO_SS);

    /* Got no where to go, just loop forever */
    while(1)
    {
		SPI_process();

    }
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
