/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    A project template for M031 MCU.
 *
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


typedef enum{
	flag_PDMA_Abort = 0 ,
	flag_PDMA_Done ,	
	
	flag_DEFAULT	
}flag_Index;


uint8_t BitFlag = 0;
#define BitFlag_ON(flag)							(BitFlag|=flag)
#define BitFlag_OFF(flag)							(BitFlag&=~flag)
#define BitFlag_READ(flag)							((BitFlag&flag)?1:0)
#define ReadBit(bit)								(uint8_t)(1<<bit)

#define is_flag_set(idx)							(BitFlag_READ(ReadBit(idx)))
#define set_flag(idx,en)							( (en == 1) ? (BitFlag_ON(ReadBit(idx))) : (BitFlag_OFF(ReadBit(idx))))

uint16_t g_au16Count[4] = {0};
volatile uint32_t g_u32IsTestOver = 0;
uint8_t duty = 30 ;
uint16_t freq = 250 ;
void PDMA_Init(void);

void CalPeriodTime(PWM_T *PWM, uint32_t u32Ch)
{
    uint16_t u16RisingTime, u16FallingTime, u16HighPeriod, u16LowPeriod, u16TotalPeriod;

    g_u32IsTestOver = 0;
    /* Wait PDMA interrupt (g_u32IsTestOver will be set at IRQ_Handler function) */
    while(g_u32IsTestOver == 0);

    u16RisingTime = g_au16Count[1];

    u16FallingTime = g_au16Count[0];

    u16HighPeriod = g_au16Count[1] - g_au16Count[2];

    u16LowPeriod = 0x10000 - g_au16Count[1];

    u16TotalPeriod = 0x10000 - g_au16Count[2];

    printf("Capture : Rising=%5d,Falling=%5d,High=%5d,Low=%5d,Total=%5d.\r\n",
           u16RisingTime, 
           u16FallingTime, 
           u16HighPeriod, 
           u16LowPeriod, 
           u16TotalPeriod);

}

void PWM_Out_DeInit(void)
{
    /* Set PWM0 channel 0 loaded value as 0 */
    PWM_Stop(PWM0, PWM_CH_0_MASK);

    /* Wait until PWM0 channel 0 Timer Stop */
    while((PWM0->CNT[0] & PWM_CNT_CNT_Msk) != 0);

    /* Disable Timer for PWM0 channel 0 */
    PWM_ForceStop(PWM0, PWM_CH_0_MASK);

    /* Disable PWM Output path for PWM0 channel 0 */
    PWM_DisableOutput(PWM0, PWM_CH_0_MASK);
}

void PWM_Out_Init(void)
{
    /* Set PWM0 channel 0 output configuration */
    PWM_ConfigOutputChannel(PWM0, 0, freq, duty);

    /* Enable PWM Output path for PWM0 channel 0 */
    PWM_EnableOutput(PWM0, PWM_CH_0_MASK);

    /* Enable Timer for PWM0 channel 0 */
    PWM_Start(PWM0, PWM_CH_0_MASK);
}

void PWM_Cap_DeInit(void)
{
    /* Set loaded value as 0 for PWM0 channel 2 */
    PWM_Stop(PWM0, PWM_CH_2_MASK);

    /* Wait until PWM0 channel 2 current counter reach to 0 */
    while((PWM0->CNT[2] & PWM_CNT_CNT_Msk) != 0);

    /* Disable Timer for PWM0 channel 2 */
    PWM_ForceStop(PWM0, PWM_CH_2_MASK);

    /* Disable Capture Function and Capture Input path for  PWM0 channel 2*/
    PWM_DisableCapture(PWM0, PWM_CH_2_MASK);

    /* Clear Capture Interrupt flag for PWM0 channel 2 */
    PWM_ClearCaptureIntFlag(PWM0, 2, PWM_CAPTURE_INT_FALLING_LATCH);

    /* Disable PDMA NVIC */
    NVIC_DisableIRQ(PDMA_IRQn);

    PDMA_Close(PDMA);
}

void PWM_Cap_Init(void)
{
	PDMA_Init();

	/* Set PWM0 channel 2 capture configuration */
	PWM_ConfigCaptureChannel(PWM0, 2, 62, 0);

	/* Enable Timer for PWM0 channel 2 */
	PWM_Start(PWM0, PWM_CH_2_MASK);

	/* Enable Capture Function for PWM0 channel 2 */
	PWM_EnableCapture(PWM0, PWM_CH_2_MASK);

	/* Enable falling capture reload */
	PWM0->CAPCTL |= PWM_CAPCTL_FCRLDEN2_Msk;

	/* Wait until PWM0 channel 2 Timer start to count */
	while((PWM0->CNT[2]) == 0);

	/* Capture the Input Waveform Data */
	CalPeriodTime(PWM0, 2);

	PWM_Out_DeInit();

	PWM_Cap_DeInit();
}

void PDMA_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS(PDMA);

    if(status & 0x1)    /* abort */
    {
        if(PDMA_GET_ABORT_STS(PDMA) & 0x1)
        {
			g_u32IsTestOver = 2;
        }
        PDMA_CLR_ABORT_FLAG(PDMA, PDMA_ABTSTS_ABTIF0_Msk);
    }
    else if(status & 0x2)      /* done */
    {
        if(PDMA_GET_TD_STS(PDMA) & 0x1)
        {
			g_u32IsTestOver = 1;
        }
        PDMA_CLR_TD_FLAG(PDMA, PDMA_TDSTS_TDIF0_Msk);
    }
    else
        printf("unknown interrupt !!\n");
}


void PDMA_Init(void)
{
    /* Open Channel 0 */
    PDMA_Open(PDMA, 0x1);

    /* Transfer width is half word(16 bit) and transfer count is 4 */
    PDMA_SetTransferCnt(PDMA, 0, PDMA_WIDTH_16, 4);

    /* Set source address as PWM capture channel PDMA register(no increment) and destination address as g_au16Count array(increment) */
    PDMA_SetTransferAddr(PDMA, 0, (uint32_t)&PWM0->PDMACAP2_3, PDMA_SAR_FIX, (uint32_t)&g_au16Count[0], PDMA_DAR_INC);

    /* Select PDMA request source as PWM RX(PWM0 channel 2 should be PWM0 pair 2) */
    PDMA_SetTransferMode(PDMA, 0, PDMA_PWM0_P2_RX, FALSE, 0);

    /* Set PDMA as single request type for PWM */
    PDMA_SetBurstType(PDMA, 0, PDMA_REQ_SINGLE, PDMA_BURST_4);

    PDMA_EnableInt(PDMA, 0, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA_IRQn);

    /* Enable PDMA for PWM0 channel 2 capture function, and set capture order as falling first, */
    /* And select capture mode as both rising and falling to do PDMA transfer. */
    PWM_EnablePDMA(PWM0, 2, FALSE, PWM_CAPTURE_PDMA_RISING_FALLING_LATCH);
}


void TMR3_IRQHandler(void)
{
//	static uint32_t LOG = 0;
	static uint16_t CNT = 0;
	
    if(TIMER_GetIntFlag(TIMER3) == 1)
    {
        TIMER_ClearIntFlag(TIMER3);
	
		if (CNT++ >= 1000)
		{		
			CNT = 0;
//        	printf("%s : %4d\r\n",__FUNCTION__,LOG++);
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
				duty += 10;
				if ( duty > 90)
				{
					duty = 90;
				}

				break;
			
			case '2':
				duty -= 10;
				if ( duty < 10)
				{
					duty = 10;
				}					
			
				break;	

			case '3':
				freq += 10;
				if ( freq > 10000)
				{
					freq = 10000;
				}
			
				break;
			
			case '4':
				freq -= 10;
				if ( freq < 250)
				{
					freq = 250;
				}
	
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

	/* Set UART receive time-out */
	UART_SetTimeoutCnt(UART0, 20);

	UART0->FIFO &= ~UART_FIFO_RFITL_4BYTES;
	UART0->FIFO |= UART_FIFO_RFITL_8BYTES;

	/* Enable UART Interrupt - */
	UART_ENABLE_INT(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_TOCNTEN_Msk | UART_INTEN_RXTOIEN_Msk);
	
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
//    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
	
    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
//    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);
	
    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART0 clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_PCLK0, CLK_CLKDIV0_UART0(1));
	
    CLK_EnableModuleClock(TMR3_MODULE);
    CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3SEL_PCLK1, 0);

    CLK_EnableModuleClock(PWM0_MODULE);
    CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL2_PWM0SEL_PCLK0, 0);

    CLK_EnableModuleClock(PDMA_MODULE);	
	
    /* Reset PWM0 module */
    SYS_ResetModule(PWM0_RST);

    /* Reset PDMA module */
    SYS_ResetModule(PDMA_RST);
	
    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk))    |       \
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB5MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk)) |
                    (SYS_GPB_MFPL_PB5MFP_PWM0_CH0 | SYS_GPB_MFPL_PB3MFP_PWM0_CH2);

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

	TIMER3_Init();

	/*
		capture : PWM0 channel 2(PB.3)
		output : PWM0 channel 0(PB.5)
	*/
	
    /* Got no where to go, just loop forever */
    while(1)
    {
	
		PWM_Out_Init();	

		PWM_Cap_Init();
    }
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
