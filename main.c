/**************************************************************************//**
 * @file     main.c
 * @brief
 *           Demonstrate how to implement a USB keyboard device.
 *           It supports to use GPIO to simulate key input.
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"
#include "hid_kb.h"

//---
//            aa,   us, rt, ch, bps, dbm, pw, nak
//  short cfg[]={1, 1000,  3,  5, 250, -12, 32,  0};
void nrf_init(short *cfg);
short nrf_input(char* p);
char nrf_output(char *p);
 

//--------------- Systick ------------------
uint32_t tick=0;
void SysTick_Handler(void){
    tick++;
};
 
void delay_ms(uint32_t ms){
    ms += tick;
    while( ms != tick){}
}
 
void init(void){
	  SystemCoreClockUpdate();
    SysTick_Config( SystemCoreClock /1000);
	  PB->PUEN |= BIT2;  //提升電阻打開
{//            aa,   us, rt, ch, bps, dbm, pw, nak
   short cfg[]={1, 1000,  3,  5, 250, -12, 32,  0};
	 nrf_init(cfg);
}
}

/*--------------------------------------------------------------------------*/
uint8_t volatile g_u8EP2Ready = 0;


/*--------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable external 12MHz HXT */
    CLK_EnableXtalRC(CLK_PWRCTL_HXT_EN_Msk);
    CLK_EnablePLL(CLK_PLLCTL_PLL_SRC_HXT, 96000000);
    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_HXT_STB_Msk | CLK_CLKSTATUS_PLL_STB_Msk);

    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_PLL, CLK_HCLK_CLK_DIVIDER(3));

    /* Select IP clock source */
    CLK_SetModuleClock(USBD_MODULE, 0, CLK_USB_CLK_DIVIDER(2));
    /* Enable IP clock */
    CLK_EnableModuleClock(USBD_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_UART_CLK_DIVIDER(1));
    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PA multi-function pins for UART0 RXD and TXD */
    SYS->PA_H_MFP &= ~( SYS_PA_H_MFP_PA15_MFP_Msk | SYS_PA_H_MFP_PA14_MFP_Msk);
    SYS->PA_H_MFP |= (SYS_PA_H_MFP_PA15_MFP_UART0_TX|SYS_PA_H_MFP_PA14_MFP_UART0_RX);

    /* Lock protected registers */
    SYS_LockReg();
}

void UART0_Init(void)
{
    /* Reset IP */
    SYS_ResetModule(UART0_RST);
    UART0->BAUD = 0x67;              /* Baud Rate:115200  OSC:12MHz */
    UART0->TLCTL = 0x03;             /* Character len is 8 bits */
}

void HID_Send(char c){
	uint8_t *buf;
	
	int32_t i;
  while(g_u8EP2Ready==0){}
	g_u8EP2Ready=0;
	buf = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2));
	for(i = 0; i < 8; i++){buf[i] = 0;}
	buf[2] = c; /* Key Spacebar 0x2C */
	USBD_SET_PAYLOAD_LEN(EP2, 8);
	
}
char sta=3;
void sw2(void){
  sta <<=1;   // sta = 100
	sta +=PB2;  
	sta &=3;
	if(sta==2){
	  HID_Send(0x2C); //spacebar 
		HID_Send(0x00);  // none
	}
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    SYS_Init();
    UART0_Init();

    printf("\n");
    printf("+--------------------------------------------------------+\n");
    printf("|          NuMicro USB HID Keyboard Sample Code          |\n");
    printf("+--------------------------------------------------------+\n");
    printf("If PB.15 = 0, just report it is key 'a'.\n");

    USBD_Open(&gsInfo, HID_ClassRequest, NULL);

    /* Endpoint configuration */
    HID_Init();
    NVIC_EnableIRQ(USBD_IRQn);
    USBD_Start();

    /* start to IN data */
    g_u8EP2Ready = 1;
    init();
    while(1)
    { 
			  delay_ms(1);
			  sw2();
        //HID_UpdateKbData();  //沒有在用的程式可砍
			//---RF Use
			{
				char rb[32]; //radio buffer
				short ln;
				ln = nrf_input(rb);
				if(ln){
 				   HID_Send(0x2C); //spacebar 
					 HID_Send(0x00);  // none
				}
				
			}
			//---
    }
}



/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/

