#include <stdio.h>
#include "lpc17xx_gpio.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_dac.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_exti.h"

//medicion true RMS del adc, calcular medicion RMS
//medir señal analogica RMS, poder meter una t
//medir valores rms de tension, hasta que frecuencia es valido el valor rms
//en funcion de la fmuestreo hasta que frecuencia es valido el true rms

#define CCLK_DAC 25000000
#define TRANSFER_SIZE 4095
#define SEÑALES 4

void configPCB(void);
void configADC(void);
void configDAC(void);
void configUART(void);
void configEINT(void);
void configTIMER0(void);
void configDMA0(void);
void configDMA1(void);
void configDMA2(void);
void configDMA3(void);

volatile uint16_t buffer_sin[]={0};
volatile uint16_t buffer_triangular[]={0};
volatile uint16_t buffer_sierra[]={0};
volatile uint16_t buffer_cuadrada[]={0};

volatile GPDMA_Channel_CFG_Type dma0={0};
volatile GPDMA_LLI_Type lli0={0};
volatile GPDMA_Channel_CFG_Type dma1={0};
volatile GPDMA_LLI_Type lli1={0};
volatile GPDMA_Channel_CFG_Type dma2={0};
volatile GPDMA_LLI_Type lli2={0};
volatile GPDMA_Channel_CFG_Type dma3={0};
volatile GPDMA_LLI_Type lli3={0};

int main(void){
	configPCB();
	configADC();
	configDAC();
	configUART();
	configEINT();
	GPDMA_Init();
	GPDMA_ChannelCmd(0,ENABLE);
	configTIMER0();

	while(1){
		__WFI();
	}
}

void configUART(void){

}

void configPCB(void){
	PINSEL_CFG_Type adc0={0};
	PINSEL_CFG_Type dac0={0};
	PINSEL_CFG_Type eint0={0};
	PINSEL_CFG_Type eint1={0};

	adc0.Portnum=0;
	adc0.Pinnum=23;
	adc0.Funcnum=1;
	adc0.Pinmode=PINSEL_PINMODE_TRISTATE;

	dac0.Portnum=0;
	dac0.Pinnum=18;
	dac0.Funcnum=1;
	dac0.Pinmode=PINSEL_PINMODE_TRISTATE;

	eint0.Portnum=2;
	eint0.Pinnum=10;
	eint0.Funcnum=1;
	eint0.Pinmode=PINSEL_PINMODE_PULLUP;

	eint1.Portnum=2;
	eint1.Pinnum=11;
	eint1.Funcnum=1;
	eint1.Pinmode=PINSEL_PINMODE_PULLUP;

	PINSEL_ConfigPin(&adc0);
	PINSEL_ConfigPin(&dac0);
	PINSEL_ConfigPin(&eint0);
	PINSEL_ConfigPin(&eint1);
}

void configEINT(void){
	EXTI_InitTypeDef eint0={0};
	EXTI_InitTypeDef eint1={0};

	eint0.EXTI_Line=0;
	eint0.EXTI_Mode=EXTI_MODE_EDGE_SENSITIVE;
	eint0.EXTI_polarity=EXTI_POLARITY_LOW_ACTIVE_OR_FALLING_EDGE;

	eint0.EXTI_Line=1;
	eint0.EXTI_Mode=EXTI_MODE_EDGE_SENSITIVE;
	eint0.EXTI_polarity=EXTI_POLARITY_LOW_ACTIVE_OR_FALLING_EDGE;

	EXTI_Init();
	EXTI_Config(&eint0);
	EXTI_Config(&eint1);
	NVIC_EnableIRQ(EINT0_IRQn);
	NVIC_EnableIRQ(EINT1_IRQn);
}

void configADC(void){
	ADC_Init(LPC_ADC,200000);
	ADC_BurstCmd(LPC_ADC,DISABLE);
	ADC_StartCmd(LPC_ADC,ADC_START_ON_EINT0);
	ADC_ChannelCmd(LPC_ADC,0,ENABLE);
	ADC_EdgeStartConfig(LPC_ADC,ADC_START_ON_FALLING);
	ADC_IntConfig(LPC_ADC,ADC_ADINTEN0,ENABLE);
	NVIC_EnableIRQ(ADC_IRQn);
}

void configDAC(void){
	DAC_CONVERTER_CFG_Type dac={0};

	dac.DBLBUF_ENA=1;
	dac.CNT_ENA=1;
	dac.DMA_ENA=1;

	DAC_Init(LPC_DAC);
	DAC_SetBias(LPC_DAC,0);
	DAC_ConfigDAConverterControl(LPC_DAC,&dac);
	DAC_SetDMATimeOut(LPC_DAC,100); //ver TIME_OUT
}

void configDMA0(void){
	dma0.ChannelNum=0;
	dma0.TransferSize=TRANSFER_SIZE;
	dma0.TransferWidth=GPDMA_WIDTH_HALFWORD;
	dma0.SrcMemAddr=(uint8_t*)buffer_sin;
	dma0.DstMemAddr=0;
	dma0.TransferType=GPDMA_TRANSFERTYPE_M2P;
	dma0.SrcConn=0;
	dma0.DstConn=GPDMA_CONN_DAC;
	dma0.DMALLI=(uint8_t*)lli0;

	lli0.SrcAddr=(uint8_t*)buffer_sin;
	lli0.DstAddr=0;
	lli0.NextLLI=(uint8_t*)lli0;
	lli0.Control=TRANSFER_SIZE|(1<<18)|(1<<21)|(1<<24)|(1<<31);
}

void configDMA1(void){
	dma1.ChannelNum=0;
	dma1.TransferSize=4095;
	dma1.TransferWidth=GPDMA_WIDTH_HALFWORD;
	dma1.SrcMemAddr=(uint8_t*)buffer_cuadrada;
	dma1.DstMemAddr=0;
	dma1.TransferType=GPDMA_TRANSFERTYPE_M2P;
	dma1.SrcConn=0;
	dma1.DstConn=GPDMA_CONN_DAC;
	dma1.DMALLI=(uint8_t*)lli1;

	lli1.SrcAddr=(uint8_t*)buffer_cuadrada;
	lli1.DstAddr=0;
	lli1.NextLLI=(uint8_t*)lli1;
	lli1.Control=TRANSFER_SIZE|(1<<18)|(1<<21)|(1<<24)|(1<<31);
}

void configDMA2(void){
	dma2.ChannelNum=0;
	dma2.TransferSize=4095;
	dma2.TransferWidth=GPDMA_WIDTH_HALFWORD;
	dma2.SrcMemAddr=(uint8_t*)buffer_triangular;
	dma2.DstMemAddr=0;
	dma2.TransferType=GPDMA_TRANSFERTYPE_M2P;
	dma2.SrcConn=0;
	dma2.DstConn=GPDMA_CONN_DAC;
	dma2.DMALLI=(uint8_t*)lli2;

	lli2.SrcAddr=(uint8_t*)buffer_triangular;
	lli2.DstAddr=0;
	lli2.NextLLI=(uint8_t*)lli2;
	lli2.Control=TRANSFER_SIZE|(1<<18)|(1<<21)|(1<<24)|(1<<31);
}

void configDMA3(void){
	dma3.ChannelNum=0;
	dma3.TransferSize=4095;
	dma3.TransferWidth=GPDMA_WIDTH_HALFWORD;
	dma3.SrcMemAddr=(uint8_t*)buffer_sierra;
	dma3.DstMemAddr=0;
	dma3.TransferType=GPDMA_TRANSFERTYPE_M2P;
	dma3.SrcConn=0;
	dma3.DstConn=GPDMA_CONN_DAC;
	dma3.DMALLI=(uint8_t*)lli3;

	lli3.SrcAddr=(uint8_t*)buffer_sierra;
	lli3.DstAddr=0;
	lli3.NextLLI=(uint8_t*)lli3;
	lli3.Control=TRANSFER_SIZE|(1<<18)|(1<<21)|(1<<24)|(1<<31);
}

void EINT1_IRQHandler(void){
	static uint8_t contador=0;
	contador=(contador+1)%SEÑALES;

	switch (contador){
	case 1: GPDMA_Setup(&dma0); break;
	case 2: GPDMA_Setup(&dma1); break;
	case 3: GPDMA_Setup(&dma2); break;
	case 4: GPDMA_Setup(&dma3); break;
	default: GPDMA_Setup(&dma0); break;
	}

	EXTI_ClearEXTIFlag(EXTI_EINT1);
}














