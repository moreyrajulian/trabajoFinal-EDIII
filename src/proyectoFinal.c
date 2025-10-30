#include <stdio.h>
#include "lpc17xx_gpio.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_dac.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_exti.h"

//medir valores rms de tension, hasta que frecuencia es valido el valor rms
//en funcion de la fmuestreo hasta que frecuencia es valido el true rms

#define CCLK_DAC 25000000

void configPCB(void);
void configADC(void);
void configDAC(void);
void configUART(void);
void configEINT(void);
void configTIMER0(void);
void configDMA(void);

volatile uint32_t valor_pote=0;
volatile uint16_t buffer_sin[]={0};
volatile uint16_t buffer_triangular[]={0};
volatile uint16_t buffer_sierra[]={0};
volatile uint16_t buffer_cuadrada[]={0};

int main(void){
	configPCB();
	configADC();
	configDAC();
	configUART();
	configEINT();
	configDMA();
	configTIMER0();

	while(1){
		__WFI();
	}
}

void configUART(void){

}

void configTIMER0(void){

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

void configDMA(void){
	GPDMA_Channel_CFG_Type dma0={0};

	dma0.ChannelNum=0;
	dma0.TransferSize=4095;
	dma0.TransferWidth=GPDMA_WIDTH_HALFWORD;
	dma0.SrcMemAddr=

	GPDMA_Init();
	GPDMA_Setup(&dma0);
}
















