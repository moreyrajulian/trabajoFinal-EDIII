#include <LPC17xx.h>
#include <stdio.h>
#include <math.h>
#include "lpc17xx_gpio.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_dac.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_exti.h"

#define VREF 3.3f

#define BUFFER_SIZE 1024

#define PCLK_DAC 25000000
#define MUESTRAS_DAC 1024
#define SIGNALS 3
#define FRECUENCIAS 4

#define FRECUENCIA_0 100
#define FRECUENCIA_1 250
#define FRECUENCIA_2 350
#define FRECUENCIA_3 50

#define SIZE_SIERRA 1024
#define SIZE_TRIANGULAR 2048
#define SIZE_SIN 2048

#define PI 3.141592653589793

#define DAC_RESOLUTION 1024

#define FRECUENCIA_MAX_2048_MUESTRAS 480
#define FRECUENCIA_MAX_1024_MUESTRAS 950

void configPCB(void);
void configADC(void);
void configDAC(void);
void configUART(void);
void configEINT(void);
void configTIMER0(void);
void configDMA0(void);
void configDMA1(void);
void configDMA2(void);
void llenar_sin(void);
void llenar_triangular(void);
void llenar_sierra(void);
float calcularRMS();
void enviarUART(char *cadena);

//buffer con valores convertidos por el ADC
volatile uint16_t buffer_adc[BUFFER_SIZE];

//buffers para generar las 4 señales mediante software
uint16_t buffer_sin[SIZE_SIN];
uint16_t buffer_triangular[SIZE_TRIANGULAR];
uint16_t buffer_sierra[SIZE_SIERRA];

GPDMA_Channel_CFG_Type dma0;
GPDMA_Channel_CFG_Type dma1;
GPDMA_Channel_CFG_Type dma2;

static GPDMA_LLI_Type lli0;
static GPDMA_LLI_Type lli1;
static GPDMA_LLI_Type lli2;

volatile uint32_t RMS=0;
volatile uint8_t onda=0;

volatile uint8_t flagCambiarOnda = 0;
volatile uint8_t flagCambiarFrec = 0;
volatile uint8_t disparar = 1;
volatile uint16_t cont_ADC = 0;

uint8_t contador0=0;
uint8_t contador1=0;
uint32_t frecuencia=0;

int main(void){

	llenar_sin();
	llenar_triangular();
	llenar_sierra();
	configPCB();
	configADC();
	configDAC();
	configUART();
	configEINT();
	GPDMA_Init();
	configDMA0();
	GPDMA_ChannelCmd(0,ENABLE);
	configTIMER0();
	enviarUART("Hola desde LPC1769, estamos con el José, debuggeando la placa y no anda XD");


	while(1){

		if(disparar){
			ADC_StartCmd(LPC_ADC, ADC_START_NOW);
			disparar = 0;
		}

	}
}


void llenar_triangular(){
	for(int i = 0; i<1024; i++){
		buffer_triangular[i]=(i << 6);
	}
	for (int i = 1024; i < 2048; i++) {
		buffer_triangular[i]=((2047-i)<< 6);
	}
}

void llenar_sierra(){
	for(int i = 0; i<1024; i++){
		buffer_sierra[i]=i<<6;
	}
}

void llenar_sin(){
    double amplitude = (DAC_RESOLUTION - 1) / 2.0;  // 511.5
    double offset = amplitude;                      // 511.5
    double step = 2.0 * PI / (double)SIZE_SIN;

    for (uint16_t i = 0; i < SIZE_SIN; i++) {
        double valor = offset + amplitude * sin(i * step);
        buffer_sin[i] = (uint16_t)(valor + 0.5)<<6;    // redondeo
    }
}

void configTIMER0(void) {
    TIM_TIMERCFG_Type timer0_conf;
    TIM_MATCHCFG_Type timer0_match;

    //TIM_TIMERCFG_Type timer1_conf;
    //TIM_MATCHCFG_Type timer1_match;

    // --- Configuración base ---
    timer0_conf.PrescaleOption = TIM_PRESCALE_TICKVAL;
    timer0_conf.PrescaleValue  = 1;  // Incrementa cada 1 µs

    // --- Configuración del match ---
    timer0_match.MatchChannel        = 0;
    timer0_match.IntOnMatch          = ENABLE;
    timer0_match.ResetOnMatch        = ENABLE;
    timer0_match.StopOnMatch         = DISABLE;
    timer0_match.ExtMatchOutputType  = TIM_EXTMATCH_NOTHING;
    timer0_match.MatchValue          = 5000000;  // 100 mili segundo
/*
    // --- Configuración base ---
	timer1_conf.PrescaleOption = TIM_PRESCALE_TICKVAL;
	timer1_conf.PrescaleValue  = 1;  // Incrementa cada 1 µs

	// --- Configuración del match ---
	timer1_match.MatchChannel        = 0;
	timer1_match.IntOnMatch          = ENABLE;
	timer1_match.ResetOnMatch        = ENABLE;
	timer1_match.StopOnMatch         = DISABLE;
	timer1_match.ExtMatchOutputType  = TIM_EXTMATCH_TOGGLE;
	timer1_match.MatchValue          = 2500000;  // 100 mili segundo
*/
    // --- Inicializar y configurar timer ---
    TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &timer0_conf);
    //TIM_Init(LPC_TIM1, TIM_TIMER_MODE, &timer1_conf);
    TIM_ConfigMatch(LPC_TIM0, &timer0_match);
    //TIM_ConfigMatch(LPC_TIM1, &timer1_match);
    TIM_Cmd(LPC_TIM0, ENABLE);
    //TIM_Cmd(LPC_TIM1, ENABLE);
    // --- Habilitar interrupción ---
	//NVIC_SetPriority(TIMER0_IRQn, (4));

    NVIC_EnableIRQ(TIMER0_IRQn);



    // --- Iniciar timer ---

}

void configPCB(void){
	PINSEL_CFG_Type adc0={0};
	PINSEL_CFG_Type dac0={0};
	PINSEL_CFG_Type eint0={0};
	PINSEL_CFG_Type eint1={0};
	PINSEL_CFG_Type uart0_tx = {0};

	uart0_tx.Portnum = 0;
	uart0_tx.Pinnum = 2;
	uart0_tx.Funcnum = 1;

	adc0.Portnum=1;
	adc0.Pinnum=31;
	adc0.Funcnum=3;
	adc0.Pinmode=PINSEL_PINMODE_TRISTATE;

	dac0.Portnum=0;
	dac0.Pinnum=26;
	dac0.Funcnum=2;
	dac0.Pinmode=0;

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
	PINSEL_ConfigPin(&uart0_tx);


}

void configEINT(void){
	EXTI_InitTypeDef eint0={0};
	EXTI_InitTypeDef eint1={0};

	eint0.EXTI_Line=0;
	eint0.EXTI_Mode=EXTI_MODE_EDGE_SENSITIVE;
	eint0.EXTI_polarity=EXTI_POLARITY_LOW_ACTIVE_OR_FALLING_EDGE;

	eint1.EXTI_Line=1;
	eint1.EXTI_Mode=EXTI_MODE_EDGE_SENSITIVE;
	eint1.EXTI_polarity=EXTI_POLARITY_LOW_ACTIVE_OR_FALLING_EDGE;

	EXTI_Init();
	EXTI_Config(&eint0);
	EXTI_Config(&eint1);
	EXTI_ClearEXTIFlag(EXTI_EINT0);
	EXTI_ClearEXTIFlag(EXTI_EINT1);

	NVIC_SetPriority(EINT0_IRQn, 0);
	NVIC_SetPriority(EINT1_IRQn, 0);

	NVIC_EnableIRQ(EINT0_IRQn);
	NVIC_EnableIRQ(EINT1_IRQn);

}

void configADC(void){

	 	ADC_Init(LPC_ADC, 200000); // 200 kHz de frecuencia ADC

	    // Habilitar canal 0
	 	ADC_BurstCmd(LPC_ADC,DISABLE);
	    ADC_ChannelCmd(LPC_ADC,5, ENABLE);

	    // --- Configurar inicio por trigger de TIMER1 MATCH0 ---

	    // --- Configurar flanco de disparo ---
	    //ADC_EdgeStartConfig(LPC_ADC, ADC_START_ON_RISING);

	    // --- Habilitar interrupción del ADC ---
	    ADC_IntConfig(LPC_ADC, ADC_ADINTEN5, ENABLE);

	    // --- Configurar NVIC ---
	    NVIC_SetPriority(ADC_IRQn, 2);
	    NVIC_EnableIRQ(ADC_IRQn);
}

void configDAC(void) {
    DAC_CONVERTER_CFG_Type dac = {0};

    dac.CNT_ENA = 1;
    dac.DMA_ENA = 1;

    DAC_Init(LPC_DAC);
    DAC_SetDMATimeOut(LPC_DAC, 24);
    DAC_ConfigDAConverterControl(LPC_DAC, &dac);
}

void configDMA0(void){
	lli0.SrcAddr=(uint32_t)buffer_sin;
	lli0.DstAddr=(uint32_t)&(LPC_DAC->DACR);
	lli0.NextLLI=(uint32_t)&lli0;
	lli0.Control=SIZE_SIN|(1<<18)|(2<<21)|(1<<26);

	dma0.ChannelNum=0;
	dma0.TransferSize=SIZE_SIN;
	dma0.TransferWidth=0;
	dma0.SrcMemAddr=(uint32_t)buffer_sin;
	dma0.DstMemAddr=0;
	dma0.TransferType=GPDMA_TRANSFERTYPE_M2P;
	dma0.SrcConn=0;
	dma0.DstConn=GPDMA_CONN_DAC;
	dma0.DMALLI=(uint32_t)&lli0;

	GPDMA_Setup(&dma0);
}

void configDMA1(void){
	lli1.SrcAddr=(uint32_t)buffer_triangular;
	lli1.DstAddr=(uint32_t)&(LPC_DAC->DACR);
	lli1.NextLLI=(uint32_t)&lli1;
	lli1.Control=SIZE_TRIANGULAR|(1<<18)|(2<<21)|(1<<26);

	dma1.ChannelNum=0;
	dma1.SrcMemAddr=(uint32_t)buffer_triangular;
	dma1.DstMemAddr=0;
	dma1.TransferSize=SIZE_TRIANGULAR;
	dma1.TransferWidth=0;
	dma1.TransferType=GPDMA_TRANSFERTYPE_M2P;
	dma1.SrcConn=0;
	dma1.DstConn=GPDMA_CONN_DAC;
	dma1.DMALLI=(uint32_t)&lli1;

	GPDMA_Setup(&dma1);
}

void configDMA2(void){
	lli2.SrcAddr=(uint32_t)buffer_sierra;
	lli2.DstAddr=(uint32_t)&(LPC_DAC->DACR);
	lli2.NextLLI=(uint32_t)&lli2;
	lli2.Control=SIZE_SIERRA|(1<<18)|(2<<21)|(1<<26);

	dma2.ChannelNum=0;
	dma2.TransferSize=SIZE_SIERRA;
	dma2.TransferWidth=0;
	dma2.SrcMemAddr=(uint32_t)buffer_sierra;
	dma2.DstMemAddr=0;
	dma2.TransferType=GPDMA_TRANSFERTYPE_M2P;
	dma2.SrcConn=0;
	dma2.DstConn=GPDMA_CONN_DAC;
	dma2.DMALLI=(uint32_t)&lli2;

	GPDMA_Setup(&dma2);
}

void configUART(){
	LPC_UART0->LCR = (3 << 0)|(0 << 2)|(0 << 3)|(1 << 7);

	LPC_UART0->DLM = 0x00;
	LPC_UART0->DLL = 0xA2;  // 162 decimal
	LPC_UART0->FDR = (1 << 4) | 0; // MULVAL=1, DIVADDVAL=0

	LPC_UART0->LCR &= ~(1 << 7); // DLAB=0

	LPC_UART0->FCR = 0x07;  // FIFO habilitado y reseteado
	LPC_UART0->TER = (1 << 7); // Habilitar TX
}

void UART0_SendByte(char c) {
    while (!(LPC_UART0->LSR & (1 << 5)));  // Esperar THR vacío
    LPC_UART0->THR = c;
}

void enviarUART(char *cadena) {
    uint32_t i = 0;
    while (cadena[i] != '\0') {
        UART0_SendByte(cadena[i]);
        i++;
    }
    UART0_SendByte('\r');
    UART0_SendByte('\r');
}

void EINT0_IRQHandler(void){
	contador0=(contador0+1)%SIGNALS;
	GPDMA_ChannelCmd(0, DISABLE);

	switch (contador0){
		case 0: configDMA0(); DAC_SetDMATimeOut(LPC_DAC,(PCLK_DAC/(FRECUENCIA_MAX_2048_MUESTRAS*SIZE_SIN))-1); onda=0; break;
		case 1: configDMA1(); DAC_SetDMATimeOut(LPC_DAC,(PCLK_DAC/(FRECUENCIA_MAX_2048_MUESTRAS*SIZE_TRIANGULAR))-1); onda=1; break;
		case 2: configDMA2(); DAC_SetDMATimeOut(LPC_DAC,(PCLK_DAC/(FRECUENCIA_MAX_1024_MUESTRAS*SIZE_SIERRA))-1); onda= 2; break;
	}

	flagCambiarOnda = 0;

	LPC_DAC->DACR = 0;

	GPDMA_ChannelCmd(0, ENABLE);    // reactiva el canal con la nueva configuración

	EXTI_ClearEXTIFlag(EXTI_EINT0);
}

void EINT1_IRQHandler(void){
	contador1=(contador1+1)%FRECUENCIAS;

	switch (contador1){
		case 0: frecuencia= FRECUENCIA_0; break;
		case 1: frecuencia= FRECUENCIA_1; break;
		case 2: frecuencia= FRECUENCIA_2; break;
		case 3: frecuencia= FRECUENCIA_3; break;
	}

	switch (onda){
		case 0: DAC_SetDMATimeOut(LPC_DAC,(PCLK_DAC/(frecuencia*SIZE_SIN))-1); break;
		case 1: DAC_SetDMATimeOut(LPC_DAC,(PCLK_DAC/(frecuencia*SIZE_TRIANGULAR))-1); break;
		case 2: DAC_SetDMATimeOut(LPC_DAC,(PCLK_DAC/(frecuencia*SIZE_SIERRA))-1); break;
	}


	EXTI_ClearEXTIFlag(EXTI_EINT1);
}


void TIMER0_IRQHandler(void) {

	if (TIM_GetIntStatus(LPC_TIM0, TIM_MR0_INT)) {
        //float valor = calcularRMS();
        //char texto[32];
        //sprintf(texto, "RMS: %.2f\r\n", valor);
        enviarUART((char *)buffer_adc);
        TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT);
    }

    //printf("Holis, interrumpio el timer");

}

void ADC_IRQHandler(void){
		//printf("entre al handler ADC\n");

		cont_ADC=(cont_ADC+1)%BUFFER_SIZE;
		//buffer_adc[cont_ADC] = ADC_ChannelGetData(LPC_ADC, 5);
		buffer_adc[cont_ADC] =cont_ADC;
		//printf("%d | ",buffer_adc[cont_ADC]);
		disparar = 1;
		NVIC_ClearPendingIRQ(ADC_IRQn);
}


float calcularRMS(){
    float suma = 0.0f;

    for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
        float val = (float)buffer_adc[i];  // Ya es el valor ADC de 0 a 4095
        suma += val * val;
    }

    float rms = sqrt(suma / BUFFER_SIZE);

    float volt_rms = (rms * VREF) / 4095.0f;

    return volt_rms;
}

