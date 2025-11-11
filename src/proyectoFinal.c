#include <LPC17xx.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
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
float calcularRMS(void);
void configTIMER1(void);

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

volatile uint8_t onda=0;

uint8_t contador0=0;
uint8_t contador1=0;
volatile uint32_t frecuencia=FRECUENCIA_MAX_2048_MUESTRAS;
const char * const NOMBRES_ONDAS[SIGNALS] = {
    "SENO",
    "TRIANGULAR",
    "SIERRA"
};

int main(void){
	llenar_sin();
	llenar_triangular();
	llenar_sierra();
	configPCB();
	configUART();
	configDAC();
	GPDMA_Init();
	configDMA0();
	GPDMA_ChannelCmd(0,ENABLE);
	configADC();
	configTIMER0();
	configTIMER1();
	configEINT();
	ADC_StartCmd(LPC_ADC, ADC_START_NOW);

	while(1){}
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
    TIM_TIMERCFG_Type timer0_conf={0};
    TIM_MATCHCFG_Type timer0_match={0};

    timer0_conf.PrescaleOption = TIM_PRESCALE_USVAL;
    timer0_conf.PrescaleValue  = 1;

    timer0_match.MatchChannel        = 0;
    timer0_match.IntOnMatch          = ENABLE;
    timer0_match.ResetOnMatch        = ENABLE;
    timer0_match.StopOnMatch         = DISABLE;
    timer0_match.ExtMatchOutputType  = TIM_EXTMATCH_NOTHING;
    timer0_match.MatchValue          = 2000000-1;

    TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &timer0_conf);
    TIM_ConfigMatch(LPC_TIM0, &timer0_match);
    TIM_Cmd(LPC_TIM0, ENABLE);
	//NVIC_SetPriority(TIMER0_IRQn, (4));
    NVIC_EnableIRQ(TIMER0_IRQn);
}

void configTIMER1(void){
    TIM_TIMERCFG_Type timer_cfg={0};
    TIM_MATCHCFG_Type match_cfg={0};

    timer_cfg.PrescaleOption = TIM_PRESCALE_USVAL;
    timer_cfg.PrescaleValue  = 1;

    match_cfg.MatchChannel       = 0;
    match_cfg.IntOnMatch         = ENABLE;
    match_cfg.ResetOnMatch       = ENABLE;
    match_cfg.StopOnMatch        = ENABLE;
    match_cfg.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    match_cfg.MatchValue         = 100000 - 1;

    TIM_Init(LPC_TIM1, TIM_TIMER_MODE, &timer_cfg);
    TIM_ConfigMatch(LPC_TIM1, &match_cfg);

    NVIC_EnableIRQ(TIMER1_IRQn);
}

void configPCB(void){
	PINSEL_CFG_Type adc0={0};
	PINSEL_CFG_Type dac0={0};
	PINSEL_CFG_Type eint0={0};
	PINSEL_CFG_Type eint1={0};
	PINSEL_CFG_Type uart3_tx = {0};

	uart3_tx.Portnum = 0;
	uart3_tx.Pinnum = 0;
	uart3_tx.Funcnum = 2;
	uart3_tx.Pinmode = PINSEL_PINMODE_TRISTATE;
	uart3_tx.OpenDrain = 0;

	adc0.Portnum=0;
	adc0.Pinnum=23;
	adc0.Funcnum=1;
	adc0.Pinmode=PINSEL_PINMODE_TRISTATE;

	dac0.Portnum=0;
	dac0.Pinnum=26;
	dac0.Funcnum=2;
	dac0.Pinmode=0;

	eint0.Portnum=2;
	eint0.Pinnum=10;
	eint0.Funcnum=1;
	eint0.Pinmode=PINSEL_PINMODE_TRISTATE;

	eint1.Portnum=2;
	eint1.Pinnum=11;
	eint1.Funcnum=1;
	eint1.Pinmode=PINSEL_PINMODE_TRISTATE;

	PINSEL_ConfigPin(&adc0);
	PINSEL_ConfigPin(&dac0);
	PINSEL_ConfigPin(&eint0);
	PINSEL_ConfigPin(&eint1);
	PINSEL_ConfigPin(&uart3_tx);
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
	LPC_SC->PCONP |= (1 << 12);
	__NOP();
	LPC_SC->PCLKSEL0 &= ~(0x3 << 24); // Limpiar bits 24 y 25
	LPC_SC->PCLKSEL0 |= (0x1 << 24);  // PCLK_ADC = CCLK/1 (01b)
	ADC_Init(LPC_ADC, 200000);
	ADC_BurstCmd(LPC_ADC,DISABLE);
	ADC_ChannelCmd(LPC_ADC,0, ENABLE);
	ADC_IntConfig(LPC_ADC, ADC_ADINTEN0, ENABLE);
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

void EINT0_IRQHandler(void){
	NVIC_DisableIRQ(EINT0_IRQn);
	contador0=(contador0+1)%SIGNALS;
	GPDMA_ChannelCmd(0, DISABLE);

	switch (contador0){
		case 0: configDMA0(); DAC_SetDMATimeOut(LPC_DAC,(PCLK_DAC/(FRECUENCIA_MAX_2048_MUESTRAS*SIZE_SIN))-1); frecuencia=FRECUENCIA_MAX_2048_MUESTRAS;onda=0;break;
		case 1: configDMA1(); DAC_SetDMATimeOut(LPC_DAC,(PCLK_DAC/(FRECUENCIA_MAX_2048_MUESTRAS*SIZE_TRIANGULAR))-1);frecuencia=FRECUENCIA_MAX_2048_MUESTRAS; onda=1;break;
		case 2: configDMA2(); DAC_SetDMATimeOut(LPC_DAC,(PCLK_DAC/(FRECUENCIA_MAX_1024_MUESTRAS*SIZE_SIERRA))-1);frecuencia=FRECUENCIA_MAX_1024_MUESTRAS; onda= 2;break;
	}

	LPC_DAC->DACR = 0;

	GPDMA_ChannelCmd(0, ENABLE);    // reactiva el canal con la nueva configuración
	
	TIM_ResetCounter(LPC_TIM1);
	TIM_Cmd(LPC_TIM1, ENABLE);

	EXTI_ClearEXTIFlag(EXTI_EINT0);
}

void EINT1_IRQHandler(void){
	NVIC_DisableIRQ(EINT1_IRQn);
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

	TIM_ResetCounter(LPC_TIM1);
	TIM_Cmd(LPC_TIM1, ENABLE);

	EXTI_ClearEXTIFlag(EXTI_EINT1);
}

void configUART(void){
	UART_CFG_Type uart3={0};
	UART_FIFO_CFG_Type fifo={0};

	fifo.FIFO_ResetRxBuf=ENABLE;
	fifo.FIFO_ResetTxBuf=ENABLE;
	fifo.FIFO_DMAMode=DISABLE;
	fifo.FIFO_Level=UART_FIFO_TRGLEV3;

	UART_ConfigStructInit(&uart3);
    UART_Init(LPC_UART3, &uart3);
    UART_FIFOConfigStructInit(&fifo);

    UART_TxCmd(LPC_UART3, ENABLE);
}

void TIMER0_IRQHandler(void) {
	float valor_rms_volts = calcularRMS();
	const char *nombreOndaActual = NOMBRES_ONDAS[onda];
    char texto[64];
    int n = sprintf(texto, "RMS: %.2f V | FRECUENCIA: %lu Hz | ONDA: %s\r\n", valor_rms_volts, frecuencia, nombreOndaActual);
    UART_Send(LPC_UART3, (uint8_t*)texto, (uint32_t)n, BLOCKING);
	TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT);
}

void TIMER1_IRQHandler(void) {
    // Limpiamos el flag del Timer1
    TIM_ClearIntPending(LPC_TIM1, TIM_MR0_INT);

    // Volvemos a habilitar las interrupciones de los botones
    // Es seguro habilitarlas aunque ya lo estuvieran.
    NVIC_EnableIRQ(EINT0_IRQn);
    NVIC_EnableIRQ(EINT1_IRQn);

    // Opcional: limpiamos cualquier rebote que haya quedado pendiente
    EXTI_ClearEXTIFlag(EXTI_EINT0);
    EXTI_ClearEXTIFlag(EXTI_EINT1);
}

void ADC_IRQHandler(void){
	static uint16_t cont_ADC = 0;
	uint32_t lectura_adc;

	lectura_adc = LPC_ADC->ADDR0;

	buffer_adc[cont_ADC] = (uint16_t)((lectura_adc >> 4) & 0x0FFF);
	cont_ADC = (cont_ADC + 1) % BUFFER_SIZE;

	ADC_StartCmd(LPC_ADC, ADC_START_NOW);
}


float calcularRMS(){
	float suma = 0.0f;
	uint16_t local_buffer[BUFFER_SIZE]; // Buffer local para copia

	// --- Sección Crítica ---
	// Copiamos el buffer de ADC rápido para que la ISR no lo pise
	NVIC_DisableIRQ(ADC_IRQn); // Apagamos la ISR del ADC
	for (int i = 0; i < BUFFER_SIZE; i++) {
		local_buffer[i] = buffer_adc[i];
	}
	NVIC_EnableIRQ(ADC_IRQn); // Prendemos la ISR del ADC
	// --- Fin Sección Crítica ---

	for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
		// Usamos la copia local para el cálculo
		float val = (float)local_buffer[i];
		suma += val * val;
	}

	float rms_digital = sqrt(suma / BUFFER_SIZE);

	// Convertimos el RMS digital (0-4095) a Volts
	float volt_rms = (rms_digital * VREF) / 4095.0f;

	return volt_rms; // ¡Devolver el valor!
}

