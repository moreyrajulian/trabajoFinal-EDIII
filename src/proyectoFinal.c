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

//medicion true RMS del adc, calcular medicion RMS
//medir señal analogica RMS, poder meter una t
//medir valores rms de tension, hasta que frecuencia es valido el valor rms
//en funcion de la fmuestreo hasta que frecuencia es valido el true rms

//Queiro leer señal de entrada por adc. Guardar en memoria
// Debo verificar quq ela velocidad de peticion al dma del adc sea más
// rapida que la del dac al dma. Ya que el dac tiene prioridad por ser canal 0

#define BUFFER_SIZE 4096 //corroborar
#define PCLK_DAC 25000000
#define MUESTRAS_DAC 1024
#define TRANSFER_SIZE_ADC 4095
#define SIGNALS 3
#define FRECUENCIAS 4
#define FRECUENCIA_0 1000
#define FRECUENCIA_1 1500
#define FRECUENCIA_2 2500
#define FRECUENCIA_3 5000
#define FRECUENCIA_4 10000
#define MAX_LENGTH 256
#define SIZE_SIERRA 1024
#define SIZE_TRIANGULAR 2048
#define PI 3.141592653589793
#define DAC_RESOLUTION 1024
#define SIZE_SIN 2048

void configPCB(void);
void configADC(void);
void configDAC(void);
void configUART(void);
void configEINT(void);
void configTIMER0(void);
void configDMA0(void);
void configDMA1(void);
void configDMA2(void);
void configDMA_ADC(void);
void llenar_sin(void);
void llenar_triangular(void);
void llenar_sierra(void);
float calcularRMS(uint16_t buffer[BUFFER_SIZE]);
void enviarUART(char *cadena);

//buffer con valores convertidos por el ADC
uint16_t buffer_adc[BUFFER_SIZE];

//buffers para generar las 4 señales mediante software
uint32_t buffer_sin[SIZE_SIN];
uint32_t buffer_triangular[SIZE_TRIANGULAR];
uint32_t buffer_sierra[SIZE_SIERRA];

GPDMA_Channel_CFG_Type dma0;
GPDMA_Channel_CFG_Type dma1;
GPDMA_Channel_CFG_Type dma2;
GPDMA_Channel_CFG_Type dma_adc;

GPDMA_LLI_Type lli0;
GPDMA_LLI_Type lli1;
GPDMA_LLI_Type lli2;
GPDMA_LLI_Type lli2;
GPDMA_LLI_Type lli_adc;

volatile uint32_t RMS=0;
volatile uint8_t onda=0;

int main(void){
	llenar_sin();
	llenar_triangular();
	llenar_sierra();
	configPCB();
	//configADC();
	configDAC();
	//configUART();
	configEINT();
	//configDMA_ADC();
	GPDMA_Init();
	configDMA0();
	GPDMA_ChannelCmd(0,ENABLE);
	//GPDMA_ChannelCmd(1,ENABLE);
	//configTIMER0();

	while(1){
		//__WFI();
	}
}

void llenar_triangular(){
	for(int i = 0; i<1024; i++){
		buffer_triangular[i]=(i << 6);
	}
	for (int i = 1024; i < 2048; i++) {
		buffer_triangular[i]=((2048-i)<< 6);
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
    TIM_TIMERCFG_Type struct_config;
    TIM_MATCHCFG_Type struct_match;

    // --- Configuración base ---
    struct_config.PrescaleOption = TIM_PRESCALE_TICKVAL;
    struct_config.PrescaleValue  = 1;  // Incrementa cada 1 µs

    // --- Configuración del match ---
    struct_match.MatchChannel        = 0;
    struct_match.IntOnMatch          = ENABLE;
    struct_match.ResetOnMatch        = ENABLE;
    struct_match.StopOnMatch         = DISABLE;
    struct_match.ExtMatchOutputType  = TIM_EXTMATCH_NOTHING;
    struct_match.MatchValue          = 1000000;  // 1 segundo

    // --- Inicializar y configurar timer ---
    TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &struct_config);
    TIM_ConfigMatch(LPC_TIM0, &struct_match);

    // --- Habilitar interrupción ---
    NVIC_EnableIRQ(TIMER0_IRQn);

    // --- Iniciar timer ---
    TIM_Cmd(LPC_TIM0, ENABLE);
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

	adc0.Portnum=0;
	adc0.Pinnum=23;
	adc0.Funcnum=1;
	adc0.Pinmode=0;

	dac0.Portnum=0;
	dac0.OpenDrain = 0;
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
	NVIC_EnableIRQ(EINT0_IRQn);
	NVIC_EnableIRQ(EINT1_IRQn);
}

void configADC(void){
	ADC_Init(LPC_ADC,200000);
	ADC_BurstCmd(LPC_ADC,ENABLE);
	ADC_ChannelCmd(LPC_ADC,0,ENABLE);
}

void configDAC(void) {
    DAC_CONVERTER_CFG_Type dac = {0};

    dac.CNT_ENA = 1;
    dac.DMA_ENA = 1;

    DAC_Init(LPC_DAC);
    DAC_SetDMATimeOut(LPC_DAC, 12);
    DAC_ConfigDAConverterControl(LPC_DAC, &dac);
    return;
    //DAC_SetDMATimeOut(LPC_DAC, (PCLK_DAC / (FRECUENCIA_0 * MUESTRAS_DAC)) - 1);

}

void configDMA_ADC(void){
	//GPDMA_LLI_Type lli_adc;

	lli_adc.SrcAddr=(uint32_t)&(LPC_ADC -> ADDR0);
	lli_adc.DstAddr=(uint32_t)&buffer_adc;
	lli_adc.NextLLI=(uint32_t)&lli_adc;
	lli_adc.Control=TRANSFER_SIZE_ADC|(1<<18)|(1<<21)|(1<<24)|(1<<31);
	//GPDMA_Init();

	dma_adc.ChannelNum=1;
	dma_adc.TransferSize=TRANSFER_SIZE_ADC;
	dma_adc.TransferWidth=0;
	dma_adc.SrcMemAddr=0;
	dma_adc.DstMemAddr=(uint32_t)&buffer_adc;
	dma_adc.TransferType=GPDMA_TRANSFERTYPE_P2M;
	dma_adc.SrcConn=GPDMA_CONN_ADC;
	dma_adc.DstConn=0;
	dma_adc.DMALLI=(uint32_t)&lli_adc;

	GPDMA_Setup(&dma_adc);
	NVIC_EnableIRQ(DMA_IRQn);
}

void configDMA0(void){
	//GPDMA_LLI_Type lli0;

	lli0.SrcAddr=(uint32_t)&buffer_sin;
	lli0.DstAddr=(uint32_t)&(LPC_DAC->DACR);
	lli0.NextLLI=(uint32_t)&lli0;
	lli0.Control=SIZE_SIN|(2<<18)|(2<<21)|(1<<26);
	//GPDMA_Init();

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
	return;
}

void configDMA1(void){
	//GPDMA_LLI_Type lli1;

	lli1.SrcAddr=(uint32_t)buffer_triangular;
	lli1.DstAddr=(uint32_t)&(LPC_DAC->DACR);
	lli1.NextLLI=(uint32_t)&lli1;
	lli1.Control=SIZE_TRIANGULAR|(2<<18)|(2<<21)|(1<<26);
	//GPDMA_Init();

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
	return;
}

void configDMA2(void){
	//GPDMA_LLI_Type lli2;

	lli2.SrcAddr=(uint32_t)&buffer_sierra;
	lli2.DstAddr=(uint32_t)&(LPC_DAC->DACR);
	lli2.NextLLI=(uint32_t)&lli2;
	lli2.Control=SIZE_SIERRA|(2<<18)|(2<<21)|(1<<26);
	//GPDMA_Init();

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
	return;
}

void configUART(){
	LPC_UART0->LCR = (3 << 0) | (0 << 2) | (0 << 3) | (1 << 6) | (1 << 7);

	LPC_UART0->DLM = 0x00;
	LPC_UART0->DLL = 0xA2;  // 162 decimal
	LPC_UART0->FDR = (1 << 4) | 0; // MULVAL=1, DIVADDVAL=0

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
    UART0_SendByte('\n');
    UART0_SendByte('\n');
}

void EINT0_IRQHandler(void){
	static uint8_t contador=0;
	contador=(contador+1)%SIGNALS;
	GPDMA_ChannelCmd(0, DISABLE);

	switch (contador){
		case 1: configDMA0(); DAC_SetDMATimeOut(LPC_DAC,(PCLK_DAC/(FRECUENCIA_0*SIZE_SIN))-1); onda=0; break;
		case 2: configDMA1(); DAC_SetDMATimeOut(LPC_DAC,(PCLK_DAC/(FRECUENCIA_0*SIZE_TRIANGULAR))-1); onda=1; break;
		case 3: configDMA2(); DAC_SetDMATimeOut(LPC_DAC,(PCLK_DAC/(FRECUENCIA_0*SIZE_SIERRA))-1); onda= 2; break;
		default: configDMA0(); DAC_SetDMATimeOut(LPC_DAC,(PCLK_DAC/(FRECUENCIA_0*SIZE_SIN))-1); onda=0; break;
	}

	GPDMA_ChannelCmd(0, ENABLE);    // reactiva el canal con la nueva configuración

	EXTI_ClearEXTIFlag(EXTI_EINT0);
}

void EINT1_IRQHandler(void){
	static uint8_t contador=0;
	static uint32_t frecuencia=0;

	contador=(contador+1)%FRECUENCIAS;

	switch (contador){
		case 1: frecuencia= 1500; break;
		case 2: frecuencia= 2500; break;
		case 3: frecuencia= 5000; break;
		case 4: frecuencia= 10000; break;
		default: frecuencia= 1000; break;
	}

	switch (onda){
		case 0: DAC_SetDMATimeOut(LPC_DAC,(PCLK_DAC/(frecuencia*SIZE_SIN))-1); break;
		case 1: DAC_SetDMATimeOut(LPC_DAC,(PCLK_DAC/(frecuencia*SIZE_TRIANGULAR))-1); break;
		case 2: DAC_SetDMATimeOut(LPC_DAC,(PCLK_DAC/(frecuencia*SIZE_SIERRA))-1); break;
		default: DAC_SetDMATimeOut(LPC_DAC,(PCLK_DAC/(frecuencia*SIZE_SIN))-1); break;
	}

	EXTI_ClearEXTIFlag(EXTI_EINT1);
}

void DMA_IRQHandler(void) {
    if (GPDMA_IntGetStatus(GPDMA_STAT_INTTC, 1)) {
        float valor = calcularRMS(buffer_adc);
        char texto[32];
        sprintf(texto, "RMS: %.2f\r\n", valor);
        enviarUART(texto);

        GPDMA_ClearIntPending(GPDMA_STATCLR_INTTC, 1);
    }
}

void TIMER0_IRQHandler(void) {
    if (TIM_GetIntStatus(LPC_TIM0, TIM_MR0_INT)) {
        TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT);  // limpiar bandera

    }
}


float calcularRMS(uint16_t buffer[BUFFER_SIZE]) {
    float suma = 0.0f;
    for (uint32_t i = 0; i < BUFFER_SIZE; i++) {
        float val = (float)buffer[i];
        suma += val * val;
    }

    float promedio = suma / BUFFER_SIZE;
    float rms = sqrt(promedio);
    return rms;
}




