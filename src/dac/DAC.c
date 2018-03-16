/*
 * DAC.c
 *
 * Created: 29.10.2015 18:54:57
 *  Author: Anton
 */ 

#include "DAC.h"

//включение и инициализация ЦАП
void DAC_init(uint16_t data)
{
	uint16_t cal;
	sysclk_enable_module(SYSCLK_PORT_B, SYSCLK_DAC);
	DACB.CTRLA = DAC_ENABLE_bm | DAC_CH0EN_bm;
	DACB.CTRLB = DAC_CHSEL_SINGLE_gc;
	DACB.CTRLC = DAC_REFSEL_AVCC_gc;
	
	//DACB.TIMCTRL = DAC_CONINTVAL_128CLK_gc | DAC_REFRESH_8192CLK_gc;
	DACB.CH0DATA = data;
}

//установка напряжения
void DAC_set(uint16_t data)
{
	DACB.CH0DATA = data;
}

//выключение
void DAC_off(void)
{
	DACB.CH0DATA = 0;
	sysclk_disable_module(SYSCLK_PORT_B, SYSCLK_DAC);
}