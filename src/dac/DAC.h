/*
 * DAC.h
 *
 * Created: 29.10.2015 18:55:17
 *  Author: Anton
 */ 


#ifndef DAC_H_
#define DAC_H_

#include <asf.h>

void DAC_init(uint16_t data);

//��������� ����������
void DAC_set(uint16_t data);

//����������
void DAC_off(void);


#endif /* DAC_H_ */