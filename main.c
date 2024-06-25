/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************

-------------------------- END-OF-HEADER -----------------------------

File    : main.c
Purpose : Generic application start


 периодическая отправка пакетов на ПК через CAN-USB-переходник. 
 Приём пакетов от ПК и передача поля данных в UART1 (9600/8/1/N) -> COM-port -> Terminal. 
 Фильтрация приема сообщений по FRAME_ID.

*/

/*
Периодически отправляются сообщения по CAN от МК в ПК. Информация о состоянии кнопок
Прием от ПК соообщения с конкретным ID, по которому МК зажигает либо гасит светодиоды.

*/


#include <stdio.h>
#include <stdlib.h>
#include "stm32f407xx.h"



void RCC_Init(void);

void GPIO_Init(void){
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
	
	//-------- GPIO for buttons -------------------
	GPIOE -> PUPDR |= GPIO_PUPDR_PUPD10_0;
	GPIOE -> PUPDR |= GPIO_PUPDR_PUPD11_0;
	GPIOE -> PUPDR |= GPIO_PUPDR_PUPD12_0;
	   
	//-------- GPIO settings for LED1 LED2 LED3 --------
	GPIOE -> MODER |=GPIO_MODER_MODE13_0;
	GPIOE -> MODER |=GPIO_MODER_MODE14_0;
	GPIOE -> MODER |=GPIO_MODER_MODE15_0;

}



void CAN1_Init(void){

}



void CAN1_Receive(void){
	// прием сообщений из CAN шины
	// опрос FIFO

	if ((CAN1->RF0R & CAN_RF0R_FMP0) != 0){	
		
	
	}
}


int main(void){

	while(1) {


	}	// while(1)

	return 0;
}





/*************************** End of file ****************************/
