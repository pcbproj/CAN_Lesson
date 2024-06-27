
/*
Поставить на плате джампер J3!

Периодическая отправка сообщений по CAN на ПК через CAN-USB-переходник.
	период отправки сообщения CAN = 300 мс
	скорость передачи CAN = 500 кБит/сек
 
Приём сообщений по CAN от ПК и передача поля данных в UART1.
Фильтрация входных CAN-сообщений по FRAME_ID = 0x567.  
	Скорость передачи USART1 115200 бит/сек. 
	8 бит данных,
	1 стоп бит,
	без бита четности
*/



#include <stdio.h>
#include <stdlib.h>
#include "stm32f407xx.h"


#define CAN_TX_TIME_MS		300		// время ожидания в мс для отправки сообщения по CAN

#define RX_FRAME_ID			0x567	// FRAME_ID сообщений, которые мы принимаем. остальные игнорируем



uint16_t ms_count = 0;


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
	RCC->APB1ENR |=	RCC_APB1ENR_CAN1EN;				// включение тактирования CAN1
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;			// включение тактирования GPIOD: PD0 = CAN1_RX, PD1 = CAN1_TX 
	
	GPIOD->MODER |= GPIO_MODER_MODE0_1;				// настройка PD0 в альтернативный режим
	GPIOD->AFR[0] |= (9U << GPIO_AFRL_AFSEL0_Pos);	// выбор альтернативной функции AF9 для PD0

	GPIOD->MODER |= GPIO_MODER_MODE1_1;				// настройка PD1 в альтернативный режим
	GPIOD->AFR[0] |= (9U << GPIO_AFRL_AFSEL1_Pos);	// выбор альтернативной функции AF9 для PD1


	CAN1->MCR |= CAN_MCR_INRQ;						// переключение CAN1 в режим инициализации
	while((CAN1->MSR & CAN_MSR_INAK) == 0){};		// ожидание, пока CAN1 не переключится в режим инициализации


	CAN1->MCR |= CAN_MCR_NART;						// Отключение автоматической ретрансляции сообщений, вероятность неудачной передачи мала
	CAN1->MCR |= CAN_MCR_AWUM;						// Включение автоматического выхода из спящего режима после приема сообщения 
	CAN1->BTR &= ~(CAN_BTR_SILM | CAN_BTR_LBKM);	// Отключение режимов Loop и Silent, нормальный режим работы
	
	/*	Настройка скорости передачи данных 500 кБит/сек:
		предделитель 6. 
		freq tq = 42 / 6 = 7 MHz
		CAN_Bit_time_tq_number = 7_000_000 Hz / 500_000 bit/s  = 14
		CAN_Bit_time = (1 + BS1 * BS2 ) = 14*tq

		BS1 + BS2 = 13*tq
		BS2 = BS1 / 7;
		BS2 = 13*tq / 7 = 1.86 ~= 2*tq
		BS1 = (13-2) * tq = 11*tq

	*/
	CAN1->BTR |= (5 << CAN_BTR_BRP_Pos);			// предделитель равен 6: 42 / 6 = 7 МГц частота тактирования CAN
	CAN1->BTR |= (10 << CAN_BTR_TS1_Pos);			// TS1 = 10, BS1 = 11
	CAN1->BTR |= (1 << CAN_BTR_TS2_Pos);			// TS2 = 1 , BS2 = 2
		
	// настройка фильтрации по FRAME_ID
	// filter list mode ID
	// принимаем сообщения только с RX_FRAME_ID = 0x567;
	CAN1->FMR |= CAN_FMR_FINIT;							// переводим фильтры в режим инициализации	
	CAN1->FM1R |= CAN_FM1R_FBM0;						// выбираем банк 0 в режиме List mode
	CAN1->FS1R &= ~(CAN_FS1R_FSC0);						// явно выбираем разрядность фильтра 16 бит 	
	CAN1->FFA1R &= ~(CAN_FFA1R_FFA0);					// явно выбираем, что сообщение после фильтра сохранится в FIFO0
	CAN1->sFilterRegister[0].FR1 = (RX_FRAME_ID << 5);	// записываем значение FRAME_ID, сообщения с которым мы принимаем
		

												
	CAN1->MCR &= ~(CAN_MCR_INRQ);					// переключение CAN1 в нормальный режим работы
	while((CAN1->MSR & CAN_MSR_INAK) != 0){};		// ожидание, пока CAN1 не переключится в нормальный режим 

}






// функция чтения из FIFO принятого сообщения по CAN
void CAN1_ReceiveMSG(	uint16_t frame_ID,			// идентификатор фрейма CAN
						uint16_t data_len_bytes,	// длина поля данных в байтах 0 - 8 байт
						char rx_array[]				// массив байтов, принятый по CAN
					){
	// проверка FIFO0 не пустое? (проверка FIFO1 не пустое?)
	// вычитывание идентификатора, DLC и данных сообщения из FIFO0/1
	// освобождение FIFO0/1 выставлением бита FROM в соотв. регистре

	if ((CAN1->RF0R & CAN_RF0R_FMP0) != 0){	
		
	
	}
}






// функция передачи сообщения по CAN1
void CAN1_SendMSG(	uint16_t frame_ID,			// идентификатор фрейма CAN 
					uint16_t data_len_bytes,	// длина поля данных в байтах 0 - 8 байт
					char tx_array[]				// массив байтов, для  отправки по CAN
					){

	// будем использовать для передачи собщений mailbox[0]

	// Настроить идентификатор фрейма FRAME_ID
	// Указать длину поля данных
	// записать данные из массива для отправки в mailbox[0]
	// Начать отправку сообщения
	// ждем пока сообщение не отправится

	
}





// настройка USART1 для передачи принятых данных в ПК по USART1
void USART1_Init(void){	
	
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;							// включение тактирования GPIOA: PA9 = TX, PA10 = RX
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;							// включение тактирования USART1 от шины APB2

	
	GPIOA->MODER  |= GPIO_MODER_MODE9_1;							// Альтернативная функция для PA9 (USART1 - TX)
	GPIOA->AFR[1] |= (7 << GPIO_AFRH_AFSEL9_Pos);					// AF7 для PA9
	GPIOA->MODER  |= GPIO_MODER_MODE10_1;                           // Альтернативная функция для PA10 (USART1 - RX)
	GPIOA->AFR[1] |= (7 << GPIO_AFRH_AFSEL10_Pos);					// AF7 для PA10

	
	/* Расчет скорости передачи данных:
		(84МГц/115200)/16 = 45.57; 
		Целая часть = 45 = 0x2D; 
		Дробная часть = 0.57*16 = 9 = 0x09 
	*/

	USART1->BRR |= 0x2D9;	// 115200
	
	/* Включение приемника и передатчика */
	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE; 
	USART1->CR1 &= ~(USART_CR1_M) | ~(USART_CR1_PCE);              // 8-бит, без контроля четности
	USART1->CR2 &= ~(USART_CR2_STOP);                              // 1 стоповый бит
	USART1->CR1 |= USART_CR1_UE;                                   // Включение USART1

}





void usart_send(uint8_t data[], uint32_t len) {
	for (uint32_t i=0; i < len; i++){
		USART1->DR = data[i];
		while ((USART1->SR & USART_SR_TXE) == 0){};
	}
}





void SysTick_Handler(void){		// прервание от Systick таймера, выполняющееся с периодом 1000 мкс
	ms_count++;
}





int main(void){
	
	SysTick_Config(168000);		// настройка SysTick таймера на время отрабатывания = 1 мс
								// 168000 = (частота_с_PLL / время_отрабатывания_таймера_в_мкс)
								// 168000 = 168 МГц / 1000 мкс; 


	while(1) {

		if (ms_count == CAN_TX_TIME_MS){	// отправка CAN-сообщения с кнопками
			ms_count = 0;

		}
		else{

		}

	}	// while(1)

	return 0;
}





/*************************** End of file ****************************/
