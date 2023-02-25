#include <stm32.h>
#include <gpio.h>
#include <delay.h>
#include <string.h>

// URUCHOM RUN /opt/arm/stm32/ocd/qfn4 

#define BUFF_SIZE 30
#define NUM_BTN 7
#define NUM_BTN_LOW_ACT 6 // The number of buttons activated with low current
#define COMMAND_LEN 3
#define ERR -1
#define MESS_NUM 14
#define LIMIT_MESS_QUEUED 100

// buttons
#define USER_BTN_GPIO GPIOC
#define USER_BTN_PIN 13

#define LEFT_BTN_GPIO GPIOB
#define LEFT_BTN_PIN 3
#define RIGHT_BTN_GPIO GPIOB
#define RIGHT_BTN_PIN 4
#define UP_BTN_GPIO GPIOB
#define UP_BTN_PIN 5
#define DOWN_BTN_GPIO GPIOB
#define DOWN_BTN_PIN 6
#define ACTION_BTN_GPIO GPIOB
#define ACTION_BTN_PIN 10

#define AT_BTN_GPIO GPIOA
#define AT_BTN_PIN 0


// button codes
#define USER_BTN 0
#define LEFT_BTN 1
#define RIGHT_BTN 2
#define UP_BTN 3
#define DOWN_BTN 4
#define ACTION_BTN 5
#define AT_MODE 6

// action codes
#define RED_ON "LR1"
#define RED_OFF "LR0"
#define RED_TOG "LRT"
#define GREEN_ON "LG1"
#define GREEN_OFF "LG0"
#define GREEN_TOG "LGT"
#define BLUE_ON "LB1"
#define BLUE_OFF "LB0"
#define BLUE_TOG "LBT"
#define GREEN2_ON "Lg1"
#define GREEN2_OFF "Lg0"
#define GREEN2_TOG "LgT"

#define PRESSED 0
#define RELEASED 1

#define LED_ON 0
#define LED_OFF 1

// CR1
// tryb
#define USART_Mode_Rx_Tx (USART_CR1_RE | \
USART_CR1_TE)
#define USART_Enable USART_CR1_UE

#define USART_WordLength_8b 0x0000
#define USART_WordLength_9b USART_CR1_M

#define USART_Parity_No 0x0000
#define USART_Parity_Even USART_CR1_PCE
#define USART_Parity_Odd (USART_CR1_PCE | \
USART_CR1_PS)

// CR2
#define USART_StopBits_1 0x0000
#define USART_StopBits_0_5 0x1000
#define USART_StopBits_2 0x2000
#define USART_StopBits_1_5 0x3000
#define USART_FlowControl_None 0x0000
		

// BRR
#define HSI_HZ 16000000U
#define PCLK1_HZ HSI_HZ
// #define BAUD_RATE 9600U
#define BAUD 9600U

// LEDs
#define RED_LED_GPIO 	GPIOA
#define GREEN_LED_GPIO 	GPIOA
#define BLUE_LED_GPIO 	GPIOB
#define GREEN2_LED_GPIO GPIOA

#define RED_LED_PIN 	6
#define GREEN_LED_PIN 	7
#define BLUE_LED_PIN 	0
#define GREEN2_LED_PIN 	5

#define RED_LED 0
#define GREEN_LED 1
#define BLUE_LED 2
#define GREEN2_LED 3

#define NUM_LEDS 4


#define RedLEDon() \
RED_LED_GPIO->BSRR = 1 << (RED_LED_PIN + 16)
#define RedLEDoff() \
RED_LED_GPIO->BSRR = 1 << RED_LED_PIN

#define GreenLEDon() \
GREEN_LED_GPIO->BSRR = 1 << (GREEN_LED_PIN + 16)
#define GreenLEDoff() \
GREEN_LED_GPIO->BSRR = 1 << GREEN_LED_PIN

#define BlueLEDon() \
BLUE_LED_GPIO->BSRR = 1 << (BLUE_LED_PIN + 16)
#define BlueLEDoff() \
BLUE_LED_GPIO->BSRR = 1 << BLUE_LED_PIN


#define Green2LEDon() \
GREEN2_LED_GPIO->BSRR = 1 << GREEN2_LED_PIN
#define Green2LEDoff() \
GREEN2_LED_GPIO->BSRR = 1 << (GREEN2_LED_PIN + 16)

int pins[NUM_BTN_LOW_ACT] = {USER_BTN_PIN, LEFT_BTN_PIN, RIGHT_BTN_PIN, UP_BTN_PIN, DOWN_BTN_PIN, ACTION_BTN_PIN};
GPIO_TypeDef* gpios[NUM_BTN_LOW_ACT] = {USER_BTN_GPIO, LEFT_BTN_GPIO, RIGHT_BTN_GPIO, UP_BTN_GPIO, DOWN_BTN_GPIO, ACTION_BTN_GPIO};

char* messages[MESS_NUM] = {"USER PRESSED\r\n", "USER RELEASED\r\n", "LEFT PRESSED\r\n", "LEFT RELEASED\r\n", "RIGHT PRESSED\r\n", "RIGH RELEASED\r\n", "UP PRESSED\r\n", 
"UP RELEASED\r\n", "DOWN PRESSED\r\n", "DOWN RELEASED\r\n", "FIRE PRESSED\r\n", 
"FIRE RELEASED\r\n", "MODE PRESSED\r\n", "MODE RELEASED\r\n"};

int mess_length[MESS_NUM];

int queued_mess_id[LIMIT_MESS_QUEUED];

int head;
int tail;

void initialize_mess_length(void) {
	for (int i = 0; i < MESS_NUM; i++) {
		mess_length[i] = strlen(messages[i]);
	}
}

// which button was here

int check_single_state(int id) {
	if (id == AT_MODE) {
		if ((AT_BTN_GPIO->IDR >> AT_BTN_PIN) & 1) {
			return PRESSED;
		}
		else {
			return RELEASED;
		}
	}
	else {
		if (!(gpios[id]->IDR >> pins[id] & 1)) {
			return PRESSED;
		}
		else {
			return RELEASED;
		}
	}
}


void configure(void) {
	// LEDS + DMA
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN |
					RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_DMA1EN;

	// Usart
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

	// SYSCFGEN (INTERRUPT)
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	// Clock needs some time to be turned on // taktowanie
	__NOP();

	// UART - TDX at PA2, RDX at PA3
	GPIOafConfigure(GPIOA,
					2,
					GPIO_OType_PP,
					GPIO_Fast_Speed,
					GPIO_PuPd_NOPULL,
					GPIO_AF_USART2);

	GPIOafConfigure(GPIOA,
					3,
					GPIO_OType_PP,
					GPIO_Fast_Speed,
					GPIO_PuPd_UP,
					GPIO_AF_USART2);

	// Transmission parameters
	USART2->CR1 = USART_CR1_RE | USART_CR1_TE;
	USART2->CR2 = 0;
	USART2->BRR = (PCLK1_HZ + (BAUD / 2U)) / BAUD;
	
	// Send and receive via DMA
	USART2->CR3 = USART_CR3_DMAT | USART_CR3_DMAR;

	RedLEDoff();
	GreenLEDoff();
	BlueLEDoff();
	Green2LEDoff();

	GPIOoutConfigure(RED_LED_GPIO,
					RED_LED_PIN,
					GPIO_OType_PP,
					GPIO_Low_Speed,
					GPIO_PuPd_NOPULL);

	GPIOoutConfigure(GREEN_LED_GPIO,
					GREEN_LED_PIN,
					GPIO_OType_PP,
					GPIO_Low_Speed,
					GPIO_PuPd_NOPULL);
	
	GPIOoutConfigure(BLUE_LED_GPIO,
					BLUE_LED_PIN,
					GPIO_OType_PP,
					GPIO_Low_Speed,
					GPIO_PuPd_NOPULL);

	GPIOoutConfigure(GREEN2_LED_GPIO,
					GREEN2_LED_PIN,
					GPIO_OType_PP,
					GPIO_Low_Speed,
					GPIO_PuPd_NOPULL);

	// IRQ
	// 
	// USER - PC13
	// GPIO_PuPd_UP before
	GPIOinConfigure(GPIOC, 13, GPIO_PuPd_NOPULL,
					EXTI_Mode_Interrupt,
					EXTI_Trigger_Rising_Falling);

	// LEFT - PB3
	GPIOinConfigure(GPIOB, 3, GPIO_PuPd_NOPULL,
					EXTI_Mode_Interrupt,
					EXTI_Trigger_Rising_Falling);

	// RIGHT - PB4
	GPIOinConfigure(GPIOB, 4, GPIO_PuPd_NOPULL,
					EXTI_Mode_Interrupt,
					EXTI_Trigger_Rising_Falling);

	// UP - PB5
	GPIOinConfigure(GPIOB, 5, GPIO_PuPd_NOPULL,
					EXTI_Mode_Interrupt,
					EXTI_Trigger_Rising_Falling);

	// DOWN - PB6
	GPIOinConfigure(GPIOB, 6, GPIO_PuPd_NOPULL,
					EXTI_Mode_Interrupt,
					EXTI_Trigger_Rising_Falling);

	// ACTION - PB10
	GPIOinConfigure(GPIOB, 10, GPIO_PuPd_NOPULL,
					EXTI_Mode_Interrupt,
					EXTI_Trigger_Rising_Falling);

	// AT MODE - PA0
	GPIOinConfigure(GPIOA, 0, GPIO_PuPd_NOPULL,
					EXTI_Mode_Interrupt,
					EXTI_Trigger_Rising_Falling);

	// Clear EXTI bits
	EXTI->PR = 	EXTI_PR_PR13 | EXTI_PR_PR10 |
				EXTI_PR_PR0 | EXTI_PR_PR6 |
				EXTI_PR_PR5 | EXTI_PR_PR4 |
				EXTI_PR_PR3;

	// Enable required IRQs
	NVIC_EnableIRQ(EXTI0_IRQn);
	NVIC_EnableIRQ(EXTI3_IRQn);
	NVIC_EnableIRQ(EXTI4_IRQn);
	NVIC_EnableIRQ(EXTI9_5_IRQn);
	NVIC_EnableIRQ(EXTI15_10_IRQn);

	// Sending stream DMA configuration
	DMA1_Stream6->CR = 	4U << 25 |
						DMA_SxCR_PL_1 |
						DMA_SxCR_MINC |
						DMA_SxCR_DIR_0 |
						DMA_SxCR_TCIE;

	DMA1_Stream6->PAR = (uint32_t)&USART2->DR;

	// Receiving stream DMA configuration
	DMA1_Stream5->CR = 	4U << 25 |
						DMA_SxCR_PL_1 |
						DMA_SxCR_MINC |
						DMA_SxCR_TCIE;

	DMA1_Stream5->PAR = (uint32_t)&USART2->DR;

	// Clear IRQ flags, enable IRQs
	DMA1->HIFCR = DMA_HIFCR_CTCIF6 | DMA_HIFCR_CTCIF5;

	NVIC_EnableIRQ(DMA1_Stream6_IRQn);
	NVIC_EnableIRQ(DMA1_Stream5_IRQn);

	// Peripheral device activation
	USART2->CR1 |= USART_CR1_UE;
}

void initialize_send_DMA(char* mess, int len) {
	DMA1_Stream6->M0AR = (uint32_t)mess;
	DMA1_Stream6->NDTR = len;
	DMA1_Stream6->CR |= DMA_SxCR_EN;
}

int empty_queue(void) {
	return head == tail;
}

int pop(void) {
	if (head == LIMIT_MESS_QUEUED || head == tail) {
		return -1;
	}

	int id = queued_mess_id[head++];

	return id;
}

void push(int id) {
	if (tail != LIMIT_MESS_QUEUED) {
		queued_mess_id[tail++] = id;
	}
	else if (head == tail) {
		head = 0;
		tail = 0;
		queued_mess_id[tail++] = id;
	}
}

int peek(void) {
	if (head == tail) {
		return -1;
	}
	else {
		return queued_mess_id[head];
	}
}

// DMA
void queue_to_send(int message_index) {
	// Transfer is possible
	if ((DMA1_Stream6->CR & DMA_SxCR_EN) == 0 &&
		(DMA1->HISR & DMA_HISR_TCIF6) == 0) {
			initialize_send_DMA(messages[message_index], 
								mess_length[message_index]);
	}
	// Needs to be queued
	else {
		push(message_index);
	}
}

// EXTI IRQ handlers implementation
void EXTI15_10_IRQHandler(void) {
	if (EXTI->PR & EXTI_PR_PR10) {
		EXTI->PR = EXTI_PR_PR10;
		queue_to_send(ACTION_BTN*2 + check_single_state(ACTION_BTN));
	}

	if (EXTI->PR & EXTI_PR_PR13) {
		EXTI->PR = EXTI_PR_PR13;
		queue_to_send(USER_BTN*2 + check_single_state(USER_BTN));
	}
}

void EXTI9_5_IRQHandler(void) {
	if (EXTI->PR & EXTI_PR_PR5) {
		EXTI->PR = EXTI_PR_PR5;
		queue_to_send(UP_BTN*2 + check_single_state(UP_BTN));
	}

	if (EXTI->PR & EXTI_PR_PR6) {
		EXTI->PR = EXTI_PR_PR6;
		queue_to_send(DOWN_BTN*2 + check_single_state(DOWN_BTN));
	}
}

void EXTI4_IRQHandler(void) {
	EXTI->PR = EXTI_PR_PR4;
	queue_to_send(RIGHT_BTN*2 + check_single_state(RIGHT_BTN));
}

void EXTI3_IRQHandler(void) {
	EXTI->PR = EXTI_PR_PR3;
	queue_to_send(LEFT_BTN*2 + check_single_state(LEFT_BTN));
}

void EXTI0_IRQHandler(void) {
	EXTI->PR = EXTI_PR_PR0;
	queue_to_send(AT_MODE*2 + check_single_state(AT_MODE));
}

// DMA1 Stream6 IRQ implementation
void DMA1_Stream6_IRQHandler() {
// Read emerged DMA1 interrupt
	uint32_t isr = DMA1->HISR;
	if (isr & DMA_HISR_TCIF6) {
		// Handle the end of transmission in stream 6; clear flag
		DMA1->HIFCR = DMA_HIFCR_CTCIF6;
		
		// Send if there is a message to send
		if (!empty_queue()) {
			int mess_id = pop();
			initialize_send_DMA(messages[mess_id], mess_length[mess_id]);
		}
	}
}



int main() {
	configure();
	initialize_mess_length();

	while (1) {
	}
	
	return 0;
}
