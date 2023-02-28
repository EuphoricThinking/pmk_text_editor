#include <stm32.h>
#include <gpio.h>
#include <delay.h>
#include <string.h>
#include <stdbool.h>
#include <lcd.h>

// URUCHOM RUN /opt/arm/stm32/ocd/qfn4 

#define BUFF_SIZE 30
#define NUM_BTN 7
#define NUM_BTN_LOW_ACT 6 // The number of buttons activated with low current
#define COMMAND_LEN 3
#define ERR -1
#define MESS_NUM 14
#define LIMIT_QUEUED 100

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


/* KEYBOARD DEFINES */
#define KEYBOARD_GPIO 	GPIOC

#define PIN_COL1		0 	// KD1
#define PIN_COL2		1	// KD2
#define PIN_COL3		2	// KD3
#define PIN_COL4		3	// KD4

#define PIN_ROW1		6	// W1
#define PIN_ROW2		7	// W2
#define PIN_ROW3		8	// W3
#define PIN_ROW4		9	// W4

#define COL1 			0
#define COL2 			1
#define COL3 			2
#define COL4 			3

#define ROW1 			4
#define ROW2 			5
#define ROW3 			6
#define ROW4 			7

#define NUM_KEYS		4

#define BSRR_UPPER_HALF	16

#define PRESS_TRIES		4

#define TIM_COUNTERMODE_UP	0

#define NOT_PRESSED		-1

#define KEY_LEN_NORMAL	4
#define NUM_NORMAL		8

#define KEY_LEN_SPECIAL	2
#define NUM_SPECIAL		4

#define UPGRADE_TRESHOLD	600 	// Minimal value of millisecond to upgrade the letter

// Action keys codes
#define SPACE 			3
#define NEWLINE			7
#define CLEAR_ONCE		11
#define DELETE_ALL		15

#define WRITE_NEW		16
#define REPEAT_KEY		17

#define NORMAL_MODE		0
#define SPECIAL_MODE 	1
#define ACTION_MODE		2

#define ACTION_COL		3
#define SPECIAL_ROW		3
#define SINGLE_SPECIAL	0

// LCD defines
#define COL_ROLLBACK	9 	// Newline needed: increment text_line, set posiotn to 0
#define ROW_ROLLBACK	5	// Needs to clear the screen

#define LAST_COLUMN		8
#define LAST_ROW		4

typedef struct event {
	int write_mode;
	char letter;
	int key_code;
} event;

int key_pins[NUM_KEYS*2] = {PIN_COL1, PIN_COL2, PIN_COL3, PIN_COL4,
							PIN_ROW1, PIN_ROW2, PIN_ROW3, PIN_ROW4};

/*
code	meaning		index in normal_keys
1		ABC2		0
2		DEF3		1
4		GHI4		2
5		JKL5		3
6		MNO6		4
8		PRS7		5
9		TUV8		6
10 		WXY9		7
*/
char normal_keys[NUM_NORMAL][KEY_LEN_NORMAL] = {
									{'A', 'B', 'C', '2'},
									{'D', 'E', 'F', '3'},
									{'G', 'H', 'I', '4'},
									{'J', 'K', 'L', '5'},
									{'M', 'N', 'O', '6'},
									{'P', 'R', 'S', '7'},
									{'T', 'U', 'V', '8'},
									{'W', 'X', 'Y', '9'}
									};

/*
code	meaning		index in special_keys
0		1 <			0
12		* :			1
13		0 Z			2
14		# Q			3
*/
char special_keys[NUM_SPECIAL][KEY_LEN_SPECIAL] = {
									{'1', '<'},
									{'*', ':'},
									{'0', 'Z'},
									{'#', 'Q'}
									};


int times_press_detected;
int text_line;
int current_cursor_col;
int setme;

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

int queued_mess_id[LIMIT_QUEUED];

event queue_events[LIMIT_QUEUED];

volatile int head;
volatile int tail;
int letter_modulo;

int previous_key;

event empty_event = {-1, '?', -1};

void turn_all_off() {
	RedLEDoff();
	BlueLEDoff();
	GreenLEDoff();
	Green2LEDoff();
}

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

/****************

QUEUE

add mutexes

Yes you can use it directly in the expression you pass as argument.

A statement like

fix_up(pQueue->heap, pQueue->size++);
is somewhat equivalent to

{
    int old_value = pQueue->size;
    pQueue->size = pQueue->size + 1;
    fix_up(pQueue->heap, old_value);
}

*****************/

/*
Thread safety analysis will be provided by the author during the appointment
because it seems to be a lenghty explanation.
*/
bool empty_queue(void) {
	return head == tail;
}

event pop(void) {
	// LCDputcharWrap('R');
	if (head == LIMIT_QUEUED || head == tail) {
		return empty_event;
	}

	event id = queue_events[head++];

	return id;
}

void push(event id) {
	if (tail != LIMIT_QUEUED) {
		// LCDputcharWrap('B');
		queue_events[tail++] = id;
	}
	else if (head == tail) {
		// LCDputcharWrap('A');
		head = 0;
		tail = 0;
		queue_events[tail++] = id;
	}
}

event peek(void) {
	if (head == tail) {
		return empty_event;
	}
	else {
		return queue_events[head];
	}
}

bool is_last_letter_same_as_current(int key_id) {
	LCDputcharWrap('^');
	LCDputcharWrap(48 + empty_queue());
	if (empty_queue()
		|| key_id != queue_events[tail - 1].key_code) {
			return false;
	}
	else {
		return true;
	}
}


/******************

KEYPAD STATES AND REGISTERS

******************/
void set_rows_EXTI_to_zero(void) {
	// Clear EXTI bits for rows: ports PC6, PC7, PC8, PC9
	EXTI->PR = 	EXTI_PR_PR6 | EXTI_PR_PR7 |
				EXTI_PR_PR8 | EXTI_PR_PR9;
}

void set_low_state(int col_row_pin) {
	KEYBOARD_GPIO->BSRR = 1 << (col_row_pin + BSRR_UPPER_HALF);
}

void set_high_state(int col_row_pin) {
	KEYBOARD_GPIO->BSRR = 1 << col_row_pin;
}

void set_columns_low_state(void) {
	int pin_index = COL1;
	while (pin_index <= COL4) {
		set_low_state(key_pins[pin_index]);
		pin_index++;
	}
}

void set_columns_high_state(void) {
	int pin_index = COL1;
	while (pin_index <= COL4) {
		set_high_state(key_pins[pin_index]);
		pin_index++;
	}
}

bool is_row_pressed(int row_pin) {
	if (!((KEYBOARD_GPIO->IDR >> row_pin) & 1)) {
		return true;
	}
	else {
		return false;
	}
}

int calculate_key_index(int row_id, int col_id) {
	return NUM_KEYS*(row_id - ROW1) + col_id;
}

int get_special_key_index(int key_id) {
	if (key_id == 0) {
		return 0;
	}
	else {
		return (key_id%10) - 1;
	}
}

int get_normal_key_index(int key_id) {
	if (key_id < 3) {
		return key_id - 1;
	}
	else if (key_id < 7) {
		return key_id - 2;
	}
	else {
		return key_id - 3;
	}
}

bool is_action_key(int key_id) {
	return key_id%NUM_KEYS == ACTION_COL;
}

event prepare_event_update_letter_modulo(int key_id, int mode) {
	event result;

	if (is_action_key(key_id)) {
		result = (event) { 
			.write_mode = ACTION_MODE, 
			.letter = '?', 
			.key_code = key_id};
	}
	else if (key_id == SINGLE_SPECIAL || key_id/NUM_KEYS == SPECIAL_ROW) {
		letter_modulo %= KEY_LEN_SPECIAL;
		result = (event) {
			.write_mode = mode,
			.letter = special_keys[get_special_key_index(key_id)][letter_modulo],
			.key_code = key_id};
	}
	else {
		letter_modulo %= KEY_LEN_NORMAL;
		result = (event) { 
			.write_mode = mode,
			.letter = normal_keys[get_normal_key_index(key_id)][letter_modulo],
			.key_code = key_id};
	}

	return result;
}

/*******************

PROCESS EVENTS

********************/

bool is_writing_possible(void) {
	if (current_cursor_col == 0) {
		if (text_line == ROW_ROLLBACK) {
			return false;
		}
		else {
			return true;
		}
	}
	else {
		return true;
	}
}

void update_text_line_current_cursor_position_forward(void) {
	current_cursor_col++;
	if (current_cursor_col == COL_ROLLBACK) {
		text_line++;
		// if (text_line != ROW_ROLLBACK) {
		// 	current_cursor_col = 0;
		// }
		current_cursor_col = 0;
	}
}

void update_text_line_current_cursor_position_backward(void) {
	if ((current_cursor_col + text_line) > 0) {
		current_cursor_col--;

		if (current_cursor_col < 0) {
			current_cursor_col = LAST_COLUMN;
			text_line--;
		}
	}
}

void process_an_event() {
	event to_display = pop();

	turn_all_off();
	GreenLEDon();
	// LCDputcharWrap('P');

	int mode_code = to_display.write_mode;

	if (mode_code == ACTION_MODE) {
		/*
		A - SPACE
		B - NEWLINE
		C - CLEAR_ONCE
		D - DELETE_ALL
		*/
		mode_code = to_display.key_code;

		if (mode_code == DELETE_ALL) {
			LCDclear();
			text_line = 0;
			current_cursor_col = 0;
		}
		else if (mode_code == CLEAR_ONCE) {
			update_text_line_current_cursor_position_backward();
			LCDgoto(text_line, current_cursor_col);
			LCDputchar(' ');
			LCDgoto(text_line, current_cursor_col);
		}
		else if (mode_code == SPACE && is_writing_possible()) {
			LCDputcharWrap(' ');
			update_text_line_current_cursor_position_forward();
		}
		else if (mode_code == NEWLINE && text_line != LAST_ROW) {
			LCDputchar('\n');
			current_cursor_col = 0;
			text_line++;
		}
	}
	else if (mode_code == REPEAT_KEY) {
		update_text_line_current_cursor_position_backward();
		LCDgoto(text_line, current_cursor_col);
		LCDputcharWrap(to_display.letter);
		update_text_line_current_cursor_position_forward();
	}
	// WRITE_NEW
	else if (is_writing_possible()) {
		LCDputcharWrap(to_display.letter);
		update_text_line_current_cursor_position_forward();
	}
}

// rowcol rowcol_init(int row_number, int col_number) {
// 	rowcol result;
// 	result.row_id = row_number;
// 	result.col_id = col_number;

// 	return result;
// }


/************************************

TIMER CONFIGURATION AND USAGE

*************************************/


/*
Initialization of the timer WITHOUT starting
*/
void set_clock_and_initial_TIM3_registers_values_without_starting(void) {
	/*
	Enable peripheral TIM3 clock - configured in the function responsible
	for TIM3 configuration in order to separate program functionalities
	*/
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	// Incremental counting mode
	// TIM_CR1_URS // Update request source, ored on slides
	TIM3->CR1 = TIM_COUNTERMODE_UP;	// A control register

	/* 
	The default frequency is 16 MHz. Changing the prescaler value turns
	an 16 MHz clock into a 1 MHz clock, which ticks once per 1 microsecond.

	-1 results from the formula for decreasing the frequency of the counter update:
	CLOCK_COUNTER3 = CLOCK_TIMER3 / (TIMER3->PSC + 1)
	*/
	TIM3->PSC = 16 - 1; // A prescaler

	/*
	Stores the value up to which the counter counts. 
	
	Since 1 microsecond = 0.0001 millisecond and we want to trigger an interruption
	after 10 ms, we decide that the counter has to count 10000 ticks.
	*/
	TIM3->ARR = 10000;	// An auto-reload register

	/*
	Force an update event in order to use the values loaded into registers
	from the first cycle.
	*/
	TIM3->EGR = TIM_EGR_UG;
}

void enable_interruptions_TIM3(void) {
	/*
	TIM3->DIER		DMA and interrupt enable register
I 	TIM3->SR 		status register


	Enable interruptions on the counter level

	TIM_SR_UIF 		Update interrupt flag
	TIM_SR_CC1IF 	Capture/compare 1 interrupt flag.
	TIM_DIER_UIE 	Update interrupt enable
	TIM_DIER_CC1IE 	Capture/compare 1 interrupt enable
	*/
	TIM3->SR = ~(TIM_SR_UIF | TIM_SR_CC1IF);
	TIM3->DIER = TIM_DIER_UIE | TIM_DIER_CC1IE;

	// Enable interruptions on NVIC level
	NVIC_EnableIRQ(TIM3_IRQn);
}

void start_bounce_timer_TIM3(void) {
	TIM3->CR1 |= TIM_CR1_CEN;
}

void stop_bounce_timer_TIM3(void) {
	TIM3->CR1 &= ~TIM_CR1_CEN;
}


/**********************

Configure TIM2 for external interuptions measurements

**********************/

void set_clock_and_initial_TIM2_registers_values_without_starting(void) {
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	TIM2->CR1 = TIM_COUNTERMODE_UP;	// A control register

	/* 
	The default frequency is 16 MHz. Changing the prescaler value turns
	an 16 MHz clock into a 0.001 MHz clock, which ticks once per 1 millisecond.
	*/
	TIM2->PSC = 16000 - 1; // A prescaler

	/*
	Enable as long counting as possible; the register size is not known
	(slides indicate 16- or 32-bit). Therefore the timer will measure on its own
	for at least one minute.
	*/
	TIM2->ARR = ~(TIM2->ARR & 0);	// An auto-reload register

	/*
	Force an update event in order to use the values loaded into registers
	from the first cycle.
	*/
	TIM2->EGR = TIM_EGR_UG;
}

void enable_interruptions_TIM2(void) {
	TIM2->SR = ~(TIM_SR_UIF | TIM_SR_CC1IF);
	TIM2->DIER = TIM_DIER_UIE | TIM_DIER_CC1IE;

	// Enable interruptions on NVIC level
	NVIC_EnableIRQ(TIM2_IRQn);
}

void start_interval_timer_TIM2(void) {
	// LCDputcharWrap('+');
	TIM2->CR1 |= TIM_CR1_CEN;
}

void stop_interval_timer_TIM2(void) {
	TIM2->CR1 &= ~TIM_CR1_CEN;
}

bool is_interval_timer_on(void) {
	// LCDputcharWrap('@');
	return ((TIM2->CR1 & TIM_CR1_CEN) != 0);
}

void TIM2_IRQHandler(void) {
	uint32_t it_status = TIM2->SR & TIM2->DIER;

	if (it_status & TIM_SR_UIF) {
		TIM2->SR = ~TIM_SR_UIF;

		stop_interval_timer_TIM2();
		TIM2->CNT = 0;
	}
	if (it_status & TIM_SR_CC1IF) {
		TIM2->SR = ~TIM_SR_CC1IF;
	}
}





int check_key_pressed_return_key_id(void) {

	BlueLEDoff();
	RedLEDoff();
	GreenLEDoff();
	Green2LEDoff();

	for (int col_id = COL1; col_id <= COL4; col_id++) {

		set_low_state(key_pins[col_id]);

		for (int row_id = ROW1; row_id <= ROW4; row_id++) {
			bool is_pressed = is_row_pressed(key_pins[row_id]);

			if (is_pressed) {
				if (row_id == ROW1) {
					RedLEDon();
				}
				else if (row_id == ROW2) {
					BlueLEDon();
				}
				else if (row_id == ROW3) {
					GreenLEDon();
				}
				else {
					Green2LEDon();
				}

				// LCDclear();
				// LCDputchar(48 + row_id);
				// LCDputchar(' ');
				// LCDputchar(48 + col_id);
				// LCDputchar('T');

				set_high_state(key_pins[col_id]); // ADDED

				return calculate_key_index(row_id, col_id);

			}
		}

		set_high_state(key_pins[col_id]);
	}

	return NOT_PRESSED;
}

void contact_vibration_cleanup(void) {
	times_press_detected = 0;
			
	stop_bounce_timer_TIM3();
	set_columns_low_state();
	set_rows_EXTI_to_zero();
	NVIC_EnableIRQ(EXTI9_5_IRQn);
}

event order_new_pos_for_writing(int key_id) {
	letter_modulo = 0;
	return prepare_event_update_letter_modulo(key_id, WRITE_NEW);
}

void printout_TIM2_cnt(int val) {
	LCDputcharWrap('|');
	if (val == 0) {
		LCDputcharWrap('0');
		return;
	}

	while (val != 0) {
		LCDputcharWrap(48 + (val%10));
		val /= 10;
	}
	LCDputcharWrap('_');
}

void TIM3_IRQHandler(void) {
	uint32_t it_status = TIM3->SR & TIM3->DIER;

	if (it_status & TIM_SR_UIF) {
		TIM3->SR = ~TIM_SR_UIF;
		// TODO
		// Handle the interruption
		// BlueLEDoff();
		// RedLEDoff();
		// GreenLEDoff();
		// Green2LEDoff();
		// RedLEDon();

		// Scan the keypad
		int pressed_key_id = check_key_pressed_return_key_id();

		// if (times_press_detected == 0) {
		// 	previous_key = pressed_key_id;
		// }
		
		if (pressed_key_id != NOT_PRESSED) { //} && pressed_key_id == previous_key) {
			if (times_press_detected < PRESS_TRIES) {
				// TODO test
				// BlueLEDoff();
				// GreenLEDoff();
				// RedLEDon();
				// Repeat the cycle several times in order to exclude the contact vibration
				times_press_detected++;
			}
			else {
				// LCDputcharWrap('C');
				// // LCDputcharWrap(48 + (pressed_key_id == 0));
				// // LCDputcharWrap('_');
				// LCDputcharWrap(48 + (pressed_key_id/10));
				// LCDputcharWrap(48 + (pressed_key_id%10));
				// LCDputcharWrap('\n');
				// TODO 
				// A key is probably REALLY pressed
				//push(calculate_key_index(row_id, col_id));
				// push(pressed_key_id);

				// LCDputcharWrap('B');
				event to_be_queued;
				//LCDputchar('H');
				
				if (is_interval_timer_on()) {
					// LCDputcharWrap('?');
					uint32_t counted_ticks = TIM2->CNT;
					
					stop_interval_timer_TIM2();
					TIM2->CNT = 0;

					// printout_TIM2_cnt(counted_ticks);
					
					// LCDputcharWrap('+');
					// LCDputcharWrap(48 + is_last_letter_same_as_current(pressed_key_id));
					/*
					1 tick per millisecond
					*/
					if (counted_ticks <= UPGRADE_TRESHOLD
						&& !is_action_key(pressed_key_id)
						&& is_last_letter_same_as_current(pressed_key_id)) {
							LCDputcharWrap('%');
							letter_modulo++;
							to_be_queued = 
								prepare_event_update_letter_modulo(pressed_key_id, REPEAT_KEY);
					}
					else {
						to_be_queued = order_new_pos_for_writing(pressed_key_id);
					}
				}
				else {
					to_be_queued = order_new_pos_for_writing(pressed_key_id);

				}
				
				push(to_be_queued);
				

				if (!empty_queue()) {
					//LCDputcharWrap('F');
					// LCDputcharWrap((tail - head) + 48);
					setme++;
				}
				else {
					//LCDputcharWrap('E');
				}


				start_interval_timer_TIM2();
				
				contact_vibration_cleanup();
			}
		}
		else {
			// None of the key is pressed or has been pressed due to the contact vibration
			
			
			contact_vibration_cleanup();
		}
	}

	if (it_status & TIM_SR_CC1IF) {
		TIM3->SR = ~TIM_SR_CC1IF;
		// TODO
	}

	// turn_all_off();
	
}

/*

0 1 2 3 4
1 2 3 4
*/



/*************************

LEDS

**************************/

void configure_clock_gpio_leds(void) {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | 
					RCC_AHB1ENR_GPIOBEN;	

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
}


/**********

KEYPAD CONFIGURATION

**********/

void configure_gpio_keypad(void) {
	// TODO

	// Configure columns
	// COL1 - PC0
	GPIOoutConfigure(GPIOC, 
					  0,
                      GPIO_OType_PP, 
					  GPIO_Low_Speed,
                      GPIO_PuPd_NOPULL);

	// COL2 - PC1
	GPIOoutConfigure(GPIOC, 
					  1,
                      GPIO_OType_PP, 
					  GPIO_Low_Speed,
                      GPIO_PuPd_NOPULL);

	// COL3 - PC2
	GPIOoutConfigure(GPIOC, 
					  2,
                      GPIO_OType_PP, 
					  GPIO_Low_Speed,
                      GPIO_PuPd_NOPULL);

	// COL4 - PC3
	GPIOoutConfigure(GPIOC, 
					  3,
                      GPIO_OType_PP, 
					  GPIO_Low_Speed,
                      GPIO_PuPd_NOPULL);


	// Configure rows
	// ROW1 - PC6
	GPIOinConfigure(GPIOC, 6, GPIO_PuPd_UP,
					EXTI_Mode_Interrupt,
					EXTI_Trigger_Falling);

	// ROW2 - PC7
	GPIOinConfigure(GPIOC, 7, GPIO_PuPd_UP,
					EXTI_Mode_Interrupt,
					EXTI_Trigger_Falling);

	// ROW3 - PC8
	GPIOinConfigure(GPIOC, 8, GPIO_PuPd_UP,
					EXTI_Mode_Interrupt,
					EXTI_Trigger_Falling);

	// ROW4 - PC9
	GPIOinConfigure(GPIOC, 9, GPIO_PuPd_UP,
					EXTI_Mode_Interrupt,
					EXTI_Trigger_Falling);

	// wyjście przewsobne - push-pull
	// rezystor podciągający pull up
	// rezystor ściągający pull down
	// TYLKO WIERSZE
	// IRQ
}

void configure_rows_EXTI_NVIC(void) {
	set_rows_EXTI_to_zero();

	// Enable required IRQs - for lines 6, 7, 8, 9
	NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void EXTI9_5_IRQHandler(void) {
	// Disable row interruptions
	NVIC_DisableIRQ(EXTI9_5_IRQn);

	// Set interrupt flags to zero
	if (EXTI->PR & EXTI_PR_PR6) {
		EXTI->PR = EXTI_PR_PR6;
		// queue_to_send(DOWN_BTN*2 + check_single_state(DOWN_BTN));
		// TODO
	}

	if (EXTI->PR & EXTI_PR_PR7) {
		EXTI->PR = EXTI_PR_PR7;
		// queue_to_send(UP_BTN*2 + check_single_state(UP_BTN));
		// TODO
	}

	if (EXTI->PR & EXTI_PR_PR8) {
		EXTI->PR = EXTI_PR_PR8;
		// queue_to_send(UP_BTN*2 + check_single_state(UP_BTN));
		// TODO
	}

	if (EXTI->PR & EXTI_PR_PR9) {
		EXTI->PR = EXTI_PR_PR9;
		// queue_to_send(UP_BTN*2 + check_single_state(UP_BTN));
		// TODO
	}

	// BlueLEDoff();
	// RedLEDon();

	set_columns_high_state();

	// Set set the register storing current counter value to zero (CNT - counter)
	TIM3->CNT = 0;

	start_bounce_timer_TIM3();
}

void configure_clock_rows_cols_interruptions(void) {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN | 		// keypad
					RCC_AHB1ENR_DMA1EN; 		// DMA
					// RCC_APB1ENR_TIM3EN;		// Timer TIM3

	// Usart 
	

	// SYSCFGEN (INTERRUPT)
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	// // Clock needs some time to be turned on // taktowanie
	// __NOP();
}

void usart_configuration(void) {
	// Clock
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

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

	// Peripheral device activation
	USART2->CR1 |= USART_CR1_UE;
}

void dma_configuration(void) {
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
}

void configure(void) {
	configure_clock_rows_cols_interruptions();
	configure_clock_gpio_leds();
	LCDconfigure();

	// Clock needs some time to be turned on // taktowanie
	__NOP();

	// Configure interval clock
	set_clock_and_initial_TIM2_registers_values_without_starting();
	enable_interruptions_TIM2();

	// Configure contact vibration detecting clock
	set_clock_and_initial_TIM3_registers_values_without_starting();
	enable_interruptions_TIM3();

	set_columns_low_state();
	configure_gpio_keypad();

	configure_rows_EXTI_NVIC();
}

void initialize_send_DMA(char* mess, int len) {
	DMA1_Stream6->M0AR = (uint32_t)mess;
	DMA1_Stream6->NDTR = len;
	DMA1_Stream6->CR |= DMA_SxCR_EN;
}

/*
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
*/

int main() {
	configure();
	// initialize_mess_length();
	BlueLEDon();
	// LCDclear();
//	LCDputchar('A');

	// if ((KEYBOARD_GPIO->IDR >> PIN_ROW1) & 1) {
	// 	BlueLEDoff();
	// 	GreenLEDoff();
	// 	RedLEDon();

	// }
	// else {
	// 	RedLEDoff();
	// 	BlueLEDoff();
	// 	GreenLEDon();

	// }

	//int prev = 0;
	// LCDputchar('W');
	while (1) {
		// turn_all_off();
		// RedLEDon();

		if (!empty_queue()) {
			// LCDputchar('O');
			
			// turn_all_off();
			// GreenLEDon();
			
			process_an_event();
		}
		// if (setme != prev) {
		// 	prev = setme;
		// 	LCDputcharWrap((tail - head) + 48);
		// 	// process_an_event();
		// }
	}
	
	return 0;
}
