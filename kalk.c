#include <stm32.h>
#include <gpio.h>
#include <delay.h>
#include <string.h>
#include <stdbool.h>
#include <lcd.h>

#define LIMIT_QUEUED 100


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

#define NUM_KEYS		4	// In a single column or row

#define BSRR_UPPER_HALF	16

#define PRESS_TRIES		6

#define TIM_COUNTERMODE_UP	0

#define NOT_PRESSED		-1

#define KEY_LEN_NORMAL	4
#define NUM_NORMAL		8

#define KEY_LEN_SPECIAL	2
#define NUM_SPECIAL		4

#define UPGRADE_TRESHOLD	500 	// Minimal value of millisecond to upgrade the letter

// Action keys codes
#define SPACE 			3	// A
#define NEWLINE			7	// B
#define CLEAR_ONCE		11	// C
#define DELETE_ALL		15	// D

#define WRITE_NEW		16
#define REPEAT_KEY		17

#define ACTION_COL		3
#define SPECIAL_ROW		3
#define SINGLE_SPECIAL	0

// LCD defines
#define COL_ROLLBACK	9 	// Newline needed: increment text_line, set posiotn to 0
#define ROW_ROLLBACK	5	// Needs to clear the screen

#define LAST_COLUMN		8
#define LAST_ROW		4

/*
A struct for queueing tasks to handle
*/
typedef struct event {
	int write_mode;			// Action_mode: key_id, WRITE_NEW, REPEAT_KEY
	char letter;
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


// How many times interruption for keys has been detected
int times_press_detected;
// The current text line at the display
int text_line;
// The current column number at the display
int current_cursor_col;

// The code of the last pressed key
int previous_key_code = -1;

/*
The queue of events (actions and letters to display)
with queue features.

The volatile keyword forces the compiler not to optimise
actions related to the distinguished variables. It indicates
that the variable vallues can be modified at any time
and the compiler should not make any assumptions
about the variables values.
*/
volatile int head;
volatile int tail;
event queue_events[LIMIT_QUEUED];

// The position of the current letter on the button list
int letter_modulo;

// For signalling purposes
event empty_event = {-1, '?'};

/****************

QUEUE

*****************/

bool empty_queue(void) {
	return head == tail;
}

event pop(void) {
	if (head == LIMIT_QUEUED || head == tail) {
		return empty_event;
	}

	event id = queue_events[head++];

	return id;
}

void push(event id) {
	if (tail != LIMIT_QUEUED) {
		queue_events[tail++] = id;
	}
	else if (head == tail) {
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

// Values: [0, 15]
int calculate_key_index(int row_id, int col_id) {
	return NUM_KEYS*(row_id - ROW1) + col_id;
}

// For special_keys table
int get_special_key_index(int key_id) {
	if (key_id == 0) {
		return 0;
	}
	else {
		return (key_id%10) - 1;
	}
}

// For normal_keys table
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

// A, B, C or D
bool is_action_key(int key_id) {
	return key_id%NUM_KEYS == ACTION_COL;
}

event prepare_event_update_letter_modulo(int key_id, int mode) {
	event result;

	if (is_action_key(key_id)) {
		result = (event) { 
			.write_mode = key_id, // ACTION: key_id
			.letter = '?'};
	}
	else if (key_id == SINGLE_SPECIAL || key_id/NUM_KEYS == SPECIAL_ROW) {
		letter_modulo %= KEY_LEN_SPECIAL;
		result = (event) {
			.write_mode = mode,
			.letter = special_keys[get_special_key_index(key_id)][letter_modulo]};
	}
	else {
		letter_modulo %= KEY_LEN_NORMAL;
		result = (event) { 
			.write_mode = mode,
			.letter = normal_keys[get_normal_key_index(key_id)][letter_modulo]};
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

	int mode_code = to_display.write_mode;

	if (is_action_key(mode_code)) {
		/*
		A - SPACE
		B - NEWLINE
		C - CLEAR_ONCE
		D - DELETE_ALL
		*/

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
			LCDputchar(' ');
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

/************************************

TIMER CONFIGURATION AND USAGE

*************************************/


/*
Initialization of the timer WITHOUT starting.
TIM3 will be used for contact vibration exclusion.
*/
void set_clock_and_initial_TIM3_registers_values_without_starting(void) {
	/*
	Enable peripheral TIM3 clock - configured in the function responsible
	for TIM3 configuration in order to separate program functionalities
	*/
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	// Incremental counting mode
	TIM3->CR1 = TIM_COUNTERMODE_UP;	// A control register 1

	/* 
	The default frequency is 16 MHz. Changing the prescaler value turns
	a 16 MHz clock into a 1 MHz clock which ticks once per 1 microsecond.

	-1 results from the formula for decreasing the frequency of the counter update:
	CLOCK_COUNTER3 = CLOCK_TIMER3 / (TIMER3->PSC + 1)
	*/
	TIM3->PSC = 16 - 1; // A prescaler

	/*
	Stores the value up to which the counter counts. 
	
	Since 1 microsecond = 0.001 millisecond and we want to trigger an interruption
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

Configure TIM2 for the measurement between external interrupts

**********************/

void set_clock_and_initial_TIM2_registers_values_without_starting(void) {
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	TIM2->CR1 = TIM_COUNTERMODE_UP;	// A control register

	/* 
	The default frequency is 16 MHz. Changing the prescaler value turns
	an 16 MHz clock into a 0.001 MHz clock which ticks once per 1 millisecond.
	*/
	TIM2->PSC = 16000 - 1; // A prescaler

	/*
	Enable counting as long as possible. The register size is not known
	(slides indicate 16- or 32-bit), therefore the timer will measure on its own
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
	TIM2->CR1 |= TIM_CR1_CEN;
}

void stop_interval_timer_TIM2(void) {
	TIM2->CR1 &= ~TIM_CR1_CEN;
}

bool is_interval_timer_on(void) {
	return ((TIM2->CR1 & TIM_CR1_CEN) != 0);
}

void TIM2_IRQHandler(void) {
	uint32_t it_status = TIM2->SR & TIM2->DIER;

	if (it_status & TIM_SR_UIF) {
		TIM2->SR = ~TIM_SR_UIF;

		/*
		The last external interrupt has occured
		too long ago to be meaningful
		*/
		stop_interval_timer_TIM2();
		TIM2->CNT = 0;
	}
	if (it_status & TIM_SR_CC1IF) {
		TIM2->SR = ~TIM_SR_CC1IF;
	}
}

/*****************

KEY PRESS DETECTION
QUEUEING EVENTS
LETTER IDENTIFICATION

******************/

int check_key_pressed_return_key_id(void) {

	for (int col_id = COL1; col_id <= COL4; col_id++) {

		set_low_state(key_pins[col_id]);

		for (int row_id = ROW1; row_id <= ROW4; row_id++) {
			bool is_pressed = is_row_pressed(key_pins[row_id]);

			if (is_pressed) {

				set_high_state(key_pins[col_id]);

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

/*
Action keys are identified in the beginning of the event processing function,
therefore passing WRITE_NEW by default is safe in the context of the function
usage.

The function is used as a subtitute for upgrading the letter on the current
button according to letter_modulo.
*/
event order_new_pos_for_writing(int key_id) {
	letter_modulo = 0;
	return prepare_event_update_letter_modulo(key_id, WRITE_NEW);
}

void TIM3_IRQHandler(void) {
	uint32_t it_status = TIM3->SR & TIM3->DIER;

	if (it_status & TIM_SR_UIF) {
		TIM3->SR = ~TIM_SR_UIF;
		// Handle the interruption

		// Scan the keypad
		int pressed_key_id = check_key_pressed_return_key_id();
		
		if (pressed_key_id != NOT_PRESSED) {
			if (times_press_detected < PRESS_TRIES) {
				// Repeat the cycle several times in order to exclude the contact vibration
				times_press_detected++;
			}
			else {
				// A key is probably REALLY pressed
				event to_be_queued;
				
				if (is_interval_timer_on()) {
					uint32_t counted_ticks = TIM2->CNT;
					
					stop_interval_timer_TIM2();
					TIM2->CNT = 0;

					/*
					1 tick per millisecond
					*/
					if (counted_ticks <= UPGRADE_TRESHOLD
						&& !is_action_key(pressed_key_id)
						&& previous_key_code == pressed_key_id) {
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

				previous_key_code = pressed_key_id;
				

				// Has been stopped before or has been turned off during the key id analysis
				start_interval_timer_TIM2();
				
				contact_vibration_cleanup();
			}
		}
		else {
			// None of the keys is pressed or has been pressed due to the contact vibration
			contact_vibration_cleanup();
		}
	}

	if (it_status & TIM_SR_CC1IF) {
		TIM3->SR = ~TIM_SR_CC1IF;
		// Input capture flag; both input capture and update flags are set
	}
	
}


/**********

KEYPAD CONFIGURATION

**********/

void configure_gpio_keypad(void) {
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
	}

	if (EXTI->PR & EXTI_PR_PR7) {
		EXTI->PR = EXTI_PR_PR7;
	}

	if (EXTI->PR & EXTI_PR_PR8) {
		EXTI->PR = EXTI_PR_PR8;
	}

	if (EXTI->PR & EXTI_PR_PR9) {
		EXTI->PR = EXTI_PR_PR9;
	}

	set_columns_high_state();

	// Set set the register storing current counter value to zero (CNT - counter)
	TIM3->CNT = 0;

	start_bounce_timer_TIM3();
}

void configure_clock_rows_cols_interruptions(void) {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; 		// keypad

	// SYSCFGEN (INTERRUPT)
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
}

void configure(void) {
	configure_clock_rows_cols_interruptions();
	LCDconfigure();

	// Clock needs some time to be turned on
	__NOP();

	// Configure the interval measurement clock
	set_clock_and_initial_TIM2_registers_values_without_starting();
	enable_interruptions_TIM2();

	// Configure the contact vibration detection clock
	set_clock_and_initial_TIM3_registers_values_without_starting();
	enable_interruptions_TIM3();

	set_columns_low_state();
	
	configure_gpio_keypad();
	configure_rows_EXTI_NVIC();
}

int main() {
	configure();

	while (1) {
		if (!empty_queue()) {
			process_an_event();
		}
	}
	
	return 0;
}
