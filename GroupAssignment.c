// Include the standard STM32 library for hardware control
#include "stm32f4xx.h"
#include<stdio.h>
#include<stdlib.h>

// Variable Declaration
//PART 1: PC13 LED Blinking
volatile uint32_t ms1 = 0; // Tracks time in milliseconds (Timer 5 increment for LED blinking)
volatile uint32_t tick1 = 0; // Tracks elapsed time for toggling the LED
//PART 3: PUSH BUTTON PA8
volatile uint8_t buttonpress = 0; // Tracks whether the button connected to PA8 is pressed or not
//PART 5: ULTRASONIC SENSOR PA1 & PA2
volatile uint32_t start_time = 0; // Stores the start time of the ultrasonic pulse
volatile uint32_t end_time = 0; // Stores the end time of the ultrasonic pulse
volatile uint32_t pulse_width = 0; // Duration of the ultrasonic pulse
volatile float distance = 0; // Calculated distance based on pulse width
volatile uint32_t detect = 0; // Flag to track if an object is detected
volatile uint32_t ms2= 0; // Tracks time in milliseconds (Timer 5 increment for ultrasonic timing)
volatile uint32_t tick2 = 0; // Tracks elapsed time for controlling the servo

//Function Declaration
void Pin_config(); // Configures GPIO pins for various components
//PART 1: PC13 LED Blinking
void Timer5_Init();  // Initializes Timer 5 to generate a 1ms interrupt
//PART 3: PUSH BUTTON PA8
void EXTI8_Init(); // Configures EXTI interrupt for the push button on PA8
//PART 4: SERVO PB1
void Timer3_PWM_Init(); // Configures Timer 3 to generate PWM signals for the servo
void Servo_Angle(uint8_t angle); // Sets the servo motor angle
//PART 5: ULTRASONIC SENSOR PA1 & PA2
void	TIM2_Init(); // Configures Timer 2 for controlling the ultrasonic trigger
void	TIM9_Init(); // Configures Timer 9 for input capture (measuring ultrasonic echo pulse width)
 
int main(void){
    Pin_config();         // Configure GPIO pins for all components
    Timer5_Init();        // Initialize Timer 5 for generating 1ms ticks
    EXTI8_Init();         // Initialize EXTI interrupt for the push button on PA8
    Timer3_PWM_Init();    // Initialize Timer 3 for PWM control of the servo motor
    TIM2_Init();          // Initialize Timer 2 for ultrasonic sensor trigger signal
    TIM9_Init();          // Initialize Timer 9 for capturing ultrasonic echo signal
	
    while (1) {

			// PART 1: LED PC13 Blinking
			if (ms1 - tick1 >= 500) { // Check if 500 ms has elapsed
            GPIOC->ODR ^= GPIO_ODR_OD13; // Toggle the state of the onboard LED (PC13)
            tick1 = ms1; // Update the tick counter
        }
			
				
			//PART 2: PA5  SENSOR AND LED PB10
			// Control PB10 LED based on the IR sensor input
			if (GPIOA->IDR & GPIO_IDR_ID5) { // If IR sensor does not detect an object
						GPIOB->ODR &= ~GPIO_ODR_OD10; // Turn off PB10 LED
      } else {
						GPIOB->ODR |= GPIO_ODR_OD10;  // Turn on PB10 LED 
			}
			
			
			//PART 3: PUSH BUTTON PA8
			if (buttonpress) { // If the button is pressed
            detect=1; // Activate the detect flag
			}

					
			//PART 5: ULTRASONIC SENSOR PA1 & PA2
				if(distance <= 10){ // If the measured distance is less than or equal to 10 cm
					  detect = 1; // Activate the detect flag
				}
				
				//PART 4: SERVO PB1
				if(detect==1){	// If the detect flag is active
					Servo_Angle(0); // Move the servo to 0° position (open the bin)
					if (ms2 - tick2 >= 3000) {  // Check if 3000 ms (3 seconds) has elapsed
            detect=0; // Deactivate the detect flag
					tick2 = ms2;} // Upadate the tick counter
				}else{
					  Servo_Angle(120);} // Move the servo to 120° position (close the bin)
	}		
}

// Pin Config
void Pin_config(void) {
	
	 RCC->AHB1ENR |= 0x07; // Enable GPIOA, GPIOB and GPIOC clocks
	 
	 // PART 1: LED PC13 Blinking
	 // Configure PC13 as outputs (On Board LED)
	 GPIOC->MODER &= ~(0b11 << 26); // Clear mode bits for PC13
	 GPIOC->MODER |= (0b01 << 26);  // Set PC13 as output (0b01)
	 GPIOC->OTYPER &= ~(1 << 13); // Clear bit 13 for push-pull
	 GPIOC->OSPEEDR &= ~(0b11 << 26); // Clear speed bits (low speed)
 	 GPIOC->PUPDR &= ~(0b11 << 26); // No pull-up, no pull-down

	
	 //PART 2: PA5  SENSOR AND LED PB10
	 // Configure PB10 as outputs (LED Indicator)
	 GPIOB->MODER &= ~(0b11 << (10 * 2)); // Clear mode bits for PB10
	 GPIOB->MODER |= (0b01 << (10 * 2)) ; // Set PB10 as output (0b01)
	 GPIOB->OTYPER &= ~(1 << 10) ; // Set Push-Pull mode for PB10
   GPIOB->OSPEEDR &= ~(0b11 << (10 * 2));  // Low speed for PB10
	 GPIOB->PUPDR &= ~(0b11 << (10 * 2)); // No pull-up, no pull-down for PB10
	
	 // Configure PA5 as inputs (IR Sensor)
   GPIOA->MODER &= ~(0b11 << (5 * 2));  // Clear mode bits for PA5
   GPIOA->PUPDR &= ~(0b10 << (5 * 2));  // Set pull-down for PA5 (to detect HIGH signal)
	
	
	 //PART 3: PUSH BUTTON PA8
	 // Configure PA8 as inputs (Push Button)
   GPIOA->MODER &= ~(0b11 << (8 * 2));  // Clear mode bits for PA8
   GPIOA->PUPDR &= ~(0b11 << (8 * 2));  // No pull-up, no pull-down (default)


   //PART 4: SERVO PB1
	 //Alternate Function Pin Config
   // Configure PB1 as Alternate Function (TIM3_CH4 - Servo)
   GPIOB->MODER &= ~(0b11 << (1 * 2)); // Clear mode bits for PB1
   GPIOB->MODER |= (0b10 << (1 * 2)); // Set PB1 to Alternate Function mode
   GPIOB->AFR[0] &= ~(0b1111 << ((1) * 4)); // Clear AF bits for PB1
   GPIOB->AFR[0] |= (0b0010 << ((1) * 4));// AF2 (TIM3_CH4)
		
		
		//PART 5: ULTRASONIC SENSOR PA1 & PA2
		// Configure PA1 as output (Trigger)
		GPIOA->MODER |= (1 << (1 * 2));  // Set MODER0[1:0] = 01 (Output mode)
		GPIOA->MODER &= ~(1 << ((1 * 2) + 1));
		GPIOA->OTYPER &= ~(1 << 1);      // Push-pull
		GPIOA->OSPEEDR |= (3 << (1 * 2)); // High speed

		// Configure PA2 as alternate function (TIM9_CH1)
		GPIOA->MODER |= (1 << (2 * 2 + 1)); // Set MODER2[1:0] = 10 (Alternate function mode)
		GPIOA->MODER &= ~(1 << (2 * 2));
		GPIOA->AFR[0] |= (3 << (2 * 4));     // Set AFRL2 to AF3 (TIM9_CH1)
}




//PART 1: PC13 LED Blinking
// Timer 5 Initialization (1ms)
void Timer5_Init(void) {
    // Enable Timer 5 clock
    RCC->APB1ENR |= (1 << 3); // Enable TIM5 clock (bit 3)

    // Configure Timer 5
    TIM5->PSC = 1600 - 1;     // Prescaler: 16 MHz / 1600 = 10 kHz (0.1 ms per tick)
    TIM5->ARR = 10 - 1;      // Auto-reload: 10 kHz / 10 = 1 kHz (1 ms period)

    // Enable update interrupt
    TIM5->DIER |= TIM_DIER_UIE;

    // Enable Timer 5 counter
    TIM5->CR1 |= TIM_CR1_CEN;

    // Enable Timer 5 interrupt in NVIC
    NVIC_EnableIRQ(TIM5_IRQn);
    NVIC_SetPriority(TIM5_IRQn, 9); // Set priority
}

// Timer 5 Interrupt Handler (Blink PC13 LED)
void TIM5_IRQHandler(void) {
    if (TIM5->SR & TIM_SR_UIF) { // Check if update interrupt flag is set
        TIM5->SR &= ~TIM_SR_UIF; // Clear the interrupt flag

			// Increment the tick count every 1 ms
        ms1 = ms1 + 1; 
				ms2 = ms2 +1; 
    }
}


//PART 3: PUSH BUTTON PA8
// EXTI Initialization for PA8 Button
void EXTI8_Init(void) {
    // Enable SYSCFG clock
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // Configure EXTI line for PA8
    SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR3_EXTI8; // Clear EXTI8 configuration
    SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI8_PA; // Map EXTI8 to PA8

    // Configure EXTI8 interrupt
    EXTI->IMR |= EXTI_IMR_MR8;   // Unmask EXTI8
    EXTI->RTSR |= EXTI_RTSR_TR8; // Enable rising edge trigger for EXTI8

    // Enable EXTI8 interrupt in NVIC
    NVIC_EnableIRQ(EXTI9_5_IRQn); // EXTI9_5 handles PA8
    NVIC_SetPriority(EXTI9_5_IRQn, 1); // Set priority
}

// EXTI9_5 Interrupt Handler (PA8 Button)
void EXTI9_5_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR8) { // Check if interrupt pending flag is set for PA8
        EXTI->PR |= EXTI_PR_PR8; // Clear the interrupt pending flag
       
			buttonpress ^= 1;   // Toggle button override state
			}
}


//PART 4: SERVO PB1
void Timer3_PWM_Init(void) {
    // Enable Timer 3 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // Configure Timer 3 for PWM mode
    TIM3->PSC = 160 - 1;       // Prescaler: 16 MHz / 160 = 100 kHz (10 µs per tick)
    TIM3->ARR = 2000 - 1;      // Auto-reload: 100 kHz / 2000 = 50 Hz (20 ms period)

    // Configure PWM Mode 1 on Channel 4 (PB1)
    TIM3->CCMR2 &= ~TIM_CCMR2_OC4M;      // Clear output compare mode bits for CH4
    TIM3->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2; // Set PWM Mode 1
    TIM3->CCMR2 |= TIM_CCMR2_OC4PE;     // Enable output compare preload

    // Enable output on CH4
    TIM3->CCER |= TIM_CCER_CC4E;

    // Set initial duty cycle (0.5 ms for 0°)
    TIM3->CCR4 = 50; // Initial pulse width: 50 ticks (0.5 ms)

    // Enable Timer 3 counter
    TIM3->CR1 |= TIM_CR1_CEN;
}

// Function to Set Servo Angle
void Servo_Angle(uint8_t angle) {
    if (angle > 180) angle = 180; // Limit angle to 180°

    // Map angle (0-180) to pulse width (50-250)
    uint16_t pulse_width = 50 + ((angle * 200) / 180);

    // Update CCR4 register for the new duty cycle
    TIM3->CCR4 = pulse_width;
}


//PART 5: ULTRASONIC SENSOR PA1 & PA2
void TIM2_Init(void) {
    // Enable TIM2 clock
    RCC->APB1ENR |= (1 << 0); // TIM2 clock enable

    // Set TIM2 for 1 Hz (1-second period)
    TIM2->PSC = 27 - 1;        // Prescaler: 84 MHz / 84 = 1 MHz
    TIM2->ARR = 20000 - 1;   // Auto-reload: 1 MHz / 1,000,000 = 1 Hz (1-second period)

    // Enable Update Interrupt
    TIM2->DIER |= TIM_DIER_UIE; // Enable update interrupt

    // Enable TIM2
    TIM2->CR1 |= TIM_CR1_CEN; // Enable counter

    // Enable TIM2 interrupt in NVIC
    NVIC_SetPriority(TIM2_IRQn, 9); // Set interrupt priority
    NVIC_EnableIRQ(TIM2_IRQn);
}

void TIM9_Init(void) {
    // Enable TIM9 clock (bit 16 in RCC_APB2ENR)
    RCC->APB2ENR |= (1 << 16);

    // Configure TIM9 for input capture on channel 1
    TIM9->CCMR1 |= (1 << 0);      // CC1 channel is input, IC1 is mapped to TI1
    TIM9->CCMR1 &= ~(1 << 1);

    TIM9->CCER |= (1 << 1);       // Capture falling edge
    TIM9->CCER |= (1 << 3);       // Configure both edges (rising and falling)
    TIM9->CCER |= (1 << 0);       // Enable capture for channel 1

    TIM9->PSC = 84 - 1;           // Prescaler: 1 Âµs resolution (assuming 84 MHz clock)
    TIM9->ARR = 0xFFFF;           // Maximum auto-reload value

    TIM9->DIER |= (1 << 1);       // Enable CC1 interrupt
    TIM9->CR1 |= (1 << 0);        // Enable timer
    NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);  // Enable TIM9 interrupt in NVIC
}

void TIM2_IRQHandler(void) { // Timer Interrupt
	if (TIM2->SR & TIM_SR_UIF) { // Check update interrupt flag
		TIM2->SR &= ~TIM_SR_UIF; // Clear update interrupt flag
	}

	GPIOA->ODR ^= GPIO_ODR_OD1; //Toggle Trig

}

void TIM1_BRK_TIM9_IRQHandler(void) {
    if (TIM9->SR & (1 << 1)) {    // Check if interrupt is from CC1
        TIM9->SR &= ~(1 << 1);    // Clear interrupt flag

        if (GPIOA->IDR & GPIO_IDR_ID2) {  // Rising edge detected
            start_time = TIM9->CCR1;  // Capture start time
        } else {                      // Falling edge detected
            end_time = TIM9->CCR1;    // Capture end time
            pulse_width = (end_time >= start_time) ? (end_time - start_time)
                                                   : (0xFFFF - start_time + end_time + 1);
            distance = (pulse_width / 58.0f);  // Convert to cm
        }
    }
}