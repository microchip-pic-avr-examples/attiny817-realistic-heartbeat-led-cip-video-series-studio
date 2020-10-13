/* 
 *  (c) 2020 Microchip Technology Inc. and its subsidiaries.
 *
 *  Subject to your compliance with these terms, you may use Microchip software
 *  and any derivatives exclusively with Microchip products. You’re responsible
 *  for complying with 3rd party license terms applicable to your use of 3rd
 *  party software (including open source software) that may accompany
 *  Microchip software.
 * 
 *  SOFTWARE IS “AS IS.” NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY,
 *  APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF
 *  NON-INFRINGEMENT, MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.
 *  
 *  IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
 *  INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
 *  WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
 *  BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
 *  FULLEST EXTENT ALLOWED BY LAW, MICROCHIP’S TOTAL LIABILITY ON ALL CLAIMS
 *  RELATED TO THE SOFTWARE WILL NOT EXCEED AMOUNT OF FEES, IF ANY, YOU PAID
 *  DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 */
 
/* 
 * --------------------------------------------------------------------------------------------------------------------------
 *                                    PERIODIC INTERRUPT HEARTBEAT LED EXAMPLE (BARE METAL)
 * --------------------------------------------------------------------------------------------------------------------------
 *
 * This example outputs a PWM signal to an LED, where the duty cycle of the PWM signal is increased or decreased to change 
 * the brightness of the LED using periodic interrupts and a state machine in order to mimic a heartbeat pulse at 
 * ~60 BPM (Beats Per Minute).
 *
 * This example uses a more traditional or common periodic interrupt solution for comparison with a more simple busy-wait
 * solution (hb-busywait) and a more power efficient solution using core independent peripherals (hb-cip), where the same
 * heartbeat effect is achieved 100% independently of the CPU after the initial set-up while using less program memory (3%)
 * and current consumption (8 uA on average). To run one of the three examples, right click on the project and 
 * "set as StartUp project" before compiling and programming the device. 
 * 
 * If the example is run on the ATtiny817 Xplained Pro Development Board, the LED0 will mimic the heartbeat. If just the 
 * ATtiny817 is being used, connect the anode (+) pin of an LED to VCC and cathode (-) to PB4 to see the same result. 
 * The example should also work with other devices in the tinyAVR-1 Series, but might require reconfiguring the output pin.
 * --------------------------------------------------------------------------------------------------------------------------
 *
 * --------------------------------------------------------------------------------------------------------------------------
 */ 

#define OSC_20M		20000000UL
#define F_CPU_DIV	64 //Prescaler value used for the system clock
#define F_CPU		(OSC_20M/F_CPU_DIV)

#define PWM_FREQ_HZ	100
#define TCA_CLK_DIV	16 //Prescaler value used for the TCA clock
#define TCA_PER		(F_CPU/(PWM_FREQ_HZ*TCA_CLK_DIV) - 1) //See ATtiny817 Datasheet Section 20.3.3.4.3

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

uint8_t duty_cycle_pct	= 0; 
uint8_t waiting_cntr	= 0; 
enum HB_FSM {RISING_SMALL, FALLING_SMALL, RISING_BIG, FALLING_BIG, WAITING} HB_state; 

/*
 * 			LED0 Light 
 *          Intensity 
 *             ^ 
 *             |                                               
 *             |           / \                              / \  
 *             |          /   \                            /   \
 *             |   / \   /     \                    / \   /     \
 *             |  /   \_/       \                  /   \_/       \
 *             | /               \________________/               \________________
 *             |----------------------------------------------------------------------------------> Time 
 *  RISING_SMALL^   ^FALLING_SMALL  		  RISING_BIG^    ^FALLING_BIG 
 */

/*
 * Set system clock (see F_CPU) as low as possible to save power consumption 
 * without compromising the timing of the heartbeat signal 
 */
void set_F_CPU(void) {   
	_PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_64X_gc | CLKCTRL_PEN_bm);
}

void init_IO_pins(void) {
	
	/*
	 * Set all IO pins as input with pull-up enabled and 
	 * digital input buffer disabled to minimize power consumption 
	 */

	PORTA.DIR = 0x00;
	PORTB.DIR = 0x00;
	PORTC.DIR = 0x00;
	
	uint8_t PORT_pin_settings = PORT_PULLUPEN_bm | PORT_ISC_INPUT_DISABLE_gc;
	
	for (uint8_t i = 0; i < 8; i++) {
		*((uint8_t *)&PORTA + 0x10 + i) = PORT_pin_settings;
		*((uint8_t *)&PORTB + 0x10 + i) = PORT_pin_settings;
		*((uint8_t *)&PORTC + 0x10 + i) = PORT_pin_settings;
	}
	
	PORTB.DIRSET	= PIN4_bm;			//Set PB4 as output (LED pin) 
	PORTB.PIN4CTRL	= PORT_INVEN_bm;	//Invert output on PB4 as LED is active low
	PORTMUX.CTRLC	= PORTMUX_TCA01_bm; //Select alternate output pin for TCA0 waveform output 1 (PB4)
}

void init_PWM(void) {
	TCA0.SINGLE.CTRLA	= TCA_SINGLE_CLKSEL_DIV16_gc | TCA_SINGLE_ENABLE_bm;
	TCA0.SINGLE.CTRLB	= TCA_SINGLE_CMP1EN_bm | TCA_SINGLE_WGMODE_SINGLESLOPE_gc;
	TCA0.SINGLE.PER		= TCA_PER; //PWM freq = 100 Hz 
}

void set_pwm_duty_cycle(int8_t percent) {
	if(percent > 100) {
		percent = 100;
	}
	uint16_t duty_cycle = (uint16_t)((float)TCA_PER*(float)percent/100);
	TCA0.SINGLE.CMP1BUF = duty_cycle;
}

void init_RTC_PIT(void) {
	
	// Set PIT CLK source from RTC to 32.7 kHz internal RC oscillator => low power consumption
	RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;
	
	// Periodic Interrupt Timer (PIT) setup
	while(RTC.PITSTATUS & RTC_CTRLBUSY_bm);					//Wait for (potential) synchronization to finish
	RTC.PITCTRLA	= RTC_PITEN_bm | RTC_PERIOD_CYC256_gc;	//32.7 kHz / 256 = 128 interrupts/s
	RTC.PITINTCTRL	= RTC_PI_bm;							//Enable periodic interrupts
}

/* 
 * Outputs an LED heartbeat at about 60 BPM, function is called 128 times per second -> 
 * takes ~128 update_heartbeat() function calls for FSM to start at the beginning. 
 */ 	
void update_heartbeat(void) {
	switch (HB_state)
	{
		case RISING_SMALL: 
			duty_cycle_pct++; 
			if(duty_cycle_pct > 15) {
				HB_state++; 
			}
		break;
		 
		case FALLING_SMALL: 
			duty_cycle_pct--; 
			if(duty_cycle_pct < 5) {
				HB_state++; 
			}
		break; 
		
		case RISING_BIG: 
			duty_cycle_pct++; 
			if(duty_cycle_pct > 30) {
				HB_state++;
			}
		break; 
		
		case FALLING_BIG: 
			duty_cycle_pct --; 
			if(duty_cycle_pct == 0) {
				HB_state++; 
			}
		break; 
		
		case WAITING: 
			waiting_cntr++; 
			if(waiting_cntr > 48) {
				waiting_cntr = 0; 
				HB_state = RISING_SMALL;  
			}
		break; 
		
		default: 
			HB_state = RISING_SMALL; 
		break; 
	}
	
	set_pwm_duty_cycle(duty_cycle_pct); 
}

ISR(RTC_PIT_vect) {
	update_heartbeat(); 
	RTC.PITINTFLAGS = RTC_PI_bm; //Clear interrupt flag
}

int main(void)
{	
	set_F_CPU(); 
	init_IO_pins(); 
	init_PWM(); 
	init_RTC_PIT(); 
	
	/*Enable global interrupts*/ 
	sei(); 
	
    while(1) {
		sleep_mode(); //Idle sleep mode - wake up on PIT interrupt
    }
}

