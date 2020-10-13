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
 *                             POWER OPTIMIZED CIP REALISTIC HEARTBEAT LED EXAMPLE (BARE METAL)
 * --------------------------------------------------------------------------------------------------------------------------
 * This example outputs a PWM signal to an LED, where the duty cycle of the PWM signal is dynamically increased and decreased
 * to change the brightness of the LED in order to mimic a heartbeat. The LED pulsates 100% independently of the CPU after 
 * the initial setup with a Beats Per Minute (BPM) of 60. This is done by using three of the core independent peripherals of 
 * the ATtiny817: TCB, TCD and CCL. After the initial setup, the CPU is turned off by using the idle sleep mode, while the
 * LED continues to pulsate.
 * 
 * WARNING: This example is power optimized and stripped down to use the Ultra Low Power 32kHz clock source, 
 * which will make running other tasks on the CPU very slow. Visit https://start.atmel.com/ and browse examples to locate 
 * the normal bare-metal and START driver-based CIP heartbeat example which include a function for changing the BPM 
 * during run-time and additionally shows how to calculate the different timer/counter register values for a specific BPM.
 * The normal examples is also available on our PIC & AVR Examples Github page.
 * 
 * This example is provided for comparison with a simple busy-wait solution (hb-busywait) and a more traditional periodic
 * interrupt solution (hb-interrupt). To run one of the three examples, right click on the project and
 * "set as StartUp project" before compiling and programming the device. 
 *
 * If the example is run on the ATtiny817 Xplained Pro Development Board, the LED0 will mimic the heartbeat. If just the 
 * ATtiny817 is being used, connect the anode (+) pin of an LED to VCC and cathode (-) to PB4 to see the same result. 
 * The example should also work with other devices in the tinyAVR-1 Series, but might require reconfiguring the output pin.
 * Included in the example is a function for changing the RPM and pulse length during run-time. Below is a detailed 
 * explanation of how the example works. Look at the set_heartbeat_BPM()-function in order to see how it is implemented in
 * practice. 
 * 
 * --------------------------------------------------------------------------------------------------------------------------
 *                                          	DETAILED EXAMPLE EXPLANATION
 * --------------------------------------------------------------------------------------------------------------------------
 * 
 * Two Timer/Counters are used to generate the heartbeat pulse output to the LED: TCB and TCD. 
 * The TCB generates *one* PWM signal: TCB waveform output (TCB_WO) with a frequency (f_TCB) and duty cycle (TCB_WO_dc). 
 * The TCD generates *two* PWM signals: waveform output A (TCD_WOA) and waveform output B (TCD_WOB), which have the *same* 
 * frequency (f_TCD), where f_TCD > f_TCB. However, TCD_WOA and TCD_WOB has a different duty cycle (TCD_WOA_dc and 
 * TCD_WOB_dc) and phase. The phase shift is represented by the percentage of the period between the falling edge of TCD_WOA
 * and the rising edge of TCD_WOB and hereby called phase_dc. One period of each PWM signal is illustrated below (where __-- 
 * indicates a transition from low to high output and --__ vice versa): 
 * 
 * 						        |  TCB_WO_dc   |
 * 						TCB_WO  ----------------_____________________________________________ f_TCB
 * 						TCD_WOA ___________-----------__________________________________      f_TCD                   
 * 						TCD_WOB ___________________________________---------------------      f_TCD 
 * 						                   |TCD_WOA_dc|  phase_dc  |     TCD_WOB_dc     |                                    
 *  
 * 
 * By using the Configurable Custom Logic (CCL) peripheral, a Look Up Table (LUT) is configured to continuously do an AND 
 * operation between either TCB_WO and TCD_WOA or TCB_WO and TCD_WOB. In boolean algebra terms:
 * 
 *      									 OUTPUT = TCB_WO * (TCD_WOA ^ TCD_WOB). 
 * 
 * 						TCB_WO  -----------------____________________________________________ f_TCB
 * 						TCD_WOA ___________-----------__________________________________      f_TCD
 * 						TCD_WOB ___________________________________---------------------      f_TCD  
 * 						OUTPUT  ___________------____________________________________________   - 
 * 						                   |O_dc|
 * 
 * Since f_TCD > f_TCB, the TCB_WO signal will drift relative to the TCD_WOA and TCD_WOB signals, changing the overlap. 
 * In addition, the duty cycles are set such that TCD_WOA_dc < phase_dc < TCB_WO_dc < TCD_WOB_dc. This means that the duty 
 * cycle of the OUTPUT signal (O_dc) will gradually increase and decrease. More specifically, when TCB_WO start to overlap
 * with TCD_WOA, O_dc will go from 0 and increase to TCD_WOA_dc, then start to decrease again. Just before the duty cycle 
 * becomes zero again, TCB_WO will overlap with TCD_WOB, increasing O_dc until it is equal to TCB_WO_dc. Since 
 * TCD_WOB_dc > TCB_WO_dc, O_dc will continue to be equal to TCB_WO_dc for a while. Eventually, the drifting of the signals
 * relative to each other results in a decrease of O_dc until it becomes zero for a long while. The cycle will then repeat.
 * An attempt to illustrate this is shown below:
 * 
 * 	    TCB_WO  ---_______---_______---_______---_______---_______---_______---_______---_______---_______---_______
 * 	    TCD_WOA ___-________-________-________-________-________-________-________-________-________-________-______
 * 	    TCD_WOB ______----_____----_____----_____----_____----_____----_____----_____----_____----_____----_____----
 * 	    OUTPUT  ____________-________-________-___________-________--_______---_______---_______--________-_________ 
 * 
 * As OUTPUT is used to turn on/off the LED, the result is that the LED will be turned off, then increase in light 
 * intensity with a short pulse, then a higher peak intensity with a bigger pulse shortly after - mimicking a heartbeat. 
 * The pulses will manifest themselves with a frequency (in Beats Per Minute (BPM)) given by: 
 * 
 *      										BPM = 60*(f_TCD - f_TCB)
 * 
 * Changing the pulse width of the heartbeat is done by increasing or decreasing all the duty cycles described above such 
 * that the ratio between the duty cycles remains the same. Finally, the LED0 on the ATtiny817 Xplained Pro development 
 * board is low side driven, i.e. the anode (+) is connected to VCC and the cathode (-) is connected to an IO pin. Thus, in
 * order to turn on the LED we need to pull the IO pin low. So, in order to make the LED pulsate like a heartbeat, the OUTPUT 
 * signal has to be inverted before it is propagated to the IO pin. This is achieved by either inverting the 
 * boolean expression in the LUT in the CCL [!(TCB_WO * (TCD_WOA ^ TCB_WOB))] or inverting the IO pin output by setting the
 * INVEN bit in the PINCTRL register of the PORT peripheral. 
 * 
 *              LED0 Light 
 *               Intensity 
 *                 ^ 
 *                 |               ____                                    ____ 
 *                 |              /     \                                 /     \  
 *                 |     __      /       \                       __      /       \
 *                 |   /    \   /         \                    /    \   /         \
 *                 |  /      \_/           \                  /      \_/           \
 *                 | /                      \________________/                      \________________
 *                 |----------------------------------------------------------------------------------> Time 
 *                  <---------------BPM period--------------> <-----Pulse Lenght----->                
 *              
 * --------------------------------------------------------------------------------------------------------------------------
 * 
 * --------------------------------------------------------------------------------------------------------------------------
 */  

/*
 * Clock constants for calculating values to set a new BPM,
 * must be changed if clock frequencies or clock divisions/prescalers are changed. 
 */

#include <avr/io.h>
#include <avr/sleep.h>

void set_F_CPU_32kHz(void) {
	_PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_OSCULP32K_gc);
	_PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, ~CLKCTRL_PEN_bm); 
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
	
	PORTB.DIRSET	= PIN4_bm;						//Set PB4 as output (LED pin) 
	PORTB.PIN4CTRL	= PORT_INVEN_bm;				//Invert output on PB4 as LED is active low / low side driven
	PORTMUX.CTRLA	= PORTMUX_LUT0_ALTERNATE_gc;	//Set LED pin (PB4) to output for CCL LUT0 alternative pin
	
}

/* 
 * Closest match with the system clock and timer/counter settings used here is 60.35 BPM.
 * For an explanation of how the TCB and TCD CMP register values are found, please refer to the normal 
 * "Realistic Heartbeat" examples available on Atmel START or Microchip PIC & AVR Examples on GitHub. 
 */ 
void init_heartbeat_60BPM() {
	
	/* When to set PWM output on TCB low */
	TCB0.CCMPL = 180;

	/* When to set PWM output on TCB high */
	TCB0.CCMPH = 28;
	 
	/* Enable TCB in 8-Bit PWM Mode with no prescaler*/
	TCB0.CTRLB = TCB_CCMPEN_bm | TCB_CNTMODE_PWM8_gc;
	TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm;
	
	/*When to set TCD PWM output A high*/
	TCD0.CMPASET = 108;
	/*When to set TCD PWM output A low*/
	TCD0.CMPACLR = 122;
	/*When to set PWM output B high*/
	TCD0.CMPBSET = 143;
	/*When to set PWM output B low and determines f_TCD*/
	TCD0.CMPBCLR = 179;

	/* Enable TCD with no prescaler */
	TCD0.CTRLA   = TCD_CLKSEL_SYSCLK_gc | TCD_ENABLE_bm | TCD_CNTPRES_DIV1_gc | TCD_SYNCPRES_DIV1_gc;

	/* Direct the two waveform outputs from TCD and the waveform from TCB to CCL */ 
	CCL.LUT0CTRLB = CCL_INSEL0_TCD0_gc | CCL_INSEL1_TCD0_gc;
	CCL.LUT0CTRLC = CCL_INSEL2_TCB0_gc;
	
	/* 
	 * Configure CCL truth table / Look Up Table (LUT) such that
	 * OUTPUT = TCB_WO * (TCD_WOA ^ TCD_WOB) 
	 * (LSb is the first entry in the table where MSb is the last) 
	 */
	CCL.TRUTH0    = 0b01100000; 
	
	/* Enable CCL and LUT*/
	CCL.LUT0CTRLA = CCL_OUTEN_bm | CCL_ENABLE_bm;
	CCL.CTRLA = CCL_ENABLE_bm;
}

int main(void)
{	
	init_IO_pins(); 
	init_heartbeat_60BPM();
	set_F_CPU_32kHz(); //Optimized for extremely low power consumption
	
	while (1) {
		sleep_mode(); //Idle sleep mode - no wake-up source
	}
}


