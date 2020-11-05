<!-- Please do not change this logo with link -->

[![MCHP](images/microchip.png)](https://www.microchip.com)

# Realistic Heartbeat LED methods | Core Independent Application Examples (CIP) Video Series

This repository is related to our [Core Independent Application Examples (CIP) - Realistic Heartbeat LED](https://youtu.be/kmxscvPfkf8) video on YouTube and contains three examples for implementing a heartbeat LED with decreasing resource consumption (in terms of memory, power and CPU usage) using three different methods: busy-wait, periodic interrupts and Core Independent Peripherals (CIP). The last example is a stripped-down version of another Realistic Heartbeat example and is optimized for low power and memory consumption. The normal CIP heartbeat example has a function for changing the Beats Per Minute (BPM) and pulse-width of the heartbeat pulse during run-time, with comments that explain the working principle behind the example, where both a bare metal and START-driver based version is available. For this reason, the normal CIP heartbeat examples are recommended as the starting point for creating a CIP heartbeat LED driver to use in an application and can be located at:

- [attiny817-realistic-heartbeat-mplab](https://github.com/microchip-pic-avr-examples/attiny817-realistic-heartbeat-mplab)
- [attiny817-realistic-heartbeat-studio](https://github.com/microchip-pic-avr-examples/attiny817-realistic-heartbeat-studio)
- [attiny817-realistic-heartbeat-mplab-start](https://github.com/microchip-pic-avr-examples/attiny817-realistic-heartbeat-mplab-start)
- [attiny817-realistic-heartbeat-studio-start](https://github.com/microchip-pic-avr-examples/attiny817-realistic-heartbeat-studio-start)

All three power optimized heartbeat examples in this repository are programmed bare metal and outputs a Pulse-Width Modulated (PWM) signal to an LED, where the duty cycle of the PWM signal is increased or decreased appropriately to change the brightness of the LED such that it mimics a periodic 60 BPM heartbeat pulse.

If the example is run on the ATtiny817 Xplained Pro Development Board, LED0 will mimic the heartbeat pulse. If just the ATtiny817 is being used: connect the anode (+) of an LED to VCC and cathode (-) to PB4 to see the same result. The example should also work with other devices in the tinyAVR® 1-series but might require reconfiguring the output pin.

### 1. Busy-wait Solution

> Avg. MCU current consumption: 464 μA
>
> Program memory consumption:   13%

The CPU is spending 100% of its resources updating the duty-cycle of the PWM output in for-loops while busy-waiting between updates using a delay-function. Peripherals used on the ATtiny817: TCA.

### 2. Periodic Interrupt Solution

> Avg. MCU current consumption: 449 μA
>
> Program memory consumption:   16%

The CPU is shut down using the IDLE sleep mode and periodically awoken by an interrupt where the duty cycle of the PWM output is updated using a state machine. The CPU is free to do any other task between duty-cycle updates but has to use an additional peripheral compared to the busy-wait solution while being interrupted frequently. Peripherals used on the ATtiny817: TCA and RTC.

### 3. Core Independent Peripherals (CIP) Solution

> Avg. MCU current consumption: 8 μA
>
> Program memory consumption:   3%

After the initial set-up, the CPU can be completely shut down while the Core Independent Peripherals (CIP) continue to output the heartbeat pulse on the LED. This is achieved by routing three PWM signals with carefully selected frequencies, duty cycles and phases to a truth table in the Configurable Custom Logic (CCL) peripheral. The output of the truth table (implemented as a lookup table (LUT) in hardware) is further routed directly to the output pin connected to the LED.

Not only does this method save a lot of power and reduce memory consumption, but the CPU is free to do any other task with true hardware parallelization. This significant difference in performance is achieved by letting the CPU sleep all of the time while using the internal Ultra Low Power 32kHz RC oscillator (OSCULP32K) as the system clock, which is not possible for the two other examples if timing requirements are to be met. Peripherals used on the ATtiny817: TCB, TCD and CCL.

## Related Documentation

- [ATtiny817 Device Page](https://www.microchip.com/wwwproducts/en/ATtiny817)

## Software Used

- [Atmel Studio](https://www.microchip.com/mplab/avr-support/atmel-studio-7) 7.0.2397 or later
- [ATtiny DFP](http://packs.download.atmel.com/) 1.6.316 or later
- AVR/GNU C Compiler (Built-in compiler) 5.4.0 or later

## Hardware Used

- [ATtiny817 Xplained Pro](https://www.microchip.com/DevelopmentTools/ProductDetails/attiny817-xpro)
- Micro-USB cable (Type-A/Micro-B)

## Operation

1. Connect the ATtiny817 Xplained Pro board to the PC using the USB cable.
2. Download the zip file or clone the example to get the source code.
3. Open the .atsln file in Atmel Studio.
4. Right click on the project you want to run (hb-busywait, hb-interrupt or hb-cip).
5. Select "Set as StartUp Project".
6. Build the solution and program the ATtiny817.
7. Observe that the LED0 pulsates like a heartbeat on the board.

## Conclusion

This example has shown the possible benefits of utilizing Core Independent Peripherals on the tinyAVR 1-series to both significantly optimize the power and memory consumption, compared to a simple busy-wait solution and a more traditional periodic interrupt approach, while freeing up the CPU to do any other task in parallel or alternatively shut down completely.