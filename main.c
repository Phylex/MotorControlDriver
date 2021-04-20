#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"

const uint LED_PIN = 25;
const uint POT_IN = 26;
const uint POT_ADC_CHAN = 0;
const uint IN1 = 0;
const uint IN2 = 1;
const uint NSLEEP = 3;
const uint SO = 27;
const uint SO_ADC_CHAN = 1;
const uint SNSOUT = 4;
const uint NFAULT = 5;
const float conversion_factor = 3.3f / (1 << 12);

int main() {
	// descriptions for the picotool program
	bi_decl(bi_program_description("Program to test the MotorControl Board"));
	bi_decl(bi_1pin_with_name(LED_PIN, "On-board LED"));
	bi_decl(bi_1pin_with_name(POT_IN, "Potentiometer Input"));
	bi_decl(bi_1pin_with_name(IN2, "Second Channel PWM"));
	bi_decl(bi_1pin_with_name(IN1, "First Channel PWM"));
	bi_decl(bi_1pin_with_name(NSLEEP, "Low Active Driver Sleep"));
	bi_decl(bi_1pin_with_name(SO, "Shunt Amplifier Out -> Current Measurement"));
	bi_decl(bi_1pin_with_name(SNSOUT, "Indicator that driver has hit current limit"));
	bi_decl(bi_1pin_with_name(NFAULT, "Driver Fault Indicator, active low, requires Pull-up resistor"));

	// initialize uart as stdout
	stdio_init_all();
	
	// initialize the gpio pins
	gpio_set_function(LED_PIN, GPIO_FUNC_PWM);
	gpio_set_function(IN1, GPIO_FUNC_PWM);
	gpio_set_function(IN2, GPIO_FUNC_PWM);
	gpio_init(NSLEEP);
	gpio_init(SNSOUT);
	gpio_init(NFAULT);

	// initialize the adc_pins
	adc_init();
	adc_gpio_init(POT_IN);
	adc_gpio_init(SO);
	
	//
	uint slice = pwm_gpio_to_slice_num(LED_PIN);
	uint channel = pwm_gpio_to_channel(LED_PIN);
	pwm_set_wrap(slice, 1024);
	pwm_set_chan_level(slice, channel, 0);
	pwm_set_enabled(slice, true);
	while (true) {
		adc_select_input(POT_ADC_CHAN);
		uint16_t adc_result = adc_read();
		float pot_voltage = adc_result * conversion_factor;
		printf("Raw value: 0x%03x, shifted value: 0x%03x voltage: %f V\n", adc_result, (adc_result >> 6), pot_voltage);
		pwm_set_chan_level(slice, channel, (adc_result >> 6));
		sleep_ms(500);
	}
}
