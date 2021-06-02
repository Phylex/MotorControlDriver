#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/dma.h"

#define CAPTURE_DEPTH 512

// define the pin constants
const uint ENABLE_BUTTON = 3;
const uint ENABLE_LED = 4;
const uint LED_PIN = 25;
const uint POT_IN = 27;
const uint POT_ADC_CHAN = 1;
const uint IN1 = 14;
const uint IN2 = 15;
const uint NSLEEP = 0;
const uint SO = 26;
const uint SO_ADC_CHAN = 0;
const uint SNSOUT = 2;
const uint NFAULT = 1;
const float conversion_factor = 3.3f / (1 << 8);

// claim the memory for the state machine that controls the motor
bool button_last_state = false;
bool button_state = false;
bool motor_elabeld = false;
bool state_change = false;

// claim the memory needed for the ADC/DMA thing to work
volatile uint8_t adc_buf[CAPTURE_DEPTH];
volatile uint adc_chan = 0;
volatile float Current = 0;
volatile float pot_setpoint = 0;
volatile bool current_captured = false;
volatile bool setpoint_captured = false;
volatile uint dma_chan;
dma_channel_config dma_cfg;

// the ADC will be left to do the free running mode where the 
void adc_handler() {
	adc_run(false);
	adc_fifo_drain();
	dma_hw->ints0 = (1u << dma_chan);
	uint32_t adc_sum = 0;
	for (int i=0; i<CAPTURE_DEPTH; i++) {
		adc_sum += adc_buf[i];
	}
	float adc_mean = (float)(adc_sum)/CAPTURE_DEPTH;
	if (adc_chan == SO_ADC_CHAN){
		Current = adc_mean * conversion_factor;
		current_captured = true;
		adc_chan = POT_ADC_CHAN;
		adc_select_input(adc_chan);
	} else if (adc_chan == POT_ADC_CHAN) {
		pot_setpoint= adc_mean;
		setpoint_captured = true;
		adc_chan = SO_ADC_CHAN;
		adc_select_input(adc_chan);
	}
	dma_channel_set_write_addr(dma_chan, adc_buf, true);
	adc_run(true);
}

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
	bi_decl(bi_1pin_with_name(ENABLE_BUTTON, "The pin connected to the button that enabels/disabels the driver"));
	bi_decl(bi_1pin_with_name(ENABLE_LED, "The LED built into the button to show the state"));

	// initialize uart as stdout
	stdio_init_all();
	
	// initialize the GPIO pins
	gpio_set_function(LED_PIN, GPIO_FUNC_PWM);
	gpio_set_function(IN1, GPIO_FUNC_PWM);
	gpio_set_function(IN2, GPIO_FUNC_PWM);
	gpio_init(NSLEEP);
	gpio_init(SNSOUT);
	gpio_init(NFAULT);

	// initialize the ADC
	adc_init();
	adc_gpio_init(POT_IN);
	adc_gpio_init(SO);
	adc_fifo_setup(
		true, // write to the sample fifo
		true, // Enable DMA request
		1,    // enable dma request when one sample is present
		false,// no error bit
		true  // shift to eight bits
	);
	adc_set_clkdiv(2400);
	
	// initialize the DMA for the ADC
	dma_chan = dma_claim_unused_channel(false);
	if (dma_chan == -1) {
		printf("dma channel could not be aquired\n");
	}
	dma_cfg = dma_channel_get_default_config(dma_chan);
	// configure to read from a constant address and write to an incrementing one
	channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_8);
	channel_config_set_read_increment(&dma_cfg, false);
	channel_config_set_write_increment(&dma_cfg, true);
	// start transfers on the request of the adc
	channel_config_set_dreq(&dma_cfg, DREQ_ADC);
	// initialize the interrupt
	dma_channel_set_irq0_enabled(dma_chan, true);
	irq_set_exclusive_handler(DMA_IRQ_0, adc_handler);
    irq_set_enabled(DMA_IRQ_0, true);
	// configure the channel and start up the dma
	adc_chan = SO_ADC_CHAN;
	adc_select_input(adc_chan);
	dma_channel_configure(dma_chan, &dma_cfg,
			adc_buf,   //dst
			&adc_hw->fifo, //src
			CAPTURE_DEPTH, //size
			true //start now
	);
	
	// set up PWM
	// led Pin
	uint led_pwm_slice = pwm_gpio_to_slice_num(LED_PIN);
	uint led_pwm_channel = pwm_gpio_to_channel(LED_PIN);
	pwm_set_wrap(led_pwm_slice, 1024);
	pwm_set_chan_level(led_pwm_slice, led_pwm_channel, 0);

	// in1 pin
	uint in_pwm_slice = pwm_gpio_to_slice_num(IN1);
	uint in1_pwm_channel = pwm_gpio_to_channel(IN1);
	uint in2_pwm_channel = pwm_gpio_to_channel(IN2);
	pwm_set_wrap(in_pwm_slice, 128 * 20);
	pwm_set_chan_level(in_pwm_slice, in1_pwm_channel, 0);
	pwm_set_chan_level(in_pwm_slice, in2_pwm_channel, 0);

	pwm_set_enabled(in_pwm_slice, false);
	pwm_set_enabled(led_pwm_slice, false);

	// set up the rest of the gpios
	gpio_init(SNSOUT);
	gpio_set_dir(SNSOUT, GPIO_IN);
	gpio_init(NSLEEP);
	gpio_set_dir(NSLEEP, GPIO_OUT);
	gpio_init(ENABLE_BUTTON);
	gpio_set_dir(ENABLE_BUTTON, GPIO_IN);
	gpio_init(ENABLE_LED);
	gpio_set_dir(ENABLE_LED, GPIO_OUT);
	gpio_init(NFAULT);
	gpio_set_dir(NFAULT, GPIO_IN);

	// start the adc and with it the dma sequence
	adc_run(true);

	// enter the loop
	while (true) {
		// state machine for enabling the driver via a button on the Breadboard
		button_state = gpio_get(ENABLE_BUTTON);
		sleep_ms(10);
		button_last_state = button_state;
		button_state = gpio_get(ENABLE_BUTTON);
		if (button_state && !(button_last_state)) {
			if (motor_elabeld) {
				motor_elabeld = false;
			} else {
				motor_elabeld = true;
			}
			state_change = true;
		}
		if (state_change) {
			state_change = false;
			pwm_set_enabled(in_pwm_slice, motor_elabeld);
			gpio_put(NSLEEP, motor_elabeld);
			gpio_put(ENABLE_LED, motor_elabeld);
			if (motor_elabeld) {
				printf("Motor enabled\n");
			} else {
				printf("Motor disabeld\n");
			}
		}

		// control the pwm duty cycle with the help of the ADC value from the potentiometer
		if (setpoint_captured) {
			int pwm_val = (int)pot_setpoint - 128;
			if (pwm_val < 0) {
				pwm_set_chan_level(in_pwm_slice, in1_pwm_channel, 0);
				pwm_set_chan_level(in_pwm_slice, in2_pwm_channel, ((uint16_t)(-pwm_val)) * 20);
			} else if (pwm_val == 0) {
				pwm_set_chan_level(in_pwm_slice, in1_pwm_channel, 0);
				pwm_set_chan_level(in_pwm_slice, in2_pwm_channel, 0);
			} else if (pwm_val > 0) {
				pwm_set_chan_level(in_pwm_slice, in1_pwm_channel, ((uint16_t)pwm_val) * 20);
				pwm_set_chan_level(in_pwm_slice, in2_pwm_channel, 0);
			}
			printf("PWM set to %f\%\n", (float)pwm_val*(100./128.));
			setpoint_captured = false;
		}
		if (current_captured) {
			printf("Current through the Motor: %f A\n");
			current_captured = false;
		}
	}
}
