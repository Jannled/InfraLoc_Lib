#include <array>

#include <Arduino.h>

#include "infraloc_utils.hpp"
#include "InfraLoc.hpp"

#ifdef MICRO_ROS_ENABLED
#include "InfraLoc_Node.hpp"
#endif

#include <pico.h>
#include <hardware/adc.h>
#include <hardware/pwm.h>
#include <hardware/clocks.h>

#ifdef ARDUINO_ARCH_MBED
#include <mbed_error.h>
#endif
#include <rmw/error_handling.h>

#define CAPTURE_DEPTH 512

#define SAMPLE_FREQ 400000u	// 200kHz
#define FREQ_BIN 2			// 99 = 38.672 kHz (98 = 38.281kHz) , 2 = 1kHz

constexpr uint8_t MUX_S0 = 6u;
constexpr uint8_t MUX_S1 = 7u;
constexpr uint8_t MUX_S2 = 8u;
constexpr uint8_t MUX_S3 = 9u;
constexpr uint8_t ADC_PIN = 28u;

constexpr uint8_t USR_LED_1 = 2;
constexpr uint8_t USR_LED_2 = 3;
constexpr uint8_t USR_BTN = 21;
constexpr uint8_t PWM_PIN = 20;

InfraLoc<CAPTURE_DEPTH>* infraLoc;

#ifdef MICRO_ROS_ENABLED
InfraNode* infraNode;
#endif // MICRO_ROS_ENABLED

void outputPWM(uint8_t gpio_pin, unsigned int frequency, unsigned int dutycycle = 500u);

void printRawADC();
void frequencySweep();
void printMagnitudes(unsigned int fourierBin);

#ifdef ARDUINO_ARCH_MBED
/**
 * @brief Called by MBed if an error is thrown / Arduino crashes
 * @see https://os.mbed.com/docs/mbed-os/v6.16/apis/error-handling.html
 * @param error_ctx 
 */
void my_mbed_error_handler(const mbed_error_ctx *error_ctx)
{
	volatile uint32_t lineNo = error_ctx->error_line_number;
	volatile int errorNo = error_ctx->error_status;

	lineNo; // Prevent complaints from the compiler about unused variables
	errorNo;

	digitalWrite(USR_LED_2, HIGH);
}
#endif

void setup()
{
	// Enable Serial, it might be needed by microROS
	Serial.begin(115200);

	#ifdef ARDUINO_ARCH_MBED
	mbed_set_error_hook(my_mbed_error_handler);
	#endif

	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(USR_LED_1, OUTPUT);
	pinMode(USR_LED_2, OUTPUT);
	pinMode(PWM_PIN, OUTPUT);
	pinMode(USR_BTN, INPUT_PULLDOWN);

	// Drive Sender
	outputPWM(PWM_PIN, 50000);
	
	// Create InfraLoc Receiver class
	infraLoc = new InfraLoc<CAPTURE_DEPTH>(ADC_PIN, MUX_S0, MUX_S1, MUX_S2, MUX_S3, SAMPLE_FREQ);

	delay(100);

	#ifdef MICRO_ROS_ENABLED
	infraNode = new InfraNode();
	infraNode->init();
	#endif
}

void loop()
{
	digitalWrite(LED_BUILTIN, HIGH);
	digitalWrite(USR_LED_1, !digitalRead(USR_LED_1));

	// Sender Frequencies: 20kHz, 30kHz, 40kHz, 50kHz
	//constexpr uint freq_1 = (38400*NUM_SAMPLES)/SAMPLE_FREQ;
	constexpr uint freq_1 = (20000*NUM_SAMPLES)/SAMPLE_FREQ;
	constexpr uint freq_2 = (30000*NUM_SAMPLES)/SAMPLE_FREQ;
	constexpr uint freq_3 = (40000*NUM_SAMPLES)/SAMPLE_FREQ;

	// Gather infrared data
	infraLoc->update();

	#ifdef MICRO_ROS_ENABLED

	// Calculate all 3 angles
	infraLoc->calculateStrength(freq_1);
	const number_t angle_a = infraLoc->calculateDirection(infraLoc->results);
	#ifdef DEBUG_INFRA_BUCKETS 
	infraNode->publishBucketStrength(infraLoc->results, angle_a); 
	#endif

	infraLoc->calculateStrength(freq_2);
	const number_t angle_b = infraLoc->calculateDirection(infraLoc->results);
	#ifdef DEBUG_INFRA_BUCKETS 
	infraNode->publishBucketStrength2(infraLoc->results, angle_b); 
	#endif

	infraLoc->calculateStrength(freq_3);
	const number_t angle_c = infraLoc->calculateDirection(infraLoc->results);
	#ifdef DEBUG_INFRA_BUCKETS
	infraNode->publishBucketStrength3(infraLoc->results, angle_c); 
	#endif

	// Use the the 3 angles for planar resection and publish to topic
	const pos2 pose = infraNode->calculatePosition(angle_a, angle_b, angle_c);
	infraNode->publishPositionMessage(pose);

	// Tick the microROS executor
	infraNode->update();

	if(infraNode->positionUpdated)
		infraNode->updatePositions();

	#else
	printMagnitudes(FREQ_BIN);
	//frequencySweep();
	//printRawADC();
	#endif
}

/**
 * https://github.com/raspberrypi/pico-examples/blob/master/pwm/hello_pwm/hello_pwm.c
*/
void outputPWM(uint8_t gpio_pin, unsigned int frequency, unsigned int dutycycle)
{
	constexpr uint16_t TOP_VALUE = 1000; // The value is TOP +1

	#if defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_ARCH_RP2040)
	volatile uint32_t f_sys = clock_get_hz(clk_sys); // Default 125MHz
	gpio_set_function(gpio_pin, GPIO_FUNC_PWM);
	uint slice_num = pwm_gpio_to_slice_num(gpio_pin);
	pwm_set_wrap(slice_num, TOP_VALUE);
	pwm_set_clkdiv(slice_num, (float) f_sys / (frequency*TOP_VALUE));
	pwm_set_gpio_level(gpio_pin, dutycycle);
	pwm_set_enabled(slice_num, true);
	#else
	#error This library only supports the Pi Pico right now.
	#endif
}

/**
 * @brief Print a list of all DFT results for each channel (only one target frequency)
 * Needed for the Matlab ServoSweep.m functions
 * @param fourierBin The target frequency as a fourier bin
 */
void printMagnitudes(unsigned int fourierBin)
{
	// Measure the light
	infraLoc->update();
	
	// Calculate Beacon 1
	infraLoc->calculateStrength(99);
	number_t angle = infraLoc->calculateDirection(infraLoc->results);
	printArray(infraLoc->results, angle);

	// Calculate Beacon 2
	infraLoc->calculateStrength(128);
	angle = infraLoc->calculateDirection(infraLoc->results);
	printArray(infraLoc->results, angle);
}

/**
 * @brief Sweep over the full frequency range one bin at a time to find the optimum frequency bin with a python script
 * 
 */
void frequencySweep()
{
	std::array<number_t, 16> results;
	results.fill(1337);

	infraLoc->update();
	for(size_t k=0; k<CAPTURE_DEPTH/2 + 1; k++)
	{
		for(uint8_t channel=0; channel<INFRALOC_NUM_CHANNELS; channel++)
			results[channel] = infraLoc->getFrequencyComponent(k, channel);

		printArray(results, (int) k);
	}

	Serial.println("------------------------------------------------------------------");
}

/**
 * @brief Print raw adc counts for each channel
 * 
 */
void printRawADC()
{
	std::array<number_t, 16> results;
	results.fill(1337);
	
	for(uint8_t channel=0; channel<INFRALOC_NUM_CHANNELS; channel++)
	{
		infraLoc->switchChannels(channel);
		delay(10);
		results[channel] = analogRead(ADC_PIN);
	}

	printArray(results, -1);
}
