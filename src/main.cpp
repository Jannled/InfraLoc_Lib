#include <array>

#include <Arduino.h>

#include "infraloc_utils.hpp"
#include "InfraLoc.hpp"
#include "hardware/adc.h"

#ifdef MICRO_ROS_ENABLED
#include "InfraLoc_Node.hpp"
#endif

#define CAPTURE_DEPTH 512

#define SAMPLE_FREQ 200000u // 200kHz
#define FREQ_BIN 99			// 99 = 38.672 kHz (98 = 38.281kHz) , 2 = 1kHz

constexpr uint8_t MUX_S0 = 6u;
constexpr uint8_t MUX_S1 = 7u;
constexpr uint8_t MUX_S2 = 8u;
constexpr uint8_t MUX_S3 = 9u;
constexpr uint8_t ADC_PIN = 28u;

InfraLoc<CAPTURE_DEPTH>* infraLoc;

#ifdef MICRO_ROS_ENABLED
InfraNode* infraNode;
#endif // MICRO_ROS_ENABLED

void printRawADC();
void frequencySweep();
void printMagnitudes(unsigned int fourierBin);

void setup()
{
	// Enable Serial, it might be needed by microROS
	Serial.begin(115200);

	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(2, OUTPUT);

	infraLoc = new InfraLoc<CAPTURE_DEPTH>(ADC_PIN, MUX_S0, MUX_S1, MUX_S2, MUX_S3, FREQ_BIN, SAMPLE_FREQ);

	delay(100);

	#ifdef MICRO_ROS_ENABLED
	infraNode = new InfraNode();
	infraNode->init();
	#endif
}

void loop()
{
	//digitalWrite(LED_BUILTIN, HIGH);
	//digitalWrite(2, !digitalRead(2));

	#ifdef MICRO_ROS_ENABLED
	// Gather infrared data
	infraLoc->update();

	// Update the microROS stuff
	infraNode->publishBucketStrength(infraLoc->results);
	infraNode->update();

	#else
	printMagnitudes(FREQ_BIN);
	//frequencySweep();
	//printRawADC();
	#endif	
}

/**
 * @brief Print a list of all DFT results for each channel (only one target frequency)
 * Needed for the Matlab ServoSweep.m functions
 * @param fourierBin The target frequency as a fourier bin
 */
void printMagnitudes(unsigned int fourierBin)
{
	std::array<number_t, 16> results;
	results.fill(1337);

	infraLoc->update();
	
	for(uint8_t channel=0; channel<INFRALOC_NUM_CHANNELS; channel++)
		results[channel] = infraLoc->getFrequencyComponent(fourierBin, channel);

	printArray(results, fourierBin);
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

		printArray(results, k);
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