#include <array>

#include <Arduino.h>

#include "InfraLoc.hpp"
#include "hardware/adc.h"

#ifdef MICRO_ROS_ENABLED
#include "InfraLoc_Node.hpp"
#endif

#define CAPTURE_DEPTH 512

#define SAMPLE_FREQ 200000u // 200kHz
#define FREQ_BIN 99			// 99 = 38.672 kHz (98 = 38.281kHz) 

constexpr uint8_t MUX_S0 = 6u;
constexpr uint8_t MUX_S1 = 7u;
constexpr uint8_t MUX_S2 = 8u;
constexpr uint8_t MUX_S3 = 9u;
constexpr uint8_t ADC_PIN = 28u;

InfraLoc<CAPTURE_DEPTH>* infraLoc;

#ifdef MICRO_ROS_ENABLED
InfraNode* infraNode;
#endif // MICRO_ROS_ENABLED

void frequencySweep();

void printArray(std::array<number_t, 16> &arr, int k)
{	
	Serial.print(k);
	Serial.print(": [");
	Serial.print(arr.at(0));

	for(size_t i=1; i<arr.size(); i++)
	{
		Serial.print(", ");
		Serial.print(arr.at(i));
	}
	Serial.println("]");
}

void setup()
{
	// Enable Serial, it might be needed by microROS
	Serial.begin(115200);

	pinMode(LED_BUILTIN, OUTPUT);

	infraLoc = new InfraLoc<CAPTURE_DEPTH>(ADC_PIN, MUX_S0, MUX_S1, MUX_S2, MUX_S3, FREQ_BIN, SAMPLE_FREQ);

	delay(100);

	#ifdef MICRO_ROS_ENABLED
	infraNode = new InfraNode();
	infraNode->init();
	#endif
}

void loop()
{
	digitalWrite(LED_BUILTIN, HIGH);

	#ifdef MICRO_ROS_ENABLED
	// Gather infrared data
	infraLoc->update();

	// Update the microROS stuff
	infraNode->publishBucketStrength(infraLoc->results);
	infraNode->update();

	#else
	frequencySweep();
	#endif	
}

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
