#include <array>

#include <Arduino.h>

#include "InfraLoc.hpp"
#include "hardware/adc.h"

#define CAPTURE_DEPTH 256

#define SAMPLE_FREQ 20000 // 20kHz
#define FREQ_BIN 204

constexpr uint8_t MUX_S0 = 9u;
constexpr uint8_t MUX_S1 = 8u;
constexpr uint8_t MUX_S2 = 7u;
constexpr uint8_t MUX_S3 = 6u;
constexpr uint8_t ADC_PIN = 28u;

InfraLoc<CAPTURE_DEPTH>* infraLoc;

void frequencySweep();

void printArray(std::array<number_t, 16> &arr)
{	
	Serial.print("[");
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
	Serial.begin(115200);
	pinMode(LED_BUILTIN, OUTPUT);

	infraLoc = new InfraLoc<CAPTURE_DEPTH>(ADC_PIN, MUX_S0, MUX_S1, MUX_S2, MUX_S3, FREQ_BIN, SAMPLE_FREQ);
	infraLoc->startSampling();

	delay(100);
}

void loop()
{
	digitalWrite(LED_BUILTIN, HIGH);

	frequencySweep();
}

void frequencySweep()
{
	for(size_t i=0; i<CAPTURE_DEPTH; i++)
	{
		infraLoc->k = i;
		infraLoc->update();
		Serial.print(i); Serial.print(": ");
		//infraLoc->printArray(infraLoc->captureBuff);
		printArray(infraLoc->results);
	}

	Serial.println("------------------------------------------------------------------");
}
