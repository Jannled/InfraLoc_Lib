#include <array>

#include <Arduino.h>

#include "InfraLoc.hpp"

#define CAPTURE_DEPTH 512

constexpr uint8_t MUX_S0 = 9u;
constexpr uint8_t MUX_S1 = 8u;
constexpr uint8_t MUX_S2 = 7u;
constexpr uint8_t MUX_S3 = 6u;
constexpr uint8_t ADC_PIN = 28u;

InfraLoc<512>* infraLoc;

std::array<unsigned int, CAPTURE_DEPTH> capture_buf;
unsigned int dma_chan;

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

	infraLoc = new InfraLoc<512>(ADC_PIN, MUX_S0, MUX_S1, MUX_S2, MUX_S3, 204);
	infraLoc->startSampling();

	delay(100);
}

void loop()
{
	digitalWrite(LED_BUILTIN, HIGH);

	for(size_t i=0; i<CAPTURE_DEPTH; i++)
	{
		infraLoc->k = i;
		infraLoc->update();
		infraLoc->update();
		printArray(infraLoc->results);
	}

	Serial.println("------------------------------------------------------------------");
}
