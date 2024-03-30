#include "infraloc_utils.hpp"

#include <Arduino.h>

void printArray(const std::array<number_t, 16> &arr, const int k)
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

void printArray(const std::array<number_t, 16> &arr, const number_t k)
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