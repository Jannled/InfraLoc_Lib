#include "infraloc_utils.hpp"

#include <Arduino.h>

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