#include "infraloc_utils.hpp"

#include <Arduino.h>

void printArray(const std::array<number_t, 16> &arr, const int k)
{	
	I_SERIAL.print(k);
	I_SERIAL.print(": [");
	I_SERIAL.print(arr.at(0));

	for(size_t i=1; i<arr.size(); i++)
	{
		I_SERIAL.print(", ");
		I_SERIAL.print(arr.at(i));
	}
	I_SERIAL.println("]");
}

void printArray(const std::array<number_t, 16> &arr, const number_t k)
{	
	I_SERIAL.print(k);
	I_SERIAL.print(": [");
	I_SERIAL.print(arr.at(0));

	for(size_t i=1; i<arr.size(); i++)
	{
		I_SERIAL.print(", ");
		I_SERIAL.print(arr.at(i));
	}
	I_SERIAL.println("]");
}
