/**
 * Compare Goertzel algorithm with standard DFT Bin calculation
*/
#include <array>

#include <Arduino.h>

#include "InfraLoc.hpp"
#include "hardware/adc.h"

#define CAPTURE_DEPTH 512

#define SAMPLE_FREQ 200000u // 200kHz
#define FREQ_BIN 204

constexpr uint8_t MUX_S0 = 9u;
constexpr uint8_t MUX_S1 = 8u;
constexpr uint8_t MUX_S2 = 7u;
constexpr uint8_t MUX_S3 = 6u;
constexpr uint8_t ADC_PIN = 28u;

InfraLoc<CAPTURE_DEPTH>* infraLoc;

void frequencySweep();

void setup()
{
	// Enable Serial, it might be needed by microROS
	Serial.begin(115200);

	pinMode(LED_BUILTIN, OUTPUT);

	infraLoc = new InfraLoc<CAPTURE_DEPTH>(ADC_PIN, MUX_S0, MUX_S1, MUX_S2, MUX_S3, FREQ_BIN, SAMPLE_FREQ);

	delay(100);
}

void dragrace()
{
	uint8_t channel = 0;
	std::array<uint16_t, CAPTURE_DEPTH> buff = infraLoc->captureBuff.at(channel);
	
	static number_t sinCache[NUM_SAMPLES] = {0};
	static number_t cosCache[NUM_SAMPLES] = {0};
	generateFourierCacheSin(sinCache, NUM_SAMPLES, FREQ_BIN);
	generateFourierCacheSin(cosCache, NUM_SAMPLES, FREQ_BIN);
	static cmplx_t goertzelCache = generateGoertzelCache(NUM_SAMPLES, FREQ_BIN);
	
	//save_and_disable_interrupts();

	/* Measure the time for Goertzel Algorithm */
	unsigned long startG = micros();
	cachedGoertzelAlgorithm(buff.data(), buff.size(), FREQ_BIN, goertzelCache);
	unsigned long endG = micros();

	/* Measure the time for simple DFT Bins */
	unsigned long startDFT = micros();
	cachedFourierComponent(buff.data(), buff.size(), FREQ_BIN, sinCache, cosCache);
	unsigned long endDFT = micros();

	Serial.print("Goertzel: ");
	Serial.print(endG - startG);
	Serial.print("us, DFT Bin: ");
	Serial.print(endDFT - startDFT);
	Serial.println("us");

	//restore_interrupts()
}

void measureMult()
{
	float x = 0;

	float numbers[100] = {};
	for(size_t i=0; i<100; i++)
		numbers[i] = (float) (micros() % 10000) / 277.0f;

	unsigned long start = micros();
	x *= numbers[0];
	x *= numbers[1];
	x *= numbers[2];
	x *= numbers[3];
	x *= numbers[4];
	x *= numbers[5];
	x *= numbers[6];
	x *= numbers[7];
	x *= numbers[8];
	x *= numbers[9];
	x *= numbers[10];
	x *= numbers[11];
	x *= numbers[12];
	x *= numbers[13];
	x *= numbers[14];
	x *= numbers[15];
	x *= numbers[16];
	x *= numbers[17];
	x *= numbers[18];
	x *= numbers[19];
	x *= numbers[20];
	x *= numbers[21];
	x *= numbers[22];
	x *= numbers[23];
	x *= numbers[24];
	x *= numbers[25];
	x *= numbers[26];
	x *= numbers[27];
	x *= numbers[28];
	x *= numbers[29];
	x *= numbers[30];
	x *= numbers[31];
	x *= numbers[32];
	x *= numbers[33];
	x *= numbers[34];
	x *= numbers[35];
	x *= numbers[36];
	x *= numbers[37];
	x *= numbers[38];
	x *= numbers[39];
	x *= numbers[40];
	x *= numbers[41];
	x *= numbers[42];
	x *= numbers[43];
	x *= numbers[44];
	x *= numbers[45];
	x *= numbers[46];
	x *= numbers[47];
	x *= numbers[48];
	x *= numbers[49];
	x *= numbers[50];
	x *= numbers[51];
	x *= numbers[52];
	x *= numbers[53];
	x *= numbers[54];
	x *= numbers[55];
	x *= numbers[56];
	x *= numbers[57];
	x *= numbers[58];
	x *= numbers[59];
	x *= numbers[60];
	x *= numbers[61];
	x *= numbers[62];
	x *= numbers[63];
	x *= numbers[64];
	x *= numbers[65];
	x *= numbers[66];
	x *= numbers[67];
	x *= numbers[68];
	x *= numbers[69];
	x *= numbers[70];
	x *= numbers[71];
	x *= numbers[72];
	x *= numbers[73];
	x *= numbers[74];
	x *= numbers[75];
	x *= numbers[76];
	x *= numbers[77];
	x *= numbers[78];
	x *= numbers[79];
	x *= numbers[80];
	x *= numbers[81];
	x *= numbers[82];
	x *= numbers[83];
	x *= numbers[84];
	x *= numbers[85];
	x *= numbers[86];
	x *= numbers[87];
	x *= numbers[88];
	x *= numbers[89];
	x *= numbers[90];
	x *= numbers[91];
	x *= numbers[92];
	x *= numbers[93];
	x *= numbers[94];
	x *= numbers[95];
	x *= numbers[96];
	x *= numbers[97];
	x *= numbers[98];
	x *= numbers[99];
	unsigned long end = micros();

	Serial.println(end - start);
}

void loop()
{
	digitalWrite(LED_BUILTIN, HIGH);

	//dragrace();
	measureMult();
}
