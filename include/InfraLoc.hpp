#ifndef INFRALOC_CPP
#define INFRALOC_CPP

#include <array>
#include <stdint.h>

#include "mymath.hpp"

#define FIRST_ADC_PIN 26
#define NUM_SAMPLES 512

#define INFRALOC_NUM_CHANNELS 16

template<size_t N>
class InfraLoc
{
private:
	uint8_t adc_pin;
	uint8_t mux_0;
	uint8_t mux_1;
	uint8_t mux_2;
	uint8_t mux_3;

	uint8_t currentChannel;

	void configureMUX();
	void enableADC_DMA(const uint8_t adc_channel);

	#if defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_ARCH_RP2040)
	unsigned int dma_chan_used;
	#endif

public:
	InfraLoc(const uint8_t adc_pin, const uint8_t mux0, const uint8_t mux1, 
		const uint8_t mux2, const uint8_t mux3, const uint32_t k, const uint32_t sample_freq
	);
	~InfraLoc();

	std::array<number_t, INFRALOC_NUM_CHANNELS> results;
	std::array<std::array<uint16_t, N>, INFRALOC_NUM_CHANNELS> captureBuff;
	std::array<number_t, N> hammingCache;
	uint32_t k;
	uint32_t sample_freq;

	void startSampling(uint16_t* buffer, size_t numSamples);
	void stopSampling();
	void switchChannels(const uint8_t channel);
	bool isSampleBufferFilled();
	bool isSampleBufferFilledBlocking();
	
	number_t getFrequencyComponent(const float k, const uint8_t channel);

	void update();

	/*void printArray(std::array<unsigned int, N> &arr);
	void printArray(std::array<uint16_t, N> &arr);*/
};

#endif // INFRALOC_CPP
