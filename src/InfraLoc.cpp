#include "InfraLoc.hpp"

#include "mymath.hpp"

// C standard libraries
#include <stdint.h>

// C++ standard libraries
#include <bitset>		// std::bitset
#include <algorithm>	// std::max_element	

// Arduino libraries
#include <Arduino.h>

// RP2040 SDK stuff
#include <hardware/adc.h>
#include <hardware/dma.h>

template<size_t N>
InfraLoc<N>::InfraLoc(uint8_t adc_pin, uint8_t mux0, uint8_t mux1, uint8_t mux2, uint8_t mux3, 
	const uint32_t sample_freq)
	: adc_pin(adc_pin), mux_0(mux0), mux_1(mux1), mux_2(mux2), mux_3(mux3), currentChannel(0), captureBuff({{0}}),
		sample_freq(sample_freq)
{
	for(size_t i=0; i<this->captureBuff.size(); i++)
		this->captureBuff[i].fill(1337u);

	configureMUX();
	enableADC_DMA(adc_pin);
}

template<size_t N>
InfraLoc<N>::~InfraLoc()
{
	
}

template<size_t N>
void InfraLoc<N>::startSampling(uint16_t* buffer, size_t numSamples)
{
	#if defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_ARCH_RP2040)
	adc_run(true);
	
	dma_channel_config cfg = dma_get_channel_config(this->dma_chan_used);

	dma_channel_configure(this->dma_chan_used, &cfg,
		buffer, 		// dst
		&adc_hw->fifo, 	// src
		numSamples, 	// transfer count
		true 			// start immediately
	);

	#else
	#error This library only supports the Pi Pico right now.
	#endif
}

template<size_t N>
void InfraLoc<N>::stopSampling()
{
	adc_run(false);
	adc_fifo_drain();
}

template<size_t N>
void InfraLoc<N>::switchChannels(uint8_t channel)
{
	std::bitset<sizeof(uint8_t)*8> channelBits(channel);
	digitalWrite(this->mux_0, channelBits.test(0));
	digitalWrite(this->mux_1, channelBits.test(1));
	digitalWrite(this->mux_2, channelBits.test(2));
	digitalWrite(this->mux_3, channelBits.test(3));
	delayMicroseconds(4);
}

template<size_t N>
number_t InfraLoc<N>::getFrequencyComponent(const float k, const uint8_t channel)
{
	std::array<uint16_t, N> buff = this->captureBuff.at(channel);
	number_t winData[buff.size()];
	bartlettWindow(buff.data(), buff.size(), winData);
	//noWindow(buff.data(), buff.size(), winData);
	
	return euclideanDistance(goertzelAlgorithm(winData, buff.size(), k));
}

/**
 * @brief 
 * See [@arbulaIndoorLocalizationBased2020]
 * @param magnitudes 
 */
template<size_t N>
number_t InfraLoc<N>::calculateDirection(const std::array<number_t, INFRALOC_NUM_CHANNELS> &magnitudes)
{
	const number_t pieSize = M_PI/INFRALOC_NUM_CHANNELS; // = 360.0f/INFRALOC_NUM_CHANNELS/2.0f;
	const number_t offset = 0.85f;

	uint8_t max_chan = 0;
	for(uint8_t i=0; i<INFRALOC_NUM_CHANNELS; i++)
	{
		if(magnitudes[i] > magnitudes[max_chan])
			max_chan = i;
	}

	const number_t val_cw  = magnitudes[(max_chan + 1) % INFRALOC_NUM_CHANNELS];
	const number_t val_ccw = magnitudes[(max_chan - 1) % INFRALOC_NUM_CHANNELS];

	const number_t seg_b = val_cw / val_ccw;
	const number_t seg_d = val_ccw / val_cw;

	return (M_TWOPI/INFRALOC_NUM_CHANNELS)*max_chan - pieSize/(seg_b+offset) + pieSize/(seg_d+offset);
}

template<size_t N>
void InfraLoc<N>::update()
{
	// Fill the sample buffer as fast as possible
	for(uint8_t channel = 0; channel<INFRALOC_NUM_CHANNELS; channel++)
	{
		// Select the desired channel and wait for MUX to complete
		switchChannels(channel);
		delayMicroseconds(2);

		// Start to collect data
		startSampling(this->captureBuff[channel].data(), N);

		// Wait until all samples are collected
		this->isSampleBufferFilledBlocking();

		// Stop adc data collection and select the next channel
		stopSampling();
	}
}

template<size_t N>
void InfraLoc<N>::calculateStrength(unsigned int k)
{
	// Pull all the desired frequencies
	for(uint8_t c=0; c<INFRALOC_NUM_CHANNELS; c++)
	{
		this->results[c] = getFrequencyComponent(k, c);
	}
}

template<size_t N>
bool InfraLoc<N>::isSampleBufferFilled()
{
	#if defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_ARCH_RP2040)
	return !dma_channel_is_busy(this->dma_chan_used);
	#else
	#error This library only supports the Pi Pico right now.
	#endif
}

template<size_t N>
bool InfraLoc<N>::isSampleBufferFilledBlocking()
{
	#if defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_ARCH_RP2040)
	dma_channel_wait_for_finish_blocking(this->dma_chan_used);
	return true;
	#else
	#error This library only supports the Pi Pico right now.
	#endif
}

template<size_t N>
void InfraLoc<N>::configureMUX()
{
	pinMode(this->mux_0, OUTPUT);
	pinMode(this->mux_1, OUTPUT);
	pinMode(this->mux_2, OUTPUT);
	pinMode(this->mux_3, OUTPUT);
}

template<size_t N>
void InfraLoc<N>::enableADC_DMA(const uint8_t adc_channel)
{
	#if defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_ARCH_RP2040)
	// https://github.com/raspberrypi/pico-examples/blob/master/adc/dma_capture/dma_capture.c
	adc_gpio_init(adc_channel);

	adc_init();
	adc_select_input((uint8_t) adc_channel - FIRST_ADC_PIN);
	adc_fifo_setup(
		true,	// Write each completed conversion to the sample FIFO
		true,	// Enable DMA data request (DREQ)
		1,		// DREQ (and IRQ) asserted when at least 1 sample present
		true,	// We won't see the ERR bit because of 8 bit reads; disable.
		false	// Shift each sample to 8 bits when pushing to FIFO
	);

	// Divisor of 0 -> full speed. Free-running capture with the divider is
	// equivalent to pressing the ADC_CS_START_ONCE button once per `div + 1`
	// cycles (div not necessarily an integer). Each conversion takes 96
	// cycles, so in general you want a divider of 0 (hold down the button
	// continuously) or > 95 (take samples less frequently than 96 cycle
	// intervals). This is all timed by the 48 MHz ADC clock.
	adc_set_clkdiv(48000000/sample_freq); // SAMPLE_FREQ = 48.000.000 / clkdiv

	// Set up the DMA to start transferring data as soon as it appears in FIFO
	this->dma_chan_used = dma_claim_unused_channel(true);
	dma_channel_config cfg = dma_channel_get_default_config(this->dma_chan_used);

	// Reading from constant address, writing to incrementing byte addresses
	channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
	channel_config_set_read_increment(&cfg, false);
	channel_config_set_write_increment(&cfg, true);

	// Pace transfers based on availability of ADC samples
	channel_config_set_dreq(&cfg, DREQ_ADC);

	dma_channel_configure(this->dma_chan_used, &cfg,
		this->captureBuff.data(), 	// dst
		&adc_hw->fifo, 				// src
		N, 		            		// transfer count
		true 						// start immediately
	);
	#else
	#error This library only supports the Pi Pico right now.
	#endif
}

/*
template<size_t N>
void InfraLoc<N>::printArray(std::array<unsigned int, N> &arr)
{	
	Serial.print("[");
	Serial.print(arr.at(0));

	for(size_t i=1; i<N; i++)
	{
		Serial.print(", ");
		Serial.print(arr.at(i));
	}
	Serial.println("]");
}

template<size_t N>
void InfraLoc<N>::printArray(std::array<uint16_t, N> &arr)
{
	Serial.print("[");
	Serial.print(arr.at(0));

	for(size_t i=1; i<N; i++)
	{
		Serial.print(", ");
		Serial.print(arr.at(i));
	}
	Serial.println("]");
}
*/

template class InfraLoc<128>;
template class InfraLoc<256>;
template class InfraLoc<512>;
template class InfraLoc<1024>;