#include "InfraLoc.hpp"

// C++ standard libraries
#include <bitset>         // std::bitset

// Arduino libraries
#include <Arduino.h>

// RP2040 SDK stuff
#include <hardware/adc.h>
#include <hardware/dma.h>

template<size_t N>
InfraLoc<N>::InfraLoc(uint8_t adc_pin, uint8_t mux0, uint8_t mux1, uint8_t mux2, uint8_t mux3, uint16_t k)
	: adc_pin(adc_pin), mux_0(mux0), mux_1(mux1), mux_2(mux2), mux_3(mux3), currentChannel(0), captureBuff({{0}}), k(k)
{
	this->captureBuff.fill(1337u);
	enableADC_DMA(adc_pin);
}

template<size_t N>
InfraLoc<N>::~InfraLoc()
{
	
}

template<size_t N>
void InfraLoc<N>::startSampling()
{
	#if defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_ARCH_RP2040)
	adc_run(true);
	
	dma_channel_config cfg = dma_get_channel_config(this->dma_chan_used);

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
	//delayMicroseconds(2);
}

template<size_t N>
number_t InfraLoc<N>::getFrequencyComponent(const float k)
{
	return euclideanDistance(goertzelAlgorithm(this->captureBuff.data(), this->captureBuff.size(), k));
}

template<size_t N>
void InfraLoc<N>::update()
{
	// Fill the sample buffer as fast as possible
	for(uint8_t c = 0; c<INFRALOC_NUM_CHANNELS; c++)
	{
		// Wait until all samples are collected
		this->isSampleBufferFilledBlocking();

		// Stop adc data collection and select the next channel
		stopSampling();
		switchChannels(c);

		// Pull all the desired frequencies
		this->results[c] = euclideanDistance(goertzelAlgorithm(this->captureBuff.data(), N, this->k));

		this->captureBuff.fill(1337);
		// Start the next
		startSampling();
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

template<size_t N>
void InfraLoc<N>::enableADC_DMA(const uint8_t adc_channel)
{
	#if defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_ARCH_RP2040)
	// https://github.com/raspberrypi/pico-examples/blob/master/adc/dma_capture/dma_capture.c
	adc_gpio_init((uint8_t) adc_channel - FIRST_ADC_PIN);

	adc_init();
	adc_select_input(adc_channel);
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
	adc_set_clkdiv(240); // ? / 48000000 = 1/200000

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

template class InfraLoc<512>;
