#include "mymath.hpp"

number_t sum(number_t values[], size_t n)
{
	int val = 0;
	
	for(size_t i=0; i<n; i++)
		val += values[i];

	return val;
}

cmplx_t sum(cmplx_t values[], size_t n)
{
	cmplx_t val = {0};
	
	for(size_t i=0; i<n; i++)
	{
		val.re += values[i].re;
		val.im += values[i].im;
	}

	return val;
}

number_t manhattanDistance(cmplx_t value)
{
	return value.re + value.im;
}

number_t euclideanDistance(cmplx_t value)
{
	return sqrt(value.re*value.re + value.im*value.im);
}

cmplx_t cachedFourierComponent(number_t data[], size_t n, number_t k, number_t sinCache[], number_t cosCache[])
{
	cmplx_t big_x = {0};

	for(size_t i=0; i<n; i++)
	{
		big_x.re += data[i] * cosCache[i];
		big_x.im += data[i] * sinCache[i];
	}

	return big_x;
}

number_t cachedFourierDC(number_t data[], size_t n, number_t k, number_t cosCacheZero[])
{
	number_t dcOffset = 0.0;

	for(size_t i=0; i<n; i++)
		dcOffset += data[i] * cosCacheZero[i];

	return dcOffset;
}

void generateFourierCacheSin(number_t cache[], size_t n, number_t k)
{
	for(size_t i=0; i<n; i++)
		cache[i] = sin((-2*MY_PI*k*i) / n);
}

void generateFourierCacheCos(number_t cache[], size_t n, number_t k)
{
	for(size_t i=0; i<n; i++)
		cache[i] = cos((-2*MY_PI*k*i) / n);
}

cmplx_t fourierComponent(number_t values[], size_t n, number_t k)
{
	cmplx_t big_x = {0};

	for(size_t i=0; i<n; i++)
	{
		big_x.re += values[i] * cos((-2*MY_PI*k*i) / n);
		big_x.im += values[i] * sin((-2*MY_PI*k*i) / n);
	}

	return big_x;
}

number_t* hammingWindow(number_t* values, size_t N)
{
	for(size_t i=0; i<N; i++)
		values[i] *= 0.54 - 0.46 * cos((2*MY_PI*i)/N);
	return values;
}

number_t* blackmanWindow(number_t* values, size_t N)
{
	for(size_t i=0; i<N; i++)
		values[i] *= 0.42 - 0.5 * cos((2*MY_PI*i)/N) + 0.08 * cos((4*MY_PI*i)/N);
	return values;
}

cmplx_t goertzelAlgorithm(uint16_t* values, unsigned int N, unsigned int k)
{
	// https://www.mstarlabs.com/dsp/goertzel/goertzel.html
	number_t W_re = 2 * cos(2 * MY_PI * k / N);
	number_t W_im = sin(2 * MY_PI * k / N);

	number_t d1 = 0;
	number_t d2 = 0;

	for(size_t n=0; n<N; n++)
	{
		number_t y = values[n] + W_re*d1 - d2;
		d2 = d1;
		d1 = y;
	}

	const cmplx_t res = {0.5f*W_re*d1 - d2, W_im*d1};
	return res;
}

number_t calculateFrequencyCenter(size_t sampleCount, number_t sampleFreq, number_t k)
{
	return (sampleFreq * k) / sampleCount;
}

number_t calculateK(size_t sampleCount, number_t sampleFreq, number_t targetFreq)
{
	return (targetFreq * sampleCount) / sampleFreq;
}
