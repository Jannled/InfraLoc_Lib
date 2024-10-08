#include "mymath.hpp"
#include <array>

number_t sum(number_t values[], const size_t n)
{
	int val = 0;
	
	for(size_t i=0; i<n; i++)
		val += values[i];

	return val;
}

cmplx_t sum(cmplx_t values[], const size_t n)
{
	cmplx_t val = {0};
	
	for(size_t i=0; i<n; i++)
	{
		val.re += values[i].re;
		val.im += values[i].im;
	}

	return val;
}

number_t manhattanDistance(const cmplx_t value)
{
	return value.re + value.im;
}

number_t euclideanDistance(const cmplx_t value)
{
	return sqrt(value.re*value.re + value.im*value.im);
}

number_t magnitude(const vec2 value)
{
	return sqrt(value.x*value.x + value.y*value.y);
}

number_t vec_angle(const vec2 vec_ba, const vec2 vec_bc)
{
	// https://muthu.co/using-the-law-of-cosines-and-vector-dot-product-formula-to-find-the-angle-between-three-points/
	// or any other textbook on math really
	const number_t nom = vec_ba.x*vec_bc.x + vec_ba.y*vec_bc.y;
	const number_t denom = (magnitude(vec_bc)*magnitude(vec_ba));
	return acos(nom/denom);
}

cmplx_t cachedFourierComponent(number_t data[], const size_t n, const number_t k, number_t sinCache[], number_t cosCache[])
{
	cmplx_t big_x = {0};

	for(size_t i=0; i<n; i++)
	{
		big_x.re += data[i] * cosCache[i];
		big_x.im += data[i] * sinCache[i];
	}

	return big_x;
}

cmplx_t cachedFourierComponent(uint16_t data[], const size_t n, const number_t k, number_t sinCache[], number_t cosCache[])
{
	cmplx_t big_x = {0};

	for(size_t i=0; i<n; i++)
	{
		big_x.re += data[i] * cosCache[i];
		big_x.im += data[i] * sinCache[i];
	}

	return big_x;
}

number_t cachedFourierDC(number_t data[], const size_t n, const number_t k, number_t cosCacheZero[])
{
	number_t dcOffset = 0.0;

	for(size_t i=0; i<n; i++)
		dcOffset += data[i] * cosCacheZero[i];

	return dcOffset;
}

void generateFourierCacheSin(number_t cache[], const size_t n, const number_t k)
{
	for(size_t i=0; i<n; i++)
		cache[i] = sin((-2*M_PI*k*i) / n);
}

void generateFourierCacheCos(number_t cache[], const size_t n, const number_t k)
{
	for(size_t i=0; i<n; i++)
		cache[i] = cos((-2*M_PI*k*i) / n);
}

cmplx_t fourierComponent(number_t values[], const size_t n, const number_t k)
{
	cmplx_t big_x = {0};

	for(size_t i=0; i<n; i++)
	{
		big_x.re += values[i] * cos((-2*M_PI*k*i) / n);
		big_x.im += values[i] * sin((-2*M_PI*k*i) / n);
	}

	return big_x;
}

number_t* noWindow(const uint16_t input[], const uint16_t big_n, number_t output[])
{
	for(size_t i=0; i<big_n; i++)
		output[i] = input[i];

	return output;
}

number_t* bartlettWindow(const uint16_t input[], const uint16_t big_n, number_t output[])
{
	output[0] = 0;
	output[big_n - 1] = 0;

	for(size_t i=1; i<big_n/2u; i++)
	{
		const size_t b = big_n - 1 - i;		// Index from the back of the array
		const number_t bf = 2.0f*i/big_n;	// Bartlett factor
		output[i] = input[i] * bf;		 	// Attack from front
		output[b] = input[b] * bf; 			// Mirrored attack from back
	}
	
	return output;
}

number_t* hammingWindow(const uint16_t input[], const uint16_t big_n, number_t output[])
{
	for(size_t i=0; i<big_n; i++)
		output[i] = input[i]*(0.54 - 0.46 * cos((2*M_PI*i)/big_n));
	return output;
}

number_t* blackmanWindow(const uint16_t input[], const uint16_t big_n, number_t output[])
{
	for(size_t i=0; i<big_n; i++)
		output[i] = input[i]*(0.42 - 0.5*cos((2*M_PI*i)/big_n) + 0.08*cos((4*M_PI*i)/big_n));
	return output;
}

cmplx_t generateGoertzelCache(const unsigned int big_n, const unsigned int k)
{
	const number_t W_re = 2 * cos(2 * M_PI * k / big_n);
	const number_t W_im = sin(2 * M_PI * k / big_n);
	return {W_re, W_im};
}

cmplx_t cachedGoertzelAlgorithm(uint16_t *values, const unsigned int big_n, const unsigned int k, const cmplx_t cache)
{
	number_t d1 = 0;
	number_t d2 = 0;

	for(size_t n=0; n<big_n; n++)
	{
		number_t y = values[n] + cache.re*d1 - d2;
		d2 = d1;
		d1 = y;
	}

	const cmplx_t res = {0.5f*cache.re*d1 - d2, cache.im*d1};
	return res;
}

cmplx_t goertzelAlgorithm(number_t* values, const unsigned int big_n, const unsigned int k)
{
	// https://www.mstarlabs.com/dsp/goertzel/goertzel.html
	const number_t W_re = 2 * cos(2 * M_PI * k / big_n);
	const number_t W_im = sin(2 * M_PI * k / big_n);

	number_t d1 = 0;
	number_t d2 = 0;

	for(size_t n=0; n<big_n; n++)
	{
		const number_t y = values[n] + W_re*d1 - d2;
		d2 = d1;
		d1 = y;
	}

	const cmplx_t res = {0.5f*W_re*d1 - d2, W_im*d1};
	return res;
}

number_t calculateFrequencyCenter(const size_t sampleCount, const number_t sampleFreq, const number_t k)
{
	return (sampleFreq * k) / sampleCount;
}

number_t calculateK(const size_t sampleCount, const number_t sampleFreq, const number_t targetFreq)
{
	return (targetFreq * sampleCount) / sampleFreq;
}

vec2 tienstraMethod(
		const vec2 pos_a, const vec2 pos_b, const vec2 pos_c, 
		const number_t alpha, const number_t beta, const number_t gamma, 
		const number_t ang_a, const number_t ang_b, const number_t ang_c
	)
{
	const number_t w1 = (sin(alpha)*sin(ang_a)) / sin(alpha - ang_a);
	const number_t w2 = (sin(beta)*sin(ang_b)) / sin(beta - ang_b);
	const number_t w3 = (sin(gamma)*sin(ang_c)) / sin(gamma - ang_c);
	const number_t w_denom = w1 + w2 + w3;
	
	vec2 pos_p = {0};
	pos_p.x = (w1*pos_a.x + w2*pos_b.x + w3*pos_c.x) / w_denom;
	pos_p.y = (w1*pos_a.y + w2*pos_b.y + w3*pos_c.y) / w_denom;

	return pos_p;
}
