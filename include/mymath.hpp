#ifndef MY_MATH_H
#define MY_MATH_H

#include <cmath>
#include <stdint.h>

#ifdef MATH_DOUBLE_PRECISION
typedef double number_t;
#else
typedef float number_t;
#endif

constexpr double MY_PI = 4272943.0/1360120.0;

typedef struct
{
	number_t re;
	number_t im;
} cmplx_t;

// https://stackoverflow.com/a/34465458
template<int N, int k>
struct SinCache {
	explicit constexpr SinCache(): lut() {
		for(int i=0; i<N; i++)
			lut[i] = sin((-2*MY_PI*k*i) / N);
	}
	number_t lut[N];
};

template<int N, int k>
struct CosCache {
	explicit constexpr CosCache(): lut() {
		for(int i=0; i<N; i++)
			lut[i] = cos((-2*MY_PI*k*i) / N);
	}
	number_t lut[N];
};

number_t sum(number_t values[], size_t n);

cmplx_t sum(cmplx_t values[], size_t n);

number_t manhattanDistance(cmplx_t value);

number_t euclideanDistance(cmplx_t value);

/**
 * @brief Calculate the frequency bin k of the discrete Fourier transformation
 * 
 * @param values 
 * @param n N The number of samples
 * @param k The desired frequency (Calculated as P(Full)/P(Event))
 * @return X
 */
cmplx_t fourierComponent(number_t values[], size_t n, number_t k);

/**
 * @brief 
 * 
 * @tparam n 
 * @tparam k 
 * @param values 
 * @return cmplx_t 
 */
template<int n, int k>
cmplx_t compiledFourierComponent(const number_t values[])
{
	constexpr auto fourierCache_cos = CosCache<n, k>();
	constexpr auto fourierCache_sin = SinCache<n, k>();

	static_assert(abs(MY_PI - 3.141) < 0.001);

	// Sanity check the correct generation of the LUT
	static_assert(abs(cos(-2*MY_PI*k*16 / n) - fourierCache_cos.lut[16]) < 0.0000000000000001);
	static_assert(abs(sin(-2*MY_PI*k*16 / n) - fourierCache_sin.lut[16]) < 0.0000000000000001);

	cmplx_t big_x = {0};

	for(size_t i=0; i<n; i++)
	{
		big_x.re += values[i] * fourierCache_cos.lut[i];
		big_x.im += values[i] * fourierCache_sin.lut[i];
	}

	return big_x;
}

/**
 * @brief Calculate the DC offset for the given values (basically k=0). 
 * 
 * The method will generate the factors for the fourier transform at compile.
 * 
 * @tparam n 
 * @param values 
 * @return number_t 
 */
template<int n>
number_t compiledFourierDC(const number_t values[])
{
	static constexpr auto fourierCache_cos = CosCache<n, 0>();

	number_t dcOffset = 0.0;

	for(size_t i=0; i<n; i++)
		dcOffset += values[i] * fourierCache_cos.lut[i];

	return dcOffset;
}

cmplx_t cachedFourierComponent(number_t data[], size_t n, number_t k, number_t sinCache[], number_t cosCache[]);

cmplx_t cachedFourierComponent(uint16_t data[], size_t n, number_t k, number_t sinCache[], number_t cosCache[]);

number_t cachedFourierDC(number_t data[], size_t n, number_t k, number_t cosCacheZero[]);

void generateFourierCacheSin(number_t cache[], size_t n, number_t k);

void generateFourierCacheCos(number_t cache[], size_t n, number_t k);

/**
 * @brief Hamming Window
 * 
 * Note: constexpr functions with loops or generally anything with more than one line need C++14
 * 
 * @param values 
 * @param N 
 * @return constexpr number_t* 
 */
number_t* hammingWindow(number_t* values, size_t N);

/**
 * @brief Blackman Window
 * 
 * Note: constexpr functions with loops or generally anything with more than one line need C++14
 * 
 * @param values Pointer to an array that will hold the calculated factors 
 * @param N length of the values
 * @return constexpr number_t* 
 */
number_t* blackmanWindow(number_t* values, size_t N);

/**
 * @brief Goertzel Algorithm
 * Implementation by Microstar Laboratories
 * https://www.mstarlabs.com/dsp/goertzel/goertzel.html
 * 
 * Referencing the following sources
 * https://gist.github.com/sebpiq/4128537
 * https://web.archive.org/web/20121113163511/http://www.embedded.com:80/Home/PrintView?contentItemId=4024443
 * https://www.embedded.com/the-goertzel-algorithm/
*/
cmplx_t goertzelAlgorithm(uint16_t* values, unsigned int N, unsigned int k);

cmplx_t generateGoertzelCache(const unsigned int N, const unsigned int k);

cmplx_t cachedGoertzelAlgorithm(uint16_t *values, unsigned int N, unsigned int k, cmplx_t cache);

/**
 * @brief Return the center frequency of the selected FFT bin.
 * @param sampleCount The number of discrete samples collected from the signal.
 * @param sampleFreq The frequency in hertz at which the signal was sampled.
 * @param k The FFT frequency bin. Should be in range from `0` to `sampleCount/2`.
 * @return The center frequency in Hertz.
 */
number_t calculateFrequencyCenter(size_t sampleCount, number_t sampleFreq, number_t k);

/**
 * @brief 
 * @param sampleCount 
 * @param sampleFreq 
 * @param targetFreq 
 * @return 
 */
number_t calculateK(size_t sampleCount, number_t sampleFreq, number_t targetFreq);

#endif // MY_MATH_H
