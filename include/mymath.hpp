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

typedef struct
{
	number_t x;
	number_t y;
} vec2;

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
cmplx_t fourierComponent(number_t values[], const size_t N, const number_t k);

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

cmplx_t cachedFourierComponent(number_t data[], const size_t N, const number_t k, number_t sinCache[], number_t cosCache[]);

cmplx_t cachedFourierComponent(uint16_t data[], const size_t N, const number_t k, number_t sinCache[], number_t cosCache[]);

number_t cachedFourierDC(number_t data[], size_t n, number_t k, number_t cosCacheZero[]);

void generateFourierCacheSin(number_t cache[], size_t n, number_t k);

void generateFourierCacheCos(number_t cache[], size_t n, number_t k);

/**
 * @brief Stupid converter to convert the ADC integer values to floats without applying a windowing function
 * 
 * @param input 
 * @param big_n 
 * @param output 
 * @return number_t* 
*/
number_t* noWindow(const uint16_t input[], const uint16_t big_n, number_t output[]);

/**
 * @brief Optimized version of a Bartlett Window by promoting 16bit numbers to 32bit fixed point
 * // Normally the order of dot operations does not matter, but due to integer arithmetic, more information would be lost otherwise
 * @param values 
 * @param big_n 
 * @return 
 */
number_t* bartlettWindow(const uint16_t input[], const uint16_t big_n, number_t output[]);

/**
 * @brief Hamming Window
 * 
 * Note: constexpr functions with loops or generally anything with more than one line need C++14
 * 
 * @param values 
 * @param N 
 * @return constexpr number_t* 
 */
number_t* hammingWindow(number_t* values, const size_t big_n);

/**
 * @brief Blackman Window
 * 
 * Note: constexpr functions with loops or generally anything with more than one line need C++14
 * 
 * @param values Pointer to an array that will hold the calculated factors 
 * @param N length of the values
 * @return constexpr number_t* 
 */
number_t* blackmanWindow(number_t* values, const size_t big_n);

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
cmplx_t goertzelAlgorithm(number_t* values, const unsigned int big_n, const unsigned int k);

cmplx_t generateGoertzelCache(const unsigned int big_n, const unsigned int k);

cmplx_t cachedGoertzelAlgorithm(uint16_t *values, const unsigned int big_n, const unsigned int k, cmplx_t cache);

/**
 * @brief Return the center frequency of the selected FFT bin.
 * @param sampleCount The number of discrete samples collected from the signal.
 * @param sampleFreq The frequency in hertz at which the signal was sampled.
 * @param k The FFT frequency bin. Should be in range from `0` to `sampleCount/2`.
 * @return The center frequency in Hertz.
 */
number_t calculateFrequencyCenter(const size_t sampleCount, const number_t sampleFreq, const number_t k);

/**
 * @brief 
 * @param sampleCount 
 * @param sampleFreq 
 * @param targetFreq 
 * @return 
 */
number_t calculateK(const size_t sampleCount, const number_t sampleFreq, const number_t targetFreq);

/**
 * @brief 
 * 
 * @param magnitudes 
 * @param length 
 * @return number_t 
 */
number_t calculateDirection(number_t *magnitudes, size_t length);

/**
 * @brief Planar resection with the Tienstra method (triangulation but the angles are measured from the unknown point)
 * 
 * @param pos_a Point A
 * @param pos_b Point B
 * @param pos_c Point C
 * @param alpha Angle between BC in Point P
 * @param beta Angle between AC in Point P
 * @param gamma Angle between AB in Point P
 * @param ang_a Angle of BC in Point A
 * @param ang_b Angle of AC in Point B
 * @param ang_c Angle of AB in Point C
 * @return vec2 The position of the unknown point P
 */
vec2 tienstraMethod(
		const vec2 pos_a, const vec2 pos_b, const vec2 pos_c, 
		const number_t alpha, const number_t beta, const number_t gamma, 
		const number_t ang_a, const number_t ang_b, const number_t ang_c
	);

#endif // MY_MATH_H
