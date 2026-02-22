/**
 * @file kal_types.h
 * @brief Shared types and constants for kalibrate-hydrasdr.
 *
 * Central definitions for:
 * - complex typedef (std::complex<float>)
 * - GSM_RATE constant
 * - FCCH_OFFSET_MAX constant
 * - M_PI fallback
 * - vectornorm2() template function
 *
 * @author Benjamin Vernoux <bvernoux@hydrasdr.com>
 * @copyright 2025 Benjamin Vernoux
 * @license BSD-2-Clause
 */

#ifndef KAL_TYPES_H
#define KAL_TYPES_H

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <complex>
#include <cmath>

/** @brief Convenience typedef for complex float samples (I/Q pairs). */
typedef std::complex<float> complex;

/** @brief GSM symbol rate (Hz): 1625000/6 = 270833.333 symbols/sec. */
static constexpr double GSM_RATE = 1625000.0 / 6.0;

/** @brief Maximum acceptable FCCH frequency offset (Hz). */
static constexpr float FCCH_OFFSET_MAX = 40e3f;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @brief Computes sum of squared magnitudes of a complex vector.
 * @tparam T Return type (float or double).
 * @param v   Pointer to complex sample array.
 * @param len Number of elements.
 * @return Sum of |v[i]|^2 for i in [0, len).
 */
template<typename T>
static inline T vectornorm2(const complex *v, const unsigned int len)
{
	unsigned int i;
	T e = T(0);
	for (i = 0; i < len; i++)
		e += std::norm(v[i]);
	return e;
}

#endif /* KAL_TYPES_H */
