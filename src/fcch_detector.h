/**
 * @file fcch_detector.h
 * @brief Frequency Correction Channel (FCCH) Detector for GSM.
 *
 * The FCCH is a pure sinusoid at GSM_RATE/4 (≈67.7 kHz) transmitted by
 * GSM base stations. This detector uses adaptive filtering to locate
 * FCCH bursts and measure their frequency offset from the expected value.
 *
 * @author Joshua Lackey (original)
 * @author Benjamin Vernoux <bvernoux@hydrasdr.com> (improvements)
 * @copyright 2010 Joshua Lackey, 2025 Benjamin Vernoux
 * @license BSD-2-Clause
 */

#ifndef __FCCH_DETECTOR_H__
#define __FCCH_DETECTOR_H__

#include <fftw3.h>
#include "circular_buffer.h"
#include "kal_types.h"

/** @brief FFT size for frequency detection. */
#define FFT_SIZE 1024

/**
 * @class fcch_detector
 * @brief Detects GSM Frequency Correction Channel bursts.
 *
 * Uses a Normalized LMS adaptive filter to identify regions of pure tone
 * (low prediction error), then FFT-based peak detection to measure the
 * exact frequency offset.
 */
class fcch_detector {
public:
	/**
	 * @brief Constructs an FCCH detector.
	 * @param sample_rate Input sample rate in Hz.
	 * @param D           Adaptive filter prediction delay (default 4).
	 * @param p           Error averaging coefficient (default 0.25).
	 * @param G           Initial adaptive filter gain (default 1.0).
	 */
	fcch_detector(const float sample_rate, const unsigned int D = 4,
		      const float p = 1.0f / 4.0f, const float G = 1.0f);
	~fcch_detector();

	/**
	 * @brief Scans input buffer for FCCH burst.
	 * @param s        Input sample buffer.
	 * @param s_len    Number of samples in buffer.
	 * @param offset   Output: detected frequency offset (Hz).
	 * @param consumed Output: number of samples consumed (may be NULL).
	 * @return 1 if FCCH found, 0 otherwise.
	 */
	unsigned int scan(const complex *s, const unsigned int s_len,
			  float *offset, unsigned int *consumed);

	/**
	 * @brief Updates internal buffers with new samples.
	 * @param s     Input sample buffer.
	 * @param s_len Number of samples.
	 * @return Number of samples written.
	 */
	unsigned int update(const complex *s, const unsigned int s_len);

	/**
	 * @brief Detects frequency of pure tone using FFT.
	 * @param s     Input sample buffer.
	 * @param s_len Number of samples.
	 * @param pm    Output: peak-to-mean ratio (quality metric).
	 * @return Detected frequency in Hz.
	 */
	float freq_detect(const complex *s, const unsigned int s_len, float *pm);

	/**
	 * @brief Computes next normalized prediction error sample.
	 * @param error Output: normalized error value.
	 * @return 0 on success, >0 if more samples needed.
	 */
	int next_norm_error(float *error);

	/** @brief Returns adaptive filter delay. */
	unsigned int get_delay();

	/** @brief Returns adaptive filter length. */
	unsigned int filter_len();

	/* Debug helpers */
	complex *dump_x(unsigned int *x_len);
	complex *dump_y(unsigned int *y_len);
	unsigned int y_buf_len();
	unsigned int x_buf_len();
	unsigned int x_purge(unsigned int len);

private:
	/* Adaptive filter parameters */
	unsigned int m_D;         /**< Prediction delay */
	float m_p;                /**< Error averaging coefficient */
	float m_G;                /**< Adaptive gain */
	float m_e;                /**< Running error average */
	float m_sample_rate;      /**< Input sample rate (Hz) */
	unsigned int m_fcch_burst_len;  /**< Expected FCCH burst length (samples) */

	/* Adaptive filter state */
	unsigned int m_filter_delay;
	unsigned int m_w_len;
	complex *m_w;

	/* Internal buffers */
	circular_buffer *m_x_cb;  /**< Input sample buffer */
	circular_buffer *m_y_cb;  /**< Filtered output buffer */
	circular_buffer *m_e_cb;  /**< Error signal buffer */

	/* FFTW resources */
	fftw_complex *m_in;
	fftw_complex *m_out;
	fftw_plan m_plan;

	/*
	 * State machine for low_to_high edge detection.
	 * These are instance variables (not static) to support multiple
	 * concurrent fcch_detector instances.
	 */
	unsigned int m_lth_count;  /**< Sample counter for edge detection */
	unsigned int m_lth_state;  /**< Current state (LOW or HIGH) */

	/** @brief Resets edge detection state machine. */
	void low_to_high_init();

	/**
	 * @brief Detects low-to-high transition in error signal.
	 * @param e Current error value.
	 * @param a Threshold.
	 * @return Length of previous low region (0 if no transition).
	 */
	unsigned int low_to_high(float e, float a);
};

#endif /* __FCCH_DETECTOR_H__ */
