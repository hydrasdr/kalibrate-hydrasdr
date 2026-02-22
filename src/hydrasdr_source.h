/**
 * @file hydrasdr_source.h
 * @brief SDR source interface for HydraSDR RFOne hardware.
 *
 * This module provides a high-level C++ interface for receiving samples from
 * HydraSDR RFOne hardware. It handles device initialization, tuning, gain
 * control, and integrates a two-stage DSP resampling pipeline to convert
 * the native 2.5 MSPS sample rate to GSM-compatible 270.833 kSPS.
 *
 * @section Architecture
 *
 * @code
 *  ┌─────────────┐    ┌──────────────┐     ┌────────────────┐     ┌──────────┐
 *  │  HydraSDR   │───▶│  USB Thread  │────▶│  DSP Pipeline  │───▶│ Circular │
 *  │  Hardware   │    │  (Callback)  │     │  (Resampler)   │     │  Buffer  │
 *  │  2.5 MSPS   │    │              │     │  270.833 kSPS  │     │          │
 *  └─────────────┘    └──────────────┘     └────────────────┘     └────┬─────┘
 *                                                                      │
 *                                                                      ▼
 *                                                               ┌──────────────┐
 *                                                               │  Main Thread │
 *                                                               │  (Consumer)  │
 *                                                               └──────────────┘
 * @endcode
 *
 * @section Threading Model
 *
 * - **USB Thread**: Invoked by HydraSDR driver via callback, runs DSP pipeline
 * - **Main Thread**: Consumes processed samples via fill() method
 * - **Synchronization**: std::mutex + std::condition_variable for thread-safe handoff
 *
 * @author Benjamin Vernoux <bvernoux@hydrasdr.com>
 * @copyright 2025 Benjamin Vernoux
 * @license BSD-2-Clause
 */

/*
 * Copyright 2025 Benjamin Vernoux <bvernoux@hydrasdr.com>
 * SPDX-License-Identifier: BSD-2-Clause
 */

#ifndef __HYDRASDR_SOURCE_H__
#define __HYDRASDR_SOURCE_H__

#include <vector>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include "circular_buffer.h"
#include "kal_types.h"
#include "dsp_resampler.h"
#include <hydrasdr.h>

/**
 * @brief Native sample rate of HydraSDR RFOne hardware (Hz).
 *
 * The hardware operates at 2.5 MSPS, which is then resampled by the
 * DSP pipeline to produce the GSM-compatible 270.833 kSPS output rate.
 */
#define HYDRASDR_2_5MSPS_NATIVE_RATE 2500000

/**
 * @class hydrasdr_source
 * @brief High-level SDR source for HydraSDR RFOne with integrated DSP resampling.
 *
 * This class encapsulates all hardware interaction and signal processing
 * required to receive and condition RF samples for GSM analysis. It manages:
 *
 * - Device lifecycle (open/close)
 * - RF front-end configuration (frequency, gain)
 * - Asynchronous sample streaming with USB callbacks
 * - Two-stage rational resampling (2.5 MSPS → 270.833 kSPS)
 * - Thread-safe sample buffering with overflow detection
 *
 * @par Example Usage:
 * @code
 *     hydrasdr_source src(10.0f);  // Initial gain
 *     src.open();
 *     src.tune(935.2e6);           // GSM-900 downlink
 *     src.start();
 *
 *     unsigned int overruns;
 *     while (running) {
 *         src.fill(1024, &overruns);
 *         complex* samples = (complex*)src.get_buffer()->peek(nullptr);
 *         // Process samples...
 *         src.get_buffer()->purge(1024);
 *     }
 *
 *     src.close();
 * @endcode
 */
class hydrasdr_source {
public:
	/**
	 * @brief Constructs a HydraSDR source instance.
	 *
	 * Initializes internal state and creates the DSP resampler pipeline.
	 * Does not open the hardware device - call open() separately.
	 *
	 * @param gain Initial RF gain value (0-21, linearity mode).
	 */
	hydrasdr_source(float gain);

	/**
	 * @brief Destructor - releases all resources.
	 *
	 * Automatically calls close() to stop streaming and release
	 * the hardware device if still open.
	 */
	~hydrasdr_source();

	/**
	 * @brief Opens and initializes the HydraSDR hardware.
	 *
	 * Performs the following initialization sequence:
	 * 1. Opens the first available HydraSDR device
	 * 2. Configures Float32 I/Q sample format
	 * 3. Sets native sample rate (2.5 MSPS)
	 * 4. Applies initial gain setting
	 * 5. Allocates circular buffer for sample handoff
	 *
	 * @return 0 on success, -1 on failure (error printed to stderr).
	 */
	int open();

	/**
	 * @brief Tunes the RF front-end to the specified frequency.
	 *
	 * Also resets the DSP filter state to prevent transients from
	 * the previous frequency from contaminating the new signal.
	 *
	 * @param freq Center frequency in Hz (e.g., 935.2e6 for GSM-900).
	 * @return 0 on success, -1 on failure.
	 */
	int tune(double freq);

	/**
	 * @brief Sets the RF front-end gain.
	 *
	 * Uses linearity gain mode for optimal dynamic range.
	 * Value is clamped to valid range [0, 21].
	 *
	 * @param gain Gain index (0-21, higher = more gain).
	 * @return 0 on success, -1 on failure.
	 */
	int set_gain(float gain);

	/**
	 * @brief Starts asynchronous sample streaming.
	 *
	 * Registers the internal callback with the HydraSDR driver and
	 * begins receiving samples. Samples are processed through the
	 * DSP pipeline and deposited into the circular buffer.
	 *
	 * @return 0 on success, -1 on failure.
	 */
	int start();

	/**
	 * @brief Stops sample streaming.
	 *
	 * Halts the USB transfer and notifies any threads waiting
	 * in fill() to allow graceful shutdown.
	 *
	 * @return 0 on success.
	 */
	int stop();

	/**
	 * @brief Closes the device and releases all resources.
	 *
	 * Calls stop() if still streaming, then releases the device
	 * handle and frees the circular buffer.
	 *
	 * @return 0 on success.
	 */
	int close();

	/**
	 * @brief Initializes the source for benchmark mode (no hardware).
	 *
	 * Creates the circular buffer and resampler without opening
	 * actual hardware. Used by run_dsp_benchmark() to test DSP
	 * pipeline performance with synthetic data.
	 *
	 * @see run_dsp_benchmark()
	 */
	void start_benchmark();

	/**
	 * @brief Returns the output sample rate after resampling.
	 * @return Sample rate in Hz (270833.333... for GSM).
	 */
	inline double sample_rate() { return m_sample_rate; }

	/**
	 * @brief Returns a pointer to the internal circular buffer.
	 *
	 * Used to access processed samples via peek()/read()/purge().
	 *
	 * @return Pointer to circular_buffer instance.
	 */
	inline circular_buffer* get_buffer() { return cb; }

	/**
	 * @brief Blocks until the requested number of samples are available.
	 *
	 * This is the primary method for the consumer thread to wait for
	 * new data. It uses a condition variable with 100ms timeout to
	 * periodically check the global exit flag (g_kal_exit_req).
	 *
	 * @param num_samples Minimum number of samples to wait for.
	 * @param overruns    Output: number of samples dropped since last call
	 *                    due to buffer overflow (can be NULL).
	 * @return 0 on success, -1 if streaming stopped or exit requested.
	 */
	int fill(unsigned int num_samples, unsigned int *overruns);

	/**
	 * @brief Discards all buffered samples and resets overflow counter.
	 * @return 0 on success.
	 */
	int flush();

	/** @brief Current center frequency in Hz. */
	double m_center_freq;

	/** @brief Frequency correction in PPM (reserved for future use). */
	int m_freq_corr;

	/**
	 * @brief USB callback entry point for processing incoming samples.
	 *
	 * Called by the HydraSDR driver from the USB thread context.
	 * Processes samples through the DSP pipeline and writes results
	 * to the circular buffer.
	 *
	 * @param transfer Pointer to transfer structure containing samples.
	 * @return 0 to continue streaming.
	 *
	 * @warning Called from USB thread - must be thread-safe.
	 */
	int fill_buffer_callback(hydrasdr_transfer_t* transfer);

private:
	/** @brief HydraSDR device handle. */
	hydrasdr_device* dev;

	/** @brief Circular buffer for producer/consumer sample handoff. */
	circular_buffer* cb;

	/** @brief Condition variable for consumer thread synchronization. */
	std::condition_variable data_ready;

	/** @brief Mutex protecting buffer access coordination. */
	std::mutex data_mutex;

	/**
	 * @brief Atomic streaming state flag.
	 *
	 * Using std::atomic<bool> to avoid data race between USB callback
	 * thread (reader) and control methods (writer). This eliminates
	 * undefined behavior that would occur with a plain bool.
	 */
	std::atomic<bool> streaming;

	/** @brief Current RF gain setting. */
	float m_gain;

	/** @brief Output sample rate after resampling (Hz). */
	double m_sample_rate;

	/** @brief Atomic overflow counter (samples dropped). */
	std::atomic<unsigned int> m_overflow_count;

	/** @brief DSP resampler instance (2.5 MSPS → 270.833 kSPS). */
	dsp_resampler* m_resampler;

	/**
	 * @brief Size of intermediate batch buffer for DSP output.
	 *
	 * Calculation for maximum USB transfer of 128K samples:
	 *   Input:  131072 samples (128K, conservative upper bound)
	 *   Stage1: 131072 / 5 = 26214 samples
	 *   Stage2: 26214 * 13/24 ≈ 14200 samples
	 *
	 * BATCH_SIZE = 32768 provides 2x safety margin.
	 *
	 * @note If libhydrasdr ever provides larger transfers, this must
	 *       be increased proportionally to avoid data loss.
	 */
	static const int BATCH_SIZE = 32768;

	/** @brief Intermediate buffer for DSP pipeline output. */
	std::complex<float> m_batch_buffer[BATCH_SIZE];
};

#endif /* __HYDRASDR_SOURCE_H__ */
