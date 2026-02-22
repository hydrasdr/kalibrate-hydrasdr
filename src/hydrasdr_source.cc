/**
 * @file hydrasdr_source.cc
 * @brief Implementation of HydraSDR RFOne source with DSP resampling pipeline.
 *
 * This file implements the hydrasdr_source class, providing:
 * - Hardware initialization and configuration via libhydrasdr
 * - Asynchronous USB sample reception with callback handling
 * - Integrated two-stage DSP resampling (2.5 MSPS → 270.833 kSPS)
 * - Thread-safe producer/consumer buffering
 *
 * @section DSP Pipeline
 *
 * The resampling pipeline converts the native hardware rate to GSM symbol rate:
 *
 * @code
 *   2,500,000 Hz ─▶ [Stage 1: ÷5] ─▶ 500,000 Hz ─▶ [Stage 2: ×13/24] ─▶ 270,833.333 Hz
 *                   (61-tap LPF)                   (729-tap Polyphase)
 * @endcode
 *
 * @see dsp_resampler for filter coefficient details.
 *
 * @author Benjamin Vernoux <bvernoux@hydrasdr.com>
 * @copyright 2025 Benjamin Vernoux
 * @license BSD-2-Clause
 */

/*
 * Copyright 2025 Benjamin Vernoux <bvernoux@hydrasdr.com>
 * SPDX-License-Identifier: BSD-2-Clause
 */

#ifdef _WIN32
#include "win_compat.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#ifndef _MSC_VER
#include <unistd.h>
#endif
#include <string.h>
#include <errno.h>
#include <math.h>
#include <algorithm>
#include <iterator>

#include "hydrasdr_source.h"
#include "kal_globals.h"

/**
 * @brief Maximum linearity gain index supported by hardware.
 *
 * Valid gain range is [0, LINEARITY_GAIN_MAX] where higher values
 * provide more RF gain but may reduce linearity.
 */
#define LINEARITY_GAIN_MAX 21

/**
 * @brief Clamps a value to specified bounds.
 * @param x     Value to clamp.
 * @param low   Minimum allowed value.
 * @param high  Maximum allowed value.
 * @return Clamped value in range [low, high].
 */
#ifndef CLAMP
#define CLAMP(x, low, high)  (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#endif

/*
 * ---------------------------------------------------------------------------
 * Static Callback Trampoline
 * ---------------------------------------------------------------------------
 */

/**
 * @brief C-style callback wrapper for HydraSDR driver.
 *
 * The HydraSDR driver C API requires a static function pointer.
 * This trampoline extracts the C++ object pointer from the transfer
 * context and forwards the call to the instance method.
 *
 * @param transfer Pointer to transfer structure from driver.
 * @return Return value from fill_buffer_callback().
 */
static int hydrasdr_callback(hydrasdr_transfer_t* transfer)
{
	hydrasdr_source* obj = (hydrasdr_source*)transfer->ctx;
	return obj->fill_buffer_callback(transfer);
}

/*
 * ---------------------------------------------------------------------------
 * Constructor / Destructor
 * ---------------------------------------------------------------------------
 */

hydrasdr_source::hydrasdr_source(float gain)
{
	m_gain = gain;

	/* Target GSM symbol rate: 13 MHz / 48 = 270833.333... Hz */
	m_sample_rate = 270833.333333;
	m_center_freq = 0.0;
	m_freq_corr = 0;
	m_overflow_count = 0;

	dev = NULL;
	cb = NULL;
	streaming = false;

	/* Initialize DSP resampling pipeline */
	m_resampler = new dsp_resampler();
}

hydrasdr_source::~hydrasdr_source()
{
	close();
	delete m_resampler;
}

/*
 * ---------------------------------------------------------------------------
 * Device Lifecycle
 * ---------------------------------------------------------------------------
 */

/**
 * @brief Opens and initializes the HydraSDR hardware.
 *
 * Uses RAII-style cleanup on failure to prevent resource leaks.
 */
int hydrasdr_source::open(void)
{
	int r;

	/* Open first available HydraSDR device */
	r = hydrasdr_open(&dev);
	if (r != HYDRASDR_SUCCESS) {
		fprintf(stderr, "Failed to open HydraSDR device: %d\n", r);
		return -1;
	}

	/* Configure hardware for Float32 I/Q sample format */
	r = hydrasdr_set_sample_type(dev, HYDRASDR_SAMPLE_FLOAT32_IQ);
	if (r != HYDRASDR_SUCCESS) {
		fprintf(stderr, "Failed to set sample type: %d\n", r);
		goto err_close_dev;
	}

	/* Set native sample rate (2.5 MSPS) */
	r = hydrasdr_set_samplerate(dev, HYDRASDR_2_5MSPS_NATIVE_RATE);
	if (r != HYDRASDR_SUCCESS) {
		fprintf(stderr, "Failed to set sample rate: %d\n", r);
		goto err_close_dev;
	}

	/* Apply initial gain setting */
	if (set_gain(m_gain) != 0) {
		fprintf(stderr, "Failed to set initial gain.\n");
		goto err_close_dev;
	}

	/*
	 * Allocate circular buffer for producer/consumer handoff.
	 * Size: 256K samples provides ~0.9 seconds of buffering at GSM rate.
	 */
	try {
		cb = new circular_buffer(256 * 1024, sizeof(complex));
	} catch (const std::exception& e) {
		fprintf(stderr, "Failed to allocate circular buffer: %s\n", e.what());
		goto err_close_dev;
	}

	return 0;

err_close_dev:
	hydrasdr_close(dev);
	dev = NULL;
	return -1;
}

int hydrasdr_source::close()
{
	stop();

	if (dev) {
		hydrasdr_close(dev);
		dev = NULL;
	}

	if (cb) {
		delete cb;
		cb = NULL;
	}

	return 0;
}

/*
 * ---------------------------------------------------------------------------
 * RF Configuration
 * ---------------------------------------------------------------------------
 */

int hydrasdr_source::tune(double freq)
{
	if (!dev)
		return -1;

	int r = hydrasdr_set_freq(dev, (uint64_t)freq);
	if (r != HYDRASDR_SUCCESS) {
		fprintf(stderr, "Failed to tune to %f Hz: %d\n", freq, r);
		return -1;
	}

	m_center_freq = freq;

	/*
	 * IMPORTANT: Reset filter history when retuning.
	 * This prevents transients from the old frequency from
	 * contaminating the new signal during filter settling.
	 */
	m_resampler->reset();

	return 0;
}

int hydrasdr_source::set_gain(float gain)
{
	if (!dev)
		return -1;

	m_gain = gain;

	/* Convert float gain to integer index, clamped to valid range */
	int gain_val = (int)round(gain);
	gain_val = CLAMP(gain_val, 0, LINEARITY_GAIN_MAX);

	int r = hydrasdr_set_gain(dev, HYDRASDR_GAIN_TYPE_LINEARITY, (uint8_t)gain_val);
	if (r != HYDRASDR_SUCCESS) {
		fprintf(stderr, "Failed to set linearity gain: %d\n", r);
		return -1;
	}

	return 0;
}

/*
 * ---------------------------------------------------------------------------
 * Streaming Control
 * ---------------------------------------------------------------------------
 */

int hydrasdr_source::start()
{
	if (!dev)
		return -1;

	/* Reset DSP state before streaming begins */
	m_resampler->reset();
	m_overflow_count = 0;

	/* Register callback and start USB transfers */
	int r = hydrasdr_start_rx(dev, hydrasdr_callback, (void*)this);
	if (r != HYDRASDR_SUCCESS) {
		fprintf(stderr, "Failed to start RX: %d\n", r);
		return -1;
	}

	/* Set streaming flag (atomic, no lock needed) */
	streaming.store(true, std::memory_order_release);

	return 0;
}

int hydrasdr_source::stop()
{
	if (dev && streaming.load(std::memory_order_acquire)) {
		hydrasdr_stop_rx(dev);

		/* Clear streaming flag (atomic) */
		streaming.store(false, std::memory_order_release);

		/* Wake up any threads waiting in fill() for graceful exit */
		data_ready.notify_all();
	}

	return 0;
}

/*
 * ---------------------------------------------------------------------------
 * Benchmark Mode
 * ---------------------------------------------------------------------------
 */

void hydrasdr_source::start_benchmark()
{
	/* Allocate buffer without opening hardware */
	if (!cb) {
		cb = new circular_buffer(256 * 1024, sizeof(complex));
	}

	/* Reset DSP pipeline (same as regular start) */
	m_resampler->reset();
	m_overflow_count = 0;

	/* Force streaming state to enable callback processing */
	streaming.store(true, std::memory_order_release);
}

/*
 * ---------------------------------------------------------------------------
 * USB Callback (Producer Thread)
 * ---------------------------------------------------------------------------
 */

int hydrasdr_source::fill_buffer_callback(hydrasdr_transfer_t* transfer)
{
	/* Check streaming flag atomically (lock-free) */
	if (!streaming.load(std::memory_order_acquire))
		return 0;

	/* Extract sample pointer and count from transfer structure */
	std::complex<float>* input = (std::complex<float>*)transfer->samples;
	size_t count = transfer->sample_count;

	/*
	 * FIX: Correctly count hardware-reported dropped samples.
	 * Previously this only incremented by 1 regardless of actual drop count.
	 */
	if (transfer->dropped_samples > 0) {
		m_overflow_count += (unsigned int)transfer->dropped_samples;
	}

	/*
	 * Sanity check: Verify input won't overflow batch buffer.
	 * Output ratio is approximately 1/9.23, so max output = count/9.23
	 * With BATCH_SIZE=32768 and ratio ~9.23, max safe input is ~302K samples.
	 */
	if (count > 262144) {
		fprintf(stderr, "Warning: USB transfer size %zu exceeds expected maximum\n", count);
		/* Process anyway, resampler will clamp output */
	}

	/*
	 * Run DSP Pipeline: 2.5 MSPS → 270.833 kSPS
	 * Stage 1: Decimate by 5 with anti-alias filter (61 taps)
	 * Stage 2: Rational resample 13/24 with polyphase filter (729 taps)
	 */
	size_t produced = m_resampler->process(input, count,
					       m_batch_buffer, BATCH_SIZE);

	/* Push processed samples to circular buffer (thread-safe) */
	if (produced > 0) {
		/*
		 * Use try_lock to avoid blocking the USB thread.
		 * If lock is contended, samples are dropped (counted as overflow).
		 * The circular_buffer has its own mutex, but we use data_mutex
		 * to coordinate with the consumer thread's wait condition.
		 */
		std::unique_lock<std::mutex> lock(data_mutex, std::defer_lock);
		if (lock.try_lock()) {
			if (cb) {
				unsigned int written = cb->write(m_batch_buffer, (unsigned int)produced);
				if (written < (unsigned int)produced) {
					/* Software overflow: buffer full */
					m_overflow_count += (unsigned int)(produced - written);
				}
			}
			lock.unlock();
			data_ready.notify_one();
		} else {
			/* Lock contention: drop samples */
			m_overflow_count += (unsigned int)produced;
		}
	}

	return 0;
}

/*
 * ---------------------------------------------------------------------------
 * Sample Consumer (Main Thread)
 * ---------------------------------------------------------------------------
 */

int hydrasdr_source::fill(unsigned int num_samples, unsigned int *overruns)
{
	if (!cb)
		return -1;

	/* Auto-start streaming if not already running */
	if (!streaming.load(std::memory_order_acquire))
		start();

	std::unique_lock<std::mutex> lock(data_mutex);

	while (true) {
		/* Check global exit flag to allow graceful shutdown */
		if (g_kal_exit_req)
			return -1;

		/* Exit loop when enough samples available or streaming stopped */
		if ((cb->data_available() >= num_samples) ||
		    !streaming.load(std::memory_order_acquire))
			break;

		/*
		 * Wait with 100ms timeout to periodically check exit flag.
		 * This prevents indefinite blocking if no samples arrive.
		 */
		data_ready.wait_for(lock, std::chrono::milliseconds(100));
	}

	if (!streaming.load(std::memory_order_acquire))
		return -1;

	/* Report and atomically reset overflow counter */
	if (overruns) {
		*overruns = m_overflow_count.exchange(0);
	}

	return 0;
}

int hydrasdr_source::flush()
{
	if (cb)
		cb->flush();

	m_overflow_count = 0;

	return 0;
}
