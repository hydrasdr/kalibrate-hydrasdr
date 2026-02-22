/**
 * @file dsp_benchmark.cc
 * @brief DSP pipeline benchmark implementation.
 *
 * Extracted from util.cc to decouple utility functions from
 * HydraSDR hardware dependencies.
 *
 * @author Benjamin Vernoux <bvernoux@hydrasdr.com>
 * @copyright 2025 Benjamin Vernoux
 * @license BSD-2-Clause
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <complex>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <iomanip>

#include "util.h"
#include "hydrasdr_source.h"
#include "kal_types.h"

#ifdef _WIN32
#include "win_compat.h"
#endif

// ---------------------------------------------------------------------------
// DSP BENCHMARK IMPLEMENTATION
// ---------------------------------------------------------------------------
void run_dsp_benchmark() {
	printf("--------------------------------------------------------\n");
	printf("HydraSDR DSP Benchmark (2.5 MSPS -> 270.833 kSPS)\n");
	printf("--------------------------------------------------------\n");

	const double FS_IN = 2500000.0;
	const double FS_OUT = 270833.333333;
	const double DURATION = 5.0;
	const size_t NUM_SAMPLES = (size_t)(FS_IN * DURATION);

	printf("Generating %.1f seconds of test signal (%zu samples)...\n", DURATION, NUM_SAMPLES);
	printf("Test Signal: ToneA'300Khz(-2dB) ToneA@67kHz(-6dB) ToneB@47kHz(-8dB) ToneC@-40kHz(-10dB) ToneD@-62kHz(-12dB) ToneD'-300Khz(-14dB)\n");

	// Create synthetic data
	std::vector<std::complex<float>> input_data(NUM_SAMPLES);

	// FIX: Use double for phase accumulation to avoid float32 precision loss
	// over 12.5M samples, which causes phase noise and spurious FFT peaks.
	const double phase_inc_outA = (2.0 * M_PI * 300000.0) / FS_IN; // Shall be filtered as output out of +135KHz

	const double phase_inc_a = (2.0 * M_PI * 67000.0) / FS_IN;
	const double phase_inc_b = (2.0 * M_PI * 47000.0) / FS_IN;
	const double phase_inc_c = (2.0 * M_PI * -40000.0) / FS_IN;
	const double phase_inc_d = (2.0 * M_PI * -62000.0) / FS_IN;

	const double phase_inc_outD = (2.0 * M_PI * -300000.0) / FS_IN; // Shall be filtered as output out of -135KHz

	for(size_t i = 0; i < NUM_SAMPLES; i++) {
		// Calculate phase using double precision
		double phase_outA = i * phase_inc_outA;

		double phase_a = i * phase_inc_a;
		double phase_b = i * phase_inc_b;
		double phase_c = i * phase_inc_c;
		double phase_d = i * phase_inc_d;

		double phase_outD = i * phase_inc_outD;

		float val_r = (float)(0.79 * cos(phase_outA) + 0.5 * cos(phase_a) + 0.4 * cos(phase_b) + 0.31 * cos(phase_c) + 0.25 * cos(phase_d) + 0.2 * cos(phase_outD));
		float val_i = (float)(0.79 * sin(phase_outA) + 0.5 * sin(phase_a) + 0.4 * sin(phase_b) + 0.31 * sin(phase_c) + 0.25 * sin(phase_d) + 0.2 * sin(phase_outD));

		input_data[i] = std::complex<float>(val_r, val_i);
	}

	// 1. Visualize Input (FULL DATASET)
	printf("\nGenerated input data 2.5 MSPS draw_ascii_fft() %zu samples:\n", input_data.size());
	draw_ascii_fft(input_data.data(), (int)input_data.size(), 120, (float)FS_IN);

	printf("\nRunning DSP Pipeline...\n");

	// Instantiate source
	hydrasdr_source* sim_src = new hydrasdr_source(10.0);

	// Mock transfer struct
	hydrasdr_transfer_t transfer;
	transfer.device = NULL;
	transfer.ctx = sim_src;
	transfer.dropped_samples = 0;
	transfer.sample_type = HYDRASDR_SAMPLE_FLOAT32_IQ;

	// Container to collect ALL processed samples
	std::vector<std::complex<float>> output_data;
	output_data.reserve((size_t)(NUM_SAMPLES * (FS_OUT/FS_IN) * 1.1));

	// Simulate realistic USB Transfer chunks
	const size_t CHUNK_SIZE = 65536;

	sim_src->start_benchmark();

	auto start = std::chrono::high_resolution_clock::now();

	for (size_t offset = 0; offset < NUM_SAMPLES; offset += CHUNK_SIZE) {
		size_t current_chunk = (std::min)(CHUNK_SIZE, NUM_SAMPLES - offset);

		transfer.samples = (float*)&input_data[offset];
		transfer.sample_count = (int)current_chunk;

		sim_src->fill_buffer_callback(&transfer);

		circular_buffer *cb = sim_src->get_buffer();
		unsigned int avail = cb->data_available();
		if (avail > 0) {
			size_t current_size = output_data.size();
			output_data.resize(current_size + avail);
			cb->read(&output_data[current_size], avail);
		}
	}

	auto end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed = end - start;

	printf("--------------------------------------------------------\n");
	printf("Processed %zu samples in %.4f seconds\n", NUM_SAMPLES, elapsed.count());
	printf("Speedup:    %.2fx realtime\n", DURATION / elapsed.count());
	printf("Throughput: %.2f MSPS\n", (NUM_SAMPLES / 1e6) / elapsed.count());
	printf("--------------------------------------------------------\n");

	// 2. Visualize Output (FULL PROCESSED DATASET)
	if (!output_data.empty()) {
		printf("\nGenerated output data 270.833 kSPS draw_ascii_fft() %zu samples:\n", output_data.size());
		draw_ascii_fft(output_data.data(), (int)output_data.size(), 120, (float)FS_OUT);
	} else {
		printf("\nError: No output data collected!\n");
	}

	delete sim_src;
	exit(0);
}
