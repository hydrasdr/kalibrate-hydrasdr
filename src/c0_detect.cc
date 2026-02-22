/**
 * @file c0_detect.cc
 * @brief Power Scan and FCCH Detection Logic.
 */

/*
 * @author Joshua Lackey (original)
 * @author Benjamin Vernoux <bvernoux@hydrasdr.com> (improvements)
 * @copyright 2010 Joshua Lackey, 2025 Benjamin Vernoux
 * @license BSD-2-Clause
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <signal.h> // For sig_atomic_t

#ifdef _WIN32
#include "win_compat.h"
#endif
#ifndef _MSC_VER
#include <unistd.h>
#endif

#include "hydrasdr_source.h"
#include "circular_buffer.h"
#include "fcch_detector.h"
#include "arfcn_freq.h"
#include "util.h"
#include "kal_globals.h"
#include "kal_types.h"

#define MAX_ARFCN 2048

/**
 * @brief Scans for a Base Station (C0 channel) in the specified band.
 * @param u Pointer to the HydraSDR source.
 * @param bi Band Indicator.
 * @return 0 on success, -1 on failure.
 */
int c0_detect(hydrasdr_source *u, int bi) {

#define NOTFOUND_MAX 10

	int i, chan_count;
	unsigned int overruns, b_len, frames_len, found_count, notfound_count, r;
	unsigned int power_scan_len; // Short capture for power scan
	
	float offset, effective_offset, min_offset, max_offset;
	
	float spower[MAX_ARFCN];
	double power[MAX_ARFCN];
	
	double freq, sps, n, a;
	complex *b;
	circular_buffer *ub;
	fcch_detector *detector = new fcch_detector((float)u->sample_rate());

	if(bi == BI_NOT_DEFINED) {
		fprintf(stderr, "error: c0_detect: band not defined\n");
		delete detector;
		return -1;
	}

	sps = u->sample_rate() / GSM_RATE;
	
	// 12 frames for FCCH detection (approx 55ms)
	frames_len = (unsigned int)ceil((12 * 8 * 156.25 + 156.25) * sps);
	
	// Optimization: Use 1 frame for Power Scan (approx 4.6ms)
	// This makes the initial scan 12x faster.
	power_scan_len = (unsigned int)ceil((8 * 156.25) * sps); 
	if (power_scan_len < 1024) power_scan_len = 1024; // Minimum safe size

	// Helper to convert Linear L2 Norm to dBFS
	// Full Scale Reference = 1.0 (Native float32 range -1.0 to 1.0)
	// Accepts l2_norm (sqrt of sum of squares) and sample count
	auto calc_dbfs = [&](double l2_norm, unsigned int len) -> double {
		if (l2_norm < 1e-9) return -120.0; // Noise floor floor
		double rms = l2_norm / sqrt((double)len);
		return 20.0 * log10(rms);
	};

	ub = u->get_buffer();

	memset(power, 0, sizeof(power));
	memset(spower, 0, sizeof(spower));

	if(g_verbosity > 2) {
		fprintf(stderr, "calculate power in each channel:\n");
	}
	u->start();
	u->flush();
	
	// --- PASS 1: Power Scan (Fast) ---
	for(i = first_chan(bi); i >= 0; i = next_chan(i, bi)) {
		if (g_kal_exit_req) break;

		// Safety check for array bounds
		if (i >= MAX_ARFCN) {
			fprintf(stderr, "warning: ARFCN %d exceeds array size, skipping.\n", i);
			continue;
		}

		freq = arfcn_to_freq(i, &bi);
		if(u->tune(freq) != 0) {
			if (g_kal_exit_req) break;
			fprintf(stderr, "error: hydrasdr_source::tune\n");
			delete detector;
			return -1;
		}

		do {
			u->flush();
			// Use short capture length
			if(u->fill(power_scan_len, &overruns)) {
				if (g_kal_exit_req) break;
				fprintf(stderr, "error: hydrasdr_source::fill\n");
				delete detector;
				return -1;
			}
		} while(overruns);
		
		if (g_kal_exit_req) break;

		b = (complex *)ub->peek(&b_len);
		n = sqrt(vectornorm2<double>(b, power_scan_len)); // Calculate norm over short length
		power[i] = n;
		if(g_verbosity > 2) {
			fprintf(stderr, "\tchan %d (%.1fMHz):\tpower: %6.1f dBFS\n",
			   i, freq / 1e6, calc_dbfs(n, power_scan_len));
		}
	}
	
	if (g_kal_exit_req) {
		delete detector;
		return 0;
	}

	chan_count = 0;
	for(i = first_chan(bi); i >= 0; i = next_chan(i, bi)) {
		if (i < MAX_ARFCN) {
		    spower[chan_count++] = (float)power[i];
		}
	}
	sort(spower, chan_count);

	if (chan_count > 0) {
		a = avg(spower, chan_count - 4 * chan_count / 10, 0);
	} else {
		a = 0.0;
	}

	if(g_verbosity > 0) {
		// Threshold calculation uses power_scan_len (from Pass 1)
		fprintf(stderr, "channel detect threshold: %6.1f dBFS\n", calc_dbfs(a, power_scan_len));
	}

	// --- PASS 2: FCCH Scan (Precise, on candidates only) ---
	printf("%s:\n", bi_to_str(bi));
	found_count = 0;
	notfound_count = 0;
	i = first_chan(bi);
	do {
		if (g_kal_exit_req) break;

		if (i >= MAX_ARFCN) {
			i = next_chan(i, bi);
			continue;
		}

		if(power[i] <= a) {
			i = next_chan(i, bi);
			continue;
		}

		freq = arfcn_to_freq(i, &bi);
		if (isatty(1)) {
			printf("...chan %d (%.1fMHz)\r", i, freq / 1e6);
			fflush(stdout);
		}
		
		if(u->tune(freq) != 0) {
			if (g_kal_exit_req) break;
			fprintf(stderr, "error: hydrasdr_source::tune\n");
			delete detector;
			return -1;
		}

		do {
			u->flush();
			// Use full capture length for detection
			if(u->fill(frames_len, &overruns)) {
				if (g_kal_exit_req) break;
				fprintf(stderr, "error: hydrasdr_source::fill\n");
				delete detector;
				return -1;
			}
		} while(overruns);
		
		if (g_kal_exit_req) break;

		b = (complex *)ub->peek(&b_len);
		r = detector->scan(b, b_len, &offset, 0);
		effective_offset = offset - (float)(GSM_RATE / 4);
		if(r && (fabsf(effective_offset) < FCCH_OFFSET_MAX)) {
			if (found_count) {
				min_offset = fmin(min_offset, effective_offset);
				max_offset = fmax(max_offset, effective_offset);
			} else {
				min_offset = max_offset = effective_offset;
			}
			found_count++;
			
			// Recalculate power for the current buffer to match FFT display
			double current_norm = sqrt(vectornorm2<double>(b, b_len));
			double current_dbfs = calc_dbfs(current_norm, b_len);

			printf(" chan: %4d (%.1fMHz ", i, freq / 1e6);
			display_freq(effective_offset);
			printf(") power: %6.1f dBFS\n", current_dbfs);

			if (g_show_fft) {
				// Found a channel, show its spectrum!
				draw_ascii_fft((std::complex<float>*)b, 2048, 70);
			}

			notfound_count = 0;
			i = next_chan(i, bi);
		} else {
			notfound_count += 1;
			if(notfound_count >= NOTFOUND_MAX) {
				notfound_count = 0;
				i = next_chan(i, bi);
			}
		}
	} while(i >= 0);

	delete detector;
	return 0;
}
