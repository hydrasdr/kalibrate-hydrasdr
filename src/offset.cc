/**
 * @file offset.cc
 * @brief Implementation of offset detection logic.
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
#ifdef __linux__
#include <sys/time.h>
#endif

#include "hydrasdr_source.h"
#include "fcch_detector.h"
#include "circular_buffer.h"
#include "util.h"
#include "kal_globals.h"

static const unsigned int TARGET_COUNT = 100; // We want 100 good samples
static const unsigned int MAX_ITERATIONS = 500; // But quit if we process 500 frames without success

/**
 * @brief Calculates the frequency offset by averaging multiple FCCH detections.
 */
int offset_detect(hydrasdr_source *u, int hz_adjust, float tuner_error) {

	unsigned int new_overruns = 0, overruns = 0;
	unsigned int notfound = 0;
	unsigned int s_len, b_len, consumed = 0;
	unsigned int count = 0;
	unsigned int iterations = 0;
	
	float offset = 0.0, min = 0.0, max = 0.0;
	double avg_offset = 0.0, stddev = 0.0;
	float sps;
	float offsets[TARGET_COUNT]; // Storage for up to 100 samples
	
	double total_ppm;
	complex *cbuf;
	fcch_detector *l;
	circular_buffer *cb;

	l = new fcch_detector((float)u->sample_rate());

	/*
	 * We grab slightly more than 1 frame length to ensure overlap
	 */
	sps = (float)(u->sample_rate() / GSM_RATE);
	s_len = (unsigned int)ceil((12 * 8 * 156.25 + 156.25) * sps);
	cb = u->get_buffer();

	u->start();
	u->flush();
	
	if (g_verbosity == 0) {
		printf("Scanning for FCCH bursts ('.' = searching, '+' = found)\n");
	}

	// Main Loop: Run until we have enough samples OR we tried too many times
	while(count < TARGET_COUNT && iterations < MAX_ITERATIONS) {
		if (g_kal_exit_req) break;

		iterations++;

		// 1. Fill Buffer
		do {
			if(u->fill(s_len, &new_overruns)) {
				// If interrupted by signal, break cleanly without error
				if (g_kal_exit_req) break;
				fprintf(stderr, "Error: Source fill failed.\n");
				delete l;
				return -1;
			}
			if(new_overruns) {
				overruns += new_overruns;
				u->flush();
			}
		} while(new_overruns);
		
		if (g_kal_exit_req) break;

		// 2. Peek at data
		cbuf = (complex *)cb->peek(&b_len);

		// FFT VISUALIZATION
		if (g_show_fft && (iterations % 5 == 0)) {
			// Draw ASCII FFT
			// 270kHz sample rate. 
			// 2048 samples gives ~130Hz resolution
			// Use width 80 or 100
			printf("\nFrame %u:", iterations);
			draw_ascii_fft((std::complex<float>*)cbuf, 2048, 80);
		}

		// 3. Scan for FCCH
		if(l->scan(cbuf, b_len, &offset, &consumed)) {
			// FOUND!
			
			// FCCH is a sine wave at GSM_RATE / 4 (approx 67.7 kHz)
			offset = offset - (float)(GSM_RATE / 4) - tuner_error;

			// Sanity check: Reject wild offsets (aliasing or false positives)
			if(fabs(offset) < FCCH_OFFSET_MAX) {
				offsets[count] = offset;
				count++;

				if(g_verbosity > 0) {
					fprintf(stderr, "  [%3u/%u] Offset: %+.2f Hz\n", count, TARGET_COUNT, offset);
				} else {
					// Visual heartbeat
					fprintf(stderr, "+"); 
					fflush(stderr);
				}
			} else {
				// Found something, but offset was crazy
				if(g_verbosity > 0) fprintf(stderr, "  [Ignored] Offset %.2f Hz out of range\n", offset);
			}
		} else {
			// NOT FOUND
			notfound++;
			
			if(g_verbosity > 0) {
			    fprintf(stderr, "  [---] No FCCH found in frame %u\n", iterations);
			} else {
				fprintf(stderr, ".");
				fflush(stderr);
			}

			// IMPORTANT: If scan failed, it might not set 'consumed'.
			// We MUST consume this block of data to move forward in time.
			if (consumed == 0) {
				consumed = s_len; // Skip this entire frame
			}
		}

		// 4. Purge used data from ring buffer
		cb->purge(consumed);
	}
	
	// End of loop cleanup
	if (g_verbosity == 0) fprintf(stderr, "\n"); // Newline after dots
	u->stop();
	delete l;
	
	if (g_kal_exit_req) return 0; // Clean exit

	// -------------------------------------------------------
	// Analysis
	// -------------------------------------------------------

	if (count == 0) {
		printf("\nError: No valid FCCH bursts found after %u attempts.\n", iterations);
		printf("Tips:\n");
		printf(" - Use '-s' scan to find a stronger channel.\n");
		printf(" - Use '-g' to increase gain.\n");
		return -1;
	}

	// Sort only the valid samples we found
	sort(offsets, count);
	
	// Calculate stats
	// If we have enough samples, drop the top/bottom 10% outliers
	unsigned int threshold = 0;
	if (count >= 10) {
		threshold = count / 10;
	}

	avg_offset = avg(offsets + threshold, count - 2 * threshold, &stddev);
	min = offsets[threshold];
	max = offsets[count - threshold - 1];

	printf("\n--------------------------------------------------\n");
	printf("Results (%u valid bursts out of %u attempts)\n", count, iterations);
	printf("--------------------------------------------------\n");
	printf("average\t\t[min, max]\t(range, stddev)\n");
	display_freq((float)avg_offset);
	printf("\t\t[%d, %d]\t(%d, %f)\n", (int)round(min), (int)round(max), (int)round(max - min), stddev);
	printf("overruns: %u\n", overruns);
	printf("not found: %u\n", notfound);

	// PPM Calculation
	// Formula: PPM = (Offset_Hz / Center_Freq_Hz) * 1e6
	total_ppm = ((avg_offset + hz_adjust) / u->m_center_freq) * 1000000.0;

	printf("\nAverage Error: %.3f ppm (%.3f ppb)\n", total_ppm, total_ppm * 1000.0);

	return 0;
}