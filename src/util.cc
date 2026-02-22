/**
 * @file util.cc
 * @brief Common utility functions.
 */

/*
 * @author Joshua Lackey (original)
 * @author Benjamin Vernoux <bvernoux@hydrasdr.com> (improvements)
 * @copyright 2010 Joshua Lackey, 2025 Benjamin Vernoux
 * @license BSD-2-Clause
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <algorithm>
#include <mutex>

#include "util.h"
#include "kal_types.h"

// Need FFTW for the visualization
#include <fftw3.h>

#ifdef _WIN32
#include "win_compat.h"
#endif

// ---------------------------------------------------------------------------
// ASCII FFT VISUALIZATION
// ---------------------------------------------------------------------------
void draw_ascii_fft(const std::complex<float> *data, int len, int width, float sample_rate) {
	static std::mutex fft_lock;
	std::lock_guard<std::mutex> guard(fft_lock);

	static fftw_complex *in = nullptr;
	static fftw_complex *out = nullptr;
	static fftw_plan p = nullptr;
	static int last_len = 0;
	static float db_offset = 0.0f;
	
	// Blackman-Harris 4-term coefficients
	const float a0 = 0.35875f;
	const float a1 = 0.48829f;
	const float a2 = 0.14128f;
	const float a3 = 0.01168f;

	// Re-allocate only if length changes
	if (len != last_len) {
		if (p) { fftw_destroy_plan(p); p = nullptr; }
		if (in) { fftw_free(in); in = nullptr; }
		if (out) { fftw_free(out); out = nullptr; }

		// Use fftw_malloc for SIMD alignment
		in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * len);
		out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * len);
		
		if (in && out) {
			p = fftw_plan_dft_1d(len, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
			if (p) {
				last_len = len;
				// CALIBRATION:
				// 1.0: Full Scale input (Normalized float -1..1)
				// a0: Blackman-Harris Window Coherent Gain (~0.36)
				float ref_amplitude = 1.0f * len * a0;
				db_offset = 20.0f * log10f(ref_amplitude);
			}
		}
		if (!p) {
			printf("Error: FFTW plan creation failed (len=%d)\n", len);
			return;
		}
	}

	// 2. Window Function (Blackman-Harris 4-term)
	for(int i=0; i<len; i++) {
		// Use double for window calculation to maintain precision for large 'len'
		double ratio = (double)i / (double)(len - 1);
		double term1 = a1 * cos(2.0 * M_PI * ratio);
		double term2 = a2 * cos(4.0 * M_PI * ratio);
		double term3 = a3 * cos(6.0 * M_PI * ratio);
		float window = (float)(a0 - term1 + term2 - term3);
		
		in[i][0] = data[i].real() * window;
		in[i][1] = data[i].imag() * window;
	}

	fftw_execute(p);

	// 3. Compute Power Spectrum (dBFS) and Find Peak
	std::vector<float> mag_db(len);
	float max_db = -1000.0f;

	for(int i=0; i<len; i++) {
		int idx = (i + len/2) % len; // FFT Shift
		float r = (float)out[idx][0];
		float im = (float)out[idx][1];
		
		float pwr = r*r + im*im; 
		float db = 10.0f * log10f(pwr + 1e-12f) - db_offset;
		
		mag_db[i] = db;
		if(db > max_db) {
			max_db = db;
		}
	}

	// 4. Downsample (Max Hold) for display width
	int plot_width = width - 20; 
	if (plot_width < 10) plot_width = 10;
	
	std::vector<float> bins(plot_width);
	for(int w=0; w<plot_width; w++) {
		float local_max = -1000.0f;
		int start_idx = w * len / plot_width;
		int end_idx = (w + 1) * len / plot_width;
		
		for(int j=start_idx; j<end_idx; j++) {
			if(j < len && mag_db[j] > local_max) local_max = mag_db[j];
		}
		bins[w] = local_max;
	}

	// 5. Draw
	const char* blocks[] = { " ", " ", "▂", "▃", "▄", "▅", "▆", "▇", "█" };
	int num_blocks = 9;
	
	float floor_db = -115.0f;
	float ceil_db = -45.0f; 
	float range = ceil_db - floor_db;

	printf("\033[36m[-BW/2] \033[0m");

	for(int w=0; w<plot_width; w++) {
		float val = bins[w];
		float norm = (val - floor_db) / range;
		
		if (norm < 0.0f) norm = 0.0f;
		if (norm > 1.0f) norm = 1.0f;
		
		int idx = (int)(norm * (num_blocks - 1));
		
		// Color coding
		if (norm < 0.20f)      printf("\033[90m"); // Gray (Noise)
		else if (norm < 0.40f) printf("\033[34m"); // Blue
		else if (norm < 0.60f) printf("\033[36m"); // Cyan
		else if (norm < 0.80f) printf("\033[32m"); // Green
		else                   printf("\033[91m"); // Red (Peak)

		printf("%s", blocks[idx]);
	}

	printf("\033[0m \033[36m[+BW/2]\033[0m");
	printf(" Max: %.1fdBFS\n", max_db);

	// 6. Find and Print Local Peaks
	if (sample_rate > 0.0f) {
		struct Peak {
			float freq;
			float db;
		};
		std::vector<Peak> peaks;

		for(int i = 1; i < len - 1; i++) {
			if (mag_db[i] > mag_db[i-1] && mag_db[i] > mag_db[i+1]) {
				// Filter spurious peaks significantly below main signal
				if (mag_db[i] > (max_db - 40.0f) && mag_db[i] > -120.0f) {
					Peak p;
					p.db = mag_db[i];
					p.freq = (i - len / 2.0f) * (sample_rate / (float)len);
					peaks.push_back(p);
				}
			}
		}

		std::sort(peaks.begin(), peaks.end(), [](const Peak& a, const Peak& b) {
			return a.db > b.db;
		});

		printf("   Peak Detection (Top 6):\n");
		int count = 0;
		for(const auto& p : peaks) {
			printf("    #%d: %9.1f Hz  (%6.1f dBFS)\n", count + 1, p.freq, p.db);
			if (++count >= 6) break;
		}
	}
	
	fflush(stdout);
}

// ---------------------------------------------------------------------------
// Existing Helpers
// ---------------------------------------------------------------------------

void display_freq(float f) {
	if(f >= 1000000.0f || f <= -1000000.0f) {
		printf("% .0fMHz", f / 1000000.0f);
	} else if(f >= 1000.0f || f <= -1000.0f) {
		printf("% .0fkHz", f / 1000.0f);
	} else {
		printf("% .0fHz", f);
	}
}

static int float_comp(const void *e1, const void *e2) {
	float f1 = *((float *)e1);
	float f2 = *((float *)e2);
	if(f1 < f2) return -1;
	if(f1 > f2) return 1;
	return 0;
}

int sort(float *data, int len) {
	qsort(data, len, sizeof(float), float_comp);
	return 0;
}

double avg(float *data, int len, double *stddev) {
	double sum = 0.0, sum_sq = 0.0, mean;
	for(int i = 0; i < len; i++) {
		sum += data[i];
		sum_sq += data[i] * data[i];
	}
	mean = sum / len;
	if(stddev) {
		*stddev = sqrt((sum_sq / len) - (mean * mean));
	}
	return mean;
}