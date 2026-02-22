/**
 * @file fcch_detector.cc
 * @brief Implementation of the FCCH Detector using FFTW.
 *
 * @author Joshua Lackey (original)
 * @author Benjamin Vernoux <bvernoux@hydrasdr.com> (improvements)
 * @copyright 2010 Joshua Lackey, 2025 Benjamin Vernoux
 * @license BSD-2-Clause
 */

#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <algorithm>

#include "fcch_detector.h"
#include "kal_globals.h"

static const char * const fftw_plan_name = ".kal_fftw_plan";
static const size_t PLAN_BUF_SIZE = 1024;

/*
 * ---------------------------------------------------------------------------
 * Constructor / Destructor
 * ---------------------------------------------------------------------------
 */

fcch_detector::fcch_detector(const float sample_rate, const unsigned int D,
			     const float p, const float G)
{
	FILE *plan_fp;
	char plan_name[PLAN_BUF_SIZE];
	const char *home;

	m_D = D;
	m_p = p;
	m_G = G;
	m_e = 0.0f;

	m_sample_rate = sample_rate;
	m_fcch_burst_len = (unsigned int)(148.0 * (m_sample_rate / GSM_RATE));

	m_filter_delay = 8;
	m_w_len = 2 * m_filter_delay + 1;

	/* Initialize all pointers to NULL for exception-safe cleanup */
	m_w = NULL;
	m_x_cb = NULL;
	m_y_cb = NULL;
	m_e_cb = NULL;
	m_in = NULL;
	m_out = NULL;
	m_plan = NULL;

	try {
		m_w = new complex[m_w_len];
		std::fill(m_w, m_w + m_w_len, complex(0.0f, 0.0f));

		m_x_cb = new circular_buffer(8192, sizeof(complex), 0);
		m_y_cb = new circular_buffer(8192, sizeof(complex), 1);
		m_e_cb = new circular_buffer(1015808, sizeof(float), 0);
	} catch (...) {
		delete[] m_w;
		delete m_x_cb;
		delete m_y_cb;
		delete m_e_cb;
		throw;
	}

	/* Initialize edge detection state machine (instance variables) */
	m_lth_count = 0;
	m_lth_state = 1;  /* HIGH */

	/* FFTW setup */
	m_in = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * FFT_SIZE);
	m_out = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * FFT_SIZE);
	if ((!m_in) || (!m_out)) {
		delete[] m_w;
		delete m_x_cb;
		delete m_y_cb;
		delete m_e_cb;
		if (m_in) fftw_free(m_in);
		if (m_out) fftw_free(m_out);
		throw std::runtime_error("fcch_detector: fftw_malloc failed!");
	}

	/* Try to load existing FFTW wisdom */
	home = getenv("HOME");
	if (!home)
		home = ".";

	snprintf(plan_name, sizeof(plan_name), "%s/%s", home, fftw_plan_name);

	if ((plan_fp = fopen(plan_name, "r"))) {
		fftw_import_wisdom_from_file(plan_fp);
		fclose(plan_fp);
	}

	m_plan = fftw_plan_dft_1d(FFT_SIZE, m_in, m_out, FFTW_FORWARD, FFTW_MEASURE);

	/* Save wisdom for future use */
	if ((plan_fp = fopen(plan_name, "w"))) {
		fftw_export_wisdom_to_file(plan_fp);
		fclose(plan_fp);
	}

	if (!m_plan) {
		delete[] m_w;
		delete m_x_cb;
		delete m_y_cb;
		delete m_e_cb;
		fftw_free(m_in);
		fftw_free(m_out);
		throw std::runtime_error("fcch_detector: fftw plan failed!");
	}
}

fcch_detector::~fcch_detector()
{
	if (m_w)
		delete[] m_w;
	if (m_x_cb)
		delete m_x_cb;
	if (m_y_cb)
		delete m_y_cb;
	if (m_e_cb)
		delete m_e_cb;

	if (m_plan)
		fftw_destroy_plan(m_plan);
	if (m_in)
		fftw_free(m_in);
	if (m_out)
		fftw_free(m_out);
}

/*
 * ---------------------------------------------------------------------------
 * Edge Detection State Machine (Instance Methods)
 * ---------------------------------------------------------------------------
 */

enum {
	LOW  = 0,
	HIGH = 1
};

void fcch_detector::low_to_high_init()
{
	m_lth_count = 0;
	m_lth_state = HIGH;
}

unsigned int fcch_detector::low_to_high(float e, float a)
{
	unsigned int r = 0;

	if (e > a) {
		if (m_lth_state == LOW) {
			r = m_lth_count;
			m_lth_state = HIGH;
			m_lth_count = 0;
		}
		m_lth_count += 1;
	} else {
		if (m_lth_state == HIGH) {
			m_lth_state = LOW;
			m_lth_count = 0;
		}
		m_lth_count += 1;
	}

	return r;
}

/*
 * ---------------------------------------------------------------------------
 * Signal Processing Helpers (Static)
 * ---------------------------------------------------------------------------
 */

static inline float sinc(const float x)
{
	if (std::abs(x) < 0.0001f)
		return 1.0f;
	return std::sin(x) / x;
}

static inline complex interpolate_point(const complex *s, const unsigned int s_len,
					const float s_i)
{
	static const unsigned int filter_len = 21;
	int start, end, i;
	unsigned int d;
	complex point(0.0f, 0.0f);

	d = (filter_len - 1) / 2;
	start = (int)(std::floor(s_i) - d);
	end = (int)(std::floor(s_i) + d + 1);

	if (start < 0)
		start = 0;
	if (end > (int)(s_len - 1))
		end = s_len - 1;

	for (i = start; i <= end; i++) {
		float arg = (float)(M_PI * (i - s_i));
		point += s[i] * sinc(arg);
	}

	return point;
}

static inline float peak_detect(const complex *s, const unsigned int s_len,
				complex *peak, float *avg_power)
{
	unsigned int i;
	float max = -1.0f, max_i = -1.0f, sample_power, sum_power, early_i, late_i, incr;
	complex early_p, late_p, cmax;

	sum_power = 0;
	for (i = 0; i < s_len; i++) {
		sample_power = std::norm(s[i]);
		sum_power += sample_power;
		if (sample_power > max) {
			max = sample_power;
			max_i = (float)i;
		}
	}

	early_i = (1 <= max_i) ? (max_i - 1) : 0;
	late_i = (max_i + 1 < s_len) ? (max_i + 1) : s_len - 1;

	incr = 0.5f;
	while (incr > 1.0f / 1024.0f) {
		early_p = interpolate_point(s, s_len, early_i);
		late_p = interpolate_point(s, s_len, late_i);
		if (std::norm(early_p) < std::norm(late_p))
			early_i += incr;
		else if (std::norm(early_p) > std::norm(late_p))
			early_i -= incr;
		else
			break;
		incr /= 2.0f;
		late_i = early_i + 2.0f;
	}

	max_i = early_i + 1.0f;
	cmax = interpolate_point(s, s_len, max_i);

	if (peak)
		*peak = cmax;
	if (avg_power)
		*avg_power = (s_len > 1) ? (sum_power - std::norm(cmax)) / (s_len - 1) : sum_power;

	return max_i;
}

static inline float itof(float index, float sample_rate, unsigned int fft_size)
{
	double r = index * (sample_rate / (double)fft_size);
	return (float)r;
}

/*
 * ---------------------------------------------------------------------------
 * Frequency Detection
 * ---------------------------------------------------------------------------
 */

float fcch_detector::freq_detect(const complex *s, const unsigned int s_len, float *pm)
{
	unsigned int i, len;
	float max_i, avg_power;
	complex fft[FFT_SIZE], peak;

	len = (s_len < FFT_SIZE) ? s_len : FFT_SIZE;

	for (i = 0; i < len; i++) {
		m_in[i][0] = s[i].real();
		m_in[i][1] = s[i].imag();
	}
	for (i = len; i < FFT_SIZE; i++) {
		m_in[i][0] = 0;
		m_in[i][1] = 0;
	}

	fftw_execute(m_plan);

	for (i = 0; i < FFT_SIZE; i++) {
		fft[i] = complex((float)m_out[i][0], (float)m_out[i][1]);
	}

	max_i = peak_detect(fft, FFT_SIZE, &peak, &avg_power);
	if (pm)
		*pm = std::norm(peak) / avg_power;

	return itof(max_i, m_sample_rate, FFT_SIZE);
}

/*
 * ---------------------------------------------------------------------------
 * Main Scan Function
 * ---------------------------------------------------------------------------
 */

/**
 * scan:
 *   1. Calculate average error
 *   2. Find neighborhoods with low error that satisfy minimum length
 *   3. For each such neighborhood, take FFT and calculate peak/mean
 *   4. If peak/mean > threshold, this is a valid FCCH finding
 */
unsigned int fcch_detector::scan(const complex *s, const unsigned int s_len,
				 float *offset, unsigned int *consumed)
{
	/*
	 * Calculate sps at runtime using instance's sample rate.
	 */
	const float sps = m_sample_rate / (float)GSM_RATE;
	const unsigned int MIN_FB_LEN = (unsigned int)(100 * sps);
	const unsigned int MIN_PM = 50;

	unsigned int len = 0, t, e_count, i, l_count, y_offset, y_len;
	float e, *a, loff = 0, pm = 0;
	double sum = 0.0, avg, limit;
	const complex *y;

	/* Batching optimization: Reduce mutex locking overhead */
	const unsigned int E_BATCH_SIZE = 512;
	float e_batch[E_BATCH_SIZE];
	unsigned int e_idx = 0;

	/* Calculate the error for each sample */
	while (len < s_len) {
		/* Fill buffer with as much data as possible */
		t = m_x_cb->write(s + len, s_len - len);
		len += t;

		/* Process all available data */
		while (!next_norm_error(&e)) {
			e_batch[e_idx++] = e;
			sum += e;

			if (e_idx >= E_BATCH_SIZE) {
				m_e_cb->write(e_batch, e_idx);
				e_idx = 0;
			}
		}
	}

	/* Flush remaining errors */
	if (e_idx > 0) {
		m_e_cb->write(e_batch, e_idx);
	}

	if (consumed)
		*consumed = len;

	/* Calculate average error over entire buffer */
	a = (float *)m_e_cb->peek(&e_count);
	if (e_count == 0)
		return 0;

	avg = sum / (double)e_count;
	limit = 0.7 * avg;

	if (g_debug) {
		printf("debug: error limit: %.1lf\n", limit);
	}

	/* Find neighborhoods where error is smaller than limit */
	low_to_high_init();
	for (i = 0; i < e_count; i++) {
		l_count = low_to_high(a[i], (float)limit);

		/* Check if region is long enough for FCCH */
		pm = 0;
		if (l_count >= MIN_FB_LEN) {
			y_offset = i - l_count;
			y_len = (l_count < m_fcch_burst_len) ? l_count : m_fcch_burst_len;

			/*
			 * Note: We use the original input 's' at y_offset.
			 * This works because len == s_len after the while loop,
			 * so s[y_offset] corresponds to the error at a[y_offset].
			 */
			y = s + y_offset;

			loff = freq_detect(y, y_len, &pm);
			if (g_debug)
				printf("debug: %.0f\t%f\t%f\n", (double)l_count / sps, pm, loff);

			if (pm > MIN_PM)
				break;
		}
	}

	/* Empty buffers for next call */
	m_e_cb->flush();
	m_x_cb->flush();
	m_y_cb->flush();

	if (pm <= MIN_PM)
		return 0;

	if (offset)
		*offset = loff;

	if (g_debug) {
		printf("debug: fcch_detector finished -----------------------------\n");
	}

	return 1;
}

/*
 * ---------------------------------------------------------------------------
 * Adaptive Filter (Normalized LMS)
 * ---------------------------------------------------------------------------
 */

int fcch_detector::next_norm_error(float *error)
{
	unsigned int i, n, max;
	float E;
	complex *x, y, e;

	/* n is "current" sample */
	n = m_w_len - 1;

	/* Ensure there are enough samples in the buffer */
	x = (complex *)m_x_cb->peek(&max);
	if (n + m_D >= max)
		return n + m_D - max + 1;

	/*
	 * Update gain (Normalized LMS).
	 * Set G = 1/E for optimal convergence, with epsilon for stability.
	 */
	E = vectornorm2<float>(x, m_w_len);
	if (E > 1e-10f) {
		m_G = 1.0f / E;
	}

	/* Calculate filtered value */
	y = complex(0.0f, 0.0f);
	for (i = 0; i < m_w_len; i++)
		y += std::conj(m_w[i]) * x[n - i];

	/* Store filtered value */
	m_y_cb->write(x + n + m_D, 1);

	/* Calculate error from desired signal */
	e = x[n + m_D] - y;

	/* Update filters with opposite gradient */
	for (i = 0; i < m_w_len; i++)
		m_w[i] += m_G * std::conj(e) * x[n - i];

	/* Update error average power */
	E /= m_w_len;
	m_e = (1.0f - m_p) * m_e + m_p * std::norm(e);

	/* Return error ratio (guard against zero-energy input) */
	if (error)
		*error = (E > 1e-20f) ? (m_e / E) : 0.0f;

	/* Remove the processed sample from the buffer */
	m_x_cb->purge(1);

	return 0;
}

/*
 * ---------------------------------------------------------------------------
 * Utility Methods
 * ---------------------------------------------------------------------------
 */

unsigned int fcch_detector::update(const complex *s, const unsigned int s_len)
{
	return m_x_cb->write(s, s_len);
}

unsigned int fcch_detector::get_delay()
{
	return m_w_len - 1 + m_D;
}

unsigned int fcch_detector::filter_len()
{
	return m_w_len;
}

complex *fcch_detector::dump_x(unsigned int *x_len)
{
	return (complex *)m_x_cb->peek(x_len);
}

complex *fcch_detector::dump_y(unsigned int *y_len)
{
	return (complex *)m_y_cb->peek(y_len);
}

unsigned int fcch_detector::y_buf_len()
{
	return m_y_cb->buf_len();
}

unsigned int fcch_detector::x_buf_len()
{
	return m_x_cb->buf_len();
}

unsigned int fcch_detector::x_purge(unsigned int len)
{
	return m_x_cb->purge(len);
}
