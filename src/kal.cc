/**
 * @file kal.cc
 * @brief Main entry point for kalibrate-hydrasdr.
 */
/*
 * @author Joshua Lackey (original)
 * @author Benjamin Vernoux <bvernoux@hydrasdr.com> (improvements)
 * @copyright 2010 Joshua Lackey, 2025 Benjamin Vernoux
 * @license BSD-2-Clause
 */
#define PACKAGE_VERSION "0.5.1"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h> 
#include <signal.h> // Added for signal handling

#ifdef _WIN32
#include "win_compat.h"
#define basename(x) "kal"
#else
#include <sys/time.h>
#include <libgen.h>
#endif
#ifndef _MSC_VER
#include <unistd.h>
#endif

#include "hydrasdr_source.h"
#include "fcch_detector.h"
#include "arfcn_freq.h"
#include "offset.h"
#include "c0_detect.h"
#include "util.h"
#include "kal_globals.h"

#define HYDRASDR_FLASH_CALIB_OFFSET (0x20000) 
#define HYDRASDR_FLASH_CALIB_HEADER (0xCA1B0001)

typedef struct {
	uint32_t header;
	uint32_t timestamp;
	int32_t correction_ppb;
} hydrasdr_calib_t;

int g_verbosity = 0;
int g_debug = 0;
int g_show_fft = 0;

// Global control flags for signal handling
volatile sig_atomic_t g_kal_exit_req = 0;

// Signal handler: Strictly async-signal-safe
void sighandler(int signum) {
	if (g_kal_exit_req) {
		// Force exit on double Ctrl-C
		const char* msg = "\nForcing exit.\n";
#ifdef _MSC_VER
		_write(2, msg, (unsigned int)strlen(msg));
#else
		write(2, msg, strlen(msg));
#endif
		_exit(1);
	}
	const char* msg = "\nSignal received, stopping...\n";
#ifdef _MSC_VER
	_write(2, msg, (unsigned int)strlen(msg));
#else
	write(2, msg, strlen(msg));
#endif
	g_kal_exit_req = 1;
}

void usage(char *prog) {
	fprintf(stderr, "kalibrate v%s-hydrasdr\n", PACKAGE_VERSION);
	fprintf(stderr, "\nUsage:\n");
	fprintf(stderr, "\tGSM Base Station Scan:\n");
	fprintf(stderr, "\t\t%s <-s band indicator> [options]\n", basename(prog));
	fprintf(stderr, "\n");
	fprintf(stderr, "\tClock Offset Calculation:\n");
	fprintf(stderr, "\t\t%s <-f frequency | -c channel> [options]\n", basename(prog));
	fprintf(stderr, "\n");
	fprintf(stderr, "\tDevice Maintenance:\n");
	fprintf(stderr, "\t\t%s -R (Read Calibration)\n", basename(prog));
	fprintf(stderr, "\t\t%s -W <ppb_error> (Write Calibration and Reset)\n", basename(prog));
	fprintf(stderr, "\n");
	fprintf(stderr, "Where options are:\n");
	fprintf(stderr, "\t-s\tband to scan (GSM850, GSM-R, GSM900, EGSM, DCS)\n");
	fprintf(stderr, "\t-f\tfrequency of nearby GSM base station\n");
	fprintf(stderr, "\t-c\tchannel of nearby GSM base station\n");
	fprintf(stderr, "\t-b\tband indicator (GSM850, GSM-R, GSM900, EGSM, DCS)\n");
	fprintf(stderr, "\t-g\tgain (0-21 for HydraSDR Linearity Gain)\n");
	fprintf(stderr, "\t-R\tRead calibration data from flash\n");
	fprintf(stderr, "\t-W\tWrite calibration data (int32 PPB) to flash and RESET\n");
	fprintf(stderr, "\t-A\tShow ASCII FFT of signal\n");
	fprintf(stderr, "\t-B\tRun DSP Benchmark and exit\n");
	fprintf(stderr, "\t-v\tverbose\n");
	fprintf(stderr, "\t-D\tenable debug messages\n");
	fprintf(stderr, "\t-h\thelp\n");
	exit(1);
}

int handle_calibration(bool write, int32_t ppb_value) {
	hydrasdr_device* dev = NULL;
	int res;
	hydrasdr_calib_t calib;

	res = hydrasdr_open(&dev); 
	if (res != HYDRASDR_SUCCESS) {
		fprintf(stderr, "Error: Failed to open HydraSDR device: %d\n", res);
		return -1;
	}

	if (write) {
		printf("[-] Erasing flash sector 2 (Calibration area)...\n");
		res = hydrasdr_spiflash_erase_sector(dev, 2);
		if (res != HYDRASDR_SUCCESS) {
			fprintf(stderr, "Error: Flash erase failed: %d\n", res);
			hydrasdr_close(dev);
			return -1;
		}

		calib.header = HYDRASDR_FLASH_CALIB_HEADER;
		calib.timestamp = (uint32_t)time(NULL);
		calib.correction_ppb = ppb_value;

		printf("[-] Writing Calibration: %d ppb (Timestamp: %u)...\n", ppb_value, calib.timestamp);
		res = hydrasdr_spiflash_write(dev, HYDRASDR_FLASH_CALIB_OFFSET, sizeof(calib), (uint8_t*)&calib);
		if (res != HYDRASDR_SUCCESS) {
			fprintf(stderr, "Error: Flash write failed: %d\n", res);
			hydrasdr_close(dev);
			return -1;
		}
		
		printf("[+] Calibration written successfully.\n");
		
		printf("[!] Resetting HydraSDR to apply changes...\n");
		res = hydrasdr_reset(dev);
		if (res != HYDRASDR_SUCCESS) {
			fprintf(stderr, "Warning: Reset command failed: %d. Please replug device.\n", res);
		} else {
			printf("[+] Device reset command sent.\n");
		}
		
		hydrasdr_close(dev); 
		return 0;

	} else {
		printf("[-] Reading calibration from flash (0x%06x)...\n", HYDRASDR_FLASH_CALIB_OFFSET);
		res = hydrasdr_spiflash_read(dev, HYDRASDR_FLASH_CALIB_OFFSET, sizeof(calib), (uint8_t*)&calib);
		if (res != HYDRASDR_SUCCESS) {
			fprintf(stderr, "Error: Flash read failed: %d\n", res);
			hydrasdr_close(dev);
			return -1;
		}

		if (calib.header == HYDRASDR_FLASH_CALIB_HEADER) {
			time_t epoch_time = calib.timestamp;
			struct tm *local_time = localtime(&epoch_time);
			char time_buf[64];
			strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", local_time);

			printf("Stored Calibration Data:\n");
			printf("  Correction: %d ppb\n", calib.correction_ppb);
			printf("  Date:       %s\n", time_buf);
		} else {
			printf("No valid calibration found (Header mismatch).\n");
			printf("Raw Header: 0x%08X (Expected 0x%08X)\n", calib.header, HYDRASDR_FLASH_CALIB_HEADER);
		}
	}

	hydrasdr_close(dev);
	return 0;
}

void check_band_limit(int bi) {
	if (bi == PCS_1900) {
		fprintf(stderr, "Error: PCS-1900 band (~1.9 GHz) is not supported by HydraSDR RFOne.\n");
		fprintf(stderr, "       Hardware frequency limit is approx 1800 MHz.\n");
		exit(1);
	}
	if (bi == DCS_1800) {
		fprintf(stderr, "Warning: DCS-1800 band (~1.8 GHz) is at the edge of HydraSDR RFOne capabilities.\n");
		fprintf(stderr, "         Reception may degrade or fail above 1800 MHz.\n");
	}
}

int main(int argc, char **argv) {
	int c;
	int bi = BI_NOT_DEFINED;
	int chan = -1;
	int bts_scan = 0;
	float gain = 10.0; 
	double freq = -1.0;
	int result = 0;
	
	bool do_read_cal = false;
	bool do_write_cal = false;
	int32_t write_cal_val = 0;
	
	hydrasdr_source *u = NULL;

	// Setup Windows Console for Unicode/ANSI
#ifdef _WIN32
	SetConsoleOutputCP(65001); // CP_UTF8
	HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
	DWORD dwMode = 0;
	GetConsoleMode(hOut, &dwMode);
	dwMode |= 0x0004; // ENABLE_VIRTUAL_TERMINAL_PROCESSING
	SetConsoleMode(hOut, dwMode);
#endif

	// Register Signal Handler
#ifdef SIGINT
	signal(SIGINT, sighandler);
#endif

	while((c = getopt(argc, argv, "f:c:s:b:g:W:RvDBAh?")) != EOF) {
		switch(c) {
			case 'f':
				freq = strtod(optarg, 0);
				break;
			case 'c':
				chan = strtoul(optarg, 0, 0);
				break;
			case 's':
				if((bi = str_to_bi(optarg)) == -1) {
					fprintf(stderr, "error: bad band indicator: ``%s''\n", optarg);
					usage(argv[0]);
				}
				check_band_limit(bi);
				bts_scan = 1;
				break;
			case 'b':
				if((bi = str_to_bi(optarg)) == -1) {
					fprintf(stderr, "error: bad band indicator: ``%s''\n", optarg);
					usage(argv[0]);
				}
				check_band_limit(bi);
				break;
			case 'g':
				gain = strtof(optarg, 0);
				break;
			case 'R':
				do_read_cal = true;
				break;
			case 'W':
				do_write_cal = true;
				write_cal_val = atoi(optarg);
				break;
			case 'B':
				run_dsp_benchmark();
				return 0; 
			case 'A':
				g_show_fft = 1;
				break;
			case 'v':
				g_verbosity++;
				break;
			case 'D':
				g_debug = 1;
				break;
			case 'h':
			case '?':
			default:
				usage(argv[0]);
				break;
		}
	}

	if (do_read_cal || do_write_cal) {
		if (do_read_cal && do_write_cal) {
			fprintf(stderr, "Error: Cannot Read (-R) and Write (-W) at the same time.\n");
			return -1;
		}
		return handle_calibration(do_write_cal, write_cal_val);
	}

	if(bts_scan) {
		if(bi == BI_NOT_DEFINED) {
			fprintf(stderr, "error: scanning requires band (-s)\n");
			usage(argv[0]);
		}
	} else {
		if(freq < 0.0) {
			if(chan < 0) {
				fprintf(stderr, "error: must enter scan band -s or channel -c or frequency -f or -R or -W to read or write calibration\n");
				usage(argv[0]);
			}
			freq = arfcn_to_freq(chan, &bi);
		}
		
		if (chan == -1 && freq != -1.0) {
			chan = freq_to_arfcn(freq, &bi);
		}
	}

	if(g_debug) {
		printf("debug: Gain                 : %f\n", gain);
	}

	u = new hydrasdr_source(gain);
	if(!u) {
		fprintf(stderr, "error: failed to allocate hydrasdr_source\n");
		return -1;
	}

	if(u->open() == -1) {
		fprintf(stderr, "error: failed to open HydraSDR device\n");
		delete u;
		return -1;
	}

	if(!bts_scan) {
		if(u->tune(freq) == -1) {
			fprintf(stderr, "error: hydrasdr_source::tune failed\n");
			result = -1;
			goto cleanup;
		}

		float tuner_error = 0.0f;

		fprintf(stderr, "%s: Calculating clock frequency offset.\n", basename(argv[0]));
		fprintf(stderr, "Using %s channel %d (%.1fMHz)\n", bi_to_str(bi), chan, freq / 1e6);
		
		result = offset_detect(u, 0, tuner_error);
		goto cleanup;
	}

	fprintf(stderr, "%s: Scanning for %s base stations.\n", basename(argv[0]), bi_to_str(bi));

	result = c0_detect(u, bi);

cleanup:
	if(u) {
		delete u;
	}
	return result;
}