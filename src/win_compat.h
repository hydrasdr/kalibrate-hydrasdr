/**
 * @file win_compat.h
 * @brief Windows compatibility layer for POSIX functions (gettimeofday, usleep, etc).
 */
/*
 * Copyright 2025 Benjamin Vernoux <bvernoux@hydrasdr.com>
 * SPDX-License-Identifier: BSD-2-Clause
 */
#ifndef WIN_COMPAT_H
#define WIN_COMPAT_H

#ifdef _WIN32

#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <time.h>
#include <stdint.h>

#ifdef _MSC_VER
#include <io.h>
#ifndef isatty
#define isatty _isatty
#endif
/* Note: do NOT #define write _write -- it breaks circular_buffer::write().
 * Use _write() directly in the few places that need POSIX write(). */
#ifndef fileno
#define fileno _fileno
#endif
/* MSVC: provide getopt inline (no unistd.h / getopt.h available) */
#include <string.h>
#include <stdio.h>

static char *optarg;
static int optind = 1;
static int opterr = 1;
static int optopt;

static int getopt(int argc, char * const argv[], const char *optstring)
{
	static int sp = 1;
	int c;
	const char *cp;

	if (sp == 1) {
		if (optind >= argc || argv[optind][0] != '-' || argv[optind][1] == '\0')
			return -1;
		if (!strcmp(argv[optind], "--")) {
			optind++;
			return -1;
		}
	}
	optopt = c = argv[optind][sp];
	cp = strchr(optstring, c);
	if (c == ':' || cp == NULL) {
		if (opterr)
			fprintf(stderr, "%s: illegal option -- %c\n", argv[0], c);
		if (argv[optind][++sp] == '\0') {
			optind++;
			sp = 1;
		}
		return '?';
	}
	if (*++cp == ':') {
		if (argv[optind][sp + 1] != '\0')
			optarg = &argv[optind++][sp + 1];
		else if (++optind >= argc) {
			if (opterr)
				fprintf(stderr, "%s: option requires an argument -- %c\n", argv[0], c);
			sp = 1;
			return '?';
		} else
			optarg = argv[optind++];
		sp = 1;
	} else {
		if (argv[optind][++sp] == '\0') {
			sp = 1;
			optind++;
		}
		optarg = NULL;
	}
	return c;
}
#else
/* MinGW / GCC: use system headers */
#include <unistd.h>
#include <getopt.h>
#endif /* _MSC_VER */

#ifndef usleep
#define usleep(x) Sleep((x)/1000)
#endif

#ifndef sleep
#define sleep(x) Sleep((x)*1000)
#endif

// Define timeval/timezone if not already provided (e.g. by Winsock)
#if !defined(_TIMEVAL_DEFINED) && !defined(_WINSOCKAPI_) && !defined(_WINSOCK2API_)
#define _TIMEVAL_DEFINED
struct timeval {
    long tv_sec;
    long tv_usec;
};
#endif
#ifndef _TIMEZONE_DEFINED
#define _TIMEZONE_DEFINED
struct timezone {
    int tz_minuteswest;
    int tz_dsttime;
};
#endif // _TIMEZONE_DEFINED

#ifndef _GETTIMEOFDAY_DEFINED
#define _GETTIMEOFDAY_DEFINED
static int gettimeofday(struct timeval * tp, struct timezone * tzp) {
    static const uint64_t EPOCH = ((uint64_t) 116444736000000000ULL);
    SYSTEMTIME  system_time;
    FILETIME    file_time;
    uint64_t    time;

    GetSystemTime( &system_time );
    SystemTimeToFileTime( &system_time, &file_time );
    time =  ((uint64_t)file_time.dwLowDateTime )      ;
    time += ((uint64_t)file_time.dwHighDateTime) << 32;

    tp->tv_sec  = (long) ((time - EPOCH) / 10000000L);
    tp->tv_usec = (long) (system_time.wMilliseconds * 1000);
    return 0;
}
#endif // _GETTIMEOFDAY_DEFINED

#endif // _WIN32
#endif // WIN_COMPAT_H