/**
 * @file kal_globals.h
 * @brief Centralized extern declarations for application globals.
 *
 * All globals are defined in kal.cc. Include this header instead of
 * scattering individual extern declarations across source files.
 */

#ifndef KAL_GLOBALS_H
#define KAL_GLOBALS_H

#include <signal.h> /* sig_atomic_t */

extern int g_verbosity;
extern int g_debug;
extern int g_show_fft;
extern volatile sig_atomic_t g_kal_exit_req;

#endif /* KAL_GLOBALS_H */
