/**
 * Copied from voxl-vision-hub.
 */

#ifndef MISC_H
#define MISC_H

#include <stdint.h>
#include <pthread.h>
#include <modal_start_stop.h>

int64_t my_time_monotonic_ns(void);
int64_t my_time_realtime_ns(void);
void my_nanosleep(uint64_t ns);
int my_loop_sleep(double rate_hz, int64_t* next_time);
float zero_bound(float);  // returns the inputted float with a lower bound of 0

// RT FIFO priorities for the most important threads
#define VIO_THREAD_PRIORITY			THREAD_PRIORITY_RT_HIGH
#define OFFBOARD_THREAD_PRIORITY	THREAD_PRIORITY_RT_MED

#endif // MISC_H
