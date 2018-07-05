/*! @file timer.cpp
 *  @version 1.0
 *  @date Jul 03 2018
 *  @author Jonathan Michel
 */

#include <pthread.h>
#include "timer.h"

long getTimeMs() {
    struct timeval tp;
    gettimeofday(&tp, nullptr);
    //get current timestamp in milliseconds
    long ms = tp.tv_sec * 1000L + tp.tv_usec / 1000;
    return ms;
}

void delay_ms(int durationMs) {
    const struct timespec duration{0, durationMs * 1000000L};
    nanosleep(&duration, NULL);
}
