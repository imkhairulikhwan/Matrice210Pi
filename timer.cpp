/*! @file timer.cpp
 *  @version 1.0
 *  @date Jul 03 2018
 *  @author Jonathan Michel
 */

#include "timer.h"

long getTimeMs() {
    struct timeval tp;
    gettimeofday(&tp, nullptr);
    //get current timestamp in milliseconds
    long ms = tp.tv_sec * 1000L + tp.tv_usec / 1000;
    return ms;
}
