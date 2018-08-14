/*! @file tiemer.h
 *  @version 1.0
 *  @date Jul 03 2018
 *  @author Jonathan Michel
 *  @brief Global time relative methods implementation
 */

#ifndef MATRICE210_TIMER_H
#define MATRICE210_TIMER_H

/**
 * Return current millisecond since the Epoch
 * @return Current timestamp [ms]
 */
long long getTimeMs();

/**
 * Thread sleep
 * @param durationMs sleep duration [ms]
 */
void delay_ms(unsigned int durationMs);

#endif //MATRICE210_TIMER_H