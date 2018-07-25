/*! @file Watchdog.h
 *  @version 1.0
 *  @date Jul 25 2018
 *  @author Jonathan Michel
 */

#ifndef MATRICE210_WATCHDOG_H
#define MATRICE210_WATCHDOG_H

#include <pthread.h>

class Watchdog {
private:
    const unsigned limit;
    unsigned counter;
    bool errorDisplayed;
public:
    explicit Watchdog(unsigned limit);
    static pthread_mutex_t mutex;
    void increment();
    void reset();
    bool isEnabled();
};


#endif //MATRICE210_WATCHDOG_H
