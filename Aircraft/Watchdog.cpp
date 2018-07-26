/*! @file Watchdog.cpp
 *  @version 1.0
 *  @date Jul 25 2018
 *  @author Jonathan Michel
 */

#include "Watchdog.h"

#include <dji_vehicle.hpp>

using namespace M210;

pthread_mutex_t Watchdog::mutex = PTHREAD_MUTEX_INITIALIZER;

Watchdog::Watchdog(unsigned limit): limit(limit){
    errorDisplayed = false;
}

void Watchdog::increment() {
    pthread_mutex_lock(&mutex);
    counter++;
    // Upper limit to avoid (improbable) out of range
    if(counter >= limit) {
        counter = limit;
    }
    pthread_mutex_unlock(&mutex);
}

void Watchdog::reset() {
    pthread_mutex_lock(&mutex);
    counter = 0;
    errorDisplayed = false;
    pthread_mutex_unlock(&mutex);
}

bool Watchdog::isEnabled() {
    if(counter < limit) {
        return false;
    }
    // Display error once
    pthread_mutex_lock(&mutex);
    if(!errorDisplayed) {
        DERROR("Watchdog enabled, please relaunch Android app");
        errorDisplayed = true;
    }
    pthread_mutex_unlock(&mutex);
    return true;
}