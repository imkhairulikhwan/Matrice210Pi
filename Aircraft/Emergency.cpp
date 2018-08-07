/*! @file Emergency.cpp
 *  @version 1.0
 *  @date juil. 25 2018
 *  @author Jonathan Michel
 */

#include "Emergency.h"

#include "dji_vehicle.hpp"

#include "../util/Log.h"

using namespace M210;

pthread_mutex_t Emergency::mutex = PTHREAD_MUTEX_INITIALIZER;

Emergency::Emergency() {
    messageDisplayed = false;
}

void Emergency::release() {
    pthread_mutex_lock(&mutex);
    this->state = false;
    // Error will be displayed next time watchdog will be enabled
    messageDisplayed = false;
    pthread_mutex_unlock(&mutex);
}

void Emergency::set() {
    // Simplest possible method to avoid any kind of bug
    pthread_mutex_lock(&mutex);
    this->state = true;
    pthread_mutex_unlock(&mutex);
}

bool Emergency::isEnabled(bool displayError) {
    if(state) {
        // Display error one unless display is forced by user
        if(!messageDisplayed || displayError) {
            LERROR("Emergency break enabled");
            messageDisplayed = true;
        }
        return true;
    }
    return false;
}