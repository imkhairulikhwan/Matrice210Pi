/*! @file Emergency.cpp
 *  @version 1.0
 *  @date juil. 25 2018
 *  @author Jonathan Michel
 */

#include "Emergency.h"

#include "dji_vehicle.hpp"

pthread_mutex_t Emergency::mutex = PTHREAD_MUTEX_INITIALIZER;

Emergency::Emergency() {
    messageDisplayed = false;
}

void Emergency::release() {
    pthread_mutex_lock(&mutex);
    this->state = false;
    messageDisplayed = false;
    pthread_mutex_unlock(&mutex);
}

void Emergency::set() {
    // Simplest possible method to avoid any kind of bug
    pthread_mutex_lock(&mutex);
    this->state = true;
    pthread_mutex_unlock(&mutex);
}

bool Emergency::isEnabled() {
    if(state) {
        if(!messageDisplayed) {
            DERROR("Emergency break enabled");
            messageDisplayed = true;
        }
        return true;
    }
    return false;
}