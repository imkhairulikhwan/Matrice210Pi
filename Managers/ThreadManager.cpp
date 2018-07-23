/*! @file ThreadManager.cpp
 *  @version 1.0
 *  @date Jul 17 2018
 *  @author Jonathan Michel
 */

#include "ThreadManager.h"

#include <dji_vehicle.hpp>

#include "../util/timer.h"

bool ThreadManager::start(string name, pthread_t *id, pthread_attr_t *attr, void *(*thread)(void *), void *arg) {
    pthread_attr_init(attr);
    pthread_attr_setdetachstate(attr, PTHREAD_CREATE_JOINABLE);

    int ret = pthread_create(id, nullptr, thread, arg);
    if (ret != 0) {
        DERROR("Fail to create thread for %s !", name.c_str());
        return false;
    }

    ret = pthread_setname_np(*id, name.c_str());
    if (ret != 0)
        DERROR("Fail to set thread name for %s !", name.c_str());

    DSTATUS("%s launched...", name.c_str());
    return true;
}

void ThreadManager::stop(pthread_t *id) {
    // TODO Doesn't work, to bugfix
    pthread_join(*id, nullptr);
}
