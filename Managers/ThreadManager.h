/*! @file ThreadManager.h
 *  @version 1.0
 *  @date Jul 17 2018
 *  @author Jonathan Michel
 */


#ifndef MATRICE210_THREADMANAGER_H
#define MATRICE210_THREADMANAGER_H

#include <pthread.h>
#include <string>

using namespace std;

namespace M210 {
    class ThreadManager {
    public:
        static bool start(string name, pthread_t *tid, pthread_attr_t *attr, void *(*thread)(void *), void *arg);
        static void stop(const pthread_t *id);
    };
}

#endif //MATRICE210_THREADMANAGER_H