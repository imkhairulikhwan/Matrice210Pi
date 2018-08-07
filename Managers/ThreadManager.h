/*! @file ThreadManager.h
 *  @version 1.0
 *  @date Jul 17 2018
 *  @author Jonathan Michel
 *  @brief This class provides static method to launch thread
 *  and verify everything is fine. Use pthread.h implementation
 */


#ifndef MATRICE210_THREADMANAGER_H
#define MATRICE210_THREADMANAGER_H

#include <pthread.h>
#include <string>

using namespace std;

namespace M210 {
    class ThreadManager {
    public:
        /**
         * Create and launch thread
         * @param name Thread name, restricted to 16 characters
         * @param tid Points to a pthread_t structure in which thread id must be returned
         * @param attr Points to a pthread_attr_t structure whose contents are used at thread creation time to determine attributes for the new
       thread
         * @param thread Thread routine declared as follow : void *myThread(void *param)
         * @param arg Will be passed as argument to thread routine
         * @return True if thread creation and launch works, false otherwise
         */
        static bool start(string name, pthread_t *tid, pthread_attr_t *attr, void *(*thread)(void *), void *arg);
        /**
         * Stop thread
         * Doesn't work - Unused
         * @param id Thread id to stop
         */
         // todo Doesn't work, to bugfix, unused now
        static void stop(const pthread_t *id);
    };
}

#endif //MATRICE210_THREADMANAGER_H