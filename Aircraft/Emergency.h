/*! @file Emergency.h
 *  @version 1.0
 *  @date Jul 25 2018
 *  @author Jonathan Michel
 */

#ifndef MATRICE210_EMERGENCY_H
#define MATRICE210_EMERGENCY_H

#include <pthread.h>

class Emergency {
public:
    const static bool displayError = true;
private:
    bool state;
    bool messageDisplayed;
    static pthread_mutex_t mutex;
public:
    Emergency();
    void set();
    void release();
    bool isEnabled(bool displayError = false);
};


#endif //MATRICE210_EMERGENCY_H
