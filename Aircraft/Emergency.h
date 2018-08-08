/*! @file Emergency.h
 *  @version 1.0
 *  @date Jul 25 2018
 *  @author Jonathan Michel
 *  @brief Handle aircraft emergency state and provide
 *  a method to verify state and display error message
 */

#ifndef MATRICE210_EMERGENCY_H
#define MATRICE210_EMERGENCY_H

#include <pthread.h>

namespace M210 {
    class Emergency {
    public:
        const static bool displayError = true;
    private:
        bool state;             /*!< Emergency state (true = emergency) */
        bool messageDisplayed;  /*!< Use to display error only once */
        static pthread_mutex_t mutex;   /*!< Protect emergency state */
    public:
        Emergency();

        /**
         * Set emergency state
         */
        void set();

        /**
         * Release emergency state
         */
        void release();

        /**
         * Verify emergency state and display error message on first call
         * (until emergency state is released)
         * @param displayError Force error message to be displayed. Use
         * Emergency::displayError for a better readability
         * @return True if emergency state is set
         */
        bool isEnabled(bool displayError = false);
    };
}

#endif //MATRICE210_EMERGENCY_H