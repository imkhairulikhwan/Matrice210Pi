/*! @file Watchdog.h
 *  @version 1.0
 *  @date Jul 25 2018
 *  @author Jonathan Michel
 *  @brief Watchdog ensures that the communication with the mobile SDK is
 *  maintained.
 *
 *  Watchdog is regularly reset on specified data received from
 *  mobile SDK and increment on each sending of moving order to aircraft.
 */

#ifndef MATRICE210_WATCHDOG_H
#define MATRICE210_WATCHDOG_H

#include <pthread.h>

namespace M210 {
    class Watchdog {
    private:
        const unsigned limit;   /*!< Numbers of orders that can be sent to aircraft before watchdog is triggered */
        unsigned counter;       /*!< Numbers of orders sent to aircraft since last reset */
        bool errorDisplayed;    /*!< Used to display error only once */
        static pthread_mutex_t mutex; /*!< Protect watchdog shared attributes */
    public:
        /**
         * Create watchdog
         * @param limit Numbers of orders that can be sent to aircraft before watchdog is triggered
         * Relative watchdog duration depends on order sending frequency. See FlightController thread
         */
        explicit Watchdog(unsigned limit);

        /**
         * Increment local counter
         */
        void increment();

        /**
         * Reset local counter
         */
        void reset();

        /**
         * Verify watchdog and display error message on first call
         * (until watchdog is reset)
         * @return True if watchdog is enabled
         */
        bool isEnabled();
    };
}

#endif //MATRICE210_WATCHDOG_H
