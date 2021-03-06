/*! @file Console.h
 *  @version 1.0
 *  @date Jul 04 2018
 *  @author Jonathan Michel
 *  @brief  This class launches a thread to get char on
 *  console and control flight controller
 */


#ifndef MATRICE210_CONSOLE_H
#define MATRICE210_CONSOLE_H

#include <pthread.h>
#include <string>
#include <dji_vehicle.hpp>

using namespace std;
using namespace DJI::OSDK;

namespace M210 {
    class FlightController;

    class Console {
    private:
        // Thread attributes
        pthread_t consoleThreadID;              /*!< Console thread id */
        pthread_attr_t consoleThreadAttr;       /*!< Console thread attributes */
        // FlightController
        FlightController *flightController;     /*!< Flight controller used */
    public:
        /**
         * Create console object
         * @param flightController FlightController to use
         */
        explicit Console(FlightController *flightController);

        /**
         * Create, set name and launch console thread
         */
        void launchThread();

    private:
        /**
         * Dedicated console thread.
         * Get command typed by user and process them
         * @param param Console object is passed as parameter to
         * grant access to FlightController
         * @return -
         */
        static void *consoleThread(void *param);

        /**
         * Display available commands list
         */
        void displayMenu() const;

        /**
         * Display available command line
         * @param command Command char
         * @param hint Command description, max hint length = 55 to assure
         * alignment
         */
        void displayMenuLine(char command, const string &hint) const;

        /**
         * Read float number from console
         * @param hint Description displayed to user
         * @return Read number
         */
        float getNumber(const string &hint) const;
    };
}

#endif //MATRICE210_CONSOLE_H