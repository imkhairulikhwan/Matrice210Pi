/*! @file Mobile.h
 *  @version 1.0
 *  @date Jul 03 2018
 *  @author Jonathan Michel
 *  @brief  This class configures Mobile-Onboard communication
 *  by setting incoming data callback and process received data.
 */

#ifndef MATRICE210_MOBILECOMMUNICATION_H
#define MATRICE210_MOBILECOMMUNICATION_H

#include <dji_vehicle.hpp>

using namespace DJI::OSDK;

#define COMMAND_CHAR '#'    /*!< If first char received is COMMAND_CHAR, frame is a command */

namespace M210 {
    class FlightController;

    class Mobile {
    private:
        FlightController *flightController; /*!< Flight controller used */
    public:
        /**
         * Create Mobile Object
         * @param flightController flight controller to use
         */
        explicit Mobile(FlightController *flightController);

        /**
         * Define callback for parsing incoming data
         */
        void setup();

        /**
         * Callback for incoming mobile data. Process data and
         * decide what to do with them
         * @param vehicle Vehicle receiving data
         * @param recvFrame Incoming data
         * @param userData Mobile object casted as void*
         */
        static void mobileCallback(Vehicle *vehicle,
                                   RecvContainer recvFrame,
                                   UserData userData);

        /**
         * Get used flight controller
         * @return Flight controller pointer
         */
        FlightController *getFlightController() const { return flightController; }
    };
}

#endif //MATRICE210_MOBILECOMMUNICATION_H