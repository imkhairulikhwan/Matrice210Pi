/*! @file Mobile.h
 *  @version 1.0
 *  @date Jul 03 2018
 *  @author Jonathan Michel
 */

#ifndef MATRICE210_MOBILECOMMUNICATION_H
#define MATRICE210_MOBILECOMMUNICATION_H

// Helpers
#include <dji_vehicle.hpp>

using namespace DJI::OSDK;

class FlightController;

/**
 *  Configure Mobile-Onboard communication by setting
 *  incoming data callback
 */
class Mobile {
private:
    FlightController* flightController;
public:
    /**
     * Create Mobile Object
     * @param flightController FlightController to use
     */
    explicit Mobile(FlightController* flightController);
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
     * Get used FlightController
     * @return FlightController pointer
     */
    FlightController* getFlightController() {return flightController; }
};

#endif //MATRICE210_MOBILECOMMUNICATION_H