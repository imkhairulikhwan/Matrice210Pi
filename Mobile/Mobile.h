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

class Mobile {
private:
    FlightController* flightController;
public:
    explicit Mobile(FlightController* flightController);
    void setup();
    // Main parser for incoming mobile data. This parser will decide what
    // vehicle API calls to make.
    static void parseFromMobileCallback(Vehicle*      vehicle,
                                 RecvContainer recvFrame,
                                 UserData      userData);
    FlightController* getFlightController() {return flightController; }
};

#endif //MATRICE210_MOBILECOMMUNICATION_H