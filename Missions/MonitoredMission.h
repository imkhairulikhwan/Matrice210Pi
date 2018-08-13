/*! @file MonitoredMission.h
 *  @version 1.0
 *  @date Jul 25 2018
 *  @author Jonathan Michel
 *  @brief This class provides monitored take-off and landing
 *  implementation. Note that methods are blocking.
 */

#ifndef MATRICE210_MONITOREDMISSION_H
#define MATRICE210_MONITOREDMISSION_H

#include <dji_vehicle.hpp>

using namespace DJI::OSDK;

namespace M210 {
    class FlightController;
    class FlightController;

    class MonitoredMission {
    private:
        FlightController *flightController;
    public:
        explicit MonitoredMission(FlightController *flightController);

        /**
         * Monitored take-off. Blocking call
         * @param timeout Timeout used on SDK method calls [s]
         * @return true if success
         */
        bool takeOff(int timeout = 1) const;

        /**
         *  Monitored landing. Blocking call
         * @param timeout Timeout used on SDK method calls [s]
         * @return true if success
         */
        bool landing(int timeout = 1) const;
    };
}
#endif //MATRICE210_MONITOREDMISSION_H
