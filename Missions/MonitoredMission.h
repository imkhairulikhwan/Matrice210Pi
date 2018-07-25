/*! @file MonitoredMission.h
 *  @version 1.0
 *  @date Jul 25 2018
 *  @author Jonathan Michel
 */

#ifndef MATRICE210_MONITOREDMISSION_H
#define MATRICE210_MONITOREDMISSION_H

#include <dji_vehicle.hpp>

using namespace DJI::OSDK;

class FlightController;

class MonitoredMission {
private:
    FlightController* flightController;
public:
    explicit MonitoredMission(FlightController* flightController);
    /**
     * Monitored take-off. Blocking call
     * @param timeout
     * @return true if success
     */
    bool takeOff(int timeout = 1) const;
    /**
     *  Monitored landing. Blocking call
     * @param timeout
     * @return true if success
     */
    bool landing(int timeout = 1) const;
};


#endif //MATRICE210_MONITOREDMISSION_H
