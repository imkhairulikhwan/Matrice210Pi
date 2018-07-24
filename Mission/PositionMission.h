/*! @file PositionMission.cpp
 *  @version 1.0
 *  @date Jul 09 2018
 *  @author Jonathan Michel
 */

#ifndef MATRICE210_POSITIONMISSION_H
#define MATRICE210_POSITIONMISSION_H

#include <dji_vehicle.hpp>

using namespace DJI;
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

class FlightController;

class PositionMission {
private:
    const FlightController* flightController;
    Vector3f position;
    float yaw;
public:
    explicit PositionMission(const FlightController *flightController);
    void move(const Vector3f *position, float yaw);
    void update() const ;
};


#endif //MATRICE210_POSITIONMISSION_H
