/*! @file PositionMission.cpp
 *  @version 1.0
 *  @date Jul 09 2018
 *  @author Jonathan Michel
 */

#ifndef MATRICE210_POSITIONMISSION_H
#define MATRICE210_POSITIONMISSION_H


#include <dji_vehicle.hpp>

#include "../timer.h"

using namespace DJI;
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

class FlightController;

class PositionMission {
private:
    FlightController* flightController;
    Vector3f position;
    float32_t yaw;
public:
    explicit PositionMission(FlightController *flightController);
    void move(Vector3f *position, float yaw);
    void update();
};


#endif //MATRICE210_POSITIONMISSION_H
