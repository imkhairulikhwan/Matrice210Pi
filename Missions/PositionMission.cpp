/*! @file PositionMission.cpp
 *  @version 1.0
 *  @date Jul 09 2018
 *  @author Jonathan Michel
 */

#include "PositionMission.h"

#include "../Aircraft/FlightController.h"
#include "../util/timer.h"
#include "../util/Log.h"

using namespace M210;

PositionMission::PositionMission(FlightController *flightController) {
    this->flightController = flightController;
}

void PositionMission::move(const Vector3f *position, float yaw) {
    LSTATUS("PositionMission move : x = % .2f m, y = % .2f m, z = % .2f m, % .2f deg",
        position->x, position->y, position->z, yaw);
    this->position.x = position->x;
    this->position.y = position->y;
    this->position.z = position->z;
    this->yaw = yaw;
}

void PositionMission::update() {
    flightController->positionAndYawCtrl(&position, yaw);
}
