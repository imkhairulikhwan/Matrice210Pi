/*! @file PositionMission.cpp
 *  @version 1.0
 *  @date Jul 09 2018
 *  @author Jonathan Michel
 */

#include "PositionMission.h"

#include "../FlightController.h"
#include "../util/timer.h"

PositionMission::PositionMission(FlightController *flightController) {
    this->flightController = flightController;
}

void PositionMission::move(Vector3f *position, float yaw) {
    DSTATUS("PositionMission move : x = % .2f m, y = % .2f m, z = % .2f m, % .2f deg",
        position->x, position->y, position->z, yaw);
    this->position.x = position->x;
    this->position.y = position->y;
    this->position.z = position->z;
    this->yaw = yaw;
}

void PositionMission::update() {
    flightController->positionAndYawCtrl(&position, yaw);
}
