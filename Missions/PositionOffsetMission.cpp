/*! @file PositionOffsetMission.cpp
 *  @version 1.0
 *  @date Jul 05 2018
 *  @author Jonathan Michel
 */

#include "PositionOffsetMission.h"

#include <cmath>

#include <dji_telemetry.hpp>

#include "../Managers/PackageManager.h"
#include "../Aircraft/FlightController.h"
#include "../Gps/GpsManip.h"
#include "../Gps/GpsAxis.h"
#include "../util/timer.h"
#include "../util/Log.h"

using namespace M210;

PositionOffsetMission::PositionOffsetMission(FlightController *flightController) {
    this->flightController = flightController;
}

bool PositionOffsetMission::move(const Vector3f *offset, float yaw,
                                 float posThreshold, float yawThreshold) {
    // Ensure an other mission is not running
    stop();
    LSTATUS("PositionOffsetMission move : x = % .2f m, y = % .2f m, z = % .2f m, yaw = % .2f deg",
            offset->x, offset->y, offset->z, yaw);
    vehicle = flightController->getVehicle();
    setOffset(offset, yaw);
    setThreshold(posThreshold, yawThreshold);

    if(!missionRunning) {
        /*/ Subscribe to package
                index : 0
                frequency : 50Hz
                content : quaternion, fused lat/lon and altitude
        //*/
        uint16_t frequency = 50;
        TopicName topics[] = {
                TOPIC_QUATERNION,
                TOPIC_GPS_FUSED
        };
        int numTopic = sizeof(topics) / sizeof(topics[0]);

        pkgIndex = PackageManager::instance().subscribe(topics, numTopic, frequency,
                                                                   false);
        if (pkgIndex < 0) {
            LERROR("PositionOffset mission aborted");
            return false;
        }

        missionRunning = true;
    }

    startTime = getTimeMs();

    // Broadcast height is used since relative height through subscription arrived
    if (!FlightController::startGlobalPositionBroadcast(vehicle))
    {
        LERROR("Failed to start global position broadcast");
        // Cleanup before return
        PackageManager::instance().unsubscribe(pkgIndex);
        return false;
    }

    // Wait for data to come in
    delay_ms(500);

    // Global position retrieved via subscription
    Telemetry::TypeMap<TOPIC_GPS_FUSED>::type currentSubscriptionGPS
        = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
    originGpsPosition = currentSubscriptionGPS;

    resetMissionCounters();

    // Basic receding setpoint position control with the setpoint always x [m] away
    // from the current position - until aircraft get within a threshold of the goal.
    // From that point on, the remaining distance is sent as the setpoint
    if (offset->x > 0)
        positionToMove.x = (offset->x < setPointDistance) ?
                           offset->x : setPointDistance;
    else if (offset->x < 0)
        positionToMove.x = (offset->x > -1 * setPointDistance) ?
                           offset->x : -1 * setPointDistance;
    else
        positionToMove.x = 0;

    if (offset->y > 0)
        positionToMove.y = (offset->y < setPointDistance) ?
                           offset->y : setPointDistance;
    else if (offset->y < 0)
        positionToMove.y = (offset->y > -1 * setPointDistance) ?
                           offset->y : -1 * setPointDistance;
    else
        positionToMove.y = 0;

    // Get the broadcast global position since we need the height for position z
    // Since subscription cannot give us a relative height, use broadcast.
    Telemetry::GlobalPosition currentBroadcastGP;
    currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
    positionToMove.z = currentBroadcastGP.height + offset->z;

    // update() has now to be called continuously

    return true;
}

bool PositionOffsetMission::update() {
    if(!missionRunning)
        return false;

    bool destinationReached = false;
    // Time management
    long currentTime = getTimeMs();
    long elapsedTime = currentTime - startTime;

    if(elapsedTime < missionTimeout) {
        flightController->positionAndYawCtrl(&positionToMove, (float32_t) targetYaw);

        // Calculate duration since last update was made
        long updateDiffTime = currentTime - lastUpdateTime;
        lastUpdateTime = currentTime;

        // Get current position in required coordinates and units
        Telemetry::TypeMap<TOPIC_QUATERNION>::type currentQuaternion
                = vehicle->subscribe->getValue<TOPIC_QUATERNION>();
        double currentYaw = GpsManip::toEulerAngle(currentQuaternion).z * RAD2DEG;
        Telemetry::TypeMap<TOPIC_GPS_FUSED>::type currentGpsPosition
                = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();

        Telemetry::Vector3f currentOffset =
                GpsManip::offsetFromGpsOffset(originGpsPosition, currentGpsPosition);

        Vector2 v{currentOffset.x, currentOffset.y};
        Vector2 projectedV = GpsAxis::instance().revertVector(v);

        // See how much farther we have to go
        double xOffsetRemaining = targetOffset.x - projectedV.x;
        double yOffsetRemaining = targetOffset.y - projectedV.y;
        double zOffsetRemaining = targetOffset.z - (-currentOffset.z);

        // See if we need to modify the setpoint
        if (abs(xOffsetRemaining) < setPointDistance)
            positionToMove.x = (float) xOffsetRemaining;

        if (abs(yOffsetRemaining) < setPointDistance)
            positionToMove.y = (float) yOffsetRemaining;

        if (abs(xOffsetRemaining) < posThreshold &&
            abs(yOffsetRemaining) < posThreshold &&
            abs(zOffsetRemaining) < zDeadband &&
            abs(currentYaw - targetYaw) < yawThreshold) {
            // 1. We are within bounds; start incrementing within bounds counter
            withinBoundsCnt += updateDiffTime;
        } else {
            if (withinBoundsCnt != 0) {
                // 2. Start incrementing an out-of-bounds counter
                outOfBoundsCnt += updateDiffTime;
            }
        }
        // 3. Reset within bounds counter if necessary
        if (outOfBoundsCnt > outOfBoundsLimit) {
            withinBoundsCnt = 0;
            outOfBoundsCnt = 0;
        }
        // 4. If within bounds, destination is reached
        if (withinBoundsCnt >= withinBoundsRequirement) {
            destinationReached = true;
        }
    } else {
        stop();
        LERROR("Position offset mission timeout");
        return false;
    }

    if(destinationReached) {
        // Stop aircraft
        stop();
        LSTATUS("Position offset mission done");
        return true;
    }

    return false;

}

void PositionOffsetMission::stop() {
    if(missionRunning) {
        brakeCnt = 0;
        while (brakeCnt < withinBoundsRequirement) {
            vehicle->control->emergencyBrake();
            delay_ms(20);
            brakeCnt += 20;
        }

        PackageManager::instance().unsubscribe(pkgIndex);
        missionRunning = false;
    }
}

void PositionOffsetMission::resetMissionCounters() {
   withinBoundsCnt = 0;
   outOfBoundsCnt = 0;
   brakeCnt = 0;
}
void PositionOffsetMission::setOffset(const Vector3f* offset, double yaw) {
    this->targetOffset.x = offset->x;
    this->targetOffset.y = offset->y;
    this->targetOffset.z = offset->z;
    this->targetYaw = (float)yaw;
}

void PositionOffsetMission::setThreshold(float posThreshold, double yawThreshold) {
   this->posThreshold = posThreshold;
   this->yawThreshold = yawThreshold;
}