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
#include "../util/timer.h"
#include "../util/Log.h"

using namespace M210;

PositionOffsetMission::PositionOffsetMission(FlightController *flightController) {
    this->flightController = flightController;
}

bool PositionOffsetMission::move(const Vector3f *offset, float yaw,
                                 float posThreshold, float yawThreshold) {
    LSTATUS("PositionOffsetMission move : x = % .2f m, y = % .2f m, z = % .2f m, yaw = % .2f degposThreshold",
            offset->x, offset->y, offset->z, yaw);
    vehicle = flightController->getVehicle();
    setOffset(offset, yaw * DEG2RAD);
    setThreshold(posThreshold, yawThreshold * DEG2RAD);

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

    // Broadcast height is used since relative height through subscription arrived
    if (!FlightController::startGlobalPositionBroadcast(vehicle))
    {
        LERROR("Failed to start global position broadcast");
        // Cleanup before return
        PackageManager::instance().unsubscribe(pkgIndex);
        return false;
    }

    // Wait for data to come in
    delay_ms(1000);

    // Global position retrieved via subscription
    Telemetry::TypeMap<TOPIC_GPS_FUSED>::type currentSubscriptionGPS;
    // Global position retrieved via broadcast
    Telemetry::GlobalPosition currentBroadcastGP;
    // Convert position offset from first position to local coordinates
    Telemetry::Vector3f localOffset;

    currentSubscriptionGPS = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
    originSubscriptionGPS = currentSubscriptionGPS;
    FlightController::localOffsetFromGpsOffset(localOffset,
                                               &currentSubscriptionGPS,
                                               &originSubscriptionGPS);

    // Get the broadcast GP since we need the height for position.z
    currentBroadcastGP = vehicle->broadcast->getGlobalPosition();

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

    // Since subscription cannot give us a relative height, use broadcast.
    positionToMove.z = currentBroadcastGP.height + offset->z;

    startTime = getTimeMs();

    // update() has now to be called continuously

    return true;
}

bool PositionOffsetMission::update() {
    if(!missionRunning)
        return false;

    bool destinationReached = false;
    long elapsedTime = getTimeMs() - startTime;
    if(elapsedTime < missionTimeout) {
        flightController->positionAndYawCtrl(&positionToMove, (float32_t) (targetYaw / DEG2RAD));

        // Get current position in required coordinates and units
        Telemetry::TypeMap<TOPIC_QUATERNION>::type subscriptionQ
                = vehicle->subscribe->getValue<TOPIC_QUATERNION>();
        double currentYaw = FlightController::toEulerAngle(&subscriptionQ).z;
        Telemetry::TypeMap<TOPIC_GPS_FUSED>::type currentSubscriptionGPS
                = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();

        Telemetry::Vector3f localOffset;
        FlightController::localOffsetFromGpsOffset(localOffset,
                                                   &currentSubscriptionGPS,
                                                   &originSubscriptionGPS);

        // See how much farther we have to go
        double xOffsetRemaining = offset.x - localOffset.x;
        double yOffsetRemaining = offset.y - localOffset.y;
        double zOffsetRemaining = offset.z - (-localOffset.z);

        // See if we need to modify the setpoint
        if (abs(xOffsetRemaining) < setPointDistance)
            positionToMove.x = (float) xOffsetRemaining;

        if (abs(yOffsetRemaining) < setPointDistance)
            positionToMove.y = (float) yOffsetRemaining;

        if (abs(xOffsetRemaining) < posThreshold &&
            abs(yOffsetRemaining) < posThreshold &&
            abs(zOffsetRemaining) < zDeadband &&
            abs(currentYaw - targetYaw) < yawThreshold) {
            // 1. We are within bounds; start incrementing our in-bound counter
            withinBoundsCnt += getCycleTimeMs();
        } else {
            if (withinBoundsCnt != 0) {
                // 2. Start incrementing an out-of-bounds counter
                outOfBoundsCnt += getCycleTimeMs();
            }
        }
        // 3. Reset withinBoundsCounter if necessary
        if (outOfBoundsCnt > getOutOfBoundsTimeLimit()) {
            withinBoundsCnt = 0;
            outOfBoundsCnt = 0;
        }
        // 4. If within bounds, destination is reached
        if (withinBoundsCnt >= getWithinBoundsTimeRequirement()) {
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
        while (brakeCnt < getWithinBoundsTimeRequirement()) {
            vehicle->control->emergencyBrake();
            delay_ms(getCycleTimeMs());
            brakeCnt += getCycleTimeMs();
        }

        missionRunning = false;

        PackageManager::instance().unsubscribe(pkgIndex);
    }
}

unsigned int PositionOffsetMission::getCycleTimeMs() const {
   return 1000 / (unsigned)controlFreq;
}

unsigned int PositionOffsetMission::getOutOfBoundsTimeLimit() const {
   return outOfBoundsLimit * getCycleTimeMs();
}

unsigned int PositionOffsetMission::getWithinBoundsTimeRequirement() const {
   return  withinBoundsRequirement * getCycleTimeMs();
}
void PositionOffsetMission::resetMissionCounters() {
   withinBoundsCnt = 0;
   outOfBoundsCnt = 0;
   brakeCnt = 0;
}
void PositionOffsetMission::setOffset(const Vector3f* offset, double yaw) {
    this->offset.x = offset->x;
    this->offset.y = offset->y;
    this->offset.z = offset->z;
    this->targetYaw = yaw;
}

void PositionOffsetMission::setThreshold(float posThreshold, double yawThreshold) {
   this->posThreshold = posThreshold;
   this->yawThreshold = yawThreshold;
}