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

using namespace M210;

PositionOffsetMission::PositionOffsetMission(FlightController *flightController) {
    this->flightController = flightController;
}

bool PositionOffsetMission::move(const Vector3f *offset, float yaw,
                                 float posThreshold, float yawThreshold) {
    DSTATUS("PositionOffsetMission move : x = % .2f m, y = % .2f m, z = % .2f m, % .2f deg",
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
            DERROR("PositionOffset mission aborted");
            return false;
        }

        missionRunning = true;
    }

    // Broadcast height is used since relative height through subscription arrived
    if (!startGlobalPositionBroadcast())
    {
        DERROR("Failed to start global position broadcast");
        // Cleanup before return
        PackageManager::instance().unsubscribe(pkgIndex);
        return false;
    }

    // Wait for data to come in
    delay_ms(1000);

    moveToPosition();

    return true;
}

bool PositionOffsetMission::moveToPosition() {
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

    /*! Calculate the inputs to send the position controller. We implement basic
    *  receding setpoint position control and the setpoint is always 2 m away
    *  from the current position - until we get within a threshold of the goal.
    *  From that point on, we send the remaining distance as the setpoint.
    */
    if (offset.x > 0)
        positionToMove.x = (offset.x < speedFactor) ?
                     offset.x : speedFactor;
    else if (offset.x < 0)
        positionToMove.x = (offset.x > -1 * speedFactor) ?
                     offset.x : -1 * speedFactor;
    else
        positionToMove.x = 0;

    if (offset.y > 0)
        positionToMove.y = (offset.y < speedFactor) ?
                     offset.y : speedFactor;
    else if (offset.y < 0)
        positionToMove.y = (offset.y > -1 * speedFactor) ?
                     offset.y : -1 * speedFactor;
    else
        positionToMove.y = 0;

    // Since subscription cannot give us a relative height, use broadcast.
    positionToMove.z = currentBroadcastGP.height + offset.z;

    startTime = getTimeMs();

    // update() has now to be called continuously
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
        if (abs(xOffsetRemaining) < speedFactor)
            positionToMove.x = (float) xOffsetRemaining;

        if (abs(yOffsetRemaining) < speedFactor)
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
        // 4. If within bounds, set flag and break
        if (withinBoundsCnt >= getWithinBoundsTimeRequirement()) {
            destinationReached = true;
        }
    } else {
        stop();
        DERROR("Position offset mission timeout");
        return false;
    }

    if(destinationReached) {
        stop();
        DSTATUS("Position offset mission done");
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

bool PositionOffsetMission::startGlobalPositionBroadcast() const {
    uint8_t freq[16];
    // Channels definition for A3/N3/M600
    freq[0]  = DataBroadcast::FREQ_HOLD; // Timestamp
    freq[1]  = DataBroadcast::FREQ_HOLD; // Attitude Quaternions
    freq[2]  = DataBroadcast::FREQ_HOLD; // Acceleration
    freq[3]  = DataBroadcast::FREQ_HOLD; // Velocity (Ground Frame)
    freq[4]  = DataBroadcast::FREQ_HOLD; // Angular Velocity (Body Frame)
    freq[5]  = DataBroadcast::FREQ_50HZ; // Position - This is the only one we want to change
    freq[6]  = DataBroadcast::FREQ_HOLD; // GPS Detailed Information
    freq[7]  = DataBroadcast::FREQ_HOLD; // RTK Detailed Information
    freq[8]  = DataBroadcast::FREQ_HOLD; // Magnetometer
    freq[9]  = DataBroadcast::FREQ_HOLD; // RC Channels Data
    freq[10] = DataBroadcast::FREQ_HOLD; //  Gimbal Data
    freq[11] = DataBroadcast::FREQ_HOLD; //  Flight Status
    freq[12] = DataBroadcast::FREQ_HOLD; //  Battery Level
    freq[13] = DataBroadcast::FREQ_HOLD; //  Control Information

    ACK::ErrorCode ack =  vehicle->broadcast->setBroadcastFreq(freq, 1);
    if (ACK::getError(ack)) {
        ACK::getErrorCodeMessage(ack, __func__);
        return false;
    }
    return true;
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
void PositionOffsetMission::setOffset(const Vector3f* o, double y) {
   offset.x = o->x;
   offset.y = o->y;
   offset.z = o->z;
   targetYaw = y;
}
void PositionOffsetMission::setThreshold(float posThreshold, double yawThreshold) {
   this->posThreshold = posThreshold;
   this->yawThreshold = yawThreshold;
}