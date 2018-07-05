/*! @file PositionOffsetMission.cpp
 *  @version 1.0
 *  @date Jul 05 2018
 *  @author Jonathan Michel
 */

#include "PositionOffsetMission.h"

#include "../PackageManager/PackageManager.h"
#include "../FlightController.h"

pthread_mutex_t  PositionOffsetMission::missionRunning_mutex = PTHREAD_MUTEX_INITIALIZER;

PositionOffsetMission::PositionOffsetMission(Vector3f *offset, float yawDesiredDeg,
                                             float posThresholdInM, float yawThresholdInDeg) {
    setOffset(offset, yawDesiredDeg * DEG2RAD);
    setThreshold(posThresholdInM, yawThresholdInDeg * DEG2RAD);
}

bool PositionOffsetMission::init(FlightController* fc) {
    // Vehicle
    this->flightController = fc;
    this->vehicle = fc->getVehicle();

    // Telemetry: Verify the subscription
    if(!PackageManager::getInstance()->verify())
        return false;

    /*/ Subscribe to package
            index : 0
            frequency : 50Hz
            content : quaternion, fused lat/lon and altitude
    //*/
    pkgIndex = 0;
    uint16_t frequency = 50;
    TopicName topics[] = {
            TOPIC_QUATERNION,
            TOPIC_GPS_FUSED
    };
    int numTopic = sizeof(topics) / sizeof(topics[0]);
    bool enableTimestamp = false;

    bool subscribed = PackageManager::getInstance()->subscribe(pkgIndex, topics, numTopic, frequency, enableTimestamp);
    if(!subscribed)
        return false;
    
    // Broadcast height is used since relative height through subscription arrived
    if (!startGlobalPositionBroadcast())
    {
        DERROR("Failed to start global position broadcast");
        // Cleanup before return
        PackageManager::getInstance()->unsubscribe(pkgIndex);
        return false;
    }

    // Wait for data to come in
    missionRunning = true;
    delay_ms(1000);
    return true;
}


bool PositionOffsetMission::moveToPosition() {
    // Global position retrieved via subscription
    Telemetry::TypeMap<TOPIC_GPS_FUSED>::type currentSubscriptionGPS;
    Telemetry::TypeMap<TOPIC_GPS_FUSED>::type originSubscriptionGPS;
    // Global position retrieved via broadcast
    Telemetry::GlobalPosition currentBroadcastGP;
    // Convert position offset from first position to local coordinates
    Telemetry::Vector3f localOffset;

    currentSubscriptionGPS = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
    originSubscriptionGPS = currentSubscriptionGPS;
    FlightController::localOffsetFromGpsOffset(localOffset,
                             static_cast<void *>(&currentSubscriptionGPS),
                             static_cast<void *>(&originSubscriptionGPS));

    // Get the broadcast GP since we need the height for position.z
    currentBroadcastGP = vehicle->broadcast->getGlobalPosition();

    // Get initial offset. We will update this in a loop later.
    double xOffsetRemaining = offset.x - localOffset.x;
    double yOffsetRemaining = offset.y - localOffset.y;
    double zOffsetRemaining = offset.z - (-localOffset.z);

    //! Get Euler angle

    // Quaternion retrieved via subscription
    Telemetry::TypeMap<TOPIC_QUATERNION>::type subscriptionQ;
    // Quaternion retrieved via broadcast
    Telemetry::Quaternion broadcastQ;

    subscriptionQ = vehicle->subscribe->getValue<TOPIC_QUATERNION>();
    double yawInRad = FlightController::toEulerAngle((static_cast<void *>(&subscriptionQ))).z;

    initMission();

    Telemetry::Vector3f position;
    // There is a deadband in position control
    // the z cmd is absolute height
    // while x and y are in relative
    float zDeadband = 0.12;

    /*! Calculate the inputs to send the position controller. We implement basic
    *  receding setpoint position control and the setpoint is always 1 m away
    *  from the current position - until we get within a threshold of the goal.
    *  From that point on, we send the remaining distance as the setpoint.
    */
    if (offset.x > 0)
        position.x = (offset.x < speedFactor) ?
                     offset.x : speedFactor;
    else if (offset.x < 0)
        position.x = (offset.x > -1 * speedFactor) ?
                     offset.x : -1 * speedFactor;
    else
        position.x = 0;

    if (offset.y > 0)
        position.y = (offset.y < speedFactor) ?
                     offset.y : speedFactor;
    else if (offset.y < 0)
        position.y = (offset.y > -1 * speedFactor) ?
                     offset.y : -1 * speedFactor;
    else
        position.y = 0;

    position.z = currentBroadcastGP.height +
                 offset.z; //Since subscription cannot give us a relative height, use broadcast.

    // Delay in ns
    //! Main closed-loop receding setpoint position control
    while (elapsedTime < missionTimeout && missionRunning) {
        flightController->positionAndYawCtrl(&position, (float32_t) (yawDesired / DEG2RAD));

        delay_ms(getCycleTimeMs());
        elapsedTime += getCycleTimeMs();

        //! Get current position in required coordinates and units
        subscriptionQ = vehicle->subscribe->getValue<TOPIC_QUATERNION>();
        yawInRad = FlightController::toEulerAngle((static_cast<void *>(&subscriptionQ))).z;
        currentSubscriptionGPS = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
        FlightController::localOffsetFromGpsOffset(localOffset,
                                 static_cast<void *>(&currentSubscriptionGPS),
                                 static_cast<void *>(&originSubscriptionGPS));

        // Get the broadcast GP since we need the height for position.z
        currentBroadcastGP = vehicle->broadcast->getGlobalPosition();

        //! See how much farther we have to go
        xOffsetRemaining = offset.x - localOffset.x;
        yOffsetRemaining = offset.y - localOffset.y;
        zOffsetRemaining = offset.z - (-localOffset.z);

        //! See if we need to modify the setpoint
        if (abs(xOffsetRemaining) < speedFactor)
            position.x = (float) xOffsetRemaining;

        if (abs(yOffsetRemaining) < speedFactor)
            position.y = (float) yOffsetRemaining;

        if (abs(xOffsetRemaining) < posThreshold &&
            abs(yOffsetRemaining) < posThreshold &&
            abs(zOffsetRemaining) < zDeadband &&
            abs(yawInRad - yawDesired) < yawThreshold) {
            //! 1. We are within bounds; start incrementing our in-bound counter
            withinBoundsCnt += getCycleTimeMs();
        } else {
            if (withinBoundsCnt != 0) {
                //! 2. Start incrementing an out-of-bounds counter
                outOfBoundsCnt += getCycleTimeMs();
            }
        }
        //! 3. Reset withinBoundsCounter if necessary
        if (outOfBoundsCnt > getOutOfBoundsTimeLimit()) {
            withinBoundsCnt = 0;
            outOfBoundsCnt = 0;
        }
        //! 4. If within bounds, set flag and break
        if (withinBoundsCnt >= getWithinBoundsTimeRequirement()) {
            break;
        }
    }

    //! Set velocity to zero, to prevent any residual velocity from position command
    while (brakeCounter < getWithinBoundsTimeRequirement()) {
        vehicle->control->emergencyBrake();
        delay_ms(getCycleTimeMs());
        brakeCounter += getCycleTimeMs();
    }

    PackageManager::getInstance()->unsubscribe(pkgIndex);

    if(!missionRunning) {
        DERROR("Position offset mission stopped");
        return false;
    }

    if (elapsedTime >= missionTimeout) {
        DERROR("Position offset mission timeout");
        return false;
    } else {
        DSTATUS("Position offset mission done");
        return true;
    }
}

bool PositionOffsetMission::startGlobalPositionBroadcast() {
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

       ACK::ErrorCode ack = vehicle->broadcast->setBroadcastFreq(freq, 1);
       if (ACK::getError(ack)) {
           ACK::getErrorCodeMessage(ack, __func__);
           return false;
       } else {
           return true;
       }
}

int PositionOffsetMission::getCycleTimeMs() {
   return 1000 / controlFreq;
}

int PositionOffsetMission::getOutOfBoundsTimeLimit() {
   return outOfBoundsLimit * getCycleTimeMs();
}

int PositionOffsetMission::getWithinBoundsTimeRequirement() {
   return  withinBoundsRequirement * getCycleTimeMs();
}
void PositionOffsetMission::initMission() {
   elapsedTime = 0;
   withinBoundsCnt = 0;
   outOfBoundsCnt = 0;
   brakeCounter = 0;
}
void PositionOffsetMission::setOffset(Vector3f* o, double y) {
   offset.x = o->x;
   offset.y = o->y;
   offset.z = o->z;
   yawDesired = y;
}
void PositionOffsetMission::setThreshold(float pos, double yaw) {
   posThreshold = pos;
   yawThreshold = yaw;
}

void PositionOffsetMission::setMissionRunning(bool state) {
   pthread_mutex_lock(&missionRunning_mutex);
   missionRunning = state;
   pthread_mutex_unlock(&missionRunning_mutex);
}

void PositionOffsetMission::stopMission() {
    setMissionRunning(false);
}
