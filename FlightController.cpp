/*! @file FlightController.cpp
 *  @version 1.0
 *  @date Jul 03 2018
 *  @author Jonathan Michel
 */

#include "FlightController.hpp"

FlightController::FlightController() {
    this->emergencyState = false;
    this->linuxEnvironment = nullptr;
    this->vehicle = nullptr;
    this->flightControllerThreadRunning = false;
    this->velocityActivated = false;
}


void FlightController::setupVehicle(int argc, char** argv) {
    // Get vehicle
    linuxEnvironment = new LinuxSetup(argc, argv);
    do{
        vehicle = linuxEnvironment->getVehicle();
        if(vehicle == nullptr)
            DERROR("Vehicle not initialized, retrying");
    } while(vehicle == nullptr);

    // Obtain control authority
    ACK::ErrorCode ack;
    do {
        ack = vehicle->obtainCtrlAuthority(1);
        if(ACK::getError(ack) != ACK::SUCCESS)
            DERROR("Obtain control authority failed, retrying");
    }while(ACK::getError(ack) != ACK::SUCCESS);

    startFlightControllerThread();
}



/**
 *  Monitored take-off. Return true if success
*/
bool FlightController::monitoredTakeoff(int timeout) {
    // Telemetry: Verify the subscription
    ACK::ErrorCode ack;
    ack = vehicle->subscribe->verify(timeout);
    if (ACK::getError(ack) != ACK::SUCCESS) {
        ACK::getErrorCodeMessage(ack, __func__);
        return false;
    }

    /*/ Subscribe to package
            index : 0
            frequency : 10Hz
            content : flight status and flight mode
    //*/
    int pkgIndex = 0;
    uint16_t frequency = 10;
    TopicName topicList[] = {
            TOPIC_STATUS_FLIGHT,
            TOPIC_STATUS_DISPLAYMODE
    };
    int  numTopic = sizeof(topicList) / sizeof(topicList[0]);
    bool enableTimestamp = false;

    bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
            pkgIndex, numTopic, topicList, enableTimestamp, frequency);
    if (!pkgStatus)
        return pkgStatus;

    ack = vehicle->subscribe->startPackage(pkgIndex, timeout);
    if (ACK::getError(ack) != ACK::SUCCESS)
    {
        ACK::getErrorCodeMessage(ack, __func__);
        // Cleanup before return
        vehicle->subscribe->removePackage(pkgIndex, timeout);
        return false;
    }

    // Start takeoff
    ack = vehicle->control->takeoff(timeout);
    if (ACK::getError(ack) != ACK::SUCCESS)
    {
        ACK::getErrorCodeMessage(ack, __func__);
        vehicle->subscribe->removePackage(pkgIndex, timeout);
        return false;
    }

    // First check: Motors started
    int motorsNotStarted = 0;
    int timeoutCycles    = 20;

    while (vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() !=
             VehicleStatus::FlightStatus::ON_GROUND &&
           vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
             VehicleStatus::DisplayMode::MODE_ENGINE_START &&
           motorsNotStarted < timeoutCycles) {
        motorsNotStarted++;
        usleep(100000);
    }

    if (motorsNotStarted == timeoutCycles) {
        std::cout << "Takeoff failed. Motors are not spinning." << std::endl;
        // Cleanup
        vehicle->subscribe->removePackage(pkgIndex, timeout);
        return false;
    } else {
        std::cout << "Motors spinning...\n";
    }


    // Second check: In air
    int stillOnGround = 0;
    timeoutCycles     = 110;

    while ( vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() !=
                VehicleStatus::FlightStatus::IN_AIR &&
            (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
                VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
            vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
                VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF) &&
            stillOnGround < timeoutCycles) {
        stillOnGround++;
        usleep(100000);
    }

    if (stillOnGround == timeoutCycles) {
        std::cout << "Takeoff failed. Aircraft is still on the ground, but the "
                   "motors are spinning."
                << std::endl;
        // Cleanup
        vehicle->subscribe->removePackage(pkgIndex, timeout);
        return false;
    } else {
        std::cout << "Ascending...\n";
    }


    // Final check: Finished takeoff
    while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
                VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
           vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
                VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF) {
        sleep(1);
    }

    if (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
            VehicleStatus::DisplayMode::MODE_P_GPS ||
        vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
            VehicleStatus::DisplayMode::MODE_ATTITUDE) {
        std::cout << "Successful takeoff!\n";
    } else {
        std::cout
        << "Takeoff finished, but the aircraft is in an unexpected mode. "
        "Please connect DJI GO.\n";
        vehicle->subscribe->removePackage(pkgIndex, timeout);
        return false;
    }


    // Cleanup
    ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
    if (ACK::getError(ack)) {
        std::cout
        << "Error unsubscribing; please restart the drone/FC to get back "
           "to a clean state.\n";
    }

    return true;
}

/**
 *  Monitored landing. Return true if success
*/
bool FlightController::monitoredLanding(int timeout)
{
    // Telemetry: Verify the subscription
    ACK::ErrorCode ack;
    ack = vehicle->subscribe->verify(timeout);
    if (ACK::getError(ack) != ACK::SUCCESS) {
        ACK::getErrorCodeMessage(ack, __func__);
        return false;
    }

    /*/ Subscribe to package
            index : 0
            frequency : 10Hz
            content : flight status and flight mode
    //*/
    int pkgIndex = 0;
    uint16_t frequency = 10;
    TopicName topicList[] = {
            TOPIC_STATUS_FLIGHT,
            TOPIC_STATUS_DISPLAYMODE
    };
    int  numTopic        = sizeof(topicList) / sizeof(topicList[0]);
    bool enableTimestamp = false;

    bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
            pkgIndex, numTopic, topicList, enableTimestamp, frequency);
    if (!pkgStatus)
        return pkgStatus;

    ack = vehicle->subscribe->startPackage(pkgIndex, timeout);
    if (ACK::getError(ack) != ACK::SUCCESS) {
        ACK::getErrorCodeMessage(ack, __func__);
        // Cleanup before return
        vehicle->subscribe->removePackage(pkgIndex, timeout);
        return false;
    }

    // Start landing
    ACK::ErrorCode landingStatus = vehicle->control->land(timeout);
    if (ACK::getError(landingStatus) != ACK::SUCCESS) {
        ACK::getErrorCodeMessage(landingStatus, __func__);
        return false;
    }

    // First check: Landing started
    int landingNotStarted = 0;
    int timeoutCycles     = 20;

    while ( vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
                VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
            landingNotStarted < timeoutCycles) {
        landingNotStarted++;
        usleep(100000);
    }


    if (landingNotStarted == timeoutCycles) {
        std::cout << "Landing failed. Aircraft is still in the air." << std::endl;
        // Cleanup before return
        ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
        if (ACK::getError(ack)) {
            std::cout << "Error unsubscribing; please restart the drone/FC to get "
                         "back to a clean state.\n";
        }
        return false;
    } else {
        std::cout << "Landing...\n";
    }

    // Second check: Finished landing
    while ( vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
                VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
            vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() ==
                VehicleStatus::FlightStatus::IN_AIR) {
        sleep(1);
    }

    if (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
            VehicleStatus::DisplayMode::MODE_P_GPS ||
        vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
            VehicleStatus::DisplayMode::MODE_ATTITUDE) {
        std::cout << "Successful landing!\n";
    } else {
        std::cout
                << "Landing finished, but the aircraft is in an unexpected mode. "
                   "Please connect DJI GO.\n";
        ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
        if (ACK::getError(ack)) {
            std::cout << "Error unsubscribing; please restart the drone/FC to get "
                         "back to a clean state.\n";
        }
        return false;
    }

    // Cleanup
    ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
    if (ACK::getError(ack)) {
        std::cout
                << "Error unsubscribing; please restart the drone/FC to get back "
                   "to a clean state.\n";
    }

    return true;
}


void FlightController::moveByVelocity(Vector3f *velocity, float yaw) {
    this->velocity.x = velocity->x;
    this->velocity.y = velocity->y;
    this->velocity.z = velocity->z;
    this->yaw = yaw;
    this->velocityActivated = true;
}


void FlightController::stopVelocity() {
    velocityActivated = false;
}

void FlightController::startFlightControllerThread() {
    if(!flightControllerThreadRunning) {
        flightControllerThreadRunning = true;
        pthread_attr_init(&flightControllerThreadAttr);
        pthread_attr_setdetachstate(&flightControllerThreadAttr, PTHREAD_CREATE_JOINABLE);
        int ret = pthread_create(&flightControllerThreadID, nullptr, flightControllerThread, (void*)this);
        string infoStr = "flightControllerThread";

        if (0 != ret)
            cout << "Fail to create thread for " << infoStr.c_str() << "!" << endl;

        ret = pthread_setname_np(flightControllerThreadID, infoStr.c_str());
        if (0 != ret)
            cout << "Fail to set thread name for " << infoStr.c_str() << "!" << endl;

        cout << infoStr.c_str() << " running..." << endl;
    }
}

void FlightController::stopFlightControllerThread() {
    flightControllerThreadRunning = false;
    void* status;
    pthread_join(flightControllerThreadID, &status);
}

void *FlightController::flightControllerThread(void *param) {
    auto fc = (FlightController*) param;
    while(fc->flightControllerThreadRunning) {
        if(fc->isvelocityActivated()) {
            fc->velocityAndYawRateCtrl(fc->getVelocity(), fc->getYaw());
            nanosleep((const struct timespec[]){{0, 500000000L}}, NULL);
        }
    }
    return nullptr;
}

/*! Position Control. Allows user to set an offset from current location.
    The aircraft will move to that position and stay there.
!*/
bool FlightController::moveByPositionOffset(Vector3f* offset, float yawDesired,
                                            float posThresholdInM, float yawThresholdInDeg)
{
    ACK::ErrorCode ack;
    int responseTimeout              = 1;
    int timeoutInMilSec              = 10000;   // Timeout to finish mission
    int controlFreqInHz              = 50;      // Hz
    int cycleTimeInMs                = 1000 / controlFreqInHz;
    int outOfControlBoundsTimeLimit  = 10 * cycleTimeInMs; // 10 cycles
    int withinControlBoundsTimeReqmt = 50 * cycleTimeInMs; // 50 cycles
    int pkgIndex;

    // Telemetry: Verify the subscription
    ack = vehicle->subscribe->verify(responseTimeout);
    if (ACK::getError(ack) != ACK::SUCCESS)  {
        ACK::getErrorCodeMessage(ack, __func__);
        return false;
    }

    /*/ Subscribe to package
            index : 0
            frequency : 50Hz
            content : quaternion, fused lat/lon and altitude
    //*/
    pkgIndex = 0;
    uint16_t frequency = 50;
    TopicName topicList[] = {
            TOPIC_QUATERNION,
            TOPIC_GPS_FUSED
    };
    int numTopic = sizeof(topicList) / sizeof(topicList[0]);
    bool enableTimestamp = false;

    bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
            pkgIndex, numTopic, topicList, enableTimestamp, frequency);
    if (!pkgStatus)
      return pkgStatus;

    ack = vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
    if (ACK::getError(ack) != ACK::SUCCESS) {
      ACK::getErrorCodeMessage(ack, __func__);
      // Cleanup before return
      vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
      return false;
    }

    // Broadcast height is used since relative height through subscription arrived
    if (!startGlobalPositionBroadcast())
    {
      // Cleanup before return
      vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
      return false;
    }

    // Wait for data to come in
    sleep(1);

    // Get data
    // Global position retrieved via subscription
    Telemetry::TypeMap<TOPIC_GPS_FUSED>::type currentSubscriptionGPS;
    Telemetry::TypeMap<TOPIC_GPS_FUSED>::type originSubscriptionGPS;
    // Global position retrieved via broadcast
    Telemetry::GlobalPosition currentBroadcastGP;
    // Convert position offset from first position to local coordinates
    Telemetry::Vector3f localOffset;

    currentSubscriptionGPS = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
    originSubscriptionGPS  = currentSubscriptionGPS;
    localOffsetFromGpsOffset(localOffset,
                             static_cast<void*>(&currentSubscriptionGPS),
                             static_cast<void*>(&originSubscriptionGPS));

    // Get the broadcast GP since we need the height for position.z
    currentBroadcastGP = vehicle->broadcast->getGlobalPosition();


    // Get initial offset. We will update this in a loop later.
    double xOffsetRemaining = offset->x - localOffset.x;
    double yOffsetRemaining = offset->y - localOffset.y;
    double zOffsetRemaining = offset->z - (-localOffset.z);

    // Conversions
    double yawDesiredRad     = DEG2RAD * yawDesired;
    double yawThresholdInRad = DEG2RAD * yawThresholdInDeg;

    //! Get Euler angle

    // Quaternion retrieved via subscription
    Telemetry::TypeMap<TOPIC_QUATERNION>::type subscriptionQ;
    // Quaternion retrieved via broadcast
    Telemetry::Quaternion broadcastQ;

    double yawInRad;

    subscriptionQ = vehicle->subscribe->getValue<TOPIC_QUATERNION>();
    yawInRad = toEulerAngle((static_cast<void*>(&subscriptionQ))).z;

    int   elapsedTimeInMs     = 0;
    int   withinBoundsCounter = 0;
    int   outOfBounds         = 0;
    int   brakeCounter        = 0;
    int   speedFactor         = 2;
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
    if (offset->x > 0)
        position.x = (offset->x < speedFactor) ? offset->x : speedFactor;
    else if (offset->x < 0)
        position.x = (offset->x > -1 * speedFactor) ? offset->x : -1 * speedFactor;
    else
        position.x = 0;

    if (offset->y > 0)
        position.y = (offset->y < speedFactor) ? offset->y : speedFactor;
    else if (offset->y < 0)
        position.y = (offset->y > -1 * speedFactor) ? offset->y : -1 * speedFactor;
    else
        position.y = 0;

    position.z = currentBroadcastGP.height + offset->z; //Since subscription cannot give us a relative height, use broadcast.

    //! Main closed-loop receding setpoint position control
    while (elapsedTimeInMs < timeoutInMilSec)
    {
        positionAndYawCtrl(&position, (float32_t)(yawDesiredRad / DEG2RAD));
        usleep((useconds_t)(cycleTimeInMs * 1000));
        elapsedTimeInMs += cycleTimeInMs;

        //! Get current position in required coordinates and units
        subscriptionQ = vehicle->subscribe->getValue<TOPIC_QUATERNION>();
        yawInRad      = toEulerAngle((static_cast<void*>(&subscriptionQ))).z;
        currentSubscriptionGPS = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
        localOffsetFromGpsOffset(localOffset,
                               static_cast<void*>(&currentSubscriptionGPS),
                               static_cast<void*>(&originSubscriptionGPS));

        // Get the broadcast GP since we need the height for position.z
        currentBroadcastGP = vehicle->broadcast->getGlobalPosition();

        //! See how much farther we have to go
        xOffsetRemaining = offset->x - localOffset.x;
        yOffsetRemaining = offset->y - localOffset.y;
        zOffsetRemaining = offset->z - (-localOffset.z);

        //! See if we need to modify the setpoint
        if (std::abs(xOffsetRemaining) < speedFactor)
            position.x = (float)xOffsetRemaining;

        if (std::abs(yOffsetRemaining) < speedFactor)
            position.y = (float)yOffsetRemaining;

        if (std::abs(xOffsetRemaining) < posThresholdInM &&
            std::abs(yOffsetRemaining) < posThresholdInM &&
            std::abs(zOffsetRemaining) < zDeadband &&
            std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad) {
            //! 1. We are within bounds; start incrementing our in-bound counter
            withinBoundsCounter += cycleTimeInMs;
        } else {
            if (withinBoundsCounter != 0) {
                //! 2. Start incrementing an out-of-bounds counter
                outOfBounds += cycleTimeInMs;
            }
        }
        //! 3. Reset withinBoundsCounter if necessary
        if (outOfBounds > outOfControlBoundsTimeLimit) {
          withinBoundsCounter = 0;
          outOfBounds         = 0;
        }
        //! 4. If within bounds, set flag and break
        if (withinBoundsCounter >= withinControlBoundsTimeReqmt) {
          break;
        }
    }

  //! Set velocity to zero, to prevent any residual velocity from position
  //! command

    while (brakeCounter < withinControlBoundsTimeReqmt) {
        vehicle->control->emergencyBrake();
        usleep((useconds_t)cycleTimeInMs * 10);
        brakeCounter += cycleTimeInMs;
    }

    if (elapsedTimeInMs >= timeoutInMilSec) {
        std::cout << "Task timeout!\n";
        ack = vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
        if (ACK::getError(ack)) {
            std::cout << "Error unsubscribing; please restart the drone/FC to get "
                     "back to a clean state.\n";
        }
        return ACK::FAIL;
    }


    ack = vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
    if (ACK::getError(ack)) {
        std::cout
            << "Error unsubscribing; please restart the drone/FC to get back "
                "to a clean state.\n";
    }

  return ACK::SUCCESS;
}

void FlightController::velocityAndYawRateCtrl(Vector3f *velocity, float32_t yaw) {
    if(!isEmergencyState()) {
        vehicle->control->velocityAndYawRateCtrl(velocity->x, velocity->y, velocity->z, yaw);
    }
}
void FlightController::positionAndYawCtrl(Vector3f* position, float32_t yaw) {
    if(!isEmergencyState()) {
        vehicle->control->positionAndYawCtrl(position->x, position->y, position->z, yaw);
    }
}

// Helper Functions

/*! Very simple calculation of local NED offset between two pairs of GPS
/coordinates.
    Accurate when distances are small.
    Functions given by DJI Programming Guide
!*/
void FlightController::localOffsetFromGpsOffset(Telemetry::Vector3f& deltaNed,
                         void* target, void* origin) {
    auto subscriptionTarget = (Telemetry::GPSFused*)target;
    auto subscriptionOrigin = (Telemetry::GPSFused*)origin;
    double deltaLon = subscriptionTarget->longitude - subscriptionOrigin->longitude;
    double deltaLat = subscriptionTarget->latitude - subscriptionOrigin->latitude;
    deltaNed.x = (float32_t)(deltaLat * C_EARTH);
    deltaNed.y = (float32_t)(deltaLon * C_EARTH *
            cos(subscriptionTarget->latitude / 2.0 + subscriptionOrigin->latitude / 2.0));
    deltaNed.z = subscriptionTarget->altitude - subscriptionOrigin->altitude;
}

Telemetry::Vector3f FlightController::toEulerAngle(void* quaternionData) {
    Telemetry::Vector3f    ans;
    auto quaternion = (Telemetry::Quaternion*)quaternionData;

    double q2sqr = quaternion->q2 * quaternion->q2;
    double t0 =
            -2.0 * (q2sqr + quaternion->q3 * quaternion->q3) + 1.0;
    double t1 =
            +2.0 * (quaternion->q1 * quaternion->q2 + quaternion->q0 * quaternion->q3);
    double t2 =
            -2.0 * (quaternion->q1 * quaternion->q3 - quaternion->q0 * quaternion->q2);
    double t3 =
            +2.0 * (quaternion->q2 * quaternion->q3 + quaternion->q0 * quaternion->q1);
    double t4 =
            -2.0 * (quaternion->q1 * quaternion->q1 + q2sqr) + 1.0;

    t2 = (t2 > 1.0) ? 1.0 : t2;
    t2 = (t2 < -1.0) ? -1.0 : t2;

    ans.x = (float32_t )asin(t2);       // pitch
    ans.y = (float32_t )atan2(t3, t4);  // roll
    ans.z = (float32_t )atan2(t1, t0);  // yaw

    return ans;
}

bool FlightController::startGlobalPositionBroadcast()
{
    uint8_t freq[16];

    /* Channels definition for A3/N3/M600
    * 0 - Timestamp
    * 1 - Attitude Quaternions
    * 2 - Acceleration
    * 3 - Velocity (Ground Frame)
    * 4 - Angular Velocity (Body Frame)
    * 5 - Position
    * 6 - GPS Detailed Information
    * 7 - RTK Detailed Information
    * 8 - Magnetometer
    * 9 - RC Channels Data
    * 10 - Gimbal Data
    * 11 - Flight Status
    * 12 - Battery Level
    * 13 - Control Information
    */
    freq[0]  = DataBroadcast::FREQ_HOLD;
    freq[1]  = DataBroadcast::FREQ_HOLD;
    freq[2]  = DataBroadcast::FREQ_HOLD;
    freq[3]  = DataBroadcast::FREQ_HOLD;
    freq[4]  = DataBroadcast::FREQ_HOLD;
    freq[5]  = DataBroadcast::FREQ_50HZ; // This is the only one we want to change
    freq[6]  = DataBroadcast::FREQ_HOLD;
    freq[7]  = DataBroadcast::FREQ_HOLD;
    freq[8]  = DataBroadcast::FREQ_HOLD;
    freq[9]  = DataBroadcast::FREQ_HOLD;
    freq[10] = DataBroadcast::FREQ_HOLD;
    freq[11] = DataBroadcast::FREQ_HOLD;
    freq[12] = DataBroadcast::FREQ_HOLD;
    freq[13] = DataBroadcast::FREQ_HOLD;

    ACK::ErrorCode ack = vehicle->broadcast->setBroadcastFreq(freq, 1);
    if (ACK::getError(ack)) {
        ACK::getErrorCodeMessage(ack, __func__);
        return false;
    } else {
        return true;
    }
}

void FlightController::sendDataToMSDK(uint8_t *data, uint8_t length) {
    vehicle->moc->sendDataToMSDK(data, length);
}

void FlightController::emergencyStop() {
    DERROR("Emergency break set !");
    emergencyState = true;
    velocityActivated = false;
    vehicle->control->emergencyBrake();
}

void FlightController::emergencyRelease() {
    DERROR("Emergency break released !");
    emergencyState = false;
}