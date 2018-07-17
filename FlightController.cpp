/*! @file FlightController.cpp
 *  @version 1.0
 *  @date Jul 03 2018
 *  @author Jonathan Michel
 */

#include "FlightController.h"

#include "PackageManager/PackageManager.h"
#include "Mission/PositionMission.h"
#include "Mission/VelocityMission.h"
#include "Mission/PositionOffsetMission.h"
#include "ThreadManager/ThreadManager.h"

pthread_mutex_t FlightController::movingMode_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t FlightController::emergencyState_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t FlightController::sendDataToMSDK_mutex = PTHREAD_MUTEX_INITIALIZER;

FlightController::FlightController() {
    this->emergencyState = false;
    this->linuxEnvironment = nullptr;
    this->vehicle = nullptr;
    this->flightControllerThreadRunning = false;
    this->movingMode = STOP;
}


void FlightController::setupVehicle(int argc, char** argv) {
    // Get vehicle
    do {
        // If linuxEnvironment is already set, an error occurred
        // delete linuxEnvironment and retry after 1 second
        if (linuxEnvironment != nullptr) {
            delete linuxEnvironment;
            delay_ms(1000);
        }
        linuxEnvironment = new LinuxSetup(argc, argv);

        vehicle = linuxEnvironment->getVehicle();
        // Retry to get vehicle until it works
        if(vehicle == nullptr) {
            DERROR("Vehicle not initialized, retrying...");
        }
    } while(vehicle == nullptr);

    bool errorMsgDisplayed = false;
    // Obtain control authority
    ACK::ErrorCode ack;
    do {
        ack = vehicle->obtainCtrlAuthority(1);
        if(ACK::getError(ack) != ACK::SUCCESS && !errorMsgDisplayed) {
            DERROR("Obtain control authority failed, retrying...");
            errorMsgDisplayed = true;
        }
    }while(ACK::getError(ack) != ACK::SUCCESS);

    launchFlightControllerThread();
}



bool FlightController::monitoredTakeoff(int timeout) {
    DSTATUS("Monitored takeoff launched");
    // Telemetry: Verify the subscription
    if(!PackageManager::getInstance()->verify())
        return false;

    /*/ Subscribe to package
            index : 0
            frequency : 10Hz
            content : flight status and flight mode
    //*/
    uint16_t frequency      = 10;
    TopicName topics[]      = {
            TOPIC_STATUS_FLIGHT,
            TOPIC_STATUS_DISPLAYMODE
    };
    int  numTopics          = sizeof(topics) / sizeof(topics[0]);

    int pkgIndex = PackageManager::getInstance()->subscribe(topics, numTopics, frequency, false);
    if(pkgIndex < 0) {
        DERROR("Monitored takeoff - Failed to start package");
        return false;
    }

    // Start takeoff
    ACK::ErrorCode ack = vehicle->control->takeoff(timeout);
    if (ACK::getError(ack) != ACK::SUCCESS)
    {
        DERROR("Start take off failed");
        ACK::getErrorCodeMessage(ack, __func__);
        PackageManager::getInstance()->unsubscribe(pkgIndex);
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
        delay_ms(100);
    }

    if (motorsNotStarted == timeoutCycles) {
        cout << "Takeoff failed. Motors are not spinning." << endl;
        // Cleanup
        PackageManager::getInstance()->unsubscribe(pkgIndex);
        return false;
    } else {
        cout << "Motors spinning...\n";
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
        delay_ms(100);
    }

    if (stillOnGround == timeoutCycles) {
        cout << "Takeoff failed. Aircraft is still on the ground, but the "
                   "motors are spinning."
                << endl;
        // Cleanup
        PackageManager::getInstance()->unsubscribe(pkgIndex);
        return false;
    } else {
        cout << "Ascending...\n";
    }


    // Final check: Finished takeoff
    while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
                VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
           vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
                VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF) {
        delay_ms(1000);
    }

    if (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
            VehicleStatus::DisplayMode::MODE_P_GPS ||
        vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
            VehicleStatus::DisplayMode::MODE_ATTITUDE) {
        cout << "Successful takeoff!\n";
    } else {
        cout
        << "Takeoff finished, but the aircraft is in an unexpected mode. "
        "Please connect DJI GO.\n";
        PackageManager::getInstance()->unsubscribe(pkgIndex);
        return false;
    }


    // Cleanup
    cout << "clear " << pkgIndex << endl;
    PackageManager::getInstance()->unsubscribe(pkgIndex);

    return true;
}

/**
 *  Monitored landing. Return true if success
*/
bool FlightController::monitoredLanding(int timeout)
{
    // Telemetry: Verify the subscription
    if(!PackageManager::getInstance()->verify())
        return false;

    /*/ Subscribe to package
            index : 0
            frequency : 10Hz
            content : flight status and flight mode
    //*/
    uint16_t frequency      = 10;
    TopicName topics[]      = {
            TOPIC_STATUS_FLIGHT,
            TOPIC_STATUS_DISPLAYMODE
    };
    int  numTopics          = sizeof(topics) / sizeof(topics[0]);
    int pkgIndex = PackageManager::getInstance()->subscribe(topics, numTopics, frequency, false);
    if(pkgIndex < 0) {
        DERROR("Monitored landing - Failed to start package");
        return false;
    }

    // Start landing
    ACK::ErrorCode landingStatus = vehicle->control->land(timeout);
    if (ACK::getError(landingStatus) != ACK::SUCCESS) {
        DERROR("Start landing failed");
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
        delay_ms(100);
    }

    if (landingNotStarted == timeoutCycles) {
        cout << "Landing failed. Aircraft is still in the air." << endl;
        // Cleanup before return
        PackageManager::getInstance()->unsubscribe(pkgIndex);
        return false;
    } else {
        cout << "Landing...\n";
    }

    // Second check: Finished landing
    while ( vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
                VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
            vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() ==
                VehicleStatus::FlightStatus::IN_AIR) {
        delay_ms(1000);
    }

    if (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
            VehicleStatus::DisplayMode::MODE_P_GPS ||
        vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
            VehicleStatus::DisplayMode::MODE_ATTITUDE) {
        cout << "Successful landing!\n";
    } else {
        cout << "Landing finished, but the aircraft is in an unexpected mode. "
                   "Please connect DJI GO.\n";
        PackageManager::getInstance()->unsubscribe(pkgIndex);
        return false;
    }

    // Cleanup
    PackageManager::getInstance()->unsubscribe(pkgIndex);

    return true;
}

void FlightController::launchFlightControllerThread() {
    if(!flightControllerThreadRunning) {
        flightControllerThreadRunning = true;
        ThreadManager::start("flightCtrThread",
                              &flightControllerThreadID, &flightControllerThreadAttr,
                              flightControllerThread, (void*)this);
    }
}

void FlightController::stopFlightControllerThread() {
    flightControllerThreadRunning = false;
    ThreadManager::stop(&flightControllerThreadID);
}

void *FlightController::flightControllerThread(void *param) {
    auto fc = (FlightController*) param;
    while(fc->flightControllerThreadRunning) {
        switch(fc->getMovingMode()) {
            case WAIT:

                break;
            case STOP:
                DSTATUS("Aircraft stopped");
                fc->getVehicle()->control->emergencyBrake();
                // Delete position offset mission TODO Remove ?
                if(fc->positionOffsetMission != nullptr) {
                    delete fc->positionOffsetMission;
                    fc->positionOffsetMission = nullptr;
                }
                fc->setMovingMode(WAIT);
                break;
            case POSITION:
                fc->positionMission->update();
                fc->setMovingMode(WAIT);
                break;
            case VELOCITY:
                fc->velocityMission->update();
                break;
            case POSITION_OFFSET:
                fc->positionOffsetMission->update();
                break;
        }
    }
    return nullptr;
}

void FlightController::moveByPosition(Vector3f *position, float yaw) {
    // Mission parameters
    if(positionMission == nullptr)
        positionMission = new PositionMission(this);
    positionMission->move(position, yaw);
    movingMode = POSITION;

}

void FlightController::moveByVelocity(Vector3f *velocity, float yaw) {
    // Mission parameters
    if(velocityMission == nullptr)
        velocityMission = new VelocityMission(this);
    velocityMission->move(velocity, yaw);
    movingMode = VELOCITY;
}


void FlightController::moveByPositionOffset(Vector3f* offset, float yaw,
                                            float posThreshold, float yawThreshold)
{
    // Mission parameters
    if(positionOffsetMission == nullptr)
        positionOffsetMission = new PositionOffsetMission(this);
    positionOffsetMission->move(offset, yaw,
                                    posThreshold, yawThreshold);
    movingMode = POSITION_OFFSET;
}

void FlightController::stopAircraft() {
    movingMode = STOP;
    PackageManager::getInstance()->clear();
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

/*! Very simple calculation of local NED offset between two pairs of GPS coordinates.
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

void FlightController::emergencyStop() {
    vehicle->control->emergencyBrake();
    setEmergencyState(true);
    setMovingMode(STOP);
    DERROR("Emergency break set !");
}

void FlightController::emergencyRelease() {
    setEmergencyState(false);
    DSTATUS("Emergency break released !");
}

void FlightController::sendDataToMSDK(uint8_t *data, size_t length) {
    pthread_mutex_lock(&sendDataToMSDK_mutex);
    vehicle->moc->sendDataToMSDK(data, (uint8_t)length);
    pthread_mutex_unlock(&sendDataToMSDK_mutex);
}

void FlightController::setMovingMode(FlightController::movingMode_ mode) {
    pthread_mutex_lock(&movingMode_mutex);
    movingMode = mode;
    pthread_mutex_unlock(&movingMode_mutex);
}

void FlightController::setEmergencyState(bool state) {
    pthread_mutex_lock(&emergencyState_mutex);
    emergencyState = state;
    pthread_mutex_unlock(&emergencyState_mutex);

}