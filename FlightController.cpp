/*! @file FlightController.cpp
 *  @version 1.0
 *  @date Jul 03 2018
 *  @author Jonathan Michel
 */

#include "FlightController.h"

#include <cmath>
#include <iostream>

#include "util/timer.h"
#include <dji_linux_helpers.hpp>

#include "Managers/PackageManager.h"
#include "Managers/ThreadManager.h"
#include "Mission/PositionMission.h"
#include "Mission/VelocityMission.h"
#include "Mission/PositionOffsetMission.h"

pthread_mutex_t FlightController::sendDataToMSDK_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t FlightController::emergencyState_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t FlightController::movingMode_mutex = PTHREAD_MUTEX_INITIALIZER;

FlightController::FlightController() {
    this->emergencyState = false;
    this->linuxEnvironment = nullptr;
    this->vehicle = nullptr;
    this->flightControllerThreadRunning = false;
    this->movingMode = STOP;
    // Mission
    positionMission = new PositionMission(this);
    velocityMission = new VelocityMission(this);
    positionOffsetMission = new PositionOffsetMission(this);
}


void FlightController::setupVehicle(int argc, char **argv) {
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
        if (vehicle == nullptr) {
            DERROR("Vehicle not initialized, retrying...");
        }
    } while (vehicle == nullptr);

    bool errorMsgDisplayed = false;
    // Obtain control authority
    ACK::ErrorCode ack;
    do {
        ack = vehicle->obtainCtrlAuthority(1);
        if (ACK::getError(ack) != ACK::SUCCESS && !errorMsgDisplayed) {
            DERROR("Obtain control authority failed, retrying...");
            errorMsgDisplayed = true;
        }
    } while (ACK::getError(ack) != ACK::SUCCESS);

    launchFlightControllerThread();
}


bool FlightController::monitoredTakeoff(int timeout) const {
    DSTATUS("Monitored takeoff launched");
    // Telemetry: Verify the subscription
    if (!PackageManager::instance().verify())
        return false;

    /*/ Subscribe to package
            index : 0
            frequency : 10Hz
            content : flight status and flight mode
    //*/
    uint16_t frequency = 10;
    TopicName topics[] = {
            TOPIC_STATUS_FLIGHT,
            TOPIC_STATUS_DISPLAYMODE
    };
    int numTopics = sizeof(topics) / sizeof(topics[0]);

    int pkgIndex = PackageManager::instance().subscribe(topics, numTopics, frequency, false);
    if (pkgIndex < 0) {
        DERROR("Monitored takeoff - Failed to start package");
        return false;
    }

    // Start takeoff
    ACK::ErrorCode ack = vehicle->control->takeoff(timeout);
    if (ACK::getError(ack) != ACK::SUCCESS) {
        DERROR("Start take off failed");
        ACK::getErrorCodeMessage(ack, __func__);
        PackageManager::instance().unsubscribe(pkgIndex);
        return false;
    }

    // First check: Motors started
    int motorsNotStarted = 0;
    int timeoutCycles = 20;

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
        PackageManager::instance().unsubscribe(pkgIndex);
        return false;
    } else {
        cout << "Motors spinning...\n";
    }


    // Second check: In air
    int stillOnGround = 0;
    timeoutCycles = 110;

    while (vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() !=
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
        PackageManager::instance().unsubscribe(pkgIndex);
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
        PackageManager::instance().unsubscribe(pkgIndex);
        return false;
    }


    // Cleanup
    PackageManager::instance().unsubscribe(pkgIndex);

    return true;
}

/**
 *  Monitored landing. Return true if success
*/
bool FlightController::monitoredLanding(int timeout) const {
    // Telemetry: Verify the subscription
    if (!PackageManager::instance().verify())
        return false;

    /*/ Subscribe to package
            index : 0
            frequency : 10Hz
            content : flight status and flight mode
    //*/
    uint16_t frequency = 10;
    TopicName topics[] = {
            TOPIC_STATUS_FLIGHT,
            TOPIC_STATUS_DISPLAYMODE
    };
    int numTopics = sizeof(topics) / sizeof(topics[0]);
    int pkgIndex = PackageManager::instance().subscribe(topics, numTopics, frequency, false);
    if (pkgIndex < 0) {
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
    int timeoutCycles = 20;

    while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
           VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
           landingNotStarted < timeoutCycles) {
        landingNotStarted++;
        delay_ms(100);
    }

    if (landingNotStarted == timeoutCycles) {
        cout << "Landing failed. Aircraft is still in the air." << endl;
        // Cleanup before return
        PackageManager::instance().unsubscribe(pkgIndex);
        return false;
    } else {
        cout << "Landing...\n";
    }

    // Second check: Finished landing
    while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
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
        PackageManager::instance().unsubscribe(pkgIndex);
        return false;
    }

    // Cleanup
    PackageManager::instance().unsubscribe(pkgIndex);

    return true;
}

void FlightController::launchFlightControllerThread() {
    if (!flightControllerThreadRunning) {
        flightControllerThreadRunning = true;
        ThreadManager::start("flightCtrThread",
                             &flightControllerThreadID, &flightControllerThreadAttr,
                             flightControllerThread, (void *) this);
    }
}

void FlightController::stopFlightControllerThread() {
    flightControllerThreadRunning = false;
    ThreadManager::stop(&flightControllerThreadID);
}

void *FlightController::flightControllerThread(void *param) {
    auto fc = (FlightController *) param;
    while (fc->flightControllerThreadRunning) {
        switch (fc->getMovingMode()) {
            case WAIT:

                break;
            case STOP:
                DSTATUS("Aircraft stopped");
                // TODO Remove if packages need to be keep while aircraft is stopped
                PackageManager::instance().clear();
                fc->setMovingMode(WAIT);
                break;
            case POSITION:
                fc->positionMission->update();
                fc->setMovingMode(WAIT);
                break;
            case VELOCITY:
                fc->velocityMission->update();
                delay_ms(20);
                break;
            case POSITION_OFFSET:
                fc->positionOffsetMission->update();
                delay_ms(fc->positionOffsetMission->getCycleTimeMs());
                break;
        }
    }
    return nullptr;
}

void FlightController::moveByPosition(const Vector3f *position, float yaw) {
    // Mission parameters
    positionMission->move(position, yaw);
    movingMode = POSITION;

}

void FlightController::moveByVelocity(const Vector3f *velocity, float yaw) {
    // Mission parameters
    velocityMission->move(velocity, yaw);
    movingMode = VELOCITY;
}


void FlightController::moveByPositionOffset(const Vector3f *offset, float yaw,
                                            float posThreshold, float yawThreshold) {
    positionOffsetMission->move(offset, yaw,
                                posThreshold, yawThreshold);
    movingMode = POSITION_OFFSET;
}

void FlightController::stopAircraft() {
    DSTATUS("Stop aircraft");
    // Stop aircraft
    getVehicle()->control->emergencyBrake();
    // Stop state machine sending moving commands
    movingMode = STOP;
}

void FlightController::velocityAndYawRateCtrl(const Vector3f *velocity, float yaw) const {
    if (!isEmergencyState()) {
        vehicle->control->velocityAndYawRateCtrl(velocity->x, velocity->y, velocity->z, yaw);
    }
}

void FlightController::positionAndYawCtrl(const Vector3f *position, float yaw) const {
    if (!isEmergencyState()) {
        vehicle->control->positionAndYawCtrl(position->x, position->y, position->z, yaw);
    }
}

void FlightController::emergencyStop() {
    // First of all set emergency state (to stop sending moving order) and stop aircraft
    setEmergencyState(true);
    vehicle->control->emergencyBrake();
    setMovingMode(STOP);
    DERROR("Emergency break set !");
}

void FlightController::emergencyRelease() {
    setEmergencyState(false);
    setMovingMode(STOP);
    DSTATUS("Emergency break released !");
}

void FlightController::sendDataToMSDK(const uint8_t *data, size_t length) const {
    pthread_mutex_lock(&sendDataToMSDK_mutex);
    vehicle->moc->sendDataToMSDK(const_cast<uint8_t*>(data), (uint8_t) length);
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

/*! Very simple calculation of local NED offset between two pairs of GPS coordinates.
    Accurate when distances are small.
    Functions given by DJI Programming Guide
!*/
void FlightController::localOffsetFromGpsOffset(Telemetry::Vector3f &deltaNed,
                                                const Telemetry::GPSFused *subscriptionTarget,
                                                const Telemetry::GPSFused *subscriptionOrigin) {
    double deltaLon = subscriptionTarget->longitude - subscriptionOrigin->longitude;
    double deltaLat = subscriptionTarget->latitude - subscriptionOrigin->latitude;
    deltaNed.x = (float32_t) (deltaLat * C_EARTH);
    deltaNed.y = (float32_t) (deltaLon * C_EARTH *
                              cos(subscriptionTarget->latitude / 2.0 + subscriptionOrigin->latitude / 2.0));
    deltaNed.z = subscriptionTarget->altitude - subscriptionOrigin->altitude;
}

Telemetry::Vector3f FlightController::toEulerAngle(const Telemetry::Quaternion *quaternion) {
    Telemetry::Vector3f ans;

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

    ans.x = (float32_t) asin(t2);       // pitch
    ans.y = (float32_t) atan2(t3, t4);  // roll
    ans.z = (float32_t) atan2(t1, t0);  // yaw

    return ans;
}