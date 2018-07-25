/*! @file MonitoredMission.cpp
 *  @version 1.0
 *  @date Jul 25 2018
 *  @author Jonathan Michel
 */

#include "MonitoredMission.h"

#include "../Aircraft/FlightController.h"
#include "../Managers/PackageManager.h"
#include "../util/timer.h"

MonitoredMission::MonitoredMission(FlightController *flightController) {
    this->flightController = flightController;
}

bool MonitoredMission::takeOff(int timeout) const {
    DSTATUS("Take-off launched");
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
        DERROR("Take-off - Failed to start package");
        return false;
    }

    // Start takeoff
    ACK::ErrorCode ack = flightController->getVehicle()->control->takeoff(timeout);
    if (ACK::getError(ack) != ACK::SUCCESS) {
        DERROR("Start take-off failed");
        ACK::getErrorCodeMessage(ack, __func__);
        PackageManager::instance().unsubscribe(pkgIndex);
        return false;
    }

    // First check: Motors started
    int motorsNotStarted = 0;
    int timeoutCycles = 20;

    while (flightController->getVehicle()->subscribe->getValue<TOPIC_STATUS_FLIGHT>() !=
           VehicleStatus::FlightStatus::ON_GROUND &&
            flightController->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
           VehicleStatus::DisplayMode::MODE_ENGINE_START &&
           motorsNotStarted < timeoutCycles) {
        motorsNotStarted++;
        delay_ms(100);
    }

    if (motorsNotStarted == timeoutCycles) {
        DERROR("Take-off failed. Motors are not spinning");
        // Cleanup
        PackageManager::instance().unsubscribe(pkgIndex);
        return false;
    }

    // Second check: In air
    int stillOnGround = 0;
    timeoutCycles = 110;

    while (flightController->getVehicle()->subscribe->getValue<TOPIC_STATUS_FLIGHT>() !=
           VehicleStatus::FlightStatus::IN_AIR &&
           (flightController->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
            VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
                   flightController->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
            VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF) &&
           stillOnGround < timeoutCycles) {
        stillOnGround++;
        delay_ms(100);
    }

    if (stillOnGround == timeoutCycles) {
        DERROR("Takeoff failed. Aircraft is still on the ground, but the motors are spinning");
        // Cleanup
        PackageManager::instance().unsubscribe(pkgIndex);
        return false;
    }

    // Final check: Finished takeoff
    while (flightController->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
           VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
           flightController->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
           VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF) {
        delay_ms(1000);
    }

    if (flightController->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
        VehicleStatus::DisplayMode::MODE_P_GPS ||
        flightController->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
        VehicleStatus::DisplayMode::MODE_ATTITUDE) {
        DSTATUS("Successful takeoff!");
    } else {
        DERROR("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO");
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
bool MonitoredMission::landing(int timeout) const {
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
        DERROR("Landing - Failed to start package");
        return false;
    }

    // Start landing
    ACK::ErrorCode landingStatus = flightController->getVehicle()->control->land(timeout);
    if (ACK::getError(landingStatus) != ACK::SUCCESS) {
        DERROR("Start landing failed");
        ACK::getErrorCodeMessage(landingStatus, __func__);
        return false;
    }

    // First check: Landing started
    int landingNotStarted = 0;
    int timeoutCycles = 20;

    while (flightController->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
           VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
           landingNotStarted < timeoutCycles) {
        landingNotStarted++;
        delay_ms(100);
    }

    if (landingNotStarted == timeoutCycles) {
        DERROR("Landing failed. Aircraft is still in the air");
        // Cleanup before return
        PackageManager::instance().unsubscribe(pkgIndex);
        return false;
    }

    // Second check: Finished landing
    while (flightController->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
           VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
           flightController->getVehicle()->subscribe->getValue<TOPIC_STATUS_FLIGHT>() ==
           VehicleStatus::FlightStatus::IN_AIR) {
        delay_ms(1000);
    }

    if (flightController->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
        VehicleStatus::DisplayMode::MODE_P_GPS ||
        flightController->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
        VehicleStatus::DisplayMode::MODE_ATTITUDE) {
        DSTATUS("Successful landing!");
    } else {
        DERROR("Landing finished, but the aircraft is in an unexpected mode. Please connect DJI GO");
        PackageManager::instance().unsubscribe(pkgIndex);
        return false;
    }

    // Cleanup
    PackageManager::instance().unsubscribe(pkgIndex);

    return true;
}