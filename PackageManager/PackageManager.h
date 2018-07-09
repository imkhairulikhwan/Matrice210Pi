/*! @file PackageManager.h
 *  @version 1.0
 *  @date Jul 05 2018
 *  @author Jonathan Michel
 */

#ifndef MATRICE210_PACKAGEMANAGER_H
#define MATRICE210_PACKAGEMANAGER_H

#include "pthread.h"

#include <dji_vehicle.hpp>

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

/**
 * Data Subscription is a paradigm proposed by DJI, it offers
 * real-time telemetry data transmission from the flight controller
 * to the Onboard SDK.
 * User can choose a set of Topics or Subscription data sets, add them
 * to a Subscription package and configure the package to arrive on
 * preferred frequency. There are total of five packages available.
 * Each package has fixed-size buffer of 300-Bytes allowing
 * user to add as many Telemetry Topics per package as desired.
 *
 * PackageManager is a singleton class providing easy use of
 * package API. When user need a package, subscribe() method
 * return used packageId (0 to 5). Subscription errors
 * (no more package available, init or start errror from API, ...)
 * are handle. Here is an example, user want STATUS_FLIGHT and
 * DISPLAYMODE 10 times par seconds :
 *
    uint16_t frequency      = 10;
    TopicName topics[]      = {
            TOPIC_STATUS_FLIGHT,
            TOPIC_STATUS_DISPLAYMODE
    };
    int  numTopics          = sizeof(topics) / sizeof(topics[0]);
    boolean enableTimestamp = false;

    int pkgIndex = PackageManager::getInstance()->subscribe(topics,
        numTopics, frequency, enableTimestamp);
    if(pkgIndex == PackageManager::PACKAGE_UNAVAILABLE) {
        DERROR("Monitored takeoff - Failed to start package");
        return false;
    }
 *
 *
 */

class PackageManager  {
public:
    const static int PACKAGE_UNAVAILABLE{-1};
private:
    static PackageManager* instance;
    Vehicle* vehicle = nullptr;
    int timeout{1};             /*!< DJI subscription method call timeout */
    int packageCnt{0};          /*!< Next available package */
    /**
     *  PackageManager is a singleton
     */
    PackageManager();
    /**
     * Verify if setVehicle() has been called
     * @return true is vehicle has been instanced
     */
    bool isVehicleInstanced();
    /**
     * Verify that index is a valid number
     * Range is 0 to DataSubscription::MAX_NUMBER_OF_PACKAGE -1
     * @param index Package index to verify
     * @return true if valid
     */
    bool validIndex(int index);
    /**
     * Verify that a package is available.
     * See DataSubscription::MAX_NUMBER_OF_PACKAGE
     * @return PackageManager::PACKAGE_UNAVAILABLE if all packages are used
     * package number if a package was allocated
     */
    int allocatePackage();
    // Mutex
    static pthread_mutex_t packageManager_mutex;
public:
    /**
     * Singleton call method
     * @return pointer to PackageManager singleton
     */
    static PackageManager* getInstance();
    /**
     * Has to be called before usage to define vehicle to send package
     * @param vehicle Pointer to used vehicle
     */
    void setVehicle(Vehicle* vehicle);
    /**
     * Verify version match
     * @return true if version match
     */
    bool verify();
    /**
     * Try to allocate package. Setup members of package and start it
     * @param topics List of Topic Names to subscribe in the package
     * @param numTopic Number of topics in topics list
     * @param frequency Package frequency
     * @param enableTimestamp Enable send of transmission package time
     * @return PackageManager::PACKAGE_UNAVAILABLE if subscription failed
     * Package index if success
     * TODO Add others return error codes
     */
    int subscribe(TopicName *topics, int numTopic, uint16_t frequency, bool enableTimestamp);
    /**
     * Unsubscribe from indexed topic
     * @param index Package index
     * @return true if success, false if error
     */
    bool unsubscribe(int index);
    /**
     * Unsubscribed to all subscribed packages
     */
    void clear();
};

#endif //MATRICE210_PACKAGEMANAGER_H