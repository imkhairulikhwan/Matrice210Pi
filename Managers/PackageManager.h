/*! @file PackageManager.h
 *  @version 1.0
 *  @date Jul 05 2018
 *  @author Jonathan Michel
 *  @brief This class provides to user an user friendly interface
 *  to manage subscription. More details below
 */

#ifndef MATRICE210_PACKAGEMANAGER_H
#define MATRICE210_PACKAGEMANAGER_H


#include <pthread.h>

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

    int pkgIndex = PackageManager::instance().subscribe(topics,
        numTopics, frequency, enableTimestamp);
    if(pkgIndex < 0) {
        DERROR("Monitored takeoff - Failed to start package");
        return false;
    }
 *
 *
 */

namespace M210 {
    class PackageManager : public Singleton<PackageManager> {
    public:
        enum RETURN_ERROR_CODE {
            VEHICLE_NOT_INSTANCED = -7,
            VERIFY_FAILED,
            INVALID_INDEX,
            START_PACKAGE_FAILED,
            INIT_PACKAGE_FAILED,
            UNSUBSCRIPTION_FAILED,
            PACKAGE_UNAVAILABLE     // -1
        };
    private:
        const Vehicle *vehicle = nullptr;
        int timeout{1};             /*!< DJI subscription method call timeout */
        bool packageAvailable[DataSubscription::MAX_NUMBER_OF_PACKAGE]; /*!< Available packages */
        /**
         * Verify if setVehicle() has been called
         * @return true is vehicle has been instanced
         */
        bool isVehicleInstanced() const;

        /**
         * Verify that index is a valid number
         * Range is 0 to DataSubscription::MAX_NUMBER_OF_PACKAGE -1
         * @param index Package index to verify
         * @return true if valid
         */
        bool validIndex(int index) const;

        /**
         * Verify that a package is available.
         * See DataSubscription::MAX_NUMBER_OF_PACKAGE
         * @return PackageManager::PACKAGE_UNAVAILABLE if all packages are used
         * package number if a package was allocated
         */
        int allocatePackage();

        /**
         * Set package available for ne w allocation
         * @param index Package index to set available
         *
         */
        void releasePackage(int index);

        /**
        * Verify version match
        * @return true if version match
        */
        bool verify() const;

        // Mutex
        static pthread_mutex_t packageManager_mutex;
    public:
        /**
        *  PackageManager is a singleton
        */
        PackageManager();

        /**
         * Has to be called before usage to define vehicle to send package
         * @param vehicle Pointer to used vehicle
         */
        void setVehicle(const Vehicle *vehicle);

        /**
         * Try to allocate package. Setup members of package and start it
         * @param topics List of Topic Names to subscribe in the package
         * @param numTopic Number of topics in topics list
         * @param frequency Package frequency
         * @param enableTimestamp Enable send of transmission package time
         * @return Negative value of PackageManager::RETURN_ERROR_CODE if subscription failed
         * Positive package index if success
         */
        int subscribe(TopicName *topics, int numTopic, uint16_t frequency, bool enableTimestamp);

        /**
         * Unsubscribe from indexed topic
         * @param index Package index
         * @return Negative value of PackageManager::RETURN_ERROR_CODE if unsubscription failed
         * 0 if success
         */
        int unsubscribe(int index);

        /**
         * Unsubscribe to all subscribed packages
         */
        void clear();
    };
}
#endif //MATRICE210_PACKAGEMANAGER_H
