/*! @file PackageManager.cpp
 *  @version 1.0
 *  @date Jul 05 2018
 *  @author Jonathan Michel
 *  @brief PackageManager.h implementation
 */

#include "PackageManager.h"

#include "../util/Log.h"

using namespace M210;

pthread_mutex_t PackageManager::packageManager_mutex = PTHREAD_MUTEX_INITIALIZER;

PackageManager::PackageManager() {
    for (bool &i : packageAvailable) {
        i = true;
    }
}

void PackageManager::setVehicle(const Vehicle *vehicle) {
    this->vehicle = vehicle;
}

bool PackageManager::verify() const {
    ACK::ErrorCode ack;
    ack = vehicle->subscribe->verify(timeout);
    if (ACK::getError(ack) != ACK::SUCCESS)  {
        DERROR("Version match failed !");
        ACK::getErrorCodeMessage(ack, __func__);
        return false;
    }
    return true;
}

int PackageManager::subscribe(TopicName *topics, int numTopic, uint16_t frequency, bool enableTimestamp) {
    if(!isVehicleInstanced())
        return VEHICLE_NOT_INSTANCED;

    if(!verify())
        return VERIFY_FAILED;

    // Try to allocate package
    int pkgIndex = allocatePackage();
    if(pkgIndex == PACKAGE_UNAVAILABLE) {
        DERROR("Cannot start package. All packages are used");
        return PACKAGE_UNAVAILABLE;
    }

    // Initialize current package
    bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
            pkgIndex, numTopic, topics,
            enableTimestamp, frequency);
    if (pkgStatus) {
        // Subscribe to current package
        ACK::ErrorCode ack = vehicle->subscribe->startPackage(pkgIndex, timeout);
        if (ACK::getError(ack) != ACK::SUCCESS)
        {
            DERROR("Error starting package %u (%u Hz)", pkgIndex, frequency);
            ACK::getErrorCodeMessage(ack, __func__);
            unsubscribe(pkgIndex);
            return START_PACKAGE_FAILED;
        }
    } else {
        DERROR("Error initializing package %u (%u Hz)", pkgIndex, frequency);
        releasePackage(pkgIndex);
        return INIT_PACKAGE_FAILED;
    }
    return pkgIndex;
}

int PackageManager::unsubscribe(int index) {
    if(!isVehicleInstanced())
        return VEHICLE_NOT_INSTANCED;

    if(!validIndex(index))
        return INVALID_INDEX;

    // Remove package from aircraft
    ACK::ErrorCode ack = vehicle->subscribe->removePackage(index, timeout);
    // Release package in local
    releasePackage(index);
    if (ACK::getError(ack)) {
        DERROR("Error unsubscribing package %u", index);
        return UNSUBSCRIPTION_FAILED;
    }

    return 0;
}

bool PackageManager::isVehicleInstanced() const {
    if(vehicle == nullptr) {
        DERROR("Vehicle not instanced. Call setVehicle() first !");
        return false;
    }
    return true;
}

bool PackageManager::validIndex(int index) const {
    // Separate test for call with PACKAGE_UNAVAILABLE to avoid useless DERROR
    if(index == PACKAGE_UNAVAILABLE)
        return false;

    if(index < 0 || index >= DataSubscription::MAX_NUMBER_OF_PACKAGE) {
        DERROR("Invalid index : [%d], must be in range 0 to %d.",
               index, DataSubscription::MAX_NUMBER_OF_PACKAGE-1);
        return false;
    }
    return true;
}

int PackageManager::allocatePackage() {
    pthread_mutex_lock(&packageManager_mutex);
    for(int i = 0; i < DataSubscription::MAX_NUMBER_OF_PACKAGE; i++) {
        if (packageAvailable[i]) {
            packageAvailable[i] = false;
            pthread_mutex_unlock(&packageManager_mutex);
            return i;
        }
    }
    pthread_mutex_unlock(&packageManager_mutex);
    return PACKAGE_UNAVAILABLE;
}

void PackageManager::releasePackage(int index) {
    if(validIndex(index)) {
        pthread_mutex_lock(&packageManager_mutex);
        packageAvailable[index] = true;
        pthread_mutex_unlock(&packageManager_mutex);
    }
}


void PackageManager::clear() {
    for(int i = 0; i < DataSubscription::MAX_NUMBER_OF_PACKAGE; i++) {
        if(!packageAvailable[i]) {
            DSTATUS("Clear package : %u", i);
            unsubscribe(i);
        }
    }
}

