/*! @file PackageManager.cpp
 *  @version 1.0
 *  @date Jul 05 2018
 *  @author Jonathan Michel
 */

#include "PackageManager.h"

pthread_mutex_t PackageManager::packageManager_mutex = PTHREAD_MUTEX_INITIALIZER;
PackageManager* PackageManager::instance = nullptr;

PackageManager::PackageManager() = default;

PackageManager *PackageManager::getInstance() {
    if(instance == nullptr)
        instance = new PackageManager();
    return instance;
}

void PackageManager::setVehicle(Vehicle *vehicle) {
    this->vehicle = vehicle;
}

bool PackageManager::verify() {
    ACK::ErrorCode ack;
    ack = vehicle->subscribe->verify(timeout);
    if (ACK::getError(ack) != ACK::SUCCESS)  {
        DERROR("PackageManager verify error !");
        ACK::getErrorCodeMessage(ack, __func__);
        return false;
    }
    return true;
}


int PackageManager::subscribe(TopicName *topics, int numTopic, uint16_t frequency, bool enableTimestamp) {
    if(!isVehicleInstanced())
        return PACKAGE_UNAVAILABLE;

    int pkgIndex = allocatePackage();
    if(pkgIndex == PACKAGE_UNAVAILABLE) {
        DERROR("Cannot start package. All packages are used");
        return PACKAGE_UNAVAILABLE;
    }

    bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
            pkgIndex, numTopic, topics,
            enableTimestamp, frequency);
    if (pkgStatus) {
        ACK::ErrorCode ack = vehicle->subscribe->startPackage(pkgIndex, timeout);
        if (ACK::getError(ack) != ACK::SUCCESS)
        {
            DERROR("Error starting package %u (%u Hz)", pkgIndex, frequency);
            ACK::getErrorCodeMessage(ack, __func__);
            unsubscribe(pkgIndex);
            return PACKAGE_UNAVAILABLE;
        }
    } else {
        DERROR("Error initializing package %u (%u Hz)", pkgIndex, frequency);
    }
    return pkgIndex;
}

bool PackageManager::unsubscribe(int index) {
    if(!isVehicleInstanced() || !validIndex(index))
        return false;

    ACK::ErrorCode ack = vehicle->subscribe->removePackage(index, timeout);
    if (ACK::getError(ack)) {
        DERROR("Error unsubscribing package %u", index);
        return false;
    }

    return true;
}

bool PackageManager::isVehicleInstanced() {
    if(vehicle == nullptr) {
        DERROR("PackageManager - Vehicle not instanced. Call setVehicle() first !");
        return false;
    }
    return true;
}

bool PackageManager::validIndex(int index) {
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
    if(packageCnt < DataSubscription::MAX_NUMBER_OF_PACKAGE) {
        packageCnt++;
        pthread_mutex_unlock(&packageManager_mutex);
        return packageCnt-1;
    }
    pthread_mutex_unlock(&packageManager_mutex);
    return PACKAGE_UNAVAILABLE;
}

void PackageManager::clear() {
    pthread_mutex_lock(&packageManager_mutex);
    // packageCtn contain nextAvailable package index
    packageCnt--;
    for (packageCnt; packageCnt >= 0; packageCnt--) {
        unsubscribe(packageCnt);
    }
    packageCnt = 0;
    pthread_mutex_unlock(&packageManager_mutex);
}