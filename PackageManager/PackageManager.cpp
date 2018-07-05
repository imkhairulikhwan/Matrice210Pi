/*! @file PackageManager.cpp
 *  @version 1.0
 *  @date Jul 05 2018
 *  @author Jonathan Michel
 */

#include "PackageManager.h"

PackageManager* PackageManager::instance = nullptr;

PackageManager::PackageManager() {
    for (bool &packagesStatu : packagesStatus) {
        packagesStatu = false;
    }
}

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
        ACK::getErrorCodeMessage(ack, __func__);
        return false;
    }
    return true;
}


bool PackageManager::subscribe(int index, TopicName *topics, int numTopic, uint16_t frequency, bool enableTimestamp) {
    if(!vehicleInstanced() || !validIndex(index))
        return false;

    bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
            index, numTopic, topics,
            enableTimestamp, frequency);
    if (pkgStatus) {
        packagesStatus[index] = true;

        ACK::ErrorCode ack = vehicle->subscribe->startPackage(index, timeout);
        if (ACK::getError(ack) != ACK::SUCCESS)
        {
            DERROR("Error starting package %u (%u Hz)", index, frequency);
            ACK::getErrorCodeMessage(ack, __func__);
            unsubscribe(index);
            return false;
        }
    } else {
        DERROR("Error initializing package %u (%u Hz)", index, frequency);
    }
    return pkgStatus;
}

bool PackageManager::unsubscribe(int index) {
    if(!vehicleInstanced() || !validIndex(index))
        return false;

    ACK::ErrorCode ack = vehicle->subscribe->removePackage(index, timeout);
    if (ACK::getError(ack)) {
        DERROR("Error unsubscribing package %u", index);
        return false;
    }

    return false;
}

bool PackageManager::vehicleInstanced() {
    if(vehicle == nullptr) {
        DERROR("PackageManager - Vehicle not instanced. Call setVehicle() !");
        return false;
    }
    return true;
}

bool PackageManager::validIndex(int index) {
    if(index < 0 || index >= MAX_NUMBER_OF_PACKAGE) {
        DERROR("Invalid index : [%d], must be in range 0 to %d.",
               index, MAX_NUMBER_OF_PACKAGE);
        return false;
    }
    return true;
}