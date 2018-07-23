/*! @file ActionData.cpp
 *  @version 1.0
 *  @date Jul 20 2018
 *  @author Jonathan Michel
 */

#include "ActionData.h"

#include <cstring>
#include <cstddef>

// TODO Find a way to do pop with RTTI (Templates ?!)

pthread_mutex_t ActionData::mutex = PTHREAD_MUTEX_INITIALIZER;

ActionData::ActionData(ActionId actionId, size_t size) {
    this->actionId = actionId;
    this->dataSize = size;
    this->dataPosCnt = 0;
    this->data = nullptr;
    if(size > 0)
        data = new char[size];
}


ActionData::~ActionData() {
    if(data != nullptr)
        delete[] data;
}

bool ActionData::checkSize(size_t size) {
    return (dataPosCnt + size <= this->dataSize);
}

bool ActionData::_push(char *c, size_t length) {
    pthread_mutex_lock(&mutex);
    if(!checkSize(length)) {
        DERROR("Unable to push data");
        pthread_mutex_unlock(&mutex);
        return false;
    }
    memcpy(data + dataPosCnt, c, length);
    dataPosCnt += length;
    pthread_mutex_unlock(&mutex);
    return true;
}

bool ActionData::push(char c) {
    return _push(&c, sizeof(char));
}

bool ActionData::push(unsigned u) {
    return _push((char *) &u, sizeof(unsigned));
}

bool ActionData::push(float f) {
    return _push((char *) &f, sizeof(float));
}

bool ActionData::push(Telemetry::Vector3f &v) {
    return _push((char *) &v, sizeof(Telemetry::Vector3f));
}

bool ActionData::popChar(char &c) {
    pthread_mutex_lock(&mutex);
    size_t length = sizeof(char);
    if(dataPosCnt < length) {
        DERROR("Unable to pop char");
        pthread_mutex_unlock(&mutex);
        return false;
    }
    dataPosCnt -= length;
    memcpy(&c, data + dataPosCnt, length);
    pthread_mutex_unlock(&mutex);
    return true;
}

bool ActionData::popUnsigned(unsigned &u) {
    pthread_mutex_lock(&mutex);
    size_t length = sizeof(unsigned);
    if(dataPosCnt < length) {
        DERROR("Unable to pop unsigned");
        pthread_mutex_unlock(&mutex);
        return false;
    }
    dataPosCnt -= length;
    memcpy(&u, data + dataPosCnt, length);
    pthread_mutex_unlock(&mutex);
    return true;
}

bool ActionData::popFloat(float &f) {
    pthread_mutex_lock(&mutex);
    size_t length = sizeof(float);
    if(dataPosCnt < length) {
        DERROR("Unable to pop float32_t");
        pthread_mutex_unlock(&mutex);
        return false;
    }
    dataPosCnt -= length;
    memcpy(&f, data + dataPosCnt, length);
    pthread_mutex_unlock(&mutex);
    return true;}

bool ActionData::popVector3f(Telemetry::Vector3f &v) {
    pthread_mutex_lock(&mutex);
    size_t length = sizeof(Telemetry::Vector3f);
    if(dataPosCnt < length) {
        DERROR("Unable to pop Vector3f");
        pthread_mutex_unlock(&mutex);
        return false;
    }
    dataPosCnt -= length;
    memcpy(&v, data + dataPosCnt, length);
    pthread_mutex_unlock(&mutex);
    return true;
}