/*! @file ActionData.cpp
 *  @version 1.0
 *  @date Jul 20 2018
 *  @author Jonathan Michel
 */

#include "ActionData.h"

#include <assert.h>
#include <cstring>
#include <cstddef>

// TODO Find a way to do pop with RTTI (Templates)
// TODO Pop in fifo instead of current filo !

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
    delete[] data;
}

bool ActionData::checkSize(size_t size) const{
    return (dataPosCnt + size <= this->dataSize);
}

bool ActionData::_push(const char *c, size_t length) {
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

bool ActionData::push(int i) {
    return _push(reinterpret_cast<const char *>(&i), sizeof(int));
}

bool ActionData::push(unsigned u) {
    return _push(reinterpret_cast<const char *>(&u), sizeof(unsigned));
}

bool ActionData::push(float f) {
    return _push(reinterpret_cast<const char *>(&f), sizeof(float));
}

bool ActionData::push(const Telemetry::Vector3f &v) {
    return _push(reinterpret_cast<const char *>(&v), sizeof(Telemetry::Vector3f));
}

bool ActionData::push(const uint8_t *data, size_t length) {
    return _push(reinterpret_cast<const char *>(data), length);
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

bool ActionData::popInt(int &i) {
    pthread_mutex_lock(&mutex);
    size_t length = sizeof(int);
    if(dataPosCnt < length) {
        DERROR("Unable to pop int");
        pthread_mutex_unlock(&mutex);
        return false;
    }
    dataPosCnt -= length;
    memcpy(&i, data + dataPosCnt, length);
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
        DERROR("Unable to pop float");
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

void ActionData::unitTest() {
    // Todo test all type
    uint8_t i = 17;
    float x = 2.52;
    float y = 5.17;
    float z = 78.35;
    float yaw = 45.25;

    auto ch = new char[16];
    memcpy(ch, reinterpret_cast<char*>(&x), 4);
    memcpy(ch+4, reinterpret_cast<char*>(&y), 4);
    memcpy(ch+8, reinterpret_cast<char*>(&z), 4);
    memcpy(ch+12, reinterpret_cast<char*>(&yaw), 4);

    ActionData ad(ActionData::stopAircraft, 17);
    ad.push((char)i);
    // Push float array
    // ad.push(reinterpret_cast<uint8_t*>(ch), 16);
    // Push float 1 by 1
    ad.push(x);
    ad.push(y);
    ad.push(z);
    ad.push(yaw);
     //*/

    uint8_t i2 = 0;
    char c2 = 0;
    float x2 = 0, y2 = 0, z2 = 0, yaw2 = 0;
    Telemetry::Vector3f v2;

    ad.popFloat(yaw2);
    ad.popFloat(z2);
    ad.popFloat(y2);
    ad.popFloat(x2);

    // Get Vector3f instead of three float
    ad.push(x);
    ad.push(y);
    ad.push(z);
    ad.popVector3f(v2);

    ad.popChar(c2);

    i2 = (uint8_t)c2;

    assert(yaw == yaw2);
    assert(z == z2);
    assert(y == y2);
    assert(x == x2);
    assert(x == v2.x);
    assert(y == v2.y);
    assert(z == v2.z);
    assert(i == i2);

    DSTATUS("ActionData test passed");
}