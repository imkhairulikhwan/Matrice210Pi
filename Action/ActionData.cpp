/*! @file ActionData.cpp
 *  @version 1.0
 *  @date Jul 20 2018
 *  @author Jonathan Michel
 */

#include "ActionData.h"

#include <cassert>
#include <cstring>
#include <cstddef>

using namespace M210;

pthread_mutex_t ActionData::mutex = PTHREAD_MUTEX_INITIALIZER;

ActionData::ActionData(ActionId actionId, size_t size) {
    this->actionId = actionId;
    this->dataSize = size;
    this->dataPosCnt = 0;
    this->dataPtr = nullptr;
    if(size > 0)
        dataPtr = new char[size];
}

ActionData::~ActionData() {
    delete[] dataPtr;
}

bool ActionData::checkSize(size_t size) const{
    return (dataPosCnt + size <= dataSize);
}

bool ActionData::push(char c) {
    _push(&c, char);
}

bool ActionData::push(int i) {
    _push(&i, int);
}

bool ActionData::push(unsigned u) {
    _push(&u, unsigned);
}

bool ActionData::push(float f) {
    _push(&f, float);
}

bool ActionData::push(const Telemetry::Vector3f &v) {
    _push(&v, Telemetry::Vector3f);
}

bool ActionData::push(const uint8_t *data, size_t length) {
    __push(data, length);
}

bool ActionData::popChar(char &c) {
    _pop(c, char);
}

bool ActionData::popInt(int &i) {
    _pop(i, int);
}

bool ActionData::popUnsigned(unsigned &u) {
    _pop(u, unsigned);
}

bool ActionData::popFloat(float &f) {
    _pop(f, float);
}

bool ActionData::popVector3f(Telemetry::Vector3f &v) {
    _pop(v, Telemetry::Vector3f);
}

void ActionData::unitTest() {
    char c = 'k';
    int i = -25178;
    unsigned u = 5891;
    float x = 2.52;
    float y = 5.17;
    float z = 78.35;
    float yaw = 45.25;

    M210::ActionData::ActionId id = M210::ActionData::ActionId::stopAircraft;
    // Allocate dynamic memory
    ActionData ad(id, sizeof(uint8_t) + sizeof(float) * 4);
    ad.push(c);
    /*/ Push float array
    auto ch = new char[16];
    memcpy(ch, reinterpret_cast<char*>(&x), 4);
    memcpy(ch+4, reinterpret_cast<char*>(&y), 4);
    memcpy(ch+8, reinterpret_cast<char*>(&z), 4);
    memcpy(ch+12, reinterpret_cast<char*>(&yaw), 4);
    ad.push(reinterpret_cast<uint8_t*>(ch), 16);
    //*/

    // Push float 1 by 1
    ad.push(x);
    ad.push(y);
    ad.push(z);
    ad.push(yaw);
    //*/

    // Try to push i, has to return false
    // because memory allocation was to small
    bool push = ad.push(i);

    // Try to get all data back
    // /!\ Lifo !
    M210::ActionData::ActionId id2 = ad.getActionId();
    int i2 = 0;
    unsigned u2 = 0;
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

    // Push and instant pop data
    ad.push(i);
    ad.push(u);

    ad.popUnsigned(u2);
    ad.popInt(i2);

    // Try to pop char, has to return false
    // because all data have been popped
    char c3;
    bool pop = ad.popChar(c3);
    //*/

    // Verify that all data recovered are identical to initials ones
    //assert(!push);
    assert(id == id2);
    assert(yaw == yaw2);
    assert(z == z2);
    assert(y == y2);
    assert(x == x2);
    assert(x == v2.x);
    assert(y == v2.y);
    assert(z == v2.z);
    assert(c == c2);
    assert(u == u2);
    assert(i == i2);
    assert(!pop);

    DSTATUS("ActionData test passed");
    DERROR("Please do not pay attention to the last two errors if ActionData test passed");
}