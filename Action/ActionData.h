/*! @file ActionData.h
 *  @version 1.0
 *  @date Jul 20 2018
 *  @author Jonathan Michel
 *  @brief ActionData objects are added in Action queue (Action.h)
 *  The goal is to provide an object with variable data size.
 *  It is useful because all actions doesn't need the same amount of
 *  data.
 *  On object creation, dynamic memory is allocated. Size depends of user
 *  need. Data can next be pushed (not all type are yet supported,
 *  only main ones). Then, data can be recovered with pop method.
 *  /!\ Push/Pop methods works as a lifo, last pushed value will be first
 *  popped
 *  Example of use in unitTest() method
 */

#ifndef MATRICE210_ACTIONDATA_H
#define MATRICE210_ACTIONDATA_H

#include <pthread.h>

#include <dji_vehicle.hpp>

/**
 * Copy data to allocated dynamic memory
 * @param _data_ Pointer to data to copy
 * @param _length_ Length of data to copy [bytes]
 * @return False if there is not enough memory allocated, true otherwise
 */
#define __push(_data_, _length_)                        \
{                                                       \
    pthread_mutex_lock(&mutex);                         \
    /* Copy data in dynamic memory allocated if there */\
    /* is enough place                                */\
    auto ptr = reinterpret_cast<const char *>(_data_);  \
    if(!checkSize(_length_)) {                          \
        DERROR("Unable to push data");                  \
        pthread_mutex_unlock(&mutex);                   \
        return false;                                   \
    }                                                   \
    memcpy(dataPtr + dataPosCnt, ptr, _length_);        \
    dataPosCnt += (_length_);                             \
    pthread_mutex_unlock(&mutex);                       \
    return true;                                        \
}

/**
 * Copy data to allocated dynamic memory
 * @param _data_ Pointer to data to copy
 * @param _type_ Type of data to copy
 * @return False if there is not enough memory allocated, true otherwise
 */
#define _push(_data_, _type_)           \
{                                       \
    __push(_data_, sizeof(_type_));     \
}

/**
 * Read data from dynamic memory
 * @param _data_ Pointer to data where write result
 * @param _type_ Type of data to read
 * @return False if there is no more data to read in allocated dynamic memory, true otherwise
 */
#define _pop(_data_, _type_)                        \
{                                                   \
    pthread_mutex_lock(&mutex);                     \
    /* Pop data from dynamic memory allocated if */ \
    /* there is remaining data                   */ \
    size_t length = sizeof(_type_);                 \
    if(dataPosCnt < length) {                       \
        DERROR("Unable to pop %s", #_type_);        \
        pthread_mutex_unlock(&mutex);               \
        return false;                               \
    }                                               \
    dataPosCnt -= length;                           \
    memcpy(&(_data_), dataPtr + dataPosCnt, length);  \
    pthread_mutex_unlock(&mutex);                   \
    return true;                                    \
}

using namespace DJI::OSDK;

namespace M210 {
    class ActionData {
    public:
        // todo move this declaration in a better place
        enum ActionId {     /*!< Indicates which action is relative to current action data object */
            takeOff,
            landing,
            mission,
            sendDataToMSDK,
            stopAircraft,
            emergencyStop,
            emergencyRelease,
            watchdog,
            helloWorld
        };
    private:
        char *dataPtr;      /*!< Pointer to dynamic memory allocated */
        size_t dataPosCnt;  /*!< Current numbers of bytes used */
        size_t dataSize;    /*!< Size allocated in dynamic memory [bytes] */
        ActionId actionId;  /*!< Action id concerned by current action data */
        static pthread_mutex_t mutex; /*!< Mutex to ensure dynamic memory operations */

        /**
         * Check if enough place has been allocated to copy data
         * @param size Size of data to copy [bytes]
         * @return True if enough place is available
         */
        bool checkSize(size_t size) const;
    public:
        /**
         * Allocate dynamic memory for specified action
         * @param actionId Action id concerned by current action data
         * @param size Size to allocate in dynamic memory [bytes]
         */
        explicit ActionData(ActionId actionId, size_t size = 0);

        /**
         * Delete dynamic memory allocated
         */
        ~ActionData();

        /**
         * Return action id concerned by current action data
         * @return Action id
         */
        ActionId getActionId() const { return actionId; }

        /**
         * Unit test to check that class is working. Called at the
         * beginning of the program. Assert if a test fails
         */
        static void unitTest();

        // todo find a way to implement all push methods with RTTI (Templates?)
        /**
         * Dedicated push methods
         * See _push() macro for details
         * @param data to push
         * @return False if there is not enough memory allocated, true otherwise
         */
        bool push(char c);
        bool push(int i);
        bool push(unsigned u);
        bool push(float f);
        bool push(const Telemetry::Vector3f &v);
        bool push(const uint8_t *data, size_t length);

        // todo replace implementation with macro
        /**
         * Dedicated pop methods
         * See _pop() macro for details
         * @param return variable
         * @return False if there is no more data to read, true otherwise
         */
        bool popChar(char &c);
        bool popInt(int &i);
        bool popUnsigned(unsigned &u);
        bool popFloat(float &f);
        bool popVector3f(Telemetry::Vector3f &v);
    };
}

#endif //MATRICE210_ACTIONDATA_H