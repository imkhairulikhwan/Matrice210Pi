/*! @file Action.cpp
 *  @version 1.0
 *  @date Jul 18 2018
 *  @author Jonathan Michel
 */

#include <fcntl.h>
#include "Action.h"

#include "ActionData.h"
#include "../FlightController.h"

pthread_mutex_t Action::mutex = PTHREAD_MUTEX_INITIALIZER;

Action::Action() {
    flightController = nullptr;
    // actionQueue is a blocking queue
    actionQueueAttr.mq_flags = 0;
    actionQueueAttr.mq_maxmsg = 5;
    actionQueueAttr.mq_msgsize = sizeof(void*);
    actionQueueAttr.mq_curmsgs = 0;
    // Queue are files shared between programs,ensure that queue
    // is empty at the beginning
    mq_unlink(ACTION_QUEUE_NAME);
    // Open queue
    actionQueue = mq_open(ACTION_QUEUE_NAME, O_CREAT | O_RDWR,
                    0666, &actionQueueAttr);
    if(actionQueue == -1) {
        int errsv = errno;  // save error code
        DERROR("Action queue creation failed, error : %i", errsv);
    }
}

Action::~Action() {
    mq_close(actionQueue);
    mq_unlink(ACTION_QUEUE_NAME);
}

void Action::add(ActionData *actionData) {
    pthread_mutex_lock(&mutex);
    static struct timespec sendTimeout;
    sendTimeout.tv_sec = 0;
    sendTimeout.tv_nsec = 250000;
    // mq_timedsend() behaves just like mq_send(), except that if the queue is full, sendTimeout specifies
    // the time for which the call will block
    int status = mq_timedsend(actionQueue, (const char *)&actionData, sizeof(void *), 1, &sendTimeout);
    if(status == -1) {
        int errsv = errno;  // save error code
        DERROR("Action added to queue failed, error : %i", errsv);
    } else {
        //DSTATUS("Action added to queue");
    }
    pthread_mutex_unlock(&mutex);
}

void Action::process() {
    if(flightController == nullptr) {
        DERROR("Please call setFlightController() first");
        return;
    }

    ActionData *action;
    // Blocking call
    int status = mq_receive(actionQueue, (char *)&action, sizeof(void *), NULL);

    if(status >= 0) {
        switch(action->getActionId()) {
            case ActionData::monitoredTakeoff:
                flightController->monitoredTakeoff();
                break;
            case ActionData::monitoredLanding:
                flightController->monitoredLanding();
                break;
            case ActionData::moveByPosition: {
                Telemetry::Vector3f position;
                float32_t yaw;
                action->popFloat(yaw);
                action->popVector3f(position);
                flightController->moveByPosition(&position, yaw);
            }
                break;
            case ActionData::moveByPositionOffset: {
                Telemetry::Vector3f position;
                float32_t yaw;
                action->popFloat(yaw);
                action->popVector3f(position);
                flightController->moveByPositionOffset(&position, yaw);
            }
                break;
            case ActionData::moveByVelocity: {
                Telemetry::Vector3f velocity;
                float32_t yaw;
                action->popFloat(yaw);
                action->popVector3f(velocity);
                flightController->moveByVelocity(&velocity, yaw);
            }
                break;
            case ActionData::sendDataToMSDK:
                break;
            case ActionData::stopAircraft: {
                flightController->stopAircraft();
            }
                break;
            case ActionData::emergencyStop:{
                flightController->emergencyStop();
            }
                break;
            case ActionData::emergencyRelease:{
                flightController->emergencyRelease();
            }
                break;
        }
        delete action;
    }
}