/*! @file Action.cpp
 *  @version 1.0
 *  @date Jul 18 2018
 *  @author Jonathan Michel
 */

#include <fcntl.h>
#include "Action.h"

#include "ActionData.h"
#include "../Aircraft/FlightController.h"
#include "../Aircraft/Watchdog.h"
#include "../util/Log.h"

using namespace M210;

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

void Action::add(const ActionData *actionData) {
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
        delete actionData;
    } else {
        //DSTATUS("Action added to queue");
    }
    pthread_mutex_unlock(&mutex);
}

void Action::process() const {
    if(flightController == nullptr) {
        DERROR("Please call setFlightController() first");
        return;
    }

    ActionData *action;
    // Blocking call
    int status = mq_receive(actionQueue, (char *)&action, sizeof(void *), NULL);

    // queue had a message, safety verification
    if(status >= 0) {
        switch(action->getActionId()) {
            case ActionData::ActionId::takeOff:
                flightController->takeoff();
                break;
            case ActionData::ActionId::landing:
                flightController->landing();
                break;
            case ActionData::ActionId::mission: {
                char mission;
                if(action->popChar(mission)) {
                    switch ((unsigned) mission){
                        case MissionType::VELOCITY:    // Velocity mission
                            velocityMission(action);
                            break;
                        case MissionType::POSITION :    // Position mission
                            positionMission(action);
                            break;
                        case MissionType::POSITION_OFFSET: // Position offset mission
                            positionOffsetMission(action);
                            break;
                        case MissionType::WAYPOINTS:        // Waypoints mission
                            waypointsMission(action);
                            break;
                        default:
                            LERROR("Mission - Unknown mission kind");
                            break;
                    }
                } else {
                    LERROR("Mission - Unable to determine mission type");
                }
            }
                break;
            case ActionData::ActionId::sendDataToMSDK:
                break;
            case ActionData::ActionId::stopAircraft:
                flightController->stopAircraft();
                break;
            case ActionData::ActionId::emergencyStop:
                flightController->emergencyStop();
                break;
            case ActionData::ActionId::emergencyRelease:
                flightController->emergencyRelease();
                break;
            case ActionData::ActionId::watchdog:
                flightController->getWatchdog()->reset();
                break;
            default:
                LERROR("Mission - Unknown action");
        }
        delete action;
    }
}

void Action::velocityMission(ActionData *action) const {
    char task;
    Vector3f v;
    float yaw;
    if(action->popChar(task)) {
        if(task == MissionAction::START) { // start mission
            action->popFloat(yaw);
            action->popVector3f(v);
            flightController->moveByVelocity(&v, yaw);
        } else {
            LERROR("velocityMission - Unknown task");
        }
    } else {
        LERROR("velocityMission - Unable to determine task");
    }
}

void Action::positionMission(ActionData *action) const {
    char task;
    Vector3f v;
    float yaw;
    if(action->popChar(task)) {
        if(task == MissionAction::START) { // start mission
            action->popFloat(yaw);
            action->popVector3f(v);
            flightController->moveByPosition(&v, yaw);
        } else {
            LERROR("positionMission - Unknown task");
        }
    } else {
        LERROR("positionMission - Unable to determine task");
    }
}

void Action::positionOffsetMission(ActionData *action) const {
    char task;
    Vector3f v;
    float yaw;
    if(action->popChar(task)) {
        if(task == MissionAction::START) { // start mission
            action->popFloat(yaw);
            action->popVector3f(v);
            flightController->moveByPositionOffset(&v, yaw);
        } else {
            LERROR("positionOffsetMission - Unknown task");
        }
    } else {
        LERROR("positionOffsetMission - Unable to determine task");
    }
}

void Action::waypointsMission(ActionData *action) const {
    char task;
    if(action->popChar(task)) {
        flightController->waypointsMissionAction((unsigned) task);
    } else {
        LERROR("waypointsMission - Unable to determine task");
    }
}
