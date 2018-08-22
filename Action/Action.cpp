/*! @file Action.cpp
 *  @version 1.0
 *  @date Jul 18 2018
 *  @author Jonathan Michel
 */

#include <fcntl.h>
#include <cassert>

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
    actionQueueAttr.mq_maxmsg = 10;
    // Queue contains a pointer to an ActionData object
    actionQueueAttr.mq_msgsize = sizeof(void*);
    actionQueueAttr.mq_curmsgs = 0;
    // Queue are files shared between programs, ensure that queue
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

bool Action::add(const ActionData *actionData) {
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
        pthread_mutex_unlock(&mutex);
        return false;
    } else {
        //DSTATUS("Action added to queue");
    }
    pthread_mutex_unlock(&mutex);
    return true;
}

void Action::process() const {
    if(flightController == nullptr) {
        DERROR("Please call setFlightController() first");
        return;
    }

    ActionData *action;
    // Blocking call, wait for data added in queue
    int status = mq_receive(actionQueue, (char *)&action, sizeof(void *), NULL);

    // Queue had a message, safety verification
    if(status >= 0) {
        // Call desired action depending on action id
        switch(action->getActionId()) {
            case ActionData::ActionId::takeOff:
                flightController->takeOff();
                break;
            case ActionData::ActionId::landing:
                flightController->landing();
                break;
            case ActionData::ActionId::mission: {
                char mission;
                // When action id is a mission, last byte indicates mission kind
                // Call dedicated mission function
                if(action->popChar(mission)) {
                    switch ((unsigned) mission){
                        case MissionType::VELOCITY:         // Velocity mission
                            velocityMission(action);
                            break;
                        case MissionType::POSITION :        // Position mission
                            positionMission(action);
                            break;
                        case MissionType::POSITION_OFFSET:  // Position offset mission
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
                // Not yet implemented by action queue
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
                // Resend watchdog to mobile SDK to indicates code is running
                flightController->sendDataToMSDK(reinterpret_cast<const uint8_t *>("#w"), 2);
                break;
            case ActionData::ActionId::helloWorld: {
                // Send a response to Mobile SDK
                char hw[] = "Hello world from Pi";
                flightController->sendDataToMSDK(reinterpret_cast<const uint8_t *>(hw), strlen(hw));
            }
                break;
            case ActionData::ActionId::obtainControlAuthority : {
                // @todo blocking call, to replace
                flightController->obtainCtrlAuthority();
            }
            default:
                LERROR("Unknown action to process");
        }
        delete action;
    }
}

void Action::velocityMission(ActionData *action) const {
    char task;
    Vector3f v;
    float yaw;
    // Last byte indicated mission task
    if(action->popChar(task)) {
        if(task == MissionAction::START) {
            // Get parameters
            bool b = true;
            b &= action->popFloat(yaw);
            b &= action->popVector3f(v);
            // Only if all parameters have been recovered return true
            if(b) {
                flightController->moveByVelocity(&v, yaw);
            } else {
                LERROR("Velocity mission - Unable to get parameters");
            }
        } else {
            LERROR("Velocity mission - Unknown task");
        }
    } else {
        LERROR("Velocity mission - Unable to determine task");
    }
}

void Action::positionMission(ActionData *action) const {
    char task;
    Vector3f v;
    float yaw;
    // Last byte indicated mission task
    if(action->popChar(task)) {
        if(task == MissionAction::START) {
            // Get parameters
            bool b = true;
            b &= action->popFloat(yaw);
            b &= action->popVector3f(v);
            // Only if all parameters have been recovered return true
            if(b) {
                flightController->moveByPosition(&v, yaw);
            } else {
                LERROR("Position mission - Unable to get parameters");
            }
        } else {
            LERROR("Position mission - Unknown task");
        }
    } else {
        LERROR("Position mission - Unable to determine task");
    }
}

void Action::positionOffsetMission(ActionData *action) const {
    char task;
    Vector3f v;
    float yaw;
    // Last byte indicated mission task
    if(action->popChar(task)) {
        if(task == MissionAction::START) {
            // Get parameters
            bool b = true;
            b &= action->popFloat(yaw);
            b &= action->popVector3f(v);
            // Only if all parameters have been recovered return true
            if(b) {
                flightController->moveByPositionOffset(&v, yaw);
            } else {
                LERROR("Position offset mission - Unable to get parameters");
            }
        } else {
            LERROR("Position offset mission - Unknown task");
        }
    } else {
        LERROR("Position offset mission - Unable to determine task");
    }
}

void Action::waypointsMission(ActionData *action) const {
    char task;
    // Last byte indicated mission task
    if(action->popChar(task)) {
        flightController->waypointsMissionAction((unsigned) task);
    } else {
        LERROR("waypoints mission - Unable to determine task");
    }
}

void Action::unitTest() {
    // Try to add action data to queue
    bool actionQueue;
    auto actionData = new ActionData(ActionData::ActionId::helloWorld);
    actionQueue = Action::instance().add(actionData);

    assert(actionQueue);
}
