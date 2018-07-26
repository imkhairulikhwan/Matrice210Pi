/*! @file Action.h
 *  @version 1.0
 *  @date Jul 18 2018
 *  @author Jonathan Michel
 */

#ifndef MATRICE210_ACTION_H
#define MATRICE210_ACTION_H

#include <pthread.h>
#include <mqueue.h>

#include <dji_vehicle.hpp>

#define ACTION_QUEUE_NAME "/actionQueue"

using namespace DJI::OSDK;

namespace M210 {
    class FlightController;
    class ActionData;

    class Action : public Singleton<Action> {
    public:
        enum MissionType {
            VELOCITY = 1,
            POSITION,
            POSITION_OFFSET,
            WAYPOINTS
        };
        enum MissionAction {
            START = 1
        };
    private:
        mq_attr actionQueueAttr;
        mqd_t actionQueue;
        // Mutex
        static pthread_mutex_t mutex;
        FlightController *flightController;

        void positionMission(ActionData *action) const;

        void velocityMission(ActionData *action) const;

        void positionOffsetMission(ActionData *action) const;

    public:
        Action();

        ~Action();

        void setFlightController(FlightController *flightController) { this->flightController = flightController; }

        void add(const ActionData *actionData);

        void process() const;
    };
}

#endif //MATRICE210_ACTION_H