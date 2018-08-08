/*! @file Action.h
 *  @version 1.0
 *  @date Jul 18 2018
 *  @author Jonathan Michel
 *  @brief This class provides a queue used to add action to do by FlightController.
 *  Queue is filled on data reception from mobile SDK or console
 *  Queue is continuously processed in main
 *  Action are ActionData objects, see ActionData.h
 *  mqueue.h is used
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
        // todo move this declaration to a better place
        enum MissionType {      /*!< Mission type, used when action concern a mission */
            VELOCITY = 1,
            POSITION,
            POSITION_OFFSET,
            WAYPOINTS
        };
        // todo move this declaration to a better place
        enum MissionAction {    /*!< Mission action, mainly used with waypoints actions */
            START = 1,
            ADD,
            RESET,
            STOP,
            PAUSE,
            RESUME
        };
    private:
        mqd_t actionQueue;              /*!< Action queue */
mq_attr actionQueueAttr;                /*!< Action queue attributes */
        static pthread_mutex_t mutex;   /*!< Action queue mutex, mqueue are supposed to be protected but safety first */
        FlightController *flightController;     /*!< Flight controller concerned by the actions */

        /**
         * Dedicated function when action is a position mission.
         * Gets all parameters and calls FlightController method
         * @param action ActionData pointer to get parameters
         */
        void positionMission(ActionData *action) const;

        /**
         * Dedicated function when action is a velocity mission.
         * Gets all parameters and calls FlightController method
         * @param action ActionData pointer to get parameters
         */
        void velocityMission(ActionData *action) const;

        /**
         * Dedicated function when action is a position offset mission.
         * Gets all parameters and calls FlightController method
         * @param action ActionData pointer to get parameters
         */
        void positionOffsetMission(ActionData *action) const;

        /**
         * Dedicated function when action is a waypoints mission.
         * Gets all parameters and calls FlightController method
         * @param action ActionData pointer to get parameters
         */
        void waypointsMission(ActionData *action) const;

    public:
        /**
         * Initialize action queue
         */
        Action();

        /**
         * Close action queue and unlink it.
         */
        ~Action();
        /**
         * Define the FlightController to whom the action should be transmitted
         * @param flightController Pointer to used FlightController
         */
        void setFlightController(FlightController *flightController) { this->flightController = flightController; }

        /**
         * Add action data to queue
         * @param actionData Pointer to ActionData object to add
         * @return true if action data has been added to queue, false otherwise
         */
        bool add(const ActionData *actionData);

        /**
         *  Receive message from queue a process it. Blocking call.
         */
        void process() const;

        /**
         * Unit test to check that class is working. Called at the
         * beginning of the program. Assert if a test fails
         */
        static void unitTest();
    };
}

#endif //MATRICE210_ACTION_H