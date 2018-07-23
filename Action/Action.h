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

class FlightController;
class ActionData;

class Action : public Singleton<Action>{
private:
    mq_attr actionQueueAttr;
    mqd_t actionQueue;
    // Mutex
    static pthread_mutex_t mutex;
    FlightController* flightController;
public:
    Action();
    ~Action();
    void setFlightController(FlightController* flightController) { this->flightController = flightController; }
    void add(ActionData *actionData);
    void process();
};

#endif //MATRICE210_ACTION_H