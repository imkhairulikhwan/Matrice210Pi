/*! @file Action.h
 *  @version 1.0
 *  @date Jul 18 2018
 *  @author Jonathan Michel
 */

#ifndef MATRICE210_ACTION_H
#define MATRICE210_ACTION_H

#include <pthread.h>
#include <csignal>
#include <cerrno>
#include <sys/time.h>
#include <sys/stat.h>
#include <mqueue.h>
#include <fcntl.h>

#include <dji_vehicle.hpp>

#define MY_MQ_NAME "/my_mq"

using namespace DJI::OSDK;

class Action : public Singleton<Action>{
private:
    mq_attr my_mq_attr;
    mqd_t my_mq;
public:
    Action();
    ~Action();
    void add(unsigned v);
    unsigned process();
};


#endif //MATRICE210_ACTION_H
