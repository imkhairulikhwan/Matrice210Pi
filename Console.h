/*! @file Console.hpp
 *  @version 1.0
 *  @date Jul 04 2018
 *  @author Jonathan Michel
 */


#ifndef MATRICE210_CONSOLE_H
#define MATRICE210_CONSOLE_H

#include <pthread.h>
#include <string>
#include <sstream>
#include <iostream>

#include "FlightController.hpp"

using namespace std;
using namespace DJI::OSDK;

class Console {
private:
    // Thread
    pthread_t consoleThreadID;
    pthread_attr_t consoleThreadAttr;
    // Vehicle
    FlightController *flightController;
public:
    explicit Console(FlightController* flightController);
    void launchThread();
private:
    static void* consoleThread(void* param);
    void displayMenu();
    void displayMenuLine(char command, const char* hint);
    float32_t getNumber(const std::string &message);

};


#endif //MATRICE210_CONSOLE_H
