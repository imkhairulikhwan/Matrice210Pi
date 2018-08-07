/*! @file Uart.h
 *  @version 1.0
 *  @date Jul 12 2018
 *  @author Jonathan Michel
 *  @brief This class handles UART communication with STM32 card.
 *  Launches a thread reading incoming uart data and process received frame.
 *  Current protocol implementation transmits ascii value of numbers
 *  | char indicates new value
 *  @ char indicates end of frame
 *  Frame example : 1|4|2.4513|123.4@
 *  Currently protocol is only used to transmit data from STM32 to Pi
 */

#ifndef MATRICE210_UART_H
#define MATRICE210_UART_H

#include <pthread.h>

#include <dji_vehicle.hpp>
#include <linux_serial_device.hpp>

// define to print received frame
//#define DEBUG_UART_FRAME
#define END_OF_FRAME_CHAR '@'
#define NEW_VALUE_CHAR '|'

using namespace std;
using namespace DJI::OSDK;

namespace M210 {
    class FlightController;

    class Uart {
    private:
        LinuxSerialDevice *serialDevice;
        const FlightController *flightController;
        // Thread attributes
        pthread_t uartRxThreadID;
        pthread_attr_t uartRxThreadAttr;
    public:
        Uart(const char *device, uint32_t baudRate);

        ~Uart();

        void send(const uint8_t *buf, size_t len) const;

        void launchRxThread();

        void
        setFlightController(const FlightController *flightController) { this->flightController = flightController; }

    private:
        static void *uartRxThread(void *param);

        size_t read(uint8_t *buf, size_t len) const;
    };
}

#endif //MATRICE210_UART_H