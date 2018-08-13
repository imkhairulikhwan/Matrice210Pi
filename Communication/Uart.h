/*! @file Uart.h
 *  @version 1.0
 *  @date Jul 12 2018
 *  @author Jonathan Michel
 *  @brief This class handles UART communication with STM32 card.
 *  It launches a thread reading incoming uart data and processes received frame.
 *
 *  Current protocol implementation transmits ascii value of numbers
 *  | char indicates new value
 *  @ char indicates end of frame
 *  Frame example : 1|4|2.4513|123.4@
 *  Currently protocol is only used to receive data from STM32
 *  Raw data can be sent to STM32
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
        pthread_t uartRxThreadID;           /*!< Uart rx thread id */
        pthread_attr_t uartRxThreadAttr;    /*!< Uart rx thread attributes */
    public:
        /**
         * Create serial device with DJI LinuxSerialDevice class and
         * initialize it
         * @param device Linux serial device port
         * @param baudRate Baud rate [Bd]
         */
        Uart(const char *device, uint32_t baudRate);

        /**
         * Delete serial device
         */
        ~Uart();

        /**
         * Send data by uart
         * @param buf Buffer pointer
         * @param len data length
         */
        void send(const uint8_t *buf, size_t len) const;

        /**
         * Launch uart rx thread
         * @return true if thread creation and launch works, false otherwise
         */
        bool launchRxThread();

        /**
         * Configure flight controller to use
         * @param flightController Flight controller pointer
         */
        void setFlightController(const FlightController *flightController) { this->flightController = flightController; }

    private:
        /**
         * Uart rx thread. Receive and process data
         * @param param Thread parameter, has to be Uart object cast in void*
         * @return -
         */
        static void *uartRxThread(void *param);

        /**
         * Read uart incoming data
         * @param buf Buffer pointer
         * @param len Data length to read
         * @return Data read size [bytes]
         */
        size_t read(uint8_t *buf, size_t len) const;
    };
}

#endif //MATRICE210_UART_H