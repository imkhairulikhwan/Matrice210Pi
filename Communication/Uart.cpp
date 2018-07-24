/*! @file Uart.cpp
 *  @version 1.0
 *  @date Jul 12 2018
 *  @author Jonathan Michel
 */


#include "Uart.h"

#include <iostream>
#include <string>
#include <sstream>

#include "../FlightController.h"
#include "../Managers/ThreadManager.h"

Uart::Uart(const char *device, uint32_t baudRate) {
    serialDevice = new LinuxSerialDevice(device, baudRate);
    serialDevice->init();
}

Uart::~Uart() {
    delete serialDevice;
}

void Uart::launchRxThread() {
    ThreadManager::start("uartRxThread",
                         &uartRxThreadID, &uartRxThreadAttr,
                         uartRxThread, (void*)this);
}

void Uart::send(const uint8_t *buf, size_t len) const {
    serialDevice->send(buf, len);
}

size_t Uart::read(uint8_t *buf, size_t len) const{
    return serialDevice->readall(buf, len);
}

void *Uart::uartRxThread(void *param) {
    DSTATUS("uartRxThread running...");
    auto uart = static_cast<Uart*>(param);
    char rxBuffer[256];
    uint8_t rxChar;
    int rxIndex = 0;

    // TODO Improve protocol
    // Current protocol implementation transmit ascii value of numbers
    // Transmit 32 bits value with predefined frame format and length
    bool running = true;
    while(running) {
        // New char received
        if(uart->read(&rxChar, 1) != 0) {
            // Add to rx buffer
            rxBuffer[rxIndex] = rxChar;
            rxIndex++;
            // '@' indicate end of transmission
            if(rxChar == '@') {
                DSTATUS("Frame received : %u", rxIndex);
                // TODO Launch in new thread
                // Process data
                long data_i[20];
                float data_f[20];
                int countValue = 0;
                char tabValues[20];
                int valuesInc = 0;
                int length = rxIndex;

#ifdef DEBUG_UART_FRAME
                std::cout << std::endl;
#endif
                for(uint16_t i = 0; i < length; i++)
                {
#ifdef DEBUG_UART_FRAME
                    std::cout << rxBuffer[i];
#endif
                    // '|' char indicate end of value
                    if(rxBuffer[i] == '|')
                    {
                        tabValues[valuesInc] = ' ';
                        data_f[countValue] = strtof(tabValues, nullptr);
                        data_i[countValue] = strtol(tabValues, nullptr, 10);
                        countValue++;
                        valuesInc = 0;
                    }
                    else
                    {
                        tabValues[valuesInc] = rxBuffer[i];
                        valuesInc++;
                    }
                }
#ifdef DEBUG_UART_FRAME
                std::cout << std::endl;
#endif
                // Last value
                tabValues[valuesInc] = ' ';
                data_f[countValue] = strtof(tabValues, nullptr);
                data_i[countValue] = strtol(tabValues, nullptr, 10);

                // Process data
                switch(data_i[0]) {
                    case 0: {
                        char tempBf[100];
                        sprintf(tempBf, "Valeur de l'antenne : %.6f", data_f[1]);
                        size_t size = strlen(tempBf);
                        uart->flightController->sendDataToMSDK((uint8_t *)tempBf, size);
                        DSTATUS("Float data received : %.6f", data_f[1]);
                    }
                        break;
                    default:
                        DERROR("Unknown data received : %u", data_i[0]);
                        break;
                }

                // Reset rx buffer
                rxIndex = 0;
            }
        }
    }
    return nullptr;
}