/*! @file Uart.cpp
 *  @version 1.0
 *  @date Jul 12 2018
 *  @author Jonathan Michel
 */

#include <iostream>

#include "Uart.h"

#include "../FlightController.h"

Uart::Uart(const char *device, uint32_t baudrate) {
    serialDevice = new LinuxSerialDevice(device, baudrate);
    serialDevice->init();
}

Uart::~Uart() {
    delete serialDevice;
}

void Uart::launchRxThread() {
    pthread_attr_init(&uartRxThreadAttr);
    pthread_attr_setdetachstate(&uartRxThreadAttr, PTHREAD_CREATE_JOINABLE);
    int ret  = pthread_create(&uartRxThreadID, nullptr, uartRxThread, (void*)this);
    string threadName = "uartRxThread";

    if (ret != 0)
        DERROR("Fail to create thread for %s !", threadName.c_str());

    ret = pthread_setname_np(uartRxThreadID, threadName.c_str());
    if (ret != 0)
        DERROR("Fail to set thread name for %s !", threadName.c_str());
}

void Uart::send(const uint8_t *buf, size_t len) {
    serialDevice->send(buf, len);
}

size_t Uart::read(uint8_t *buf, size_t len) {
    return serialDevice->readall(buf, len);
}

void *Uart::uartRxThread(void *param) {
    DSTATUS("uartRxThread running...");
    auto uart = (Uart*)param;
    char rxBuffer[256];
    uint8_t rxChar;
    int rxIndex = 0;

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

                std::cout << std::endl;
                for(uint16_t i = 0; i < length; i++)
                {
                    std::cout << rxBuffer[i];
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
                std::cout << std::endl;
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

void Uart::setFlightController(FlightController* flightController) {
    this->flightController = flightController;
}

