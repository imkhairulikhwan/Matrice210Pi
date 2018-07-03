/*! @file mobile.cpp
 *  @version 1.0
 *  @date Jul 03 2018
 *  @author Jonathan Michel
 */

#include "mobile.h"

// mobileDataID for keeping track of command from mobile
uint16_t mobileDataID_glob = 0;
// keepLoopRunning to maintain state of when to stop checking mobile command state
bool keepLoopRunning = true;

bool setupMocParsing(Vehicle *vehicle, LinuxSetup *linuxEnvironment) {
    // Register the callback for parsing mobile data
    vehicle->moc->setFromMSDKCallback(parseFromMobileCallback, linuxEnvironment);

    // Setup a thread to poll the incoming data for large functions
    //pthread_t threadID = setupSamplePollingThread(vehicle);

    return true;
}

void resetMocParing() {
    /*/ Shut off the polling thread
    keepLoopRunning = false;
    void* status;
    pthread_join(threadID, &status);
    //*/
}

pthread_t setupSamplePollingThread(Vehicle* vehicle)  {
    int         ret = -1;
    std::string infoStr;

    pthread_t      threadID;
    pthread_attr_t attr;

    // Initialize and set thread detached attribute
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

    ret     = pthread_create(&threadID, nullptr, mobileSamplePoll, (void*)vehicle);
    infoStr = "mobilePoll";

    if (0 != ret)
        DERROR("fail to create thread for %s!\n", infoStr.c_str());

    ret = pthread_setname_np(threadID, infoStr.c_str());
    if (0 != ret)
        DERROR("fail to set thread name for %s!\n", infoStr.c_str());

    return threadID;
}

void parseFromMobileCallback(Vehicle* vehicle,
                             RecvContainer recvFrame,
                             UserData userData) {

    // First, lets cast the userData to LinuxSetup*
    auto *linuxEnvironment = (LinuxSetup *) userData;

    uint8_t formatFrameLength = OpenProtocol::PackageMin + OpenProtocol::CRCHead;
    uint16_t msgLength = recvFrame.recvInfo.len - formatFrameLength;
    uint8_t* data = recvFrame.recvData.raw_ack_array;


    if(data[0] == '#') {
        switch (data[1]) {
            case '1':			// toggle LED
                DSTATUS("MOC - Led toggled");
                break;
            default:
                DERROR("MOC - Unknown command");
                break;
        }
    } else {
        // Data received displayed as string
        if(msgLength == 100) {
            msgLength = 99;
        }
        data[msgLength] = '\0';
        DSTATUS("MOC - String received : %s", recvFrame.recvData.raw_ack_array);
    }
    vehicle->moc->sendDataToMSDK(data, (uint8_t)msgLength);
}

void *
mobileSamplePoll(void* vehiclePtr)
{
    auto vehicle = (Vehicle *)vehiclePtr;

    // Initialize variables so as to not cross case statements
    bool coreMissionStatus = false;

    // Run polling loop until we're told to stop
    while (keepLoopRunning)
    {
        /*/ Check global variable to find out if we need to execute an action
        switch (mobileDataID_glob)
        {
            case 0x3E:
                coreMissionStatus = runPositionControlSample(vehicle);
                sendAckToMobile(vehicle, 0x3E, coreMissionStatus);
                mobileDataID_glob = 0;
                break;
            default:
                break;
        }
        //*/
        usleep(500000);
    }

}