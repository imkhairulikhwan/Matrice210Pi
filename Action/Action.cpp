/*! @file Action.cpp
 *  @version 1.0
 *  @date Jul 18 2018
 *  @author Jonathan Michel
 */

#include "Action.h"

Action::Action() {
    pthread_attr_t attr;
    my_mq_attr.mq_maxmsg = 10;
    my_mq_attr.mq_msgsize = sizeof(unsigned);

    my_mq = mq_open(MY_MQ_NAME, O_CREAT | O_RDWR,
                    0666, &my_mq_attr);
    if(my_mq == -1) {
        DERROR("Action queue creation failed");
    }
}

Action::~Action() {
    mq_close(my_mq);
    mq_unlink(MY_MQ_NAME);
}

void Action::add(unsigned v) {
    int status = mq_send(my_mq, (const char*)&v, sizeof(v), 1);
    if(status == -1) {
        DERROR("Action added to queue failed: %u", v);
    } else {
        DSTATUS("Action added to queue : %u", v);
    }

}

unsigned Action::process() {
    unsigned rcv;
    int status = mq_receive(my_mq, (char*)&rcv, \
                            sizeof(rcv), NULL);
    if (status > 0) {
        return rcv;
    }
    return 0;
}
