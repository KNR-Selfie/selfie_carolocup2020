#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <selfie_scheduler/scheduler_enums.h>
#include <selfie_scheduler/client_interface.h>

class Scheduler
{
    program_state car_state_;
    ClientInterface *current_action_;
public:
    Scheduler();

    void setAction(); //close actions and start new one
    void stopAction();


};

#endif // SCHEDULER_H
