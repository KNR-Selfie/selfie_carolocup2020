#ifndef SCHEDULER_ENUMS_H
#define SCHEDULER_ENUMS_H
typedef enum program_states
{
    SELFIE_READY = 0,// – auto gotowe do jazdy, oczekuje na otrzymanie sygnału startu,
    START_SIGN,// – auto otrzymało sygnał startu,
    START_DRIVE,// – auto rozpoczęło jazdę,
    END_DRIVE, // – auto przejechało wymagany dystans
}feedback_var;

#endif // SCHEDULER_ENUMS_H
