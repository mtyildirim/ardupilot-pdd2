#pragma once

// This structure provides information on the internal member data of
// a PID.  It provides an abstract way to pass PID information around,
// useful for logging and sending mavlink messages.

// It is also used to pass PID information into controllers...

struct AP_PDD2Info {
    float target;
    float actual;
    float error;
    float P;
    float D;
    float D2;
    float FF;
    float Dmod;
    float slew_rate;
    bool  limit;
};
