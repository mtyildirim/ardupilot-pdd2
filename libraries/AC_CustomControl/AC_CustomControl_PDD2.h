#pragma once
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include "AC_CustomControl_Backend.h"

#ifndef CUSTOMCONTROL_PDD2_ENABLED
    #define CUSTOMCONTROL_PDD2_ENABLED AP_CUSTOMCONTROL_ENABLED
#endif

#if CUSTOMCONTROL_PDD2_ENABLED

class AC_CustomControl_PDD2 : public AC_CustomControl_Backend {
public:
    AC_CustomControl_PDD2(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl_Multi*& att_control, AP_MotorsMulticopter*& motors, float dt);


    Vector3f update(void) override;
    void reset(void) override;

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // declare parameters here
    AC_PDD2 _pdd2_atti_rate_roll;
    AC_PDD2 _pdd2_atti_rate_pitch;
    AC_PDD2 _pdd2_atti_rate_yaw;
};

#endif
