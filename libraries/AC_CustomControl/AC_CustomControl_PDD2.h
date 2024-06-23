#pragma once
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AC_PID/AC_PDD2.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include "AC_CustomControl_Backend.h"

#ifndef CUSTOMCONTROL_PDD2_ENABLED
    #define CUSTOMCONTROL_PDD2_ENABLED AP_CUSTOMCONTROL_ENABLED
#endif

#if CUSTOMCONTROL_PDD2_ENABLED

#define AC_PDD2_DEFAULT_P_ROLL    1.0f
#define AC_PDD2_DEFAULT_D_ROLL    0.25f
#define AC_PDD2_DEFAULT_D2_ROLL   0.02f

#define AC_PDD2_DEFAULT_P_PITCH   1.0f
#define AC_PDD2_DEFAULT_D_PITCH   0.25f
#define AC_PDD2_DEFAULT_D2_PITCH  0.02f

#define AC_PDD2_DEFAULT_P_YAW     1.0f
#define AC_PDD2_DEFAULT_D_YAW     0.25f
#define AC_PDD2_DEFAULT_D2_YAW    0.02f



#define AC_PDD2_TFILT_HZ_DEFAULT  0.0f   // default input filter frequency
#define AC_PDD2_EFILT_HZ_DEFAULT  0.0f   // default input filter frequency
#define AC_PDD2_DFILT_HZ_DEFAULT  20.0f   // default input filter frequency
#define AC_PDD2_D2FILT_HZ_DEFAULT 20.0f   // default input filter frequency
#define AC_PDD2_RESET_TC          0.16f   // Time constant for integrator reset decay to zero


class AC_CustomControl_PDD2 : public AC_CustomControl_Backend {
public:
    AC_CustomControl_PDD2(AC_CustomControl& frontend, AP_AHRS*& ahrs, AC_AttitudeControl_Multi*& att_control,AC_PosControl*& pos_control ,AP_MotorsMulticopter*& motors, float dt);


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
