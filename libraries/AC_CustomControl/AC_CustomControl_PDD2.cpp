#include "AC_CustomControl_PDD2.h"

#if CUSTOMCONTROL_PDD2_ENABLED

#include <GCS_MAVLink/GCS.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_CustomControl_PDD2::var_info[] = {
    // @Param: PDD2_RLL_P
    // @DisplayName: Roll axis PDD2 controller P gain
    // @Description: Roll axis PDD2 controller P gain.  Corrects in proportion to the difference between the desired roll rate vs actual roll rate
    // @Range: 0.01 0.5
    // @Increment: 0.005
    // @User: Standard

    // @Param: PDD2_RLL_D
    // @DisplayName: Roll axis PDD2 controller D gain
    // @Description: Roll axis PDD2 controller D gain.  Compensates for short-term change in desired roll rate vs actual roll rate
    // @Range: 0.0 0.05
    // @Increment: 0.001
    // @User: Standard

    // @Param: PDD2_RLL_D2
    // @DisplayName: Roll axis PDD2 controller D2 gain
    // @Description: Roll axis PDD2 controller D2 gain.  Corrects long-term difference in desired roll rate vs actual roll rate
    // @Range: 0.001 0.02
    // @Increment: 0.001
    // @User: Standard

    // @Param: PDD2_RLL_FLTT
    // @DisplayName: Roll axis rate controller target frequency in Hz
    // @Description: Roll axis rate controller target frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: PDD2_RLL_FLTE
    // @DisplayName: Roll axis rate controller error frequency in Hz
    // @Description: Roll axis rate controller error frequency in Hz
    // @Range: 0 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: PDD2_RLL_FLTD
    // @DisplayName: Roll axis rate controller derivative frequency in Hz
    // @Description: Roll axis rate controller derivative frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: PDD2_RLL_FLTD2
    // @DisplayName: Roll axis rate controller derivative 2 frequency in Hz
    // @Description: Roll axis rate controller derivative 2 frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: PDD2_RLL_SMAX
    // @DisplayName: Roll slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    AP_SUBGROUPINFO(_pdd2_atti_rate_roll,"PDD2_Roll", 1, AC_CustomControl_PDD2,AC_PDD2),

    // @Param: PDD2_PIT_P
    // @DisplayName: Pitch axis PDD2 controller P gain
    // @Description: Pitch axis PDD2 controller P gain.  Corrects in proportion to the difference between the desired Pitch rate vs actual Pitch rate
    // @Range: 0.01 0.5
    // @Increment: 0.005
    // @User: Standard

    // @Param: PDD2_PIT_D
    // @DisplayName: Pitch axis PDD2 controller D gain
    // @Description: Pitch axis PDD2 controller D gain.  Compensates for short-term change in desired Pitch rate vs actual Pitch rate
    // @Range: 0.0 0.05
    // @Increment: 0.001
    // @User: Standard

    // @Param: PDD2_PIT_D2
    // @DisplayName: Pitch axis PDD2 controller D2 gain
    // @Description: Pitch axis PDD2 controller D2 gain.  Corrects long-term difference in desired Pitch rate vs actual Pitch rate
    // @Range: 0.001 0.02
    // @Increment: 0.001
    // @User: Standard

    // @Param: PDD2_PIT_FLTT
    // @DisplayName: Pitch axis rate controller target frequency in Hz
    // @Description: Pitch axis rate controller target frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: PDD2_PIT_FLTE
    // @DisplayName: Pitch axis rate controller error frequency in Hz
    // @Description: Pitch axis rate controller error frequency in Hz
    // @Range: 0 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: PDD2_PIT_FLTD
    // @DisplayName: Pitch axis rate controller derivative frequency in Hz
    // @Description: Pitch axis rate controller derivative frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: PDD2_PIT_FLTD2
    // @DisplayName: Pitch axis rate controller derivative 2 frequency in Hz
    // @Description: Pitch axis rate controller derivative 2 frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: PDD2_PIT_SMAX
    // @DisplayName: Pitch slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced
    AP_SUBGROUPINFO(_pdd2_atti_rate_pitch,"PDD2_Pitch", 2, AC_CustomControl_PDD2,AC_PDD2),

    // @Param: PDD2_YAW_P
    // @DisplayName: Yaw axis PDD2 controller P gain
    // @Description: Yaw axis PDD2 controller P gain.  Corrects in proportion to the difference between the desired Yaw rate vs actual Yaw rate
    // @Range: 0.01 0.5
    // @Increment: 0.005
    // @User: Standard

    // @Param: PDD2_YAW_D
    // @DisplayName: Yaw axis PDD2 controller D gain
    // @Description: Yaw axis PDD2 controller D gain.  Compensates for short-term change in desired Yaw rate vs actual Yaw rate
    // @Range: 0.0 0.05
    // @Increment: 0.001
    // @User: Standard

    // @Param: PDD2_YAW_D2
    // @DisplayName: Yaw axis PDD2 controller D2 gain
    // @Description: Yaw axis PDD2 controller D2 gain.  Corrects long-term difference in desired Yaw rate vs actual Yaw rate
    // @Range: 0.001 0.02
    // @Increment: 0.001
    // @User: Standard

    // @Param: PDD2_YAW_FLTT
    // @DisplayName: Yaw axis rate controller target frequency in Hz
    // @Description: Yaw axis rate controller target frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: PDD2_YAW_FLTE
    // @DisplayName: Yaw axis rate controller error frequency in Hz
    // @Description: Yaw axis rate controller error frequency in Hz
    // @Range: 0 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: PDD2_YAW_FLTD
    // @DisplayName: Yaw axis rate controller derivative frequency in Hz
    // @Description: Yaw axis rate controller derivative frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: PDD2_YAW_FLTD2
    // @DisplayName: Yaw axis rate controller derivative 2 frequency in Hz
    // @Description: Yaw axis rate controller derivative 2 frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: PDD2_YAW_SMAX
    // @DisplayName: Yaw slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced
    AP_SUBGROUPINFO(_pdd2_atti_rate_yaw,"PDD2_Yaw", 3, AC_CustomControl_PDD2,AC_PDD2),

    AP_GROUPEND
};

// initialize in the constructor
AC_CustomControl_PDD2::AC_CustomControl_PDD2(AC_CustomControl& frontend, AP_AHRS*& ahrs, AC_AttitudeControl_Multi*& att_control, AC_PosControl*& pos_control , AP_MotorsMulticopter*& motors, float dt) :
    AC_CustomControl_Backend(frontend, ahrs, att_control, pos_control ,motors, dt),
    _pdd2_atti_rate_roll(AC_PDD2_DEFAULT_P_ROLL,AC_PDD2_DEFAULT_D_ROLL,AC_PDD2_DEFAULT_D2_ROLL,AC_PDD2_TFILT_HZ_DEFAULT,AC_PDD2_EFILT_HZ_DEFAULT,AC_PDD2_DFILT_HZ_DEFAULT,AC_PDD2_D2FILT_HZ_DEFAULT),
    _pdd2_atti_rate_pitch(AC_PDD2_DEFAULT_P_PITCH,AC_PDD2_DEFAULT_D_PITCH,AC_PDD2_DEFAULT_D2_PITCH,AC_PDD2_TFILT_HZ_DEFAULT,AC_PDD2_EFILT_HZ_DEFAULT,AC_PDD2_DFILT_HZ_DEFAULT,AC_PDD2_D2FILT_HZ_DEFAULT),
    _pdd2_atti_rate_yaw(AC_PDD2_DEFAULT_P_YAW,AC_PDD2_DEFAULT_D_YAW,AC_PDD2_DEFAULT_D2_YAW,AC_PDD2_TFILT_HZ_DEFAULT,AC_PDD2_EFILT_HZ_DEFAULT,AC_PDD2_DFILT_HZ_DEFAULT,AC_PDD2_D2FILT_HZ_DEFAULT)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// update controller
// return roll, pitch, yaw controller output
Vector3f AC_CustomControl_PDD2::update()
{
    // reset controller based on spool state
    switch (_motors->get_spool_state()) {
        case AP_Motors::SpoolState::SHUT_DOWN:
        case AP_Motors::SpoolState::GROUND_IDLE:
            // We are still at the ground. Reset custom controller to avoid
            // build up, ex: integrator
            reset();
            break;

        case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        case AP_Motors::SpoolState::SPOOLING_UP:
        case AP_Motors::SpoolState::SPOOLING_DOWN:
            // we are off the ground
            break;
    }

    gcs().send_text(MAV_SEVERITY_INFO, "PDD2 custom controller working");
    // run custom controller after here
    Quaternion attitude_body, attitude_target;

    Vector3f EulerOrientation,EulerTargets,Angular_Accelerations;

    _ahrs->get_quat_body_to_ned(attitude_body);
    attitude_body.to_euler(EulerOrientation.x, EulerOrientation.y, EulerOrientation.z);

    //EulerOrientation.x= _ahrs->get_roll();
    //EulerOrientation.y= _ahrs->get_pitch();                //bunlardeğil
    //EulerOrientation.z= _ahrs->get_yaw();

    EulerTargets.x = _pos_control->get_roll_cd();
    EulerTargets.y = _pos_control->get_pitch_cd();
    EulerTargets.z = _pos_control->get_yaw_cd();

    Angular_Accelerations = _ahrs->get_ang_acc();
    attitude_target = _att_control->get_attitude_target_quat();
    // This vector represents the angular error to rotate the thrust vector using x and y and heading using z
    Vector3f attitude_error;
    float _thrust_angle, _thrust_error_angle;
    _att_control->thrust_heading_rotation_angles(attitude_target, attitude_body, attitude_error, _thrust_angle, _thrust_error_angle);


    // run rate controller
    Vector3f gyro_latest = _ahrs->get_gyro_latest();
    Vector3f motor_out;
    motor_out.x = _pdd2_atti_rate_roll.update_all(EulerTargets.x, EulerOrientation.x, gyro_latest.x,Angular_Accelerations.x,1);
    motor_out.y = _pdd2_atti_rate_pitch.update_all(EulerTargets.y, EulerOrientation.y, gyro_latest.y,Angular_Accelerations.y,1);
    motor_out.z = _pdd2_atti_rate_yaw.update_all(EulerTargets.z, EulerOrientation.z, gyro_latest.z,Angular_Accelerations.z,1);

    return motor_out;
}

// reset controller to avoid build up on the ground
// or to provide bumpless transfer from arducopter main controller
void AC_CustomControl_PDD2::reset(void)
{

    _pdd2_atti_rate_roll.reset_filter();
    _pdd2_atti_rate_pitch.reset_filter();
    _pdd2_atti_rate_yaw.reset_filter();

}

#endif
