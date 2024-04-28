#include "AC_CustomControl_PDD2.h"

#if CUSTOMCONTROL_PDD2_ENABLED

#include <GCS_MAVLink/GCS.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_CustomControl_PDD2::var_info[] = {
    // @Param: PARAM1
    // @DisplayName: PDD2 param1
    // @Description: Dumy parameter for PDD2 custom controller backend
    // @User: Advanced
    AP_SUBGROUPINFO("PARAM1", 1, AC_CustomControl_PDD2,_pdd2_atti_rate_roll , 0.0f),

    // @Param: PARAM2
    // @DisplayName: PDD2 param2
    // @Description: Dumy parameter for PDD2 custom controller backend
    // @User: Advanced
    AP_SUBGROUPINFO("PARAM2", 2, AC_CustomControl_PDD2, _pdd2_atti_rate_pitch, 0.0f),

    // @Param: PARAM3
    // @DisplayName: PDD2 param3
    // @Description: Dumy parameter for PDD2 custom controller backend
    // @User: Advanced
    AP_SUBGROUPINFO("PARAM3", 3, AC_CustomControl_PDD2, _pdd2_atti_rate_yaw, 0.0f),

    AP_GROUPEND
};

// initialize in the constructor
AC_CustomControl_PDD2::AC_CustomControl_PDD2(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl_Multi*& att_control, AP_MotorsMulticopter*& motors, float dt) :
    AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt),
    _pdd2_atti_rate_roll(),
    _pdd2_atti_rate_pitch(),
    _pdd2_atti_rate_yaw()
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
    _ahrs->get_quat_body_to_ned(attitude_body);

    attitude_target = _att_control->get_attitude_target_quat();
    // This vector represents the angular error to rotate the thrust vector using x and y and heading using z
    Vector3f attitude_error;
    float _thrust_angle, _thrust_error_angle;
    _att_control->thrust_heading_rotation_angles(attitude_target, attitude_body, attitude_error, _thrust_angle, _thrust_error_angle);


    // run rate controller
    Vector3f gyro_latest = _ahrs->get_gyro_latest();
    Vector3f motor_out;
    motor_out.x = _pid_atti_rate_roll.update_all(target_rate[0], gyro_latest[0], 1);
    motor_out.y = _pid_atti_rate_pitch.update_all(target_rate[1], gyro_latest[1], 1);
    motor_out.z = _pid_atti_rate_yaw.update_all(target_rate[2], gyro_latest[2], 1);

    return motor_out;
}

// reset controller to avoid build up on the ground
// or to provide bumpless transfer from arducopter main controller
void AC_CustomControl_PDD2::reset(void)
{
}

#endif
