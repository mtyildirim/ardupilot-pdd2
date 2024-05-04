#include "AC_CustomControl_PDD2.h"

#if CUSTOMCONTROL_PDD2_ENABLED

#include <GCS_MAVLink/GCS.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_CustomControl_PDD2::var_info[] = {
    // @Param: PARAM1
    // @DisplayName: PDD2 param1
    // @Description: Dumy parameter for PDD2 custom controller backend
    // @User: Advanced
    AP_SUBGROUPINFO(_pdd2_atti_rate_roll,"PDD2_1", 1, AC_CustomControl_PDD2,AC_PDD2),

    // @Param: PARAM2
    // @DisplayName: PDD2 param2
    // @Description: Dumy parameter for PDD2 custom controller backend
    // @User: Advanced
    AP_SUBGROUPINFO(_pdd2_atti_rate_pitch,"PDD2_2", 2, AC_CustomControl_PDD2,AC_PDD2),

    // @Param: PARAM3
    // @DisplayName: PDD2 param3
    // @Description: Dumy parameter for PDD2 custom controller backend
    // @User: Advanced
    AP_SUBGROUPINFO(_pdd2_atti_rate_yaw,"PDD2_3", 3, AC_CustomControl_PDD2,AC_PDD2),

    AP_GROUPEND
};

// initialize in the constructor
AC_CustomControl_PDD2::AC_CustomControl_PDD2(AC_CustomControl& frontend, AP_AHRS*& ahrs, AC_AttitudeControl_Multi*& att_control, AC_PosControl*& pos_control , AP_MotorsMulticopter*& motors, float dt) :
    AC_CustomControl_Backend(frontend, ahrs, att_control, pos_control ,motors, dt),
    _pdd2_atti_rate_roll(AC_PDD2_DEFAULT_P_ROLL,AC_PDD2_DEFAULT_D_ROLL,AC_PDD2_DEFAULT_D2_ROLL,0,AC_PDD2_TFILT_HZ_DEFAULT,AC_PDD2_EFILT_HZ_DEFAULT,AC_PDD2_DFILT_HZ_DEFAULT,AC_PDD2_D2FILT_HZ_DEFAULT),
    _pdd2_atti_rate_pitch(AC_PDD2_DEFAULT_P_PITCH,AC_PDD2_DEFAULT_D_PITCH,AC_PDD2_DEFAULT_D2_PITCH,0,AC_PDD2_TFILT_HZ_DEFAULT,AC_PDD2_EFILT_HZ_DEFAULT,AC_PDD2_DFILT_HZ_DEFAULT,AC_PDD2_D2FILT_HZ_DEFAULT),
    _pdd2_atti_rate_yaw(AC_PDD2_DEFAULT_P_YAW,AC_PDD2_DEFAULT_D_YAW,AC_PDD2_DEFAULT_D2_YAW,0,AC_PDD2_TFILT_HZ_DEFAULT,AC_PDD2_EFILT_HZ_DEFAULT,AC_PDD2_DFILT_HZ_DEFAULT,AC_PDD2_D2FILT_HZ_DEFAULT)
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
    EulerOrientation[0]= _ahrs->get_roll();
    EulerOrientation[1]= _ahrs->get_pitch();
    EulerOrientation[2]= _ahrs->get_yaw();

    EulerTargets[0] = _pos_control->get_roll_cd();
    EulerTargets[1] = _pos_control->get_pitch_cd();
    EulerTargets[2] = _pos_control->get_yaw_cd();

    Angular_Accelerations = _ahrs->get_ang_acc();

    attitude_target = _att_control->get_attitude_target_quat();
    // This vector represents the angular error to rotate the thrust vector using x and y and heading using z
    Vector3f attitude_error;
    float _thrust_angle, _thrust_error_angle;
    _att_control->thrust_heading_rotation_angles(attitude_target, attitude_body, attitude_error, _thrust_angle, _thrust_error_angle);


    // run rate controller
    Vector3f gyro_latest = _ahrs->get_gyro_latest();
    Vector3f motor_out;
    motor_out.x = _pdd2_atti_rate_roll.update_all(EulerTargets[0], EulerOrientation[0], gyro_latest[0],Angular_Accelerations[0],1);
    motor_out.y = _pdd2_atti_rate_pitch.update_all(EulerTargets[1], EulerOrientation[1], gyro_latest[1],Angular_Accelerations[1],1);
    motor_out.z = _pdd2_atti_rate_yaw.update_all(EulerTargets[2], EulerOrientation[2], gyro_latest[2],Angular_Accelerations[2],1);

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
