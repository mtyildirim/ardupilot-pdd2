#pragma once

/// @file	AC_PDD2.h
/// @brief	Generic PDD2 algorithm, with EEPROM-backed storage of constants.

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <stdlib.h>
#include <cmath>
#include <Filter/SlewLimiter.h>



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

#include "AP_PIDInfo.h"

/// @class	AC_PDD2
/// @brief	Copter PDD2 control class
class AC_PDD2 {
public:

    // Constructor for PDD2
    AC_PDD2(float initial_p, float initial_d, float initial_d2, float initial_ff, float initial_filt_T_hz, float initial_filt_E_hz, float initial_filt_D_hz,float initial_filt_D2_hz,
           float initial_srmax=0, float initial_srtau=1.0);

    CLASS_NO_COPY(AC_PDD2);

    //  update_all - set target and measured inputs to PDD2 controller and calculate outputs
    //  target and error are filtered
    //  the derivative is then calculated and filtered
    //  the integral is then updated based on the setting of the limit flag
    float update_all(float target, float angle, float gyro, float ang_acc, float dt, bool limit = false, float boost = 1.0f);

    //  update_error - set error input to PDD2 controller and calculate outputs
    //  target is set to zero and error is set and filtered
    //  the derivative then is calculated and filtered
    //  the integral is then updated based on the setting of the limit flag
    //  Target and Measured must be set manually for logging purposes.
    // todo: remove function when it is no longer used.
    float update_error(float error,float gyro, float ang_acc, float dt, bool limit = false);

    // get_PDD2 - get results from PDD2 controller
    float get_PDD2() const;
    float get_p() const;
    float get_d() const;
    float get_d2() const;
    float get_ff();


    // reset_filter - input filter will be reset to the next value provided to set_input()
    void reset_filter() {
        _flags._reset_filter = true;
    }

    // load gain from eeprom
    void load_gains();

    // save gain to eeprom
    void save_gains();

    /// operator function call for easy initialisation
    void operator()(float p_val, float d_val, float d2_val, float ff_val, float input_filt_T_hz, float input_filt_E_hz, float input_filt_D_hz,float input_filt_D2_hz);

    // get accessors
    AP_Float &kP() { return _kp; }
    AP_Float &kD() { return _kd; }
    AP_Float &kD2() { return _kd2; }
    AP_Float &ff() { return _kff;}
    AP_Float &filt_T_hz() { return _filt_T_hz; }
    AP_Float &filt_E_hz() { return _filt_E_hz; }
    AP_Float &filt_D_hz() { return _filt_D_hz; }
    AP_Float &slew_limit() { return _slew_rate_max; }

    float get_filt_T_alpha(float dt) const;
    float get_filt_E_alpha(float dt) const;
    float get_filt_D_alpha(float dt) const;
    float get_filt_D2_alpha(float dt) const;
    // set accessors
    void kP(const float v) { _kp.set(v); }
    void kD(const float v) { _kd.set(v); }
    void kD2(const float v) { _kd2.set(v); }
    void ff(const float v) { _kff.set(v); }
    void filt_T_hz(const float v);
    void filt_E_hz(const float v);
    void filt_D_hz(const float v);
    void filt_D2_hz(const float v);   
    void slew_limit(const float v);

    // set the desired and actual rates (for logging purposes)
    void set_target_rate(float target) { _PDD2_info.target = target; }
    void set_actual_rate(float actual) { _PDD2_info.actual = actual; }

    // set slew limiter scale factor
    void set_slew_limit_scale(int8_t scale) { _slew_limit_scale = scale; }

    // return current slew rate of slew limiter. Will return 0 if SMAX is zero
    float get_slew_rate(void) const { return _slew_limiter.get_slew_rate(); }

    const AP_PIDInfo& get_PDD2_info(void) const { return _PDD2_info; }

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

    // the time constant tau is not currently configurable, but is set
    // as an AP_Float to make it easy to make it configurable for a
    // single user of AC_PDD2 by adding the parameter in the param
    // table of the parent class. It is made public for this reason
    AP_Float _slew_rate_tau;
    
protected:

    // parameters
    AP_Float _kp;
    AP_Float _kd;
    AP_Float _kd2;
    AP_Float _kff;
    AP_Float _filt_T_hz;         // PDD2 target filter frequency in Hz
    AP_Float _filt_E_hz;         // PDD2 error filter frequency in Hz
    AP_Float _filt_D_hz;         // PDD2 derivative filter frequency in Hz
    AP_Float _filt_D2_hz;
    AP_Float _slew_rate_max;

    SlewLimiter _slew_limiter{_slew_rate_max, _slew_rate_tau};

    // flags
    struct ac_PDD2_flags {
        bool _reset_filter :1; // true when input filter should be reset during next call to set_input
    } _flags;

    // internal variables
    float _target;            // target value to enable filtering
    float _error;             // error value to enable filtering
    float _derivative;        // derivative value to enable filtering
    float _derivative2;
    float _last_derivative;
    int8_t _slew_limit_scale;

    AP_PIDInfo _PDD2_info;

private:
    const float default_kp;
    const float default_kd;
    const float default_kd2;
    const float default_kff;
    const float default_filt_T_hz;
    const float default_filt_E_hz;
    const float default_filt_D_hz;
    const float default_filt_D2_hz;
    const float default_slew_rate_max;
};
