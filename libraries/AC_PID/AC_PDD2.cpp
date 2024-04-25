/// @file	AC_PDD2.cpp
/// @brief	Generic PDD2 algorithm

#include <AP_Math/AP_Math.h>
#include "AC_PDD2.h"

const AP_Param::GroupInfo AC_PDD2::var_info[] = {
    // @Param: P
    // @DisplayName: PDD2 Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("P", 0, AC_PDD2, _kp, default_kp),

    // @Param: I
    // @DisplayName: PDD2 Integral Gain
    // @Description: I Gain which produces an output that is proportional to both the magnitude and the duration of the error
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("D", 1, AC_PDD2, _kd, default_kd),

    // @Param: D
    // @DisplayName: PDD2 Derivative Gain
    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("D2", 2, AC_PDD2, _kd, default_kd2),

    // 3 was for uint16 IMAX

    // @Param: FF
    // @DisplayName: FF FeedForward Gain
    // @Description: FF Gain which produces an output value that is proportional to the demanded input
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FF", 3, AC_PDD2, _kff, default_kff),

    // 6 was for float FILT

    // 7 is for float ILMI and FF

    // index 8 was for AFF

    // @Param: FLTT
    // @DisplayName: PDD2 Target filter frequency in Hz
    // @Description: Target filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FLTT", 4, AC_PDD2, _filt_T_hz, default_filt_T_hz),

    // @Param: FLTE
    // @DisplayName: PDD2 Error filter frequency in Hz
    // @Description: Error filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FLTE", 5, AC_PDD2, _filt_E_hz, default_filt_E_hz),

    // @Param: FLTD
    // @DisplayName: PDD2 Derivative term filter frequency in Hz
    // @Description: Derivative filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FLTD", 6, AC_PDD2, _filt_D_hz, default_filt_D_hz),
    
    // @Param: FLTD2
    // @DisplayName: PDD2 Derivative 2 term filter frequency in Hz
    // @Description: Derivative 2 filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FLTD2", 7, AC_PDD2, _filt_D2_hz, default_filt_D2_hz),

    // @Param: SMAX
    // @DisplayName: Slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("SMAX", 8, AC_PDD2, _slew_rate_max, default_slew_rate_max),

    AP_GROUPEND
};

// Constructor
AC_PDD2::AC_PDD2(float initial_p, float initial_d, float initial_d2, float initial_ff, float initial_filt_T_hz, float initial_filt_E_hz, float initial_filt_D_hz,
               float initial_filt_D2_hz, float initial_srmax, float initial_srtau) :
    default_kp(initial_p),
    default_kd(initial_d),
    default_kd2(initial_d2),
    default_kff(initial_ff),
    default_filt_T_hz(initial_filt_T_hz),
    default_filt_E_hz(initial_filt_E_hz),
    default_filt_D_hz(initial_filt_D_hz),
    default_filt_D2_hz(initial_filt_D2_hz),
    default_slew_rate_max(initial_srmax)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);

    // this param is not in the table, so its default is no loaded in the call above
    _slew_rate_tau.set(initial_srtau);

    // reset input filter to first value received
    _flags._reset_filter = true;

    memset(&_PDD2_info, 0, sizeof(_PDD2_info));

    // slew limit scaler allows for plane to use degrees/sec slew
    // limit
    _slew_limit_scale = 1;
}

// filt_T_hz - set target filter hz
void AC_PDD2::filt_T_hz(float hz)
{
    _filt_T_hz.set(fabsf(hz));
}

// filt_E_hz - set error filter hz
void AC_PDD2::filt_E_hz(float hz)
{
    _filt_E_hz.set(fabsf(hz));
}

// filt_D_hz - set derivative filter hz
void AC_PDD2::filt_D_hz(float hz)
{
    _filt_D_hz.set(fabsf(hz));
}

// slew_limit - set slew limit
void AC_PDD2::slew_limit(float smax)
{
    _slew_rate_max.set(fabsf(smax));
}

//  update_all - set target and measured inputs to PDD2 controller and calculate outputs
//  target and error are filtered
//  the derivative is then calculated and filtered
//  the integral is then updated based on the setting of the limit flag
float AC_PDD2::update_all(float target, float measurement, float dt, bool limit, float boost)
{
    // don't process inf or NaN
    if (!isfinite(target) || !isfinite(measurement)) {
        return 0.0f;
    }

    // reset input filter to value received
    if (_flags._reset_filter) {
        _flags._reset_filter = false;
        _target = target;
        _error = _target - measurement;
        _derivative = 0.0f;
    } else {
        float error_last = _error;
        _target += get_filt_T_alpha(dt) * (target - _target);
        _error += get_filt_E_alpha(dt) * ((_target - measurement) - _error);

        // calculate and filter derivative
        if (is_positive(dt)) {
            float derivative = (_error - error_last) / dt;
            _derivative += get_filt_D_alpha(dt) * (derivative - _derivative);
        }

        if (is_positive(dt)) {
            float derivative2 = (_error - error_last) / dt;
            _derivative2 += get_filt_D_alpha(dt) * (derivative2 - _derivative2);
        }
    
    }
    
    float P_out = (_error * _kp);
    float D_out = (_derivative * _kd);
    float D2_out = (_derivative2 * _kd2);

    // calculate slew limit modifier for P+D
    _PDD2_info.Dmod = _slew_limiter.modifier((_PDD2_info.P + _PDD2_info.D + _PDD2_info.D2) * _slew_limit_scale, dt);
    _PDD2_info.slew_rate = _slew_limiter.get_slew_rate();

    P_out *= _PDD2_info.Dmod;
    D_out *= _PDD2_info.Dmod;

    // boost output if required
    P_out *= boost;
    D_out *= boost;

    _PDD2_info.target = _target;
    _PDD2_info.actual = measurement;
    _PDD2_info.error = _error;
    _PDD2_info.P = P_out;
    _PDD2_info.D = D_out;
    _PDD2_info.D2 = D2_out;

    return P_out + D_out + D2_out;
}

//  update_error - set error input to PDD2 controller and calculate outputs
//  target is set to zero and error is set and filtered
//  the derivative then is calculated and filtered
//  the integral is then updated based on the setting of the limit flag
//  Target and Measured must be set manually for logging purposes.
// todo: remove function when it is no longer used.
float AC_PDD2::update_error(float error, float dt, bool limit)
{
    // don't process inf or NaN
    if (!isfinite(error)) {
        return 0.0f;
    }

    _target = 0.0f;

    // reset input filter to value received
    if (_flags._reset_filter) {
        _flags._reset_filter = false;
        _error = error;
        _derivative = 0.0f;
    } else {
        float error_last = _error;
        _error += get_filt_E_alpha(dt) * (error - _error);

        // calculate and filter derivative
        if (is_positive(dt)) {
            float derivative = (_error - error_last) / dt;
            _derivative += get_filt_D_alpha(dt) * (derivative - _derivative);
        }
    
        if (is_positive(dt)) {
            float derivative2 = (_error - error_last) / dt;
            _derivative2 += get_filt_D2_alpha(dt) * (derivative2 - _derivative2);
        }
    }

    float P_out = (_error * _kp);
    float D_out = (_derivative * _kd);
    float D2_out = (_derivative2 * _kd2);

    // calculate slew limit modifier for P+D
    _PDD2_info.Dmod = _slew_limiter.modifier((_PDD2_info.P + _PDD2_info.D) * _slew_limit_scale, dt);
    _PDD2_info.slew_rate = _slew_limiter.get_slew_rate();

    P_out *= _PDD2_info.Dmod;
    D_out *= _PDD2_info.Dmod;
    
    _PDD2_info.target = 0.0f;
    _PDD2_info.actual = 0.0f;
    _PDD2_info.error = _error;
    _PDD2_info.P = P_out;
    _PDD2_info.D = D_out;

    return P_out + D_out + D2_out;
}

float AC_PDD2::get_p() const
{
    return _error * _kp;
}

float AC_PDD2::get_d() const
{
    return _kd * _derivative;
}

float AC_PDD2::get_d2() const
{
    return _kd2 * _derivative2;
}

float AC_PDD2::get_ff()
{
    _PDD2_info.FF = _target * _kff;
    return _target * _kff;
}

void AC_PDD2::load_gains()
{
    _kp.load();
    _kd.load();
    _kd2.load();
    _kff.load();
    _filt_T_hz.load();
    _filt_E_hz.load();
    _filt_D_hz.load();
}

// save_gains - save gains to eeprom
void AC_PDD2::save_gains()
{
    _kp.save();
    _kd.save();
    _kd2.save();
    _kff.save();
    _filt_T_hz.save();
    _filt_E_hz.save();
    _filt_D_hz.save();
}

/// Overload the function call operator to permit easy initialisation
void AC_PDD2::operator()(float p_val, float d_val, float d2_val, float ff_val, float input_filt_T_hz, float input_filt_E_hz, float input_filt_D_hz,float input_filt_D2_hz)
{
    _kp.set(p_val);
    _kd.set(d_val);
    _kd2.set(d2_val);
    _kff.set(ff_val);
    _filt_T_hz.set(input_filt_T_hz);
    _filt_E_hz.set(input_filt_E_hz);
    _filt_D_hz.set(input_filt_D_hz);
    _filt_D2_hz.set(input_filt_D2_hz);
}

// get_filt_T_alpha - get the target filter alpha
float AC_PDD2::get_filt_T_alpha(float dt) const
{
    return calc_lowpass_alpha_dt(dt, _filt_T_hz);
}

// get_filt_E_alpha - get the error filter alpha
float AC_PDD2::get_filt_E_alpha(float dt) const
{
    return calc_lowpass_alpha_dt(dt, _filt_E_hz);
}

// get_filt_D_alpha - get the derivative filter alpha
float AC_PDD2::get_filt_D_alpha(float dt) const
{
    return calc_lowpass_alpha_dt(dt, _filt_D_hz);
}

// get_filt_D2_alpha - get the derivative2 filter alpha
float AC_PDD2::get_filt_D2_alpha(float dt) const
{
    return calc_lowpass_alpha_dt(dt, _filt_D2_hz);
}
