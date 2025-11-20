#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <algorithm>
#include <cmath>
#include <../data/constants.h>
#include <box2d/box2d.h>

/**
 * @brief PIDController for steering angle and longitudinal force.
 *
 * Two independent PIDs are used:
 *  - Steering PID acts on heading error (radians).
 *  - Speed PID acts on speed error (m/s) and outputs force (N).
 *
 * Features:
 *  - Configurable gains and limits
 *  - Output saturation + conditional anti-windup
 *  - Optional integral clamping
 *  - Derivative-on-measurement option for throttle to reduce kick
 *  - Reset for bumpless restart
 *
 * All functions are noexcept where practical.
 */
class PIDController {
public:
    /// Controller output, consisting of steering angle and motor force.
    struct ControllerOutput {
        float steering_angle;  ///< [rad]
        float motor_force;     ///< [N]
    };

    /// Configuration bundle for readability.
    struct Gains {
        float kp, ki, kd;
    };

    /// Limits for each loop.
    struct Limits {
        float out_min;     ///< output minimum (rad or N)
        float out_max;     ///< output maximum (rad or N)
        float i_min;       ///< integral clamp min (term before ki multiply)
        float i_max;       ///< integral clamp max
    };

    /**
     * @brief Construct controller with defaults (tune these to your vehicle).
     */
    PIDController() noexcept
        : steer_{/*kp*/1.0f, /*ki*/0.5f, /*kd*/3.0f}
        , speed_{/*kp*/200.0f, /*ki*/1.7f, /*kd*/600.0f}
        , lim_steer_{-0.6f, 0.6f, -0.2f, 0.2f}   // ~±34° steering, modest I clamp
        , lim_speed_{-9000.0f, 9000.0f, -50.0f, 50.0f} // ±5 kN force, I clamp
        , use_derivative_on_measurement_speed_{true}
        , d_lpf_alpha_{0.15f}                    // low-pass for derivative
        , prev_angle_error_{0.0f}
        , prev_speed_meas_{0.0f}
        , prev_speed_error_{0.0f}
        , i_steer_{0.0f}
        , i_speed_{0.0f}
        , dmeas_speed_filt_{0.0f}
    {}

    /// Resets integrators and history (use on enable/disable, test setup).
    void reset() noexcept {
        prev_angle_error_ = 0.0f;
        prev_speed_meas_  = 0.0f;
        prev_speed_error_ = 0.0f;
        dmeas_speed_filt_ = 0.0f;
        i_steer_ = 0.0f;
        i_speed_ = 0.0f;
    }

    /// Set gains (live-tunable).
    void setSteerGains(Gains g) noexcept { steer_ = g; }
    void setSpeedGains(Gains g) noexcept { speed_ = g; }

    /// Set limits (outputs and integrals).
    void setSteerLimits(Limits l) noexcept { lim_steer_ = l; }
    void setSpeedLimits(Limits l) noexcept { lim_speed_ = l; }

    /// Enable/disable derivative-on-measurement for speed loop.
    void setDerivativeOnMeasurementForSpeed(bool enabled) noexcept {
        use_derivative_on_measurement_speed_ = enabled;
    }

    /// Set low-pass alpha for derivative filtering in (0,1]; smaller = heavier filtering.
    void setDerivativeLPFAlpha(float alpha) noexcept {
        d_lpf_alpha_ = std::clamp(alpha, 0.01f, 1.0f);
    }

    /**
     * @brief Compute controller output.
     * @param current_angle Vehicle orientation (rad).
     * @param current_pos   Vehicle position (m).
     * @param target_pos    Target position (m).
     * @param current_speed Current speed (m/s).
     * @param target_speed  Target (m/s).
     * @param dt            Time step (seconds).
     * @return Command {steering_angle (rad), motor_force (N)}.
     * @note Implements output saturation + conditional anti-windup.
     */
    ControllerOutput compute(float current_angle,
                    const b2Vec2& current_pos,
                    const b2Vec2& target_pos,
                    float current_speed,
                    float target_speed,
                    float dt) noexcept
    {
        dt = std::clamp(dt, 1e-3f, 0.1f); // avoid div0
        // --- Steering loop ---
        const float desired_angle = std::atan2(target_pos.y - current_pos.y,
                                               target_pos.x - current_pos.x);
        float angle_error = wrapToPi(desired_angle - current_angle);

        // Derivative (on error) with simple LPF optional for noisy headings
        const float d_err = (angle_error - prev_angle_error_) / dt;
        prev_angle_error_ = angle_error;

        // Provisional integral (apply conditional anti-windup after output clamp)
        float i_steer_new = i_steer_ + angle_error * dt;
        i_steer_new = std::clamp(i_steer_new, lim_steer_.i_min, lim_steer_.i_max);

        float u_steer = steer_.kp * angle_error
                      + steer_.ki * i_steer_new
                      + steer_.kd * d_err;

        // Output clamp
        const float u_steer_sat = std::clamp(u_steer, lim_steer_.out_min, lim_steer_.out_max);

        // Conditional integration: accept integrator only if not driving deeper into saturation
        if ((u_steer == u_steer_sat) || ((u_steer > u_steer_sat) && angle_error < 0.0f) ||
            ((u_steer < u_steer_sat) && angle_error > 0.0f))
        {
            i_steer_ = i_steer_new;
        }
        // else: freeze i_steer_

        // --- Speed loop ---
        const float speed_error = target_speed - current_speed;

        float d_term_speed = 0.0f;
        if (use_derivative_on_measurement_speed_) {
            // Derivative on measurement to avoid kick
            float dmeas = (current_speed - prev_speed_meas_) / dt;
            // Low-pass filter derivative
            dmeas_speed_filt_ = d_lpf_alpha_ * dmeas + (1.0f - d_lpf_alpha_) * dmeas_speed_filt_;
            d_term_speed = -speed_.kd * dmeas_speed_filt_;
            prev_speed_meas_ = current_speed;
        } else {
            // Classic derivative on error
            const float d_err_speed = (speed_error - prev_speed_error_) / dt;
            d_term_speed = speed_.kd * d_err_speed;
            prev_speed_error_ = speed_error;
        }

        float i_speed_new = i_speed_ + speed_error * dt;
        i_speed_new = std::clamp(i_speed_new, lim_speed_.i_min, lim_speed_.i_max);

        float u_speed = speed_.kp * speed_error + speed_.ki * i_speed_new + d_term_speed;

        const float u_speed_sat = std::clamp(u_speed, lim_speed_.out_min, lim_speed_.out_max);

        // Conditional anti-windup for speed
        if ((u_speed == u_speed_sat) || ((u_speed > u_speed_sat) && speed_error < 0.0f) ||
            ((u_speed < u_speed_sat) && speed_error > 0.0f))
        {
            i_speed_ = i_speed_new;
        }

        return { u_steer_sat, u_speed_sat };
    }

private:
    // Fast angle wrap to [-pi, pi]
    static float wrapToPi(float a) noexcept {
        // std::remainder maps to (-pi, pi]; ensure [-pi, pi]
        float r = std::remainder(a, 2.0f * static_cast<float>(M_PI));
        if (r <= -static_cast<float>(M_PI)) r += 2.0f * static_cast<float>(M_PI);
        return r;
    }

    Gains  steer_;
    Gains  speed_;
    Limits lim_steer_;
    Limits lim_speed_;

    bool  use_derivative_on_measurement_speed_;
    float d_lpf_alpha_;

    // State
    float prev_angle_error_;
    float prev_speed_meas_;
    float prev_speed_error_;
    float i_steer_;
    float i_speed_;
    float dmeas_speed_filt_;
};

#endif // PIDCONTROLLER_H
