/**
 * @file MotorCtrl.hpp
 * @brief Motor control functions
 * @brief control protocol is Bidirectional DSHOT-300
 */
#ifndef MOTORCTRL_HPP__
#define MOTORCTRL_HPP__

#include <cstdint>
#include <Circbuffer.hpp>

namespace Motor
{
    struct Motor_FB_t
    {
        int64_t time;
        uint32_t eRPM;
    };

    // max range is 2047, 0 throttle is 48.
    constexpr uint32_t DSHOT_THROTTLE_MAX = 1999;

    /**
     * @brief Initialize the motor controller and start the 'keep-alive' task.
     * @note This code startup the motor controller with a sequence of 0
     * throttles. It will also launch a high priority task that send out a 0
     * throttle command if no other throttle values are sent over 100ms, just to
     * keep the motor controller online, and to ensure that if something went
     * wrong the throttle will be set to 0.
     */
    void Init();

    enum class DShot_Command
    {
        // Commands below are only executed when motors are stopped
        MOTOR_STOP = 0,
        BEEP1 = 1,
        BEEP2 = 2,
        BEEP3 = 3,
        BEEP4 = 4,
        BEEP5 = 5,
        ESC_INFO = 6,
        SPIN_DIRECTION_1 = 7,
        SPIN_DIRECTION_2 = 8,
        SWITCH_3D_MODE_OFF = 9,
        SWITCH_3D_MODE_ON = 10,
        SETTINGS_REQUEST = 11,
        SAVE_SETTINGS = 12,
        SPIN_DIRECTION_NORMAL = 20,
        SPIN_DIRECTION_REVERSED = 21,
        LED0_ON = 22,
        LED1_ON = 23,
        LED2_ON = 24,
        LED3_ON = 25,
        LED0_OFF = 26,
        LED1_OFF = 27,
        LED2_OFF = 28,
        LED3_OFF = 29,
        TOGGLE_AUDIO_STREAM_MODE = 30,
        TOGGLE_SILENT_MODE = 31,
        SIGNAL_LINE_TELEMETRY_DISABLE = 32,
        SIGNAL_LINE_TELEMETRY_ENABLE = 33,
        SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY = 34,
        SIGNAL_LINE_CONTINUOUS_ERPM_PERIOD_TELEMETRY = 35,

        // Commands below are executed at any time
        SIGNAL_LINE_TEMPERATURE_TELEMETRY = 42,
        SIGNAL_LINE_VOLTAGE_TELEMETRY = 43,
        SIGNAL_LINE_CURRENT_TELEMETRY = 44,
        SIGNAL_LINE_CONSUMPTION_TELEMETRY = 45,
        SIGNAL_LINE_ERPM_TELEMETRY = 46,
        SIGNAL_LINE_ERPM_PERIOD_TELEMETRY = 47,
    };

    /**
     * @brief set the motor thrust and get the motor speed feedback
     *
     * @param throttle throttle value from 0 to 1999
     * @return uint32_t electrical RPM in us, convert that to frequency using
     * 10^6/(pole count * eRPM), pole count is usually 12 or 14 for larger
     * motors.
     *
     * @note the procedure is:
     * 1. In TX mode, send the Bidirectional Dshot frame to set the throttle.
     * 2. After TX frame is sent, switch to RX mode to listen for the motor
     * speed feedback.
     * 3. After received RX feedback, switch back to TX mode + infinite loops
     * but no longer listen for RX.
     */
    void Set_throttle(uint32_t throttle);

    /**
     * @brief send command to ESC
     *
     * @param throttle throttle value from 0 to 1999
     * @return uint32_t electrical RPM in us, convert that to frequency using
     * 10^6/(pole count * eRPM), pole count is usually 12 or 14 for larger
     * motors.
     *
     * @note the procedure is:
     * 1. In TX mode, send the Bidirectional Dshot frame to set the throttle.
     * 2. After TX frame is sent, switch to RX mode to listen for the motor
     * speed feedback.
     * 3. After received RX feedback, switch back to TX mode + infinite loops
     * but no longer listen for RX.
     */
    void Send_command(DShot_Command cmd);

    // size of the motor buffer
    constexpr size_t Motor_FB_buffer_size = 100 + 3;

    Circbuffer_copycat<Motor_FB_t, Motor_FB_buffer_size> Get_motor_speed_buffer();
} // namespace Motor

#endif