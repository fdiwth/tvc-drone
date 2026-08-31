#ifndef TYPES_H
#define TYPES_H

#include <stdint.h>
#include "Fusion.h"

/* Bitmask of optional fields captured in a flight log record.
 * timestamp_ms and drone_state are always present regardless of mask —
 * they're needed to make sense of / bound every record. */
typedef enum {
    LOG_FIELD_ACCEL   = 1u << 0,
    LOG_FIELD_GYRO    = 1u << 1,
    LOG_FIELD_MAG     = 1u << 2,
    LOG_FIELD_BARO    = 1u << 3,
    LOG_FIELD_TOF     = 1u << 4,
    LOG_FIELD_FLOW    = 1u << 5,
    LOG_FIELD_VOLTAGE = 1u << 6,
    LOG_FIELD_ORI     = 1u << 7,   // filtered rx, ry, rz, pz
    LOG_FIELD_POS     = 1u << 8,   // filtered pos x, y, z
    LOG_FIELD_POS_P   = 1u << 9,   // pos_p_x/pos_p_y output
    LOG_FIELD_VEL_PID = 1u << 10,  // vel_pid_x/vel_pid_y output (post thrust-scale)
    LOG_FIELD_LQR     = 1u << 11,  // lqr.u[0..3]
    LOG_FIELD_SERVO   = 1u << 12,
    LOG_FIELD_MOTOR   = 1u << 13,
} log_field_flags_t;

#define LOG_MASK_DEFAULT   (LOG_FIELD_ORI | LOG_FIELD_POS | LOG_FIELD_SERVO | LOG_FIELD_MOTOR)
#define LOG_MASK_ALL       0x3FFFu
#define LOG_MASK_ERASED    0xFFFFFFFFu  // sentinel: matches erased-flash 0xFF fill

/* Worst-case serialized size of one record (LOG_MASK_ALL active):
 *   header: mask(4) + timestamp_ms(4) + drone_state(1)          =   9
 *   accel(12) + gyro(12) + mag(12) + baro(4) + tof(4)
 *     + flow(8) + voltage(4)                                    =  56
 *   ori(16) + pos(12) + pos_p(8) + vel_pid(8) + lqr(16)          =  60
 *   servo(8) + motor(8)                                         =  16
 *   total = 141, rounded up with margin
 */
#define FLIGHT_LOG_MAX_RECORD_SIZE 160

/* Full in-RAM representation of everything that COULD be logged in one
 * control cycle. Always fully populated by ControlTask; g_log_field_mask
 * determines which subset actually gets serialized to flash. */
typedef struct {
    uint32_t timestamp_ms;
    uint8_t  drone_state;

    FusionVector accel;
    FusionVector gyro;
    FusionVector mag;
    float        baro_altitude_m;
    float        tof_altitude_m;
    float        flow_dx;
    float        flow_dy;
    float        voltage;

    float rx, ry, rz;                  // filtered orientation
    float pos_x, pos_y, pos_z;             // filtered position

    float pos_p_out_x, pos_p_out_y;        // pos_p_x / pos_p_y output
    float vel_pid_out_x, vel_pid_out_y;    // vel_pid_x / vel_pid_y output
    float lqr_u[4];                        // lqr.u[0..3]

    float servo_x, servo_y;
    float motor1, motor2;
} flight_log_t;

typedef struct {
    FusionVector accel;
    FusionVector gyro;
    FusionVector mag;
    float        flow_dx;
    float        flow_dy;
    float        baro_altitude_m;   // was baro_pressure_Pa — now height, so it's directly comparable to tof
    float        tof_altitude_m;
    float        bus_voltage_V;
    FusionVector pos;
    FusionEuler  ori;
} telemetry_packet_t;

typedef enum {
    DRONE_DISARMED = 0,
    DRONE_INIT,
    DRONE_ARMED,
    DRONE_FLYING
} drone_state_t;

#endif
