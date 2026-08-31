#ifndef ODRIVE_ENDPOINTS_HPP
#define ODRIVE_ENDPOINTS_HPP

#include <cstdint>

// Constants for CAN communication
static constexpr uint16_t per_axis_offset = 0x100;

// Common endpoints
#define CLEAR_ERRORS 0x0001
#define VBUS_VOLTAGE 0x0002

// Per-axis endpoints (add per_axis_offset * axis for actual address)
#define AXIS__REQUESTED_STATE 0x0003
#define AXIS__ERROR 0x0004
#define AXIS__CONFIG__WATCHDOG_TIMEOUT 0x0005
#define AXIS__CONFIG__ENABLE_WATCHDOG 0x0006
#define AXIS__WATCHDOG_FEED 0x0007
#define AXIS__ENCODER__POS_ESTIMATE 0x0008
#define AXIS__ENCODER__VEL_ESTIMATE 0x0009
#define AXIS__MOTOR__CONFIG__TORQUE_CONSTANT 0x000A
#define AXIS__MOTOR__CURRENT_CONTROL__IQ_MEASURED 0x000B
#define AXIS__MOTOR__ERROR 0x000C
#define AXIS__ENCODER__ERROR 0x000D
#define AXIS__CONTROLLER__ERROR 0x000E
#define AXIS__MOTOR__FET_THERMISTOR__TEMPERATURE 0x000F
#define AXIS__MOTOR__MOTOR_THERMISTOR__TEMPERATURE 0x0010

// Axis states
#define AXIS_STATE_IDLE 1
#define AXIS_STATE_CLOSED_LOOP_CONTROL 8

// Helper function to calculate axis-specific endpoint
static constexpr uint16_t get_axis_endpoint(uint16_t base_endpoint, uint8_t axis) {
    return base_endpoint + per_axis_offset * axis;
}

// CAN-specific endpoints for optimized communication
#define CAN_GET_VBUS_VOLTAGE 0x0011
#define CAN_GET_ENCODER_ESTIMATES 0x0012
#define CAN_GET_IQ_MEASURED 0x0013
#define CAN_SET_INPUT_POS 0x0014
#define CAN_SET_INPUT_VEL 0x0015
#define CAN_SET_INPUT_TORQUE 0x0016
#define CAN_SET_CONTROLLER_MODE 0x0017
#define CAN_SET_AXIS_STATE 0x0018

#endif // ODRIVE_ENDPOINTS_HPP