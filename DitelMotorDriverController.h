#ifndef DITEL_MOTOR_DRIVER_CONTROLLER_H
#define DITEL_MOTOR_DRIVER_CONTROLLER_H

#include "main.h"

#define DITEL_MOTOR_FORWARD 0x10
#define DITEL_MOTOR_REVERSAL 0x11
#define DITEL_MOTOR_NEUTRAL 0x12
#define DITEL_MOTOR_BRAKE 0x13

#define DITEL_MOTOR_PID_FORWARD 0x30
#define DITEL_MOTOR_PID_REVERSAL 0x31
#define DITEL_MOTOR_PID_NEUTRAL 0x32
#define DITEL_MOTOR_PID_BRAKE 0x33

#define DITEL_MOTOR_PID_GAIN_SET 0x3A
#define DITEL_MOTOR_PID_CONDITION 0x3B
#define DITEL_MOTOR_PID_ENABLE 1
#define DITEL_MOTOR_PID_DISABLE 0

#define DITEL_NONE 0

#define DITEL_MOTOR_MAX_OUTPUT 800
#define DITEL_MOTOR_MAX_PID_SPEED 20000
#define DITEL_MOTOR_PID_GAIN_MAX 50000

int DitelMotorDriverRotate(CAN_HandleTypeDef *hcan, uint8_t _motorDriverAddress, uint8_t _mode, uint16_t output);
int DitelMotor(CAN_HandleTypeDef *hcan, int motor_address, int output);

int DitelMotorDriverSetPIDGain(CAN_HandleTypeDef *_hcan, int _motorDriverAddress, uint16_t _kp, uint16_t _ki, uint16_t _kd,int8_t _scale);
int DitelMotorDriverPIDRotate(CAN_HandleTypeDef *_hcan, uint8_t _motorDriverAddress, uint8_t _mode, uint16_t _speed);
int DitelMotorPID(CAN_HandleTypeDef *_hcan, int _motorDriverAddress, int _speed);
int DitelMotorDriverPIDCondition(CAN_HandleTypeDef *_hcan, int _motorDriverAddress, int _condition);

#endif // DITEL_MOTOR_DRIVER_CONTROLLER_H
