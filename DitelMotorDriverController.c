#include "DitelMotorDriverController/DitelMotorDriverController.h"

int DitelMotorDriverRotate(CAN_HandleTypeDef *hcan, uint8_t _motorDriverAddress, uint8_t _mode, uint16_t output)
{
    static uint8_t lastMode[16]; // 0000～1111
    if (lastMode[_motorDriverAddress] != _mode && lastMode[_motorDriverAddress] != DITEL_MOTOR_NEUTRAL)
    {
        _mode = DITEL_MOTOR_NEUTRAL;
        output = 0;
    }
    lastMode[_motorDriverAddress] = _mode;

    uint8_t canSendData[8] = {};

    if (output > DITEL_MOTOR_MAX_OUTPUT)
        output = DITEL_MOTOR_MAX_OUTPUT;

    canSendData[0] = _mode;
    canSendData[1] = output >> 8;
    canSendData[2] = output & 0x00FF;
    canSendData[3] = 0;
    canSendData[4] = 0;
    canSendData[5] = 0;
    canSendData[6] = 0;
    canSendData[7] = 0;

    if (0 < HAL_CAN_GetTxMailboxesFreeLevel(hcan))
    {
        CAN_TxHeaderTypeDef TxHeader;
        uint32_t TxMailbox;
        TxHeader.StdId = _motorDriverAddress;  // CAN ID
        TxHeader.RTR = CAN_RTR_DATA;           // フレームタイプはデータフレーム
        TxHeader.IDE = CAN_ID_STD;             // 標準ID(11ﾋﾞｯﾄ)
        TxHeader.DLC = 8;                      // データ長は8バイトに
        TxHeader.TransmitGlobalTime = DISABLE; // ???
        HAL_CAN_AddTxMessage(hcan, &TxHeader, canSendData, &TxMailbox);

        return 0;
    }
    else
    {
        return 1;
    }
}

int DitelMotor(CAN_HandleTypeDef *hcan, int motor_address, int output)
{
    output *= 1;
    if (output > DITEL_MOTOR_MAX_OUTPUT)
        output = DITEL_MOTOR_MAX_OUTPUT;
    if (output < -DITEL_MOTOR_MAX_OUTPUT)
        output = -DITEL_MOTOR_MAX_OUTPUT;

    if (output > 0)
        return DitelMotorDriverRotate(hcan, motor_address, DITEL_MOTOR_FORWARD, 1 * output);
    else if (output < 0)
        return DitelMotorDriverRotate(hcan, motor_address, DITEL_MOTOR_REVERSAL, -1 * output);
    else
        // return DitelMotorDriverRotate(hcan,motor_address, DITEL_MOTOR_BRAKE,DITEL_NONE);
        return DitelMotorDriverRotate(hcan, motor_address, DITEL_MOTOR_NEUTRAL, DITEL_NONE);

    return 1;
}

int DitelMotorDriverSetPIDGain(CAN_HandleTypeDef *_hcan, int _motorDriverAddress, uint16_t _kp, uint16_t _ki, uint16_t _kd, int8_t _scale)
{
    if (_kp > DITEL_MOTOR_PID_GAIN_MAX)
    {
        _kp = DITEL_MOTOR_PID_GAIN_MAX;
    }
    // else if (_kp < -DITEL_MOTOR_PID_GAIN_MAX)
    // {
    //     _kp = -DITEL_MOTOR_PID_GAIN_MAX;
    // }
    if (_ki > DITEL_MOTOR_PID_GAIN_MAX)
    {
        _ki = DITEL_MOTOR_PID_GAIN_MAX;
    }
    // else if (_ki < -DITEL_MOTOR_PID_GAIN_MAX)
    // {
    //     _ki = -DITEL_MOTOR_PID_GAIN_MAX;
    // }
    if (_kd > DITEL_MOTOR_PID_GAIN_MAX)
    {
        _kd = DITEL_MOTOR_PID_GAIN_MAX;
    }
    // else if (_kd < -DITEL_MOTOR_PID_GAIN_MAX)
    // {
    //     _kd = -DITEL_MOTOR_PID_GAIN_MAX;
    // }

    uint8_t canSendData[8] = {};

    canSendData[0] = DITEL_MOTOR_PID_GAIN_SET; // PID Gain Set
    canSendData[1] = (_kp & 0xFF00) >> 8;
    canSendData[2] = (_kp & 0x00FF);
    canSendData[3] = (_ki & 0xFF00) >> 8;
    canSendData[4] = (_ki & 0x00FF);
    canSendData[5] = (_kd & 0xFF00) >> 8;
    canSendData[6] = _kd & 0x00FF;
    canSendData[7] = _scale;

    if (0 < HAL_CAN_GetTxMailboxesFreeLevel(_hcan))
    {
        CAN_TxHeaderTypeDef TxHeader;
        uint32_t TxMailbox;
        TxHeader.StdId = _motorDriverAddress;  // CAN ID
        TxHeader.RTR = CAN_RTR_DATA;           // フレームタイプはデータフレーム
        TxHeader.IDE = CAN_ID_STD;             // 標準ID(11ﾋﾞｯﾄ)
        TxHeader.DLC = 8;                      // データ長は8バイトに
        TxHeader.TransmitGlobalTime = DISABLE; // ???
        HAL_CAN_AddTxMessage(_hcan, &TxHeader, canSendData, &TxMailbox);

        return 0;
    }
    else
    {
        return 1;
    }
}

int DitelMotorDriverPIDRotate(CAN_HandleTypeDef *_hcan, uint8_t _motorDriverAddress, uint8_t _mode, uint16_t output)
{
    static uint8_t lastMode[16]; // 0000～1111
    if (lastMode[_motorDriverAddress] != _mode && lastMode[_motorDriverAddress] != DITEL_MOTOR_PID_NEUTRAL)
    {
        _mode = DITEL_MOTOR_PID_NEUTRAL;
        output = 0;
    }
    lastMode[_motorDriverAddress] = _mode;

    uint8_t canSendData[8] = {};

    if (output > DITEL_MOTOR_MAX_PID_SPEED)
        output = DITEL_MOTOR_MAX_PID_SPEED;

    canSendData[0] = _mode;
    canSendData[1] = output >> 8;
    canSendData[2] = output & 0x00FF;
    canSendData[3] = 0;
    canSendData[4] = 0;
    canSendData[5] = 0;
    canSendData[6] = 0;
    canSendData[7] = 0;

    if (0 < HAL_CAN_GetTxMailboxesFreeLevel(_hcan))
    {
        CAN_TxHeaderTypeDef TxHeader;
        uint32_t TxMailbox;
        TxHeader.StdId = _motorDriverAddress;  // CAN ID
        TxHeader.RTR = CAN_RTR_DATA;           // フレームタイプはデータフレーム
        TxHeader.IDE = CAN_ID_STD;             // 標準ID(11ﾋﾞｯﾄ)
        TxHeader.DLC = 8;                      // データ長は8バイトに
        TxHeader.TransmitGlobalTime = DISABLE; // ???
        HAL_CAN_AddTxMessage(_hcan, &TxHeader, canSendData, &TxMailbox);

        return 0;
    }
    else
    {
        return 1;
    }
}

int DitelMotorPID(CAN_HandleTypeDef *_hcan, int _motorDriverAddress, int _speed)
{
    _speed *= 1;
    if (_speed > DITEL_MOTOR_MAX_PID_SPEED)
        _speed = DITEL_MOTOR_MAX_PID_SPEED;
    if (_speed < -DITEL_MOTOR_MAX_PID_SPEED)
        _speed = -DITEL_MOTOR_MAX_PID_SPEED;

    if (_speed > 0)
        return DitelMotorDriverPIDRotate(_hcan, _motorDriverAddress, DITEL_MOTOR_PID_FORWARD, 1 * _speed);
    else if (_speed < 0)
        return DitelMotorDriverPIDRotate(_hcan, _motorDriverAddress, DITEL_MOTOR_PID_REVERSAL, -1 * _speed);
    else
        // return DitelMotorDriverPIDRotate(_hcan, _motorDriverAddress, DITEL_MOTOR_PID_BRAKE, DITEL_NONE);
        return DitelMotorDriverPIDRotate(_hcan, _motorDriverAddress, DITEL_MOTOR_PID_NEUTRAL, DITEL_NONE);

    return 1;
}

int DitelMotorDriverPIDCondition(CAN_HandleTypeDef *_hcan, int _motorDriverAddress, int _condition)
{
    uint8_t canSendData[8] = {};

    canSendData[0] = DITEL_MOTOR_PID_CONDITION;
    canSendData[1] = _condition;
    canSendData[2] = 0;
    canSendData[3] = 0;
    canSendData[4] = 0;
    canSendData[5] = 0;
    canSendData[6] = 0;
    canSendData[7] = 0;

    if (0 < HAL_CAN_GetTxMailboxesFreeLevel(_hcan))
    {
        CAN_TxHeaderTypeDef TxHeader;
        uint32_t TxMailbox;
        TxHeader.StdId = _motorDriverAddress;  // CAN ID
        TxHeader.RTR = CAN_RTR_DATA;           // フレームタイプはデータフレーム
        TxHeader.IDE = CAN_ID_STD;             // 標準ID(11ﾋﾞｯﾄ)
        TxHeader.DLC = 8;                      // データ長は8バイトに
        TxHeader.TransmitGlobalTime = DISABLE; // ???
        HAL_CAN_AddTxMessage(_hcan, &TxHeader, canSendData, &TxMailbox);

        return 0;
    }
    else
    {
        return 1;
    }
}