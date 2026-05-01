#ifndef INC_MODBUSRTU_H_
#define INC_MODBUSRTU_H_

#include "stm32g4xx_hal.h"
#include "string.h"

#define MODBUS_MESSAGEBUFFER_SIZE 300

typedef union
{
    uint16_t U16;
    uint8_t U8[2];
} u16u8_t;

typedef enum
{
    Modbus_state_Init,
    Modbus_state_Idle,
    Modbus_state_Reception,
    Modbus_state_ControlAndWaiting,
    Modbus_state_Emission
} ModbusStateTypedef;

typedef enum
{
    Modbus_function_Read_Holding_Register = 0x03,
    Modbus_function_Write_Single_Register = 0x06
} ModbusFunctionCode;

typedef enum
{
    Modbus_RecvFrame_Null = -2,
    Modbus_RecvFrame_FrameError = -1,
    Modbus_RecvFrame_Normal = 0,
    Modbus_RecvFrame_IllegalFunction = 0x01,
    Modbus_RecvFrame_IllegalDataAddress = 0x02,
    Modbus_RecvFrame_IllegalDataValue = 0x03,
    Modbus_RecvFrame_SlaveDeviceFailure = 0x04
} ModbusRecvFrameStatus;

typedef struct
{
    uint8_t RxBuffer[MODBUS_MESSAGEBUFFER_SIZE];
    uint16_t RxTail;
    uint8_t TxBuffer[MODBUS_MESSAGEBUFFER_SIZE];
    uint16_t TxTail;
} ModbusUartStructure;

typedef struct
{
    uint8_t slaveAddress;
    u16u8_t *RegisterAddress;
    uint32_t RegisterSize;

    UART_HandleTypeDef* huart;
    TIM_HandleTypeDef* htim;

    uint8_t Flag_T15TimeOut;
    uint8_t Flag_T35TimeOut;

    ModbusRecvFrameStatus RecvStatus;
    ModbusStateTypedef Mstatus;

    uint8_t RxFrame[MODBUS_MESSAGEBUFFER_SIZE];
    uint8_t TxFrame[MODBUS_MESSAGEBUFFER_SIZE];
    uint8_t TxCount;

    ModbusUartStructure UartStructure;
} ModbusHandleTypedef;

void Modbus_init(ModbusHandleTypedef* hmodbus, u16u8_t* RegisterStartAddress);
void Modbus_Protocol_Worker(ModbusHandleTypedef* hmodbus);

#endif /* INC_MODBUSRTU_H_ */
