#ifndef MODBUS_BRIDGE_H
#define MODBUS_BRIDGE_H

#include "main.h"

/**
 * @brief Initialize the Modbus bridge
 */
void Modbus_Init(void);

/**
 * @brief Process incoming serial data for Modbus
 * @param data Received byte
 */
void Modbus_ProcessByte(uint8_t data);

/**
 * @brief Sync internal motor variables to Modbus registers (Read map)
 * Should be called periodically (e.g., in the 100Hz loop)
 */
void Modbus_UpdateRegisters(void);

#endif /* MODBUS_BRIDGE_H */
