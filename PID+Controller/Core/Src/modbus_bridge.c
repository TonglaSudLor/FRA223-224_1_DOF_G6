#include "modbus_bridge.h"
#include "motor_pid_controller.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

/* --- Modbus Configuration --- */
#define SLAVE_ID            21
#define REG_COUNT           0x32  // Up to 0x31

/* --- Register Storage --- */
static uint16_t holding_registers[REG_COUNT];

/* --- Modbus RTU State Machine --- */
#define MODBUS_BUF_SIZE     64
static uint8_t rx_buf[MODBUS_BUF_SIZE];
static uint8_t rx_idx = 0;
static uint32_t last_rx_time = 0;

/* --- CRC16 Calculation (Modbus) --- */
static uint16_t Modbus_CRC16(uint8_t *nData, uint16_t wLength)
{
    uint16_t wCRCWord = 0xFFFF;
    while (wLength--) {
        wCRCWord ^= *(nData++);
        for (int nBitCnt = 0; nBitCnt < 8; nBitCnt++) {
            if (wCRCWord & 0x01) {
                wCRCWord = (wCRCWord >> 1) ^ 0xA001;
            } else {
                wCRCWord >>= 1;
            }
        }
    }
    return wCRCWord;
}

/* --- Placeholder Action Handlers --- */

static void Handle_OperatingMode(uint16_t val) {
    // 0x01: Home, Jog, Auto, Set Home, Test
    printf("Modbus: Set Operating Mode bits: 0x%04X\n", val);
}

static void Handle_JogCommand(int16_t degrees) {
    // 0x05: Signed step size
    printf("Modbus: Jog %d degrees\n", degrees);
}

static void Handle_TestConfig(uint16_t type, uint16_t vel, uint16_t accel) {
    // 0x06, 0x07, 0x08
    printf("Modbus: Test Config - Type: %u, Vel: %u, Accel: %u\n", type, vel, accel);
}

static void Handle_PrecisionTest(uint16_t start, uint16_t end, int16_t repeat) {
    // 0x09, 0x10, 0x11
    printf("Modbus: Precision Test - Start: %u, End: %u, Repeat: %d\n", start, end, repeat);
}

static void Handle_PickPlacePlan(uint16_t pairs) {
    // 0x22: Number of pairs (each pair is a Pick and a Place)
    // Sequence slots: 0x12 to 0x21
    printf("Modbus: Execute Pick & Place plan with %u pairs\n", pairs);
    
    for (int i = 0; i < pairs * 2; i++) {
        if (i >= 16) break; // Limit to available slots (0x12-0x21 is 16 regs)
        
        int16_t hole_index = (int16_t)holding_registers[0x12 + i];
        float target_deg = hole_index * 10.0f;
        
        printf("  Slot %d: Hole Index %d -> Target %.1f deg\n", i, hole_index, target_deg);
        
        // TODO: In the real implementation, you would add these to a queue 
        // and have your motion sequencer visit each target_deg.
    }
}

void Modbus_Init(void)
{
    memset(holding_registers, 0, sizeof(holding_registers));
    holding_registers[0x00] = 22881; // "YA"
}

/**
 * @brief Handles Write commands from the PC (Master)
 */
static void Modbus_HandleWrite(uint16_t addr, uint16_t val)
{
    if (addr >= REG_COUNT) return;
    holding_registers[addr] = val;

    switch (addr) {
        case 0x00: // Heartbeat reply
            if (val == 18537) printf("Modbus: Heartbeat HI received\n");
            break;

        case 0x01: Handle_OperatingMode(val); break;
        
        case 0x02: // Manual Gripper
            if (val == 1) Gripper_Up();
            else if (val == 2) Gripper_Down();
            else if (val == 4) Gripper_Open();
            else if (val == 8) Gripper_Close();
            break;

        case 0x03: // Gripper Sequence
            if (val == 1) Gripper_Sequence_Pick();
            else if (val == 2) Gripper_Sequence_Place();
            break;

        case 0x04: // Gripper in AUTO toggle
            printf("Modbus: Gripper in AUTO: %s\n", val ? "ENABLED" : "DISABLED");
            break;

        case 0x05: Handle_JogCommand((int16_t)val); break;

        case 0x06: // Test Type
        case 0x07: // Performance Vel
        case 0x08: // Performance Accel
            Handle_TestConfig(holding_registers[0x06], holding_registers[0x07], holding_registers[0x08]);
            break;

        case 0x09: // Precision Start
        case 0x10: // Precision End
        case 0x11: // Precision Repeat
            Handle_PrecisionTest(holding_registers[0x09], holding_registers[0x10], (int16_t)holding_registers[0x11]);
            break;

        case 0x22: Handle_PickPlacePlan(val); break;

        case 0x23: // P2P Unit (Degree vs Index)
            printf("Modbus: P2P Unit set to %s\n", val ? "INDEX" : "DEGREE");
            break;

        case 0x24: // P2P Target
            printf("Modbus: P2P Target: %d\n", (int16_t)val);
            Motor_MoveToPosition((float)((int16_t)val));
            break;

        case 0x25: // Safety / Soft Stop
            if (val & 0x01) {
                Emergency_stop = true;
                printf("Modbus: SOFT STOP REQUESTED\n");
            } else {
                Emergency_stop = false;
            }
            break;

        default: break;
    }
}

void Modbus_ProcessByte(uint8_t data)
{
    uint32_t now = HAL_GetTick();
    if (now - last_rx_time > 5) rx_idx = 0;
    last_rx_time = now;

    if (rx_idx < MODBUS_BUF_SIZE) rx_buf[rx_idx++] = data;

    if (rx_idx >= 8) {
        if (rx_buf[0] == SLAVE_ID) {
            uint16_t crc_calc = Modbus_CRC16(rx_buf, rx_idx - 2);
            uint16_t crc_recv = rx_buf[rx_idx - 2] | (rx_buf[rx_idx - 1] << 8);

            if (crc_calc == crc_recv) {
                uint8_t func = rx_buf[1];
                uint16_t addr = (rx_buf[2] << 8) | rx_buf[3];
                
                if (func == 0x03) { // Read Holding Registers
                    uint16_t count = (rx_buf[4] << 8) | rx_buf[5];
                    if (addr + count <= REG_COUNT) {
                        uint8_t tx_buf[MODBUS_BUF_SIZE];
                        tx_buf[0] = SLAVE_ID;
                        tx_buf[1] = 0x03;
                        tx_buf[2] = count * 2;
                        for (int i = 0; i < count; i++) {
                            uint16_t val = holding_registers[addr + i];
                            tx_buf[3 + i * 2] = val >> 8;
                            tx_buf[4 + i * 2] = val & 0xFF;
                        }
                        uint16_t tx_crc = Modbus_CRC16(tx_buf, 3 + count * 2);
                        tx_buf[3 + count * 2] = tx_crc & 0xFF;
                        tx_buf[4 + count * 2] = tx_crc >> 8;
                        HAL_UART_Transmit(&hlpuart1, tx_buf, 5 + count * 2, 100);
                    }
                }
                else if (func == 0x06) { // Write Single Register
                    uint16_t val = (rx_buf[4] << 8) | rx_buf[5];
                    Modbus_HandleWrite(addr, val);
                    HAL_UART_Transmit(&hlpuart1, rx_buf, rx_idx, 100);
                }
                rx_idx = 0;
            }
        }
    }
}

void Modbus_UpdateRegisters(void)
{
    // --- INFO Registers (PC Reads these) ---
    
    // 0x00: Heartbeat (Robot sends YA)
    holding_registers[0x00] = 22881; 

    // 0x26: Lead/Reed Sensors (Placeholder bits)
    // Bit 0: Reed 1, Bit 1: Reed 2, Bit 2: Reed 3
    holding_registers[0x26] = 0; 

    // 0x27: Current robot task (Status Bits)
    uint16_t task_bits = 0;
    if (current_mode == MOTOR_MODE_STOPPED) task_bits = 0; // Idle
    else if (current_mode == MOTOR_MODE_POSITION) task_bits |= (1 << 1); // Jogging/Moving
    // Map more modes to README bits (Homing, Pick, Place, etc.)
    holding_registers[0x27] = task_bits;

    // 0x28: Position (divided by 10 in UI, so we multiply by 10 here)
    holding_registers[0x28] = (int16_t)(Motor_GetPosition() * 10.0f);

    // 0x29: Velocity
    holding_registers[0x29] = (int16_t)(Motor_GetSpeed() * 10.0f);

    // 0x30: Acceleration
    holding_registers[0x30] = 0; // Placeholder

    // 0x31: Emergency / Safety state
    holding_registers[0x31] = Emergency_stop ? 1 : 0;
}
