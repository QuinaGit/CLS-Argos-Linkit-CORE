#ifndef __ARTIC_REGS_HPP_
#define __ARTIC_REGS_HPP_

#include <stdint.h>

#include "bsp.hpp"

#ifndef GTEST
#define SAT_TURN_ON_TIME_S  (16)
#else
#define SAT_TURN_ON_TIME_S  ( 0) // Note: Unit testing makes more sense if we ignore this parameter
#endif

#define SAT_ARTIC_DELAY_BOOT_MS              (1000)
#define SAT_ARTIC_DELAY_POWER_ON_MS          (1000)
#define SAT_ARTIC_DELAY_INTERRUPT_MS         (10000)
#define SAT_ARTIC_DELAY_INTERRUPT_1_PROG_MS  (1000)
#define SAT_ARTIC_TIMEOUT_SEND_TX_MS         (20000)
#define SAT_ARTIC_DELAY_RESET_MS             (5)
#define SAT_ARTIC_DELAY_BURST_MS             (5)
#define SAT_ARTIC_DELAY_SET_BURST_MS         (22)
#define SAT_ARTIC_DELAY_FINISH_BURST_MS      (13)
#define SAT_ARTIC_DELAY_TRANSFER_US          (50)
#define NUM_FIRMWARE_FILES_ARTIC 3

#define BURST_ADDRESS  (0)
#define ADDRESS_DSP    (1)

#define NUM_BYTES_BEFORE_WAIT   60

#define ARTIC_WRITE_ADDRESS(x) (((x << 1) & 0xF7))
#define ARTIC_READ_ADDRESS(x)  (((x << 1) & 0xF7) | 1) // LSB set to indicate a read

#define BURST_MODE_SHIFT_BITMASK    (0x080000)
#define MEM_SEL_BITMASK             (0x060000)
#define MEM_SEL_SHIFT               (17)
#define BURST_R_NW_MODE_BITMASK     (0x010000)
#define BURST_START_ADDRESS_BITMASK (0x00FFFF)

#define FIRMWARE_VERSION_ADDRESS (0x0010)
#define CRC_ADDRESS              (0x0371)
#define INTERRUPT_ADDRESS        (0x8018)
#define TX_PAYLOAD_ADDRESS       (0x0273)
#define TCXO_WARMUP_TIME_ADDRESS (0x036F)

#define TX_FREQUENCY_ARGOS_2_3	 (0x034F)
#define TX_FREQUENCY_ARGOS_4   	 (0x035F)

#define FIRMWARE_ADDRESS_LENGTH       (3)
#define SIZE_SPI_REG_XMEM_YMEM_IOMEM  (3)
#define SIZE_SPI_REG_PMEM             (4)

#define SPI_MAX_BYTE_READ  (8192)

#define MAX_BURST  (2048)

#define MAX_TX_SIZE_BYTES  (31)

#define MAX_BUFFER_READ  (256)

#define INTERRUPT_1  (1)
#define INTERRUPT_2  (2)

#define TOTAL_NUMBER_STATUS_FLAG  (24)


#define ID_SIZE_BYTE (4)

#define ARTIC_PACKET_HDR_BYTES (3)

#define ARTIC_MSG_LEN_BITS      ( 4)
#define ARTIC_MSG_ID_SIZE_BITS  (28)
#define ARTIC_MSG_ID_BITMASK    (0x0FFFFFFF)

#define ARTIC_ZTE_MAX_USER_BITS            (0)
#define ARTIC_ZTE_MAX_USER_BYTES           (0)
#define ARTIC_ZTE_MSG_NUM_TAIL_BITS        (8)
#define ARTIC_ZTE_MSG_TOTAL_BITS           (ARTIC_MSG_ID_SIZE_BITS + ARTIC_ZTE_MAX_USER_BITS + ARTIC_ZTE_MSG_NUM_TAIL_BITS)
#define ARTIC_ZTE_BYTES_TO_SEND            (9) // CEIL(ARTIC_ZTE_MSG_TOTAL_BITS + ARTIC_MSG_LEN_BITS) / 8) + ARTIC_PACKET_HDR_BYTES // Must be divisible by 3

#define ARTIC_PTT_A3_24_MAX_USER_BITS      (24)
#define ARTIC_PTT_A3_24_MAX_USER_BYTES     ((unsigned int) ARTIC_PTT_A3_24_MAX_USER_BITS / 8)
#define ARTIC_PTT_A3_24_MSG_LEN_FIELD      (0x00)
#define ARTIC_PTT_A3_24_MSG_NUM_TAIL_BITS  (7)
#define ARTIC_PTT_A3_24_MSG_TOTAL_BITS     (ARTIC_MSG_LEN_BITS + ARTIC_MSG_ID_SIZE_BITS + ARTIC_PTT_A3_24_MAX_USER_BITS + ARTIC_PTT_A3_24_MSG_NUM_TAIL_BITS)
#define ARTIC_PTT_A3_24_BYTES_TO_SEND      (12)

#define ARTIC_PTT_A3_56_MAX_USER_BITS      (56)
#define ARTIC_PTT_A3_56_MAX_USER_BYTES     ((unsigned int) ARTIC_PTT_A3_56_MAX_USER_BITS / 8)
#define ARTIC_PTT_A3_56_MSG_LEN_FIELD      (0x30)
#define ARTIC_PTT_A3_56_MSG_NUM_TAIL_BITS  (8)
#define ARTIC_PTT_A3_56_MSG_TOTAL_BITS     (ARTIC_MSG_LEN_BITS + ARTIC_MSG_ID_SIZE_BITS + ARTIC_PTT_A3_56_MAX_USER_BITS + ARTIC_PTT_A3_56_MSG_NUM_TAIL_BITS)
#define ARTIC_PTT_A3_56_BYTES_TO_SEND      (15)

#define ARTIC_PTT_A3_88_MAX_USER_BITS      (88)
#define ARTIC_PTT_A3_88_MAX_USER_BYTES     ((unsigned int) ARTIC_PTT_A3_88_MAX_USER_BITS / 8)
#define ARTIC_PTT_A3_88_MSG_LEN_FIELD      (0x50)
#define ARTIC_PTT_A3_88_MSG_NUM_TAIL_BITS  (9)
#define ARTIC_PTT_A3_88_MSG_TOTAL_BITS     (ARTIC_MSG_LEN_BITS + ARTIC_MSG_ID_SIZE_BITS + ARTIC_PTT_A3_88_MAX_USER_BITS + ARTIC_PTT_A3_88_MSG_NUM_TAIL_BITS)
#define ARTIC_PTT_A3_88_BYTES_TO_SEND      (21)

#define ARTIC_PTT_A3_120_MAX_USER_BITS     (120)
#define ARTIC_PTT_A3_120_MAX_USER_BYTES    ((unsigned int) ARTIC_PTT_A3_120_MAX_USER_BITS / 8)
#define ARTIC_PTT_A3_120_MSG_LEN_FIELD     (0x60)
#define ARTIC_PTT_A3_120_MSG_NUM_TAIL_BITS (7)
#define ARTIC_PTT_A3_120_MSG_TOTAL_BITS    (ARTIC_MSG_LEN_BITS + ARTIC_MSG_ID_SIZE_BITS + ARTIC_PTT_A3_120_MAX_USER_BITS + ARTIC_PTT_A3_120_MSG_NUM_TAIL_BITS)
#define ARTIC_PTT_A3_120_BYTES_TO_SEND     (24)

#define ARTIC_PTT_A3_152_MAX_USER_BITS     (152)
#define ARTIC_PTT_A3_152_MAX_USER_BYTES    ((unsigned int) ARTIC_PTT_A3_152_MAX_USER_BITS / 8)
#define ARTIC_PTT_A3_152_MSG_LEN_FIELD     (0x90)
#define ARTIC_PTT_A3_152_MSG_NUM_TAIL_BITS (8)
#define ARTIC_PTT_A3_152_MSG_TOTAL_BITS    (ARTIC_MSG_LEN_BITS + ARTIC_MSG_ID_SIZE_BITS + ARTIC_PTT_A3_152_MAX_USER_BITS + ARTIC_PTT_A3_152_MSG_NUM_TAIL_BITS)
#define ARTIC_PTT_A3_152_BYTES_TO_SEND     (27)

#define ARTIC_PTT_A3_184_MAX_USER_BITS     (184)
#define ARTIC_PTT_A3_184_MAX_USER_BYTES    ((unsigned int) ARTIC_PTT_A3_184_MAX_USER_BITS / 8)
#define ARTIC_PTT_A3_184_MSG_LEN_FIELD     (0xA0)
#define ARTIC_PTT_A3_184_MSG_NUM_TAIL_BITS (9)
#define ARTIC_PTT_A3_184_MSG_TOTAL_BITS    (ARTIC_MSG_LEN_BITS + ARTIC_MSG_ID_SIZE_BITS + ARTIC_PTT_A3_184_MAX_USER_BITS + ARTIC_PTT_A3_184_MSG_NUM_TAIL_BITS)
#define ARTIC_PTT_A3_184_BYTES_TO_SEND     (33)

#define ARTIC_PTT_A3_216_MAX_USER_BITS     (216)
#define ARTIC_PTT_A3_216_MAX_USER_BYTES    ((unsigned int) ARTIC_PTT_A3_216_MAX_USER_BITS / 8)
#define ARTIC_PTT_A3_216_MSG_LEN_FIELD     (0xC0)
#define ARTIC_PTT_A3_216_MSG_NUM_TAIL_BITS (7)
#define ARTIC_PTT_A3_216_MSG_TOTAL_BITS    (ARTIC_MSG_LEN_BITS + ARTIC_MSG_ID_SIZE_BITS + ARTIC_PTT_A3_216_MAX_USER_BITS + ARTIC_PTT_A3_216_MSG_NUM_TAIL_BITS)
#define ARTIC_PTT_A3_216_BYTES_TO_SEND     (36)

#define ARTIC_PTT_A3_248_MAX_USER_BITS     (248)
#define ARTIC_PTT_A3_248_MAX_USER_BYTES    ((unsigned int) ARTIC_PTT_A3_248_MAX_USER_BITS / 8)
#define ARTIC_PTT_A3_248_MSG_LEN_FIELD     (0xF0)
#define ARTIC_PTT_A3_248_MSG_NUM_TAIL_BITS (8)
#define ARTIC_PTT_A3_248_MSG_TOTAL_BITS    (ARTIC_MSG_LEN_BITS + ARTIC_MSG_ID_SIZE_BITS + ARTIC_PTT_A3_248_MAX_USER_BITS + ARTIC_PTT_A3_248_MSG_NUM_TAIL_BITS)
#define ARTIC_PTT_A3_248_BYTES_TO_SEND     (39)

#define ARTIC_MSG_MAX_SIZE (ARTIC_PTT_A3_248_BYTES_TO_SEND)

#define TAIL_BITS_VALUE  0

#define MAX_WORD_A3 7
#define TOTAL_LEN_MAX_A3 288

#define MAXIMUM_READ_FIRMWARE_OPERATION (FIRMWARE_ADDRESS_LENGTH + SIZE_SPI_REG_PMEM)

// COMMANDS //

// Artic setting commands
#define ARTIC_CMD_SET_ARGOS_4_RX_MODE          (0x01)  // 00000001b
#define ARTIC_CMD_SET_ARGOS_3_RX_MODE          (0x02)  // 00000010b
#define ARTIC_CMD_SET_ARGOS_3_RX_BACKUP_MODE   (0x03)  // 00000011b
#define ARTIC_CMD_SET_PTT_A2_TX_MODE           (0x04)  // 00000100b
#define ARTIC_CMD_SET_PTT_A3_TX_MODE           (0x05)  // 00000101b
#define ARTIC_CMD_SET_PTT_ZE_TX_MODE           (0x06)  // 00000110b
#define ARTIC_CMD_SET_ARGOS_3_PTT_HD_TX_MODE   (0x07)  // 00000111b
#define ARTIC_CMD_SET_ARGOS_4_PTT_HD_TX_MODE   (0x08)  // 00001000b
#define ARTIC_CMD_SET_ARGOS_4_PTT_MD_TX_MODE   (0x09)  // 00001001b
#define ARTIC_CMD_SET_PTT_VLD_TX_MODE          (0x0A)  // 00001010b

// Artic instruction commands
#define ARTIC_CMD_START_RX_CONT                (0x41)  // 01000001b
#define ARTIC_CMD_START_RX_1M                  (0x42)  // 01000010b
#define ARTIC_CMD_START_RX_2M                  (0x43)  // 01000011b
#define ARTIC_CMD_START_RX_TIMED               (0x46)  // 01000110b
#define ARTIC_CMD_START_TX_1M_SLEEP            (0x48)  // 01001000b
#define ARTIC_CMD_START_TX_1M_RX_TIMED         (0x49)  // 01001001b
#define ARTIC_CMD_SLEEP                        (0x50)  // 01010000b
#define ARTIC_CMD_SATELLITE_DETECTION          (0x55)  // 01010101b

// Artic commands to clear interrupt flags
#define ARTIC_CMD_CLEAR_INT1  (0x80)   // 10XXXXXXb
#define ARTIC_CMD_CLEAR_INT2  (0xC0)   // 11XXXXXXb

#endif // __ARTIC_REGS_HPP_
