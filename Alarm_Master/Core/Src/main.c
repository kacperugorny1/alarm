/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "keyboard.h"
#include "state_machine.h"
#include "flash_interface.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


alarm_state state = DISARMED;
<<<<<<< HEAD
#define NRF_REG_CONFIG      0x00
#define NRF_REG_EN_AA       0x01
#define NRF_REG_EN_RXADDR   0x02
#define NRF_REG_SETUP_AW    0x03
#define NRF_REG_SETUP_RETR  0x04
#define NRF_REG_RF_CH       0x05
#define NRF_REG_RF_SETUP    0x06
#define NRF_REG_STATUS      0x07
#define NRF_REG_RX_ADDR_P0  0x0A
#define NRF_REG_TX_ADDR     0x10
#define NRF_REG_RX_PW_P0    0x11 // Note: For dynamic payload, actual payload width is read via R_RX_PL_WID
#define NRF_REG_FIFO_STATUS 0x17
#define NRF_REG_DYNPD       0x1C
#define NRF_REG_FEATURE     0x1D

// --- NRF24L01+ Command Definitions ---
#define NRF_CMD_R_REGISTER    0x00 // Read command and status registers. AAAAA = 5 bit register map address
#define NRF_CMD_W_REGISTER    0x20 // Write command and status registers. AAAAA = 5 bit register map address. Executable in power down or standby modes only.
#define NRF_CMD_R_RX_PAYLOAD  0x61 // Read RX-payload: 1 – 32 bytes. A read operation always starts at byte 0. Payload is deleted from FIFO after it is read. Used in RX mode.
#define NRF_CMD_W_TX_PAYLOAD  0xA0 // Write TX-payload: 1 – 32 bytes. A write operation always starts at byte 0. Used in TX payload.
#define NRF_CMD_FLUSH_TX      0xE1 // Flush TX FIFO, used in TX mode
#define NRF_CMD_FLUSH_RX      0xE2 // Flush RX FIFO, used in RX mode. Should not be executed during transmission of ACK packet.
#define NRF_CMD_R_RX_PL_WID   0x60 // Read RX-payload width for the top R_RX_PAYLOAD in the RX FIFO.
#define NRF_CMD_NOP           0xFF // No Operation. Might be used to read the STATUS register

// --- STATUS Register Bits ---
#define NRF_STATUS_RX_DR    (1<<6) // Data Ready RX FIFO interrupt
#define NRF_STATUS_TX_DS    (1<<5) // Data Sent TX FIFO interrupt
#define NRF_STATUS_MAX_RT   (1<<4) // Maximum number of TX retransmits interrupt

// --- CONFIG Register Bits ---
#define NRF_CONFIG_MASK_RX_DR  (1<<6) // Mask interrupt caused by RX_DR
#define NRF_CONFIG_MASK_TX_DS  (1<<5) // Mask interrupt caused by TX_DS
#define NRF_CONFIG_MASK_MAX_RT (1<<4) // Mask interrupt caused by MAX_RT
#define NRF_CONFIG_EN_CRC      (1<<3) // Enable CRC. Forced high if one of the bits in EN_AA is high
#define NRF_CONFIG_CRCO        (1<<2) // CRC encoding scheme, '0':1 byte, '1':2 bytes
#define NRF_CONFIG_PWR_UP      (1<<1) // 1: POWER UP, 0:POWER DOWN
#define NRF_CONFIG_PRIM_RX     (1<<0) // RX/TX control, '1':PRX, '0':PTX


HAL_StatusTypeDef nrf_write_register(uint8_t reg_addr, uint8_t* data, uint8_t size);
HAL_StatusTypeDef nrf_read_register_multi(uint8_t reg_addr, uint8_t* read_data, uint8_t size);
uint8_t nrf_read_single_register(uint8_t reg_addr);
uint8_t nrf_read_status_register(void);
void nrf_clear_status_flags(uint8_t flags_to_clear);
uint8_t nrf_read_rx_payload_width(void);
HAL_StatusTypeDef nrf_read_rx_payload(uint8_t* payload_buffer, uint8_t width);
void nrf_flush_tx_fifo(void);
void nrf_flush_rx_fifo(void);
void init_nrf_slave(void);
void transmit_nrf_payload(uint8_t size, char* data);
void init_nrf_master(void);


/*
 * @brief Writes data to an NRF24L01+ register.
 * @param reg_addr: The register address (5 LSBs are used).
 * @param data: Pointer to the data to write.
 * @param size: Number of bytes to write.
 * @retval HAL_StatusTypeDef from SPI transaction.
 */
HAL_StatusTypeDef nrf_write_register(uint8_t reg_addr, uint8_t* data, uint8_t size) {
    uint8_t tx_buf[size + 1];
    uint8_t rx_buf[size + 1]; // To capture status bytes clocked out
    HAL_StatusTypeDef status;

    tx_buf[0] = NRF_CMD_W_REGISTER | (reg_addr & 0x1F); // Command OR'd with register address
    memcpy(tx_buf + 1, data, size);

    HAL_GPIO_WritePin(SPI_SW_CSN_GPIO_Port, SPI_SW_CSN_Pin, GPIO_PIN_RESET);
    status = HAL_SPI_TransmitReceive(&hspi2, tx_buf, rx_buf, size + 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SPI_SW_CSN_GPIO_Port, SPI_SW_CSN_Pin, GPIO_PIN_SET);

    // rx_buf[0] contains the STATUS register value at the time of command byte transmission.
    // You can print or log rx_buf[0] for debugging if needed.
    // printf("NRF_Write: Reg 0x%02X, Status during write: 0x%02X\n", reg_addr, rx_buf[0]);

    if (status != HAL_OK) {
        printf("SPI Error in nrf_write_register: %d for reg 0x%02X\n", (int)status, reg_addr);
    }
    return status;
}

/*
 * @brief Reads multiple bytes from an NRF24L01+ register.
 * @param reg_addr: The register address (5 LSBs are used).
 * @param read_data: Pointer to a buffer to store the read data.
 * @param size: Number of bytes to read.
 * @retval HAL_StatusTypeDef from SPI transaction.
 */
HAL_StatusTypeDef nrf_read_register_multi(uint8_t reg_addr, uint8_t* read_data, uint8_t size) {
    uint8_t cmd = NRF_CMD_R_REGISTER | (reg_addr & 0x1F);
    uint8_t status_byte; // To store the status clocked out during command transmission
    HAL_StatusTypeDef status;

    HAL_GPIO_WritePin(SPI_SW_CSN_GPIO_Port, SPI_SW_CSN_Pin, GPIO_PIN_RESET);
    // Transmit the read command, receive status byte
    status = HAL_SPI_TransmitReceive(&hspi2, &cmd, &status_byte, 1, HAL_MAX_DELAY);
    if (status == HAL_OK) {
        // Receive the register data (clocks out NOPs on MOSI if HAL_SPI_Receive requires it,
        // or send dummy bytes if using HAL_SPI_TransmitReceive for reading)
        // Using HAL_SPI_Receive is cleaner here.
        status = HAL_SPI_Receive(&hspi2, read_data, size, HAL_MAX_DELAY);
    }
    HAL_GPIO_WritePin(SPI_SW_CSN_GPIO_Port, SPI_SW_CSN_Pin, GPIO_PIN_SET);

    // printf("NRF_Read_Multi: Reg 0x%02X, Status during read cmd: 0x%02X\n", reg_addr, status_byte);

    if (status != HAL_OK) {
        printf("SPI Error in nrf_read_register_multi: %d for reg 0x%02X\n", (int)status, reg_addr);
    }
    return status;
}
=======

>>>>>>> c1d5507 (packet_list and sim800L alert)

/*
 * @brief Reads a single byte from an NRF24L01+ register.
 * @param reg_addr: The register address.
 * @retval The byte read from the register. Returns 0xFF on SPI error for simplicity.
 */
uint8_t nrf_read_single_register(uint8_t reg_addr) {
    uint8_t data_byte;
    if (nrf_read_register_multi(reg_addr, &data_byte, 1) == HAL_OK) {
        return data_byte;
    }
    printf("Error reading single register 0x%02X\n", reg_addr);
    return 0xFF; // Indicate error
}

/*
 * @brief Reads the NRF24L01+ STATUS register using NOP command.
 * @retval The STATUS register value. Returns 0xFF on SPI error.
 */
uint8_t nrf_read_status_register(void) {
    uint8_t status_val;
    uint8_t nop_cmd = NRF_CMD_NOP;
    HAL_StatusTypeDef spi_status;
    HAL_GPIO_WritePin(SPI_SW_CSN_GPIO_Port, SPI_SW_CSN_Pin, GPIO_PIN_RESET);
    spi_status = HAL_SPI_TransmitReceive(&hspi2, &nop_cmd, &status_val, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SPI_SW_CSN_GPIO_Port, SPI_SW_CSN_Pin, GPIO_PIN_SET);
    if (spi_status != HAL_OK) {
        printf("SPI Error reading STATUS register: %d\n", (int)spi_status);
        return 0xFF; // Error
    }
    return status_val;
}

/*
 * @brief Clears specified IRQ flags in the STATUS register.
 * @param flags_to_clear: Bitmask of flags (NRF_STATUS_RX_DR, NRF_STATUS_TX_DS, NRF_STATUS_MAX_RT).
 */
void nrf_clear_status_flags(uint8_t flags_to_clear) {
    // To clear a flag, write '1' to its bit position in the STATUS register
    nrf_write_register(NRF_REG_STATUS, &flags_to_clear, 1);
}

/*
 * @brief Reads the RX payload width from the NRF24L01+ using R_RX_PL_WID command.
 * @retval Payload width (1-32). Returns 0 if FIFO is empty or error, or width > 32.
 */
uint8_t nrf_read_rx_payload_width(void) {
    uint8_t cmd = NRF_CMD_R_RX_PL_WID;
    uint8_t width;
    uint8_t status_at_cmd_time; // Status byte returned when sending the command
    HAL_StatusTypeDef spi_status;

    HAL_GPIO_WritePin(SPI_SW_CSN_GPIO_Port, SPI_SW_CSN_Pin, GPIO_PIN_RESET);
    // Send command, get status
    spi_status = HAL_SPI_TransmitReceive(&hspi2, &cmd, &status_at_cmd_time, 1, HAL_MAX_DELAY);
    if (spi_status == HAL_OK) {
        // Get width (send NOP to clock it out, or just receive if SPI driver supports it)
        // Using a NOP to clock out the width byte is a common practice.
        uint8_t nop = NRF_CMD_NOP;
        spi_status = HAL_SPI_TransmitReceive(&hspi2, &nop, &width, 1, HAL_MAX_DELAY);
    }
    HAL_GPIO_WritePin(SPI_SW_CSN_GPIO_Port, SPI_SW_CSN_Pin, GPIO_PIN_SET);

    if (spi_status != HAL_OK) {
        printf("SPI Error in nrf_read_rx_payload_width: %d\n", (int)spi_status);
        return 0; // Error
    }

    // If width > 32, it indicates an error or empty FIFO.
    // The NRF24L01+ datasheet states: "A payload width value > 32 is an error and should be discarded."
    if (width > 32) {
        // This often means the RX FIFO is empty.
        // printf("NRF_Read_RX_PL_WID: Width > 32 (0x%02X), likely empty FIFO. Status was 0x%02X\n", width, status_at_cmd_time);
        return 0;
    }
    return width;
}

/*
 * @brief Reads the received payload from the RX FIFO.
 * @param payload_buffer: Buffer to store the payload.
 * @param width: The width of the payload to read (obtained from nrf_read_rx_payload_width).
 * @retval HAL_StatusTypeDef from SPI transaction.
 */
HAL_StatusTypeDef nrf_read_rx_payload(uint8_t* payload_buffer, uint8_t width) {
    uint8_t cmd = NRF_CMD_R_RX_PAYLOAD;
    uint8_t status_at_cmd_time; // Status byte returned when sending the command
    HAL_StatusTypeDef spi_status;

    if (width == 0 || width > 32) {
        printf("NRF_Read_RX_Payload: Invalid width %d\n", width);
        return HAL_ERROR;
    }

    HAL_GPIO_WritePin(SPI_SW_CSN_GPIO_Port, SPI_SW_CSN_Pin, GPIO_PIN_RESET);
    // Send command, get status
    spi_status = HAL_SPI_TransmitReceive(&hspi2, &cmd, &status_at_cmd_time, 1, HAL_MAX_DELAY);
    if (spi_status == HAL_OK) {
        // Get payload
        // Send dummy bytes if using HAL_SPI_TransmitReceive, or just receive.
        spi_status = HAL_SPI_Receive(&hspi2, payload_buffer, width, HAL_MAX_DELAY);
    }
    HAL_GPIO_WritePin(SPI_SW_CSN_GPIO_Port, SPI_SW_CSN_Pin, GPIO_PIN_SET);

    // printf("NRF_Read_RX_Payload: Status during R_RX_PAYLOAD cmd: 0x%02X\n", status_at_cmd_time);
    if (spi_status != HAL_OK) {
        printf("SPI Error in nrf_read_rx_payload: %d\n", (int)spi_status);
    }
    // After reading the payload, the RX_DR bit in STATUS is cleared if DYNPD is disabled and payload width matches RX_PW_Px.
    // If DYNPD is enabled, RX_DR must be cleared manually.
    return spi_status;
}

/*
 * @brief Flushes the TX FIFO.
 */
void nrf_flush_tx_fifo(void) {
    uint8_t cmd = NRF_CMD_FLUSH_TX;
    uint8_t status_byte;
    HAL_GPIO_WritePin(SPI_SW_CSN_GPIO_Port, SPI_SW_CSN_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi2, &cmd, &status_byte, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SPI_SW_CSN_GPIO_Port, SPI_SW_CSN_Pin, GPIO_PIN_SET);
    // printf("NRF_Flush_TX: Status during flush: 0x%02X\n", status_byte);
}

/*
 * @brief Flushes the RX FIFO.
 */
void nrf_flush_rx_fifo(void) {
    uint8_t cmd = NRF_CMD_FLUSH_RX;
    uint8_t status_byte;
    HAL_GPIO_WritePin(SPI_SW_CSN_GPIO_Port, SPI_SW_CSN_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi2, &cmd, &status_byte, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SPI_SW_CSN_GPIO_Port, SPI_SW_CSN_Pin, GPIO_PIN_SET);
    // printf("NRF_Flush_RX: Status during flush: 0x%02X\n", status_byte);
}


// --- Slave (Transmitter) Functions ---
void init_nrf_slave(void) {
    uint8_t data_val[5]; // Buffer for register values
    uint8_t addr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7}; // Example address, ensure it's consistent

    HAL_GPIO_WritePin(SPI_SW_CE_GPIO_Port, SPI_SW_CE_Pin, GPIO_PIN_RESET); // Start with CE low
    HAL_Delay(100); // Allow NRF to settle after power on (datasheet says 100ms from PWDN to Standby-I)

    printf("--- Initializing NRF Slave (PTX) ---\n");

    // CONFIG: Power Up, 2-byte CRC, PTX mode. Mask RX_DR IRQ, leave TX_DS and MAX_RT IRQs unmasked.
    data_val[0] = NRF_CONFIG_PWR_UP | NRF_CONFIG_EN_CRC | NRF_CONFIG_CRCO | NRF_CONFIG_MASK_RX_DR;
    // Your original was 0x0E, which is (PWR_UP | EN_CRC | CRCO | PRIM_RX=0). This is correct for PTX.
    // Masking RX_DR is fine if slave only transmits.
    nrf_write_register(NRF_REG_CONFIG, data_val, 1);
    HAL_Delay(2); // Allow time for state change (power up takes 1.5ms)

    // Enable Auto-Acknowledgement on Pipe 0 (for receiving ACKs from master)
    data_val[0] = 0x00; nrf_write_register(NRF_REG_EN_AA, data_val, 1);

    // Enable RX Pipe 0 (needed to receive ACKs)
    data_val[0] = 0x01; nrf_write_register(NRF_REG_EN_RXADDR, data_val, 1);

    // Address Width: 5 bytes
    data_val[0] = 0x03; nrf_write_register(NRF_REG_SETUP_AW, data_val, 1);

    // Auto Retransmit Delay: 500us, Auto Retransmit Count: 10 retries
    // Your original 0x0F (250us, 15 retries) is also fine.
    // 0x1A = (0001 << 4) | (1010) = 500us delay, 10 retries.
    data_val[0] = 0x1A; nrf_write_register(NRF_REG_SETUP_RETR, data_val, 1);

    // RF Channel: e.g., Channel 60 (2460 MHz)
    // Your original 0x3C (Channel 60) is good.
    data_val[0] = 0x3C; nrf_write_register(NRF_REG_RF_CH, data_val, 1);

    // RF Setup: Data Rate 1Mbps, Output Power 0dBm
    // Your original 0x27 (250kbps, 0dBm). Ensure master matches.
    // For testing, 1Mbps (0x07) or 2Mbps (0x0F) can be more robust if range is short.
    // Let's stick to 1Mbps (0x07) for now for consistency with master example.
    data_val[0] = 0x07; nrf_write_register(NRF_REG_RF_SETUP, data_val, 1);

    // TX Address (slave transmits on this address)
    nrf_write_register(NRF_REG_TX_ADDR, addr, 5);

    // RX Address Pipe 0 (slave receives ACKs on this address, must match TX_ADDR for ACKs)
    nrf_write_register(NRF_REG_RX_ADDR_P0, addr, 5);

    // Enable Dynamic Payload Length for Pipe 0 (for ACKs if master sends ACK payloads, though usually not needed for basic ACK)
    // And globally enable DPL feature.
    // Your original 0x01 for DYNPD (Pipe0) and 0x04 for FEATURE (EN_DPL)
    data_val[0] = 0x01; nrf_write_register(NRF_REG_DYNPD, data_val, 1);
    data_val[0] = (1<<2); // FEATURE: EN_DPL bit (bit 2)
    // Your original 0x04 for FEATURE is (EN_DPL | EN_ACK_PAY). If you don't use ACK payloads, just EN_DPL is enough.
    // Let's use EN_DPL only: data_val[0] = (1<<2);
    // If you need ACK payloads: data_val[0] = (1<<2) | (1<<1);
    // For simplicity, let's assume no ACK payloads for now.
    nrf_write_register(NRF_REG_FEATURE, data_val, 1);


    // Clear any pending IRQ flags from startup
    nrf_clear_status_flags(NRF_STATUS_RX_DR | NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);
    nrf_flush_tx_fifo(); // Important
    nrf_flush_rx_fifo(); // Important

    HAL_Delay(5); // Allow settings to take effect

    // --- Verification Prints ---
    printf("Slave CONFIG: 0x%02X (Expected PTX: PWR_UP, CRC_EN, CRC_2B, MASK_RX_DR)\n", nrf_read_single_register(NRF_REG_CONFIG));
    printf("Slave RF_SETUP: 0x%02X (Expected 1Mbps, 0dBm: 0x07)\n", nrf_read_single_register(NRF_REG_RF_SETUP));
    printf("Slave RF_CH: 0x%02X (Expected 0x3C)\n", nrf_read_single_register(NRF_REG_RF_CH));
    uint8_t rxtx_addr_check[5];
    nrf_read_register_multi(NRF_REG_TX_ADDR, rxtx_addr_check, 5);
    printf("Slave TX_ADDR: %02X:%02X:%02X:%02X:%02X\n", rxtx_addr_check[0],rxtx_addr_check[1],rxtx_addr_check[2],rxtx_addr_check[3],rxtx_addr_check[4]);
    nrf_read_register_multi(NRF_REG_RX_ADDR_P0, rxtx_addr_check, 5);
    printf("Slave RX_ADDR_P0: %02X:%02X:%02X:%02X:%02X\n", rxtx_addr_check[0],rxtx_addr_check[1],rxtx_addr_check[2],rxtx_addr_check[3],rxtx_addr_check[4]);
    printf("Slave STATUS: 0x%02X\n", nrf_read_status_register());
    printf("Slave FIFO_STATUS: 0x%02X\n", nrf_read_single_register(NRF_REG_FIFO_STATUS));
    printf("--- NRF Slave Initialized ---\n");
}

void transmit_nrf_payload(uint8_t size, char* data) {
    if (size == 0 || size > 32) {
        printf("Transmit_NRF: Invalid payload size %d\n", size);
        return;
    }

    // Optional: Check if NRF is in PTX mode and powered up
    // uint8_t config_reg = nrf_read_single_register(NRF_REG_CONFIG);
    // if (!(config_reg & NRF_CONFIG_PWR_UP) || (config_reg & NRF_CONFIG_PRIM_RX)) {
    //    printf("Transmit_NRF: Not in PTX mode or not powered up! CONFIG: 0x%02X\n", config_reg);
    //    // Potentially re-initialize or set to PTX mode here
    //    return;
    // }

    // Ensure CE is low before writing payload
    HAL_GPIO_WritePin(SPI_SW_CE_GPIO_Port, SPI_SW_CE_Pin, GPIO_PIN_RESET);

    // Write payload to TX FIFO using W_TX_PAYLOAD command
    uint8_t tx_payload_cmd = 0xB0; // W_TX_PAYLOAD_NOACK;
    uint8_t status_byte_during_cmd;

    HAL_GPIO_WritePin(SPI_SW_CSN_GPIO_Port, SPI_SW_CSN_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi2, &tx_payload_cmd, &status_byte_during_cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi2, (uint8_t*)data, size, HAL_MAX_DELAY); // Send payload data
    HAL_GPIO_WritePin(SPI_SW_CSN_GPIO_Port, SPI_SW_CSN_Pin, GPIO_PIN_SET);
    // printf("Transmit_NRF: Status during W_TX_PAYLOAD cmd: 0x%02X\n", status_byte_during_cmd);

    // Pulse CE to trigger transmission
    HAL_GPIO_WritePin(SPI_SW_CE_GPIO_Port, SPI_SW_CE_Pin, GPIO_PIN_SET);
    // Keep CE high for at least 10us for transmission to start.
    // HAL_Delay(1) is more than enough. For faster systems, a microsecond delay is better.
    // For STM32, DWT_Delay_us can be used if initialized.
    volatile uint32_t delay_count = 1000; // Simple busy loop for short delay, adjust as needed
    while(delay_count--); // This is a rough delay, use proper us delay if available
    // HAL_Delay(1); // Using HAL_Delay for simplicity, though it's ms resolution
    HAL_GPIO_WritePin(SPI_SW_CE_GPIO_Port, SPI_SW_CE_Pin, GPIO_PIN_RESET);

    // After pulsing CE, the NRF attempts transmission.
    // The IRQ pin will go low if TX_DS or MAX_RT occurs (if unmasked in CONFIG).
    // Or, you can poll the STATUS register.
    // It's good practice to wait for TX_DS or MAX_RT, or implement a timeout.
}

// --- Master (Receiver) Functions ---
void init_nrf_master(void) {
    uint8_t data_val[5];
    uint8_t addr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7}; // Must match slave's TX_ADDR and RX_ADDR_P0 for ACKs

    HAL_GPIO_WritePin(SPI_SW_CE_GPIO_Port, SPI_SW_CE_Pin, GPIO_PIN_RESET); // Start with CE low
    HAL_Delay(100); // Allow NRF to settle

    printf("--- Initializing NRF Master (PRX) ---\n");

    // CONFIG: Power Up, 2-byte CRC, PRX mode. Unmask RX_DR IRQ, mask TX_DS and MAX_RT.
    data_val[0] = NRF_CONFIG_PWR_UP | NRF_CONFIG_EN_CRC | NRF_CONFIG_CRCO | NRF_CONFIG_PRIM_RX |
                  NRF_CONFIG_MASK_TX_DS | NRF_CONFIG_MASK_MAX_RT;
    // Your original 0x0F is (PWR_UP | EN_CRC | CRCO | PRIM_RX). This is correct for PRX.
    // The masking part is important for IRQ behavior.
    nrf_write_register(NRF_REG_CONFIG, data_val, 1);
    HAL_Delay(2); // Allow time for state change

    // Enable Auto-Acknowledgement on Pipe 0 (master will send ACKs automatically)
    data_val[0] = 0x00; nrf_write_register(NRF_REG_EN_AA, data_val, 1);

    // Enable RX Pipe 0
    data_val[0] = 0x01; nrf_write_register(NRF_REG_EN_RXADDR, data_val, 1);

    // Address Width: 5 bytes
    data_val[0] = 0x03; nrf_write_register(NRF_REG_SETUP_AW, data_val, 1);

    // RF Channel: (Must match slave)
    data_val[0] = 0x01; nrf_write_register(NRF_REG_RF_CH, data_val, 1);

    // RF Setup: Data Rate 1Mbps, Output Power 0dBm (Must match slave)
    data_val[0] = 0x07; nrf_write_register(NRF_REG_RF_SETUP, data_val, 1);

    // RX Address for Pipe 0 (master listens on this address)
    nrf_write_register(NRF_REG_RX_ADDR_P0, addr, 5);

    // If master also needs to transmit data (not just ACKs), set TX_ADDR.
    // For basic ACK, P0 address is used automatically by NRF.
    // nrf_write_register(NRF_REG_TX_ADDR, master_tx_addr_if_needed, 5);

    // Enable Dynamic Payload Length for Pipe 0 and globally enable DPL feature.
    data_val[0] = 0x01; nrf_write_register(NRF_REG_DYNPD, data_val, 1);
    data_val[0] = (1<<2); // FEATURE: EN_DPL bit
    nrf_write_register(NRF_REG_FEATURE, data_val, 1);

    // Static payload length for Pipe 0 (if not using DPL, or as a fallback if DPL read fails)
    // This is often set even if DPL is used, but DPL overrides it.
    // data_val[0] = 32; // Max payload size, or your expected fixed size
    // nrf_write_register(NRF_REG_RX_PW_P0, data_val, 1);

    // Clear any pending IRQ flags
    nrf_clear_status_flags(NRF_STATUS_RX_DR | NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);
    nrf_flush_tx_fifo();
    nrf_flush_rx_fifo();

    HAL_Delay(5); // Allow settings to take effect

    // --- Verification Prints ---
    printf("Master CONFIG: 0x%02X (Expected PRX: PWR_UP, CRC_EN, CRC_2B, PRIM_RX, MASK_TX_DS, MASK_MAX_RT)\n", nrf_read_single_register(NRF_REG_CONFIG));
    printf("Master RF_SETUP: 0x%02X (Expected 1Mbps, 0dBm: 0x07)\n", nrf_read_single_register(NRF_REG_RF_SETUP));
    printf("Master RF_CH: 0x%02X (Expected 0x3C)\n", nrf_read_single_register(NRF_REG_RF_CH));
    uint8_t rx_p0_addr_check[5];
    nrf_read_register_multi(NRF_REG_RX_ADDR_P0, rx_p0_addr_check, 5);
    printf("Master RX_ADDR_P0: %02X:%02X:%02X:%02X:%02X\n", rx_p0_addr_check[0],rx_p0_addr_check[1],rx_p0_addr_check[2],rx_p0_addr_check[3],rx_p0_addr_check[4]);
    printf("Master STATUS: 0x%02X\n", nrf_read_status_register());
    printf("Master FIFO_STATUS: 0x%02X\n", nrf_read_single_register(NRF_REG_FIFO_STATUS));
    printf("--- NRF Master Initialized ---\n");

    // IMPORTANT: After initialization, put master in RX mode by setting CE HIGH
    HAL_GPIO_WritePin(SPI_SW_CE_GPIO_Port, SPI_SW_CE_Pin, GPIO_PIN_SET);
    printf("Master CE is HIGH, listening for packets.\n");
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */


  //INIT WRITE next 4 lines
//  HAL_Delay(1000);
//  char data_in[65] = "************************************************1235****120*****";
//  flash_write_erase_sector7();
//  flash_write_multiple_word(0x08060000, (uint32_t *)data_in, 16);

  //READOUT
  {
	char data_out[64];
	flash_read_multiple_words(0x08060000, (uint32_t *)data_out, 16);
	//PARSING TO VARIABLES
	state_machine_init(data_out, &huart1);
  }
  lcd_init(hi2c1);
  char x;
<<<<<<< HEAD
<<<<<<< HEAD
  init_nrf_master(); // CE will be set high at the end of this function

  // OPTION 1: POLLING METHOD (Recommended for initial debugging)
  printf("Master: Starting in POLLING mode.\n");
  uint8_t received_payload[33]; // Buffer for payload + null terminator
  uint8_t payload_width;

=======
//  init_nrf_master();
//  HAL_GPIO_WritePin(SPI_SW_CE_GPIO_Port, SPI_SW_CE_Pin, GPIO_PIN_SET);
//  while(HAL_GPIO_ReadPin(SPI_IRQ_GPIO_Port, SPI_IRQ_Pin));
//  HAL_GPIO_WritePin(SPI_SW_CE_GPIO_Port, SPI_SW_CE_Pin, GPIO_PIN_RESET);
>>>>>>> 0207771 (switch changed to function array)

=======
  printf("Hello\n");
>>>>>>> c1d5507 (packet_list and sim800L alert)

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // Check IRQ Pin (Active LOW) - This simulates an EXTI interrupt check
      // if (HAL_GPIO_ReadPin(SPI_IRQ_GPIO_Port, SPI_IRQ_Pin) == GPIO_PIN_RESET) {
      //    printf("Master: IRQ Pin is LOW!\n");
      // }
	  static uint32_t last_master_print = 0;
	  if (HAL_GetTick() - last_master_print > 250) { // Print every 250ms
	      last_master_print = HAL_GetTick();
	      uint8_t mst_status = nrf_read_status_register();
	      uint8_t mst_fifo = nrf_read_single_register(NRF_REG_FIFO_STATUS);
	      printf("Master Poll: STATUS=0x%02X, FIFO=0x%02X\n", mst_status, mst_fifo);
	  }

      uint8_t status = nrf_read_status_register();

      if (status & NRF_STATUS_RX_DR) { // Check RX_DR flag in STATUS register
          printf("Master: RX_DR detected! STATUS: 0x%02X\n", status);

          // Optional: Temporarily lower CE while processing. Not strictly necessary for simple RX.
          // HAL_GPIO_WritePin(SPI_SW_CE_GPIO_Port, SPI_SW_CE_Pin, GPIO_PIN_RESET);

          payload_width = nrf_read_rx_payload_width();
          if (payload_width > 0 && payload_width <= 32) {
              if (nrf_read_rx_payload(received_payload, payload_width) == HAL_OK) {
                  received_payload[payload_width] = '\0'; // Null-terminate for printf
                  printf("Master: Payload Received (width %d): \"%s\"\n", payload_width, (char*)received_payload);
              } else {
                  printf("Master: Error reading payload after width indicated data.\n");
              }
          } else if (payload_width == 0) {
               // This can happen if R_RX_PL_WID is called when FIFO is empty, even if RX_DR was set.
               // Or if there was an error reading width.
               printf("Master: RX_DR set, but payload width is 0. Flushing RX FIFO.\n");
               nrf_flush_rx_fifo(); // Good practice to flush if width is unexpected
          } else { // payload_width > 32 (error)
               printf("Master: RX_DR set, but payload width invalid (%d). Flushing RX FIFO.\n", payload_width);
               nrf_flush_rx_fifo();
          }

          // CRITICAL: Clear the RX_DR flag in the STATUS register.
          // Otherwise, IRQ pin might stay asserted (if using actual IRQ) or this flag will persist.
          nrf_clear_status_flags(NRF_STATUS_RX_DR);

          // Optional: Ensure CE is high again if it was lowered.
          // HAL_GPIO_WritePin(SPI_SW_CE_GPIO_Port, SPI_SW_CE_Pin, GPIO_PIN_SET);
          printf("Master: RX_DR cleared. Listening...\n");

      } else {
          // No RX_DR. You can print status periodically for debugging if nothing is received.
          // static uint32_t last_poll_time = 0;
          // if (HAL_GetTick() - last_poll_time > 1000) { // Print status every second
          //    last_poll_time = HAL_GetTick();
          //    printf("Master: Polling... STATUS: 0x%02X, FIFO_STATUS: 0x%02X, IRQ_PIN: %d\n",
          //           status, nrf_read_single_register(NRF_REG_FIFO_STATUS),
          //           HAL_GPIO_ReadPin(SPI_IRQ_GPIO_Port, SPI_IRQ_Pin));
          // }
      }

      // Your original `while(HAL_GPIO_ReadPin(SPI_IRQ_GPIO_Port, SPI_IRQ_Pin));`
      // This is for edge-triggered IRQ simulation. The polling loop above is more for level-triggered.
      // If using the IRQ pin directly:
      // if (HAL_GPIO_ReadPin(SPI_IRQ_GPIO_Port, SPI_IRQ_Pin) == GPIO_PIN_RESET) {
      //     // IRQ is active, process it (as done in the RX_DR block above)
      //     // Then clear status flags, etc.
      // }


      // The rest of your master application logic can go here.
      // For example, check_keyboard() and state_machine_run(x);
      // uint8_t x = check_keyboard();
      // if(x){
      //    if(x < 10) x += '0';
      //    else if(x == 10) x = '*';
      //    else if(x == 11) x = '0';
      //    else if(x == 12) x = '#';
      // }
      // state_machine_run(x);

      HAL_Delay(100); // Polling interval for STATUS register. Adjust as needed.





//	  x = check_keyboard();
//	  if(x){
//		  if(x < 10) x += '0';
//		  else if(x == 10) x = '*';
//		  else if(x == 11) x = '0';
//		  else if(x == 12) x = '#';
//	  }
//	  state_machine_run(x);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 256;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Keyboard_col3_Pin|Keyboard_col1_Pin|Keyboard_col2_Pin|SPI_SW_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_SW_CE_GPIO_Port, SPI_SW_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Keyboard_row2_Pin Keyboard_row3_Pin Keyboard_row4_Pin Keyboard_row1_Pin
                           Alarm_Signal_Pin */
  GPIO_InitStruct.Pin = Keyboard_row2_Pin|Keyboard_row3_Pin|Keyboard_row4_Pin|Keyboard_row1_Pin
                          |Alarm_Signal_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Keyboard_col3_Pin Keyboard_col1_Pin Keyboard_col2_Pin SPI_SW_CSN_Pin */
  GPIO_InitStruct.Pin = Keyboard_col3_Pin|Keyboard_col1_Pin|Keyboard_col2_Pin|SPI_SW_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_SW_CE_Pin */
  GPIO_InitStruct.Pin = SPI_SW_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_SW_CE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_IRQ_Pin */
  GPIO_InitStruct.Pin = SPI_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPI_IRQ_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
  (void)file;
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
