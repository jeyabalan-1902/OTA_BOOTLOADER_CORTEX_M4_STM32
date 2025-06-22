/*
 * bootloader.h
 *
 *  Created on: May 21, 2025
 *      Author: kjeyabalan
 */

#ifndef INC_BOOTLOADER_H_
#define INC_BOOTLOADER_H_

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdint.h>

#include "main.h"


#define BL_DEBUG_MSG_EN

#define D_UART    &huart2
#define BL_RX_LEN  200

#define UART_TIMEOUT_MS        3000
#define TOTAL_TIMEOUT_MS       3000
#define POLL_INTERVAL_MS       10
#define BOOT_CMD               0x50

#define FLASH_SECTOR2_BASE_ADDRESS 0x08008000U      //Application base address

//version 1.0
#define BL_VERSION              0x10
#define BL_GET_VER				0x51
#define BL_GET_HELP				0x52
#define BL_GET_CID				0x53
#define BL_GET_RDP_STATUS		0x54
#define BL_GO_TO_ADDR			0x55
#define BL_FLASH_ERASE          0x56
#define BL_MEM_WRITE			0x57
#define BL_EN_RW_PROTECT		0x58
#define BL_MEM_READ				0x59
#define BL_READ_SECTOR_P_STATUS	0x5A

#define BL_OTP_READ				0x5B
#define BL_DIS_R_W_PROTECT		0x5C

/* ACK and NACK bytes*/
#define BL_ACK   0XA5
#define BL_NACK  0X7F

/*CRC*/
#define VERIFY_CRC_FAIL    1
#define VERIFY_CRC_SUCCESS 0

#define ADDR_VALID     0x00
#define ADDR_INVALID   0x01

#define INVALID_SECTOR 0x04

/*Memories of STM32F446RET6 MCU */
#define SRAM1_SIZE            112*1024     // STM32F446RE has 112KB of SRAM1
#define SRAM1_END             (SRAM1_BASE + SRAM1_SIZE)
#define SRAM2_SIZE            16*1024     // STM32F446RE has 16KB of SRAM2
#define SRAM2_END             (SRAM2_BASE + SRAM2_SIZE)
#define FLASH_SIZE             512*1024     // STM32F446RE has 512KB of SRAM2
#define BKPSRAM_SIZE           4*1024     // STM32F446RE has 4KB of SRAM2
#define BKPSRAM_END            (BKPSRAM_BASE + BKPSRAM_SIZE)


extern uint8_t bl_rx_buffer[BL_RX_LEN];
extern UART_HandleTypeDef *C_UART;

extern CRC_HandleTypeDef hcrc;

extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

void printmsg(char *format,...);

void  bootloader_uart_read_data(void);
void bootloader_jump_to_user_app(void);

void bootloader_handle_getver_cmd(uint8_t *bl_rx_buffer);
void bootloader_handle_gethelp_cmd(uint8_t *pBuffer);
void bootloader_handle_getcid_cmd(uint8_t *pBuffer);
void bootloader_handle_getrdp_cmd(uint8_t *pBuffer);
void bootloader_handle_go_cmd(uint8_t *pBuffer);
void bootloader_handle_flash_erase_cmd(uint8_t *pBuffer);
void bootloader_handle_mem_write_cmd(uint8_t *pBuffer);
void bootloader_handle_en_rw_protect(uint8_t *pBuffer);
void bootloader_handle_mem_read (uint8_t *pBuffer);
void bootloader_handle_read_sector_protection_status(uint8_t *pBuffer);
void bootloader_handle_read_otp(uint8_t *pBuffer);
void bootloader_handle_dis_rw_protect(uint8_t *pBuffer);

void bootloader_send_ack(uint8_t command_code, uint8_t follow_len);
void bootloader_send_nack(void);

uint8_t bootloader_verify_crc (uint8_t *pData, uint32_t len,uint32_t crc_host);
uint8_t get_bootloader_version(void);
void bootloader_uart_write_data(uint8_t *pBuffer,uint32_t len);

uint16_t get_mcu_chip_id(void);
uint8_t get_flash_rdp_level(void);
uint8_t verify_address(uint32_t go_address);
uint8_t execute_flash_erase(uint8_t sector_number , uint8_t number_of_sector);
uint8_t execute_mem_write(uint8_t *pBuffer, uint32_t mem_address, uint32_t len);

uint8_t configure_flash_sector_rw_protection(uint8_t sector_details, uint8_t protection_mode, uint8_t disable);

uint16_t read_OB_rw_protection_status(void);


#endif /* INC_BOOTLOADER_H_ */
