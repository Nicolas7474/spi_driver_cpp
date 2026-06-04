#include "stdint.h"

#ifndef INC_W25QXX_H_
#define INC_W25QXX_H_

// 0x02 Page program (the Page Program instruction allows from 1 to 256 bytes (a page) of data to be programmed at previously erased (FFh) memory locations)
// 0x03 Enable Read
// 0x04 Disable write
// 0x05 Read Status Register
// 0x06 Enable write (5ms)
// 0x12 Page program with 4-Byte Address
// 0x13 Read Data with 4-Byte Address
// 0x20 Erase sector
// 0x21 Erase Sector with 32bit address
// 0x52 32Ko BLock Erase
// 0xD8 64Ko Block Erase
// 0x7C Chip Erase
// 0x0B Enable Fast Read
// 0x0C Fast Read with 4-Byte Address
// 0xB9 Power-down

// States
typedef enum {
    W25Q_READY,
	W25Q_BUSY,
	W25Q_ERROR,
	W25Q_TIMEOUT,
} W25Q_Status;


W25Q_Status W25Q_Reset (void);
uint32_t W25Q_ReadID (void);

uint8_t W25Q_Read_Status_Reg(void);

W25Q_Status W25Q_Read (uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData);
W25Q_Status W25Q_FastRead (uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData);

W25Q_Status W25Q_Erase_Sector (uint16_t numsector);

W25Q_Status W25Q_Write_Clean(uint32_t page, uint16_t offset, uint32_t size, uint8_t *data);
W25Q_Status W25Q_Write (uint32_t page, uint16_t offset, uint32_t size, uint8_t *data);

W25Q_Status W25Q_Write_Byte (uint32_t Addr, uint8_t data);
uint8_t W25Q_Read_Byte (uint32_t Addr);

float W25Q_Read_NUM (uint32_t page, uint16_t offset);
W25Q_Status W25Q_Write_NUM (uint32_t page, uint16_t offset, float data);

W25Q_Status W25Q_Read_32B (uint32_t page, uint16_t offset, uint32_t size, uint32_t *data);
W25Q_Status W25Q_Write_32B (uint32_t page, uint16_t offset, uint32_t size, uint32_t *data);

void write_enable (void);
void write_disable(void);

void floatToBytes(uint8_t * ftoa_bytes_temp, float float_variable);
float BytesTofloat(uint8_t * ftoa_bytes_temp);
uint32_t bytestoWrite (uint32_t size, uint16_t offset);
uint32_t bytestoModify (uint32_t size, uint16_t offset);




#endif /* INC_W25QXX_H_ */
