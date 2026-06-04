// ******* 	Winbond W25Qxx EEPROM - Bare-Metal Driver - DMA  ***** //
// ******* 			Must be combined with spi1.c  	 	  ***** //


#include <spi1.hpp>
#include <stdint.h>
#include <string.h>
#include "Flash_W25Q.h"
#include "timers.h"

// Port handler to define and to add among the functions parameters ?
//#define W25Q_SPI hspi1  //extern SPI_HandleTypeDef hspi1;

#define numBLOCK 128  // number of total blocks - 128 for 64M-bit flash

uint8_t tempBytes[4];
W25Q_Status State_W25Q = W25Q_READY; // global variable of SPI state

void write_enable (void)
{
	uint8_t tData = 0x06;  // enable write
	csLOW1();
	SPI1_Transmit_DMA(&tData, 1);
	csHIGH1();
	while(GetState_SPI1() != SPI_READY); //NBdelay_ms(5); // delay not necessary
}

void write_disable(void)
{
	uint8_t tData = 0x04;  // disable write
	csLOW1();
	SPI1_Transmit_DMA(&tData, 1);
	csHIGH1();
	while(GetState_SPI1() != SPI_READY); // NBdelay_ms(5); // delay not necessary
}

W25Q_Status W25Q_Reset (void)
{
	uint8_t tData[2];
	tData[0] = 0x66;  // enable Reset
	tData[1] = 0x99;  // Reset
	csLOW1();
	SPI1_Transmit_DMA(tData, 2);
	csHIGH1();
	NBdelay_ms(100);

	return State_W25Q = W25Q_READY;
}

uint32_t W25Q_ReadID (void)
{
	uint8_t tData = 0x9F;  // Read JEDEC ID
	uint8_t rData[3];
	uint32_t timeout_counter = GetSysTick();

	csLOW1();
	SPI1_Transmit_DMA(&tData, 1);
	while(GetState_SPI1() != SPI_READY) {if (GetSysTick() - timeout_counter > 10) return W25Q_TIMEOUT;} // wait long as State_SPI1 == SPI_BUSY_TX
	SPI1_Receive_DMA(rData, 3);
	while(GetState_SPI1() != SPI_READY) {if (GetSysTick() - timeout_counter > 100) return W25Q_TIMEOUT;} // as long as State_SPI1 == SPI_BUSY_RX
	csHIGH1();

	return ((rData[0]<<16)|(rData[1]<<8)|rData[2]);
}

uint8_t W25Q_Read_Status_Reg(void)
{
	uint8_t tData = 0x05;  // Status Register -1
	uint8_t rData;
	uint32_t timeout_counter = GetSysTick();
	csLOW1();
	SPI1_Transmit_DMA(&tData, 1);
	while(GetState_SPI1() != SPI_READY) {if (GetSysTick() - timeout_counter > 10) return W25Q_TIMEOUT;}
	SPI1_Receive_DMA(&rData, 1);
	while(GetState_SPI1() != SPI_READY) {if (GetSysTick() - timeout_counter > 10) return W25Q_TIMEOUT;}
	csHIGH1();

	return rData;
}


W25Q_Status W25Q_Read (uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData)
{
	uint8_t tData[5];
	uint32_t memAddr = (startPage*256) + offset;

	if (numBLOCK<512)   // Chip Size<256Mb
	{
		tData[0] = 0x03;  // enable Read
		tData[1] = (memAddr>>16)&0xFF;  // MSB of the memory Address
		tData[2] = (memAddr>>8)&0xFF;
		tData[3] = (memAddr)&0xFF; // LSB of the memory Address
	}
	else  // we use 32bit memory address for chips >= 256Mb
	{
		tData[0] = 0x13;  // Read Data with 4-Byte Address
		tData[1] = (memAddr>>24)&0xFF;  // MSB of the memory Address
		tData[2] = (memAddr>>16)&0xFF;
		tData[3] = (memAddr>>8)&0xFF;
		tData[4] = (memAddr)&0xFF; // LSB of the memory Address
	}

	csLOW1();  // pull the CS Low
	if (numBLOCK<512)
	{
		SPI1_Transmit_DMA(tData, 4);  // send read instruction along with the 24 bit memory address
	}
	else
	{
		SPI1_Transmit_DMA(tData, 5);  // send read instruction along with the 32 bit memory address
	}
	uint32_t timeout_counter = GetSysTick();

	while(GetState_SPI1() != SPI_READY) {if (GetSysTick() - timeout_counter > 100) return W25Q_TIMEOUT;}
	SPI1_Receive_DMA(rData, size);  // Read the data
	while(GetState_SPI1() != SPI_READY) {if (GetSysTick() - timeout_counter > 3000) return W25Q_TIMEOUT;}
	csHIGH1();

	return State_W25Q = W25Q_READY;
}


W25Q_Status W25Q_FastRead (uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData)
{
	uint8_t tData[6];
	uint32_t memAddr = (startPage*256) + offset;

	if (numBLOCK<512)   // Chip Size<256Mb
	{
		tData[0] = 0x0B;  // enable Fast Read
		tData[1] = (memAddr>>16)&0xFF;  // MSB of the memory Address
		tData[2] = (memAddr>>8)&0xFF;
		tData[3] = (memAddr)&0xFF; // LSB of the memory Address
		tData[4] = 0;  // Dummy clock for Fast Read
	}
	else  // we use 32bit memory address for chips >= 256Mb
	{
		tData[0] = 0x0C;  // Fast Read with 4-Byte Address
		tData[1] = (memAddr>>24)&0xFF;  // MSB of the memory Address
		tData[2] = (memAddr>>16)&0xFF;
		tData[3] = (memAddr>>8)&0xFF;
		tData[4] = (memAddr)&0xFF; // LSB of the memory Address
		tData[5] = 0;  // Adding 8x Dummy clock for Fast Read, allowing more time to setting up the initial address (operate at the highest possible freq. of Fr)
	}

	csLOW1();  // pull the CS Low
	if (numBLOCK<512)
	{
		SPI1_Transmit_DMA(tData, 5);  // send read instruction along with the 24 bit memory address
	}
	else
	{
		SPI1_Transmit_DMA(tData, 6);  // send read instruction along with the 32 bit memory address
	}
	uint32_t timeout_counter = GetSysTick();

	while(GetState_SPI1() != SPI_READY) {if (GetSysTick() - timeout_counter > 100) return W25Q_TIMEOUT;}
	SPI1_Receive_DMA(rData, size);  // Read the data
	while(GetState_SPI1() != SPI_READY) {if (GetSysTick() - timeout_counter > 3000) return W25Q_TIMEOUT;}
	csHIGH1();  // pull the CS High

	return State_W25Q = W25Q_READY;
}


W25Q_Status W25Q_Write (uint32_t page, uint16_t offset, uint32_t size, uint8_t *data)
{
	uint16_t startSector  = page/16; // a sector = 16 pages (cannot erase less than a sector)
	uint16_t endSector  = (page + ((size+offset-1)/256))/16; // add all the bytes to be written plus their offset to find the end sector
	uint16_t numSectors = endSector-startSector+1;

	uint8_t previousData[4096]; // buffer holding the previous datas of the sector
	uint32_t sectorOffset = ((page%16)*256)+offset; // get the offset in bytes (which page*256 + bytes offset) of the datas in the startSector
	uint32_t dataindx = 0; // keep track of the increment the pointer in the user-provided data array

	uint32_t timeout_counter = GetSysTick();

	for (uint16_t i=0; i < numSectors; i++) 	// Loop through each sector	{
	{
		uint32_t startPage = startSector*16; // first page of the first sector
		W25Q_FastRead(startPage, 0, 4096, previousData); // function * FAST READ * (0x0B) -> load the previous data of the current sector

		uint16_t bytesRemaining = bytestoModify(size, sectorOffset); // bytes remaining to write in the current sector// bytestoModify = if((size+offset)<4096) return size; else return 4096-offset;
		for (uint16_t i=0; i<bytesRemaining; i++)
		{
			previousData[i+sectorOffset] = data[i+dataindx]; // modifying the sector with the new user-provided data
		}
		while(GetState_SPI1() != SPI_READY) {if (GetSysTick() - timeout_counter > 1000) return W25Q_TIMEOUT;}
		W25Q_Write_Clean(startPage, 0, 4096, previousData); // to function -> * WRITE_CLEAN *

		startSector++;
		sectorOffset = 0; // on the next loop iteration we start from the beginning of the sector
		dataindx = dataindx+bytesRemaining;
		size = size-bytesRemaining;
	}
	return State_W25Q = W25Q_READY;
}


W25Q_Status W25Q_Write_Clean (uint32_t page, uint16_t offset, uint32_t size, uint8_t *data) // data has 4096 bytes
{
	uint8_t tData[261];
	uint32_t startPage = page;
	uint32_t endPage  = startPage + ((size + offset - 1) / 256);
	uint32_t numPages = endPage - startPage + 1;

	uint16_t startSector  = startPage / 16;
	uint16_t endSector  = endPage / 16;
	uint16_t numSectors = endSector - startSector + 1;

	for (uint16_t i=0; i < numSectors; i++)
	{
		W25Q_Erase_Sector(startSector++); /* function * ERASE SECTOR * (0x20) Erase time (with timeout delay) already included in function */
	}

	uint32_t dataPosition = 0;

	// write the data
	for (uint32_t i=0; i < numPages; i++) 	// LOOP THROUGH EACH PAGE
	{
		uint32_t memAddr = (startPage * 256) + offset;
		uint16_t bytesremaining = bytestoWrite(size, offset);
		uint32_t indx = 0;

		write_enable();

		if (numBLOCK<512) 	 // Chip Size < 256Mb
		{
			tData[0] = 0x02; // The Page Program instruction allows from 1 to 256 bytes (a page) of data to be programmed at previously erased (FFh) memory locations
			tData[1] = (memAddr>>16)&0xFF;  // MSB of the memory Address
			tData[2] = (memAddr>>8)&0xFF;
			tData[3] = (memAddr)&0xFF; // LSB of the memory Address
			indx = 4;
		}
		else // we use 32bit memory address for chips >= 256Mb
		{
			tData[0] = 0x12;  // page program with 4-Byte Address
			tData[1] = (memAddr>>24)&0xFF;  // MSB of the memory Address
			tData[2] = (memAddr>>16)&0xFF;
			tData[3] = (memAddr>>8)&0xFF;
			tData[4] = (memAddr)&0xFF; // LSB of the memory Address
			indx = 5;
		}
		uint16_t bytestosend  = bytesremaining + indx;

		for (uint16_t k=0; k < bytesremaining; k++)
		{
			tData[indx++] = data[k + dataPosition];
		}

		csLOW1();
		SPI1_Transmit_DMA(tData, bytestosend); // * WRITE DATA Page Program (0x02) *
		while(GetState_SPI1() != SPI_READY);
		csHIGH1();
		write_disable();
		while(W25Q_Read_Status_Reg() && 0x01) {
			Delay_us_TIM7(500);	// Polling Status Register to get end of Page Program Write, Delay max = 3ms, Typ. = 0.7ms
		}

		startPage++;
		offset = 0; // offset is always 0 even at the 1st iteration (for the 1st page), since we write the complete page after erase
		size = size - bytesremaining;
		dataPosition = dataPosition + bytesremaining;

	}
	return State_W25Q = W25Q_READY;
}


uint8_t W25Q_Read_Byte (uint32_t Addr)
{
	uint32_t timeout_counter = GetSysTick();
	uint8_t tData[5];
	uint8_t rData;

	if (numBLOCK<512)   // with Chip Size < 256Mb
	{
		tData[0] = 0x03;  // enable Read
		tData[1] = (Addr>>16)&0xFF;  // MSB of the memory Address
		tData[2] = (Addr>>8)&0xFF;
		tData[3] = (Addr)&0xFF; // LSB of the memory Address
	}
	else  // with 32bit memory address for chips >= 256Mb
	{
		tData[0] = 0x13;  // Read Data with 4-Byte Address
		tData[1] = (Addr>>24)&0xFF;  // MSB of the memory Address
		tData[2] = (Addr>>16)&0xFF;
		tData[3] = (Addr>>8)&0xFF;
		tData[4] = (Addr)&0xFF; // LSB of the memory Address
	}

	csLOW1();  // pull the CS Low
	if (numBLOCK<512)
	{
		SPI1_Transmit_DMA(tData, 4);  // send read instruction along with the 24 bit memory address
	}
	else
	{
		SPI1_Transmit_DMA(tData, 5);  // send read instruction along with the 32 bit memory address
	}
	while(GetState_SPI1() != SPI_READY) {if (GetSysTick() - timeout_counter > 300) return W25Q_TIMEOUT;}
	SPI1_Receive_DMA(&rData, 1);  // Read the data
	while(GetState_SPI1() != SPI_READY) {if (GetSysTick() - timeout_counter > 500) return W25Q_TIMEOUT;}
	csHIGH1();  // pull the CS High

	return rData;
}

W25Q_Status W25Q_Write_Byte (uint32_t Addr, uint8_t data)
{
	uint32_t timeout_counter = GetSysTick();
	uint8_t tData[6];
	uint8_t indx;

	if (numBLOCK < 512)   // Chip Size < 256Mb
	{
		tData[0] = 0x02;  // page program
		tData[1] = (Addr>>16)&0xFF;  // MSB of the memory Address
		tData[2] = (Addr>>8)&0xFF;
		tData[3] = (Addr)&0xFF; // LSB of the memory Address
		tData[4] = data;
		indx = 5;
	}
	else  // we use 32bit memory address for chips >= 256Mb
	{
		tData[0] = 0x12;  // Write Data with 4-Byte Address
		tData[1] = (Addr>>24)&0xFF;  // MSB of the memory Address
		tData[2] = (Addr>>16)&0xFF;
		tData[3] = (Addr>>8)&0xFF;
		tData[4] = (Addr)&0xFF; // LSB of the memory Address
		tData[5] = data;
		indx = 6;
	}

	if (W25Q_Read_Byte(Addr) == 0xFF)
	{
		while(GetState_SPI1() != SPI_READY);
		write_enable();
		csLOW1();
		SPI1_Transmit_DMA(tData, indx);
		while(GetState_SPI1() != SPI_READY) {if (GetSysTick() - timeout_counter > 500) return W25Q_TIMEOUT;}
		csHIGH1();
		write_disable();
	}
	return State_W25Q = W25Q_READY;
}


W25Q_Status W25Q_Write_NUM (uint32_t page, uint16_t offset, float data)
{
	uint32_t timeout_counter = GetSysTick();
	floatToBytes(tempBytes, data);

	/* write using single byte function */
	uint32_t Addr = (page * 256) + offset;
	for (int i=0; i < 4; i++)
	{
		W25Q_Write_Byte(i + Addr, tempBytes[i]);
		while(GetState_SPI1() != SPI_READY) {if (GetSysTick() - timeout_counter > 300) return W25Q_TIMEOUT;}
	}

	/* Write using sector update function */
	W25Q_Write(page, offset, 4, tempBytes);

	return State_W25Q = W25Q_READY;
}

float W25Q_Read_NUM (uint32_t page, uint16_t offset)
{
	uint8_t rData[4];
	W25Q_Read(page, offset, 4, rData);

	return (BytesTofloat(rData));
}

W25Q_Status W25Q_Write_32B (uint32_t page, uint16_t offset, uint32_t size, uint32_t *data)
{
	uint8_t data8[size*4];
	uint32_t indx = 0;

	for (uint32_t i=0; i<size; i++)
	{
		data8[indx++] = data[i]&0xFF;   // extract LSB
		data8[indx++] = (data[i]>>8)&0xFF;
		data8[indx++] = (data[i]>>16)&0xFF;
		data8[indx++] = (data[i]>>24)&0xFF;
	}
	W25Q_Write(page, offset, indx, data8);

	return State_W25Q = W25Q_READY;
}

W25Q_Status W25Q_Read_32B (uint32_t page, uint16_t offset, uint32_t size, uint32_t *data)
{
	uint8_t data8[size*4];
	W25Q_FastRead(page, offset, size*4, data8);

	for (uint32_t i=0; i < size; i++)
	{
		data[i] = (data8[0+(i*4)]) | (data8[1+(i*4)]<<8) | (data8[2+(i*4)]<<16) | (data8[3+(i*4)]<<24);
		// avoid the Warning operation on 'idx' may be undefined, using (data8[idx++)]) | (data8[idx++]<<8)...
	}
	return State_W25Q = W25Q_READY;
}

W25Q_Status W25Q_Erase_Sector (uint16_t numsector)
{
	uint32_t timeout_counter = GetSysTick();
	uint8_t tData[6];
	uint32_t memAddr = numsector*16*256;   // Each sector contains 16 pages * 256 bytes

	write_enable();

	if (numBLOCK<512)   // Chip Size < 256Mb
	{
		tData[0] = 0x20;  // Erase sector (a sector = 4KB = 16 pages of 256 bytes each)
		tData[1] = (memAddr>>16)&0xFF;  // MSB of the memory Address
		tData[2] = (memAddr>>8)&0xFF;
		tData[3] = (memAddr)&0xFF; // LSB of the memory Address

		csLOW1();
		SPI1_Transmit_DMA(tData, 4);
		while(GetState_SPI1() != SPI_READY) {if (GetSysTick() - timeout_counter > 100) return W25Q_TIMEOUT;}
		csHIGH1();
	}
	else  // we use 32bit memory address for chips >= 256Mb
	{
		tData[0] = 0x21;  // ERASE Sector with 32bit address
		tData[1] = (memAddr>>24)&0xFF;
		tData[2] = (memAddr>>16)&0xFF;
		tData[3] = (memAddr>>8)&0xFF;
		tData[4] = memAddr&0xFF;

		csLOW1();  // pull the CS LOW
		SPI1_Transmit_DMA(tData, 5);
		while(GetState_SPI1() != SPI_READY) {if (GetSysTick() - timeout_counter > 100) return W25Q_TIMEOUT;}
		csHIGH1();  // pull the HIGH
	}

	while(W25Q_Read_Status_Reg() && 0x01) {
		NBdelay_ms(5);   // Sector erase max time = 400ms, polling status register every 5ms shows actually below 40ms
		if (GetSysTick() - timeout_counter > 500)
			return W25Q_TIMEOUT;
	}
	write_disable();

	return State_W25Q = W25Q_READY;
}


uint32_t bytestoWrite (uint32_t size, uint16_t offset)
{
	if ((size+offset)<256) return size;
	else return 256-offset;
}

uint32_t bytestoModify (uint32_t size, uint16_t offset)
{
	if ((size + offset) < 4096) return size;
	else return 4096 - offset;
}

void floatToBytes(uint8_t * ftoa_bytes_temp,float float_variable)
{
	union {
		float a;
		uint8_t bytes[4];
	} thing;

	thing.a = float_variable;

	for (uint8_t i = 0; i < 4; i++) {
		ftoa_bytes_temp[i] = thing.bytes[i];
	}
}

float BytesTofloat(uint8_t * ftoa_bytes_temp)
{
	union {
		float a;
		uint8_t bytes[4];
	} thing;

	for (uint8_t i = 0; i < 4; i++) {
		thing.bytes[i] = ftoa_bytes_temp[i];
	}

	float float_variable =  thing.a;

	return float_variable;
}
