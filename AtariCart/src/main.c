/*
 * --------------------------------------
 * UNOCart Firmware (c)2016 Robin Edwards
 * --------------------------------------
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define _GNU_SOURCE

#include "defines.h"
#include "stm32f4xx.h"
#include "tm_stm32f4_fatfs.h"
#include "fatfs_sd_sdio.h"
#include "tm_stm32f4_delay.h"
#include <stdio.h>
#include <string.h>
#include "loader_rom.h"

unsigned char cart_ram1[64*1024];
unsigned char cart_ram2[64*1024] __attribute__((section(".ccmram")));
unsigned char cart_d5xx[256] = {0};
//char errorBuf[40];
unsigned char CartType,NewCart;
DWORD FATBeginLBA, ClusterBeginLBA, FileBytesRemaining, Cluster, lba;
int BufPtr, SectorOffset, SectorsPerCluster, CAR, FAT16, EOFFlag;
//unsigned char MountBuf[18];
unsigned char HeaderBuf[16];


#define IDE_DATA			0xf0
#define IDE_ERR				0xf1
#define IDE_SCNT			0xf2
#define IDE_SNUM			0xf3
#define IDE_CYLL			0xf4
#define IDE_CYLH			0xf5
#define IDE_HEAD			0xf6
#define IDE_STAT			0xf7
#define IDE_CTRL			0xf8
#define SIDE2_SIG			0xf8
#define SIDE2_REMOVE_FLAG	0xf9
#define	SDX_FLAG			0xfc
#define IDE_SIG				0xfd


#define ATA_CMD_READ			0x20
#define ATA_CMD_WRITE			0x30
#define ATA_CMD_IDENTIFY_DEVICE	0xEC
#define ATA_CMD_SET_FEATURES	0xEF
#define ATA_RST_LOW				0x00
#define ATA_RST_HIGH			0x01
#define CMD_SET_RAM_BANK		0xFE
#define CMD_SET_CART_TYPE		0xFF

#define CART_CMD_OPEN_ITEM			0x00
#define CART_CMD_READ_CUR_DIR		0x01
#define CART_CMD_GET_DIR_ENTRY		0x02
#define CART_CMD_UP_DIR				0x03
#define CART_CMD_ROOT_DIR			0x04
#define CART_CMD_SEARCH				0x05
#define CART_CMD_LOAD_SOFT_OS		0x10
#define CART_CMD_SOFT_OS_CHUNK		0x11
#define CART_CMD_MOUNT_ATR			0x20	// unused, done automatically by firmware
#define CART_CMD_READ_ATR_SECTOR	0x21
#define CART_CMD_WRITE_ATR_SECTOR	0x22
#define CART_CMD_ATR_HEADER			0x23
#define CART_CMD_NO_CART			0xFE
#define CART_CMD_ACTIVATE_CART  	0xFF

#define CART_TYPE_NONE				0
#define CART_TYPE_8K				1	// 8k
#define CART_TYPE_16K				2	// 16K
#define CART_TYPE_XEGS_32K			12	// 32K
#define CART_TYPE_XEGS_64K			13	// 64K
#define CART_TYPE_XEGS_128K			14	// 128K
#define CART_TYPE_SW_XEGS_32K		33	// 32K
#define CART_TYPE_SW_XEGS_64K		34	// 64K
#define CART_TYPE_SW_XEGS_128K		35	// 128K
#define CART_TYPE_MEGACART_16K		26	// 16K
#define CART_TYPE_MEGACART_32K		27	// 32K
#define CART_TYPE_MEGACART_64K		28	// 64K
#define CART_TYPE_MEGACART_128K		29	// 128K
#define CART_TYPE_BOUNTY_BOB		18	// 40K
#define CART_TYPE_ATARIMAX_1MBIT	41	// 128K
#define CART_TYPE_WILLIAMS_64K		22	// 64K
#define CART_TYPE_WILLIAMS_32K		8	// 32K
#define CART_TYPE_OSS_16K_TYPE_B	15	// 16K
#define CART_TYPE_OSS_8K			44	// 8K
#define CART_TYPE_OSS_16K_034M		3	// 16K
#define CART_TYPE_OSS_16K_043M		45	// 16K
#define CART_TYPE_SIC_128K			54	// 128K
#define CART_TYPE_SDX_64K			11	// 64K
#define CART_TYPE_SDX_128K			43	// 128K
#define CART_TYPE_DIAMOND_64K		10	// 64K
#define CART_TYPE_EXPRESS_64K		9	// 64K
#define CART_TYPE_BLIZZARD_16K		40	// 16K
#define CART_TYPE_LOADER			99	// 16K

unsigned char ATASectorBuffer[512];

int	WriteActive, ReadActive;
uint16_t ATAByte;

uint16_t addr, data, c;
int RAMBank;
int RAMBase;


/* CARTRIDGE/XEX HANDLING */

/* int load_file(char *filename) {
	TM_DELAY_Init();
	FATFS FatFs;
	int cart_type = CART_TYPE_NONE;
	int car_file = 0, xex_file = 0, expectedSize = 0;
	unsigned char carFileHeader[16];
	UINT br, size = 0;

	if (strncasecmp(filename+strlen(filename)-4, ".CAR", 4) == 0)
		car_file = 1;
	if (strncasecmp(filename+strlen(filename)-4, ".XEX", 4) == 0)
		xex_file = 1;

	if (f_mount(&FatFs, "", 1) != FR_OK) {
		strcpy(errorBuf, "Can't read SD card");
		return 0;
	}
	FIL fil;
	if (f_open(&fil, filename, FA_READ) != FR_OK) {
		strcpy(errorBuf, "Can't open file");
		goto cleanup;
	}

	// read the .CAR file header?
	if (car_file) {
		if (f_read(&fil, carFileHeader, 16, &br) != FR_OK || br != 16) {
			strcpy(errorBuf, "Bad CAR file");
			goto closefile;
		}
		int car_type = carFileHeader[7];
		if (car_type == 1)			{ cart_type = CART_TYPE_8K; expectedSize = 8192; }
		else if (car_type == 2)		{ cart_type = CART_TYPE_16K; expectedSize = 16384; }
		else if (car_type == 3) 	{ cart_type = CART_TYPE_OSS_16K_034M; expectedSize = 16384; }
		else if (car_type == 8)		{ cart_type = CART_TYPE_WILLIAMS_64K; expectedSize = 65536; }
		else if (car_type == 9)		{ cart_type = CART_TYPE_EXPRESS_64K; expectedSize = 65536; }
		else if (car_type == 10)	{ cart_type = CART_TYPE_DIAMOND_64K; expectedSize = 65536; }
		else if (car_type == 11)	{ cart_type = CART_TYPE_SDX_64K; expectedSize = 65536; }
		else if (car_type == 12) 	{ cart_type = CART_TYPE_XEGS_32K; expectedSize = 32768; }
		else if (car_type == 13) 	{ cart_type = CART_TYPE_XEGS_64K; expectedSize = 65536; }
		else if (car_type == 14) 	{ cart_type = CART_TYPE_XEGS_128K; expectedSize = 131072; }
		else if (car_type == 15) 	{ cart_type = CART_TYPE_OSS_16K_TYPE_B; expectedSize = 16384; }
		else if (car_type == 18) 	{ cart_type = CART_TYPE_BOUNTY_BOB; expectedSize = 40960; }
		else if (car_type == 22)	{ cart_type = CART_TYPE_WILLIAMS_64K; expectedSize = 32768; }
		else if (car_type == 26)	{ cart_type = CART_TYPE_MEGACART_16K; expectedSize = 16384; }
		else if (car_type == 27)	{ cart_type = CART_TYPE_MEGACART_32K; expectedSize = 32768; }
		else if (car_type == 28)	{ cart_type = CART_TYPE_MEGACART_64K; expectedSize = 65536; }
		else if (car_type == 29)	{ cart_type = CART_TYPE_MEGACART_128K; expectedSize = 131072; }
		else if (car_type == 33)	{ cart_type = CART_TYPE_SW_XEGS_32K; expectedSize = 32768; }
		else if (car_type == 34)	{ cart_type = CART_TYPE_SW_XEGS_64K; expectedSize = 65536; }
		else if (car_type == 35)	{ cart_type = CART_TYPE_SW_XEGS_128K; expectedSize = 131072; }
		else if (car_type == 40)	{ cart_type = CART_TYPE_BLIZZARD_16K; expectedSize = 16384; }
		else if (car_type == 41)	{ cart_type = CART_TYPE_ATARIMAX_1MBIT; expectedSize = 131072; }
		else if (car_type == 43)	{ cart_type = CART_TYPE_SDX_128K; expectedSize = 131072; }
		else if (car_type == 44)	{ cart_type = CART_TYPE_OSS_8K; expectedSize = 8192; }
		else if (car_type == 45) 	{ cart_type = CART_TYPE_OSS_16K_043M; expectedSize = 16384; }
		else if (car_type == 54)	{ cart_type = CART_TYPE_SIC_128K; expectedSize = 131072; }
		else {
			strcpy(errorBuf, "Unsupported CAR type");
			goto closefile;
		}
	}

	// set a default error
	strcpy(errorBuf, "Can't read file");

	unsigned char *dst = &cart_ram1[0];
	int bytes_to_read = 64 * 1024;
	if (xex_file) {
		dst += 4;	// leave room for the file length at the start of sram
		bytes_to_read -= 4;
	}
	// read the file in two 64k chunks to each area of SRAM
	if (f_read(&fil, dst, bytes_to_read, &br) != FR_OK) {
		cart_type = CART_TYPE_NONE;
		goto closefile;
	}
	size += br;
	if (br == bytes_to_read) {
		// first 64k was complete, so try to load 64k more
		if (f_read(&fil, &cart_ram2[0], 64*1024, &br) != FR_OK) {
			cart_type = CART_TYPE_NONE;
			goto closefile;
		}
		size += br;
		if (br == 64*1024) {
			// that's 128k read, is there any more?
			if (f_read(&fil, carFileHeader, 1, &br) != FR_OK) {
				cart_type = CART_TYPE_NONE;
				goto closefile;
			}
			if	(br == 1) {
				strcpy(errorBuf, "Cart file/XEX too big (>128k)");
				cart_type = CART_TYPE_NONE;
				goto closefile;
			}
		}
	}

	// set the correct cart type based on the size
	if (car_file) {
		if (size != expectedSize) {
			strcpy(errorBuf, "CAR file is wrong size");
			cart_type = CART_TYPE_NONE;
			goto closefile;
		}
	}
	else if (xex_file) {
		cart_type = CART_TYPE_XEX;
		// stick the size of the file as the first 4 bytes (little endian)
		cart_ram1[0] = size & 0xFF;
		cart_ram1[1] = (size >> 8) & 0xFF;
		cart_ram1[2] = (size >> 16) & 0xFF;
		cart_ram1[3] = 0;	// has to be zero!
	}
	else {	// not a car/xex file - guess the type based on size
		if (size == 8*1024) cart_type = CART_TYPE_8K;
		else if (size == 16*1024) cart_type = CART_TYPE_16K;
		else if (size == 32*1024) cart_type = CART_TYPE_XEGS_32K;
		else if (size == 64*1024) cart_type = CART_TYPE_XEGS_64K;
		else if (size == 128*1024) cart_type = CART_TYPE_XEGS_128K;
		else {
			strcpy(errorBuf, "Unsupported ROM size ");
			cart_type = CART_TYPE_NONE;
			goto closefile;
		}
	}

closefile:
	f_close(&fil);
cleanup:
	f_mount(0, "", 1);
	return cart_type;
} */

#define RD5_LOW GPIOB->BSRRH = GPIO_Pin_2;
#define RD4_LOW GPIOB->BSRRH = GPIO_Pin_4;
#define RD5_HIGH GPIOB->BSRRL = GPIO_Pin_2;
#define RD4_HIGH GPIOB->BSRRL = GPIO_Pin_4;

#define CONTROL_IN GPIOC->IDR
#define ADDR_IN GPIOD->IDR
#define DATA_IN GPIOE->IDR
#define DATA_OUT GPIOE->ODR

#define PHI2_RD (GPIOC->IDR & 0x0001)
#define S5_RD (GPIOC->IDR & 0x0002)
#define S4_RD (GPIOC->IDR & 0x0004)
#define S4_AND_S5_HIGH (GPIOC->IDR & 0x0006) == 0x6

#define PHI2	0x0001
#define S5		0x0002
#define S4		0x0004
#define CCTL	0x0010
#define RW		0x0020

#define SET_DATA_MODE_IN GPIOE->MODER = 0x00000000;
#define SET_DATA_MODE_OUT GPIOE->MODER = 0x55550000;

#define GREEN_LED_OFF GPIOB->BSRRH = GPIO_Pin_0;
#define RED_LED_OFF GPIOB->BSRRH = GPIO_Pin_1;
#define GREEN_LED_ON GPIOB->BSRRL = GPIO_Pin_0;
#define RED_LED_ON GPIOB->BSRRL = GPIO_Pin_1;

GPIO_InitTypeDef  GPIO_InitStructure;

/* Green LED -> PB0, Red LED -> PB1, RD5 -> PB2, RD4 -> PB4 */
void config_gpio_leds_RD45()
{
	/* GPIOB Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	/* Configure PB0, PB1in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/* Input Signals GPIO pins on CLK -> PC0, /S5 -> PC1, /S4 ->PC2, CCTL -> PC4, R/W -> PC5 */
void config_gpio_sig(void) {
	/* GPIOC Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	/* Configure GPIO Settings */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/* Input/Output data GPIO pins on PE{8..15} */
void config_gpio_data(void) {
	/* GPIOE Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	/* Configure GPIO Settings */
	GPIO_InitStructure.GPIO_Pin =
		GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 |
		GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;	// avoid sharp edges
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
}

/* Input Address GPIO pins on PD{0..15} */
void config_gpio_addr(void) {
	/* GPIOD Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/* Configure GPIO Settings */
	GPIO_InitStructure.GPIO_Pin =
		GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |
		GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 |
		GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 |
		GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}


void ReadSector() {
	DRESULT status;
	DWORD lba = cart_d5xx[IDE_SNUM];
	lba += cart_d5xx[IDE_CYLL]<<8;
	lba += cart_d5xx[IDE_CYLH]<<16;
	lba += cart_d5xx[IDE_HEAD]&0x0f<<24;
	status = TM_FATFS_SD_disk_read (ATASectorBuffer, lba, 1);
//	SD_ReadBlock(ATASectorBuffer,lba*512,512)
	if (status == RES_ERROR) cart_d5xx[IDE_STAT] = 0x01;
	else {
		cart_d5xx[IDE_STAT] = 0x58;	// SKC, RDY and DRQ
		cart_d5xx[IDE_DATA] = ATASectorBuffer[0];
		ReadActive = 1;
		ATAByte = 1;
		RED_LED_ON
	}
}



void WriteSector() {
	DRESULT status;
	DWORD lba = cart_d5xx[IDE_SNUM];
	lba += cart_d5xx[IDE_CYLL]<<8;
	lba += cart_d5xx[IDE_CYLH]<<16;
	lba += cart_d5xx[IDE_HEAD]&0x0f<<24;
	status = TM_FATFS_SD_disk_write (ATASectorBuffer, lba, 1);
	if (status == RES_ERROR) cart_d5xx[IDE_STAT] = 0x01;
	else {
		cart_d5xx[IDE_STAT] = 0x50; // RDY and SKC
		WriteActive = 0;
	}
}





void IdentifyDevice() {
	int i;
	unsigned char a;
//	SD_CardInfo SDCardInfo;
//	SD_Error ErrorStatus = SD_GetCardInfo(&SDCardInfo);
//	DWORD lba = SDCardInfo.CardCapacity>>9;
	for(i=0;i<512;i++) ATASectorBuffer[i]=0;
	char buf[40+1] = {};
	TM_FATFS_SD_disk_ioctl(GET_SECTOR_COUNT,buf);
	DWORD lba = buf[0];
	lba+= buf[1]<<8;
	lba+= buf[2]<<16;
	lba+= buf[3]<<24;

	// words 10-19: serial number
	ATASectorBuffer[10*2] = '0';
	ATASectorBuffer[10*2+1] = '7';
	ATASectorBuffer[11*2] = '4';
	ATASectorBuffer[11*2+1] = '7';
	ATASectorBuffer[12*2] = '7';
	ATASectorBuffer[12*2+1] = '6';
	ATASectorBuffer[13*2] = '0';
	ATASectorBuffer[13*2+1] = '2';
	ATASectorBuffer[14*2] = '8';
	ATASectorBuffer[14*2+1] = '1';

	for(i=15*2;i<20*2;i++) ATASectorBuffer[i]=' ';

	// words 23-26: firmware revision
	ATASectorBuffer[23*2] = '.';
	ATASectorBuffer[23*2+1] = '1';
	ATASectorBuffer[24*2] = ' ';
	ATASectorBuffer[24*2+1] = '0';
	ATASectorBuffer[25*2] = ' ';
	ATASectorBuffer[25*2+1] = ' ';
	ATASectorBuffer[26*2] = ' ';
	ATASectorBuffer[26*2+1] = ' ';

	// words 27-46: model name (byte swapped)
	sprintf(buf, "SD Card");
	memset(&ATASectorBuffer[27*2], ' ', 40);
	memcpy(&ATASectorBuffer[27*2], buf, strlen(buf));

	for(i=0; i<40; i+=2) {
		a=ATASectorBuffer[27*2+i];	// swap bytes
		ATASectorBuffer[27*2+i]=ATASectorBuffer[27*2+i+1];
		ATASectorBuffer[27*2+i+1]=a;
	}

	// words 60-61: number of user-addressable lba sectors
	ATASectorBuffer[60*2] = lba&0xff;
	ATASectorBuffer[60*2+1] = (lba>>8)&0xff;
	ATASectorBuffer[60*2+2] = (lba>>16)&0xff;
	ATASectorBuffer[60*2+3] = (lba>>24)&0xff;

	uint16_t Cylinders = lba/(15*63);
	// word 1: cylinder count
	ATASectorBuffer[1*2]=Cylinders&0xff;
	ATASectorBuffer[1*2+1]=(Cylinders>>8)&0xff;

	// word 3: head count
	ATASectorBuffer[3*2]=15;
	ATASectorBuffer[3*2+1]=0;

	// word 6: sector count
	ATASectorBuffer[6*2]=63;
	ATASectorBuffer[6*2+1]=0;

	cart_d5xx[IDE_STAT] = 0x58;	// SKC, RDY and DRQ
	cart_d5xx[IDE_DATA] = ATASectorBuffer[0];
	ReadActive = 1;
	ATAByte = 1;
	RED_LED_ON
}




/*
 ******************************************************************************************************
 * Cart loading functions
 ******************************************************************************************************
 */



void GetNextCluster() {
	int Offset;
	DRESULT status;
	if (FAT16) {
		lba = Cluster/256;
		Offset = (Cluster % 256) * 2;
	}
	else {
		lba = Cluster/128;
		Offset = (Cluster % 128) * 4;
	}
	lba+=FATBeginLBA;	// add to absolute FAT start
	status = TM_FATFS_SD_disk_read (ATASectorBuffer, lba, 1);	// get sector

	Cluster = ATASectorBuffer[Offset];
	Cluster += ATASectorBuffer[Offset+1]<<8;
	if (!FAT16) {
		Cluster += ATASectorBuffer[Offset+2]<<16;
		Cluster += (ATASectorBuffer[Offset+3]&0x0f)<<24;
		EOFFlag = (Cluster == 0x0FFFFFF8);
	}
	else EOFFlag = (Cluster == 0xFFF8);
}



void NextSector() {
	SectorOffset++;
	if(SectorOffset == SectorsPerCluster) {
		GetNextCluster();
		SectorOffset = 0;
	}
	lba = ((Cluster - 2) * SectorsPerCluster) + ClusterBeginLBA + SectorOffset;	// get sector address
}





DWORD ReadBuffer(BYTE *Address, int ioBufLen) {
	DRESULT status;
	DWORD BytesRead = 0;
	unsigned char c;
	RED_LED_ON
	while (FileBytesRemaining && ioBufLen) {
		if ((BufPtr == 512) && (FileBytesRemaining >= 512) && (ioBufLen >= 512)) {	// if we need a full sector, burst read it
			NextSector();
			status = TM_FATFS_SD_disk_read(Address, lba, 1);
			ioBufLen -= 512;
			Address += 512;
			FileBytesRemaining -= 512;
			BytesRead += 512;
			BufPtr = 512;
		}
		else {
			if (BufPtr == 512) {
				NextSector();
				BufPtr = 0;
				status = TM_FATFS_SD_disk_read(ATASectorBuffer, lba, 1);
			}
			*Address++ = ATASectorBuffer[BufPtr++];
			BytesRead++;
			FileBytesRemaining--;
			ioBufLen--;
		}
	}
	RED_LED_OFF
	return(BytesRead);
}




/*
 * Fast loader for cart images
 * Loader provides:
 * LBA address of FAT (4)
 * LBA address of cluster area (4)
 * FAT width (1)
 * Sectors Per Cluster (1)
 * Image type (1)
 * Cluster number of cart image (4)
 * Length of cart image file in bytes (4)
 */

int LoadCartridge() {
;	unsigned char HeaderBuf[16];
	DWORD expectedSize,Size;
	SectorOffset = -1;
	BufPtr = 512;
	FATBeginLBA = ATASectorBuffer[0];
	FATBeginLBA += ATASectorBuffer[1]<<8;
	FATBeginLBA += ATASectorBuffer[2]<<16;
	FATBeginLBA += ATASectorBuffer[3]<<24;
	ClusterBeginLBA = ATASectorBuffer[4];
	ClusterBeginLBA += ATASectorBuffer[5]<<8;
	ClusterBeginLBA += ATASectorBuffer[6]<<16;
	ClusterBeginLBA += ATASectorBuffer[7]<<24;
	FAT16 = ATASectorBuffer[8]&0xff;
	SectorsPerCluster = ATASectorBuffer[9]&0xff;
	CAR = ATASectorBuffer[10]&0xff;
	Cluster = ATASectorBuffer[11];
	Cluster += ATASectorBuffer[12]<<8;
	Cluster += ATASectorBuffer[13]<<16;
	Cluster += (ATASectorBuffer[14]&0x0f)<<24;
	FileBytesRemaining = ATASectorBuffer[15];
	FileBytesRemaining += ATASectorBuffer[16]<<8;
	FileBytesRemaining += ATASectorBuffer[17]<<16;
	FileBytesRemaining += ATASectorBuffer[18]<<24;
	if(CAR) {
		Size = ReadBuffer(HeaderBuf,16);
		CartType = HeaderBuf[7];
		if (CartType == CART_TYPE_8K) expectedSize = 8192;
		else if (CartType == CART_TYPE_16K) expectedSize = 16384;
		else if (CartType == CART_TYPE_OSS_16K_034M) expectedSize = 16384;
		else if (CartType == CART_TYPE_WILLIAMS_64K) expectedSize = 65536;
		else if (CartType == CART_TYPE_EXPRESS_64K)	expectedSize = 65536;
		else if (CartType == CART_TYPE_DIAMOND_64K) expectedSize = 65536;
		else if (CartType == CART_TYPE_SDX_64K) expectedSize = 65536;
		else if (CartType == CART_TYPE_XEGS_32K) expectedSize = 32768;
		else if (CartType == CART_TYPE_XEGS_64K) expectedSize = 65536;
		else if (CartType == CART_TYPE_XEGS_128K) expectedSize = 131072;
		else if (CartType == CART_TYPE_OSS_16K_TYPE_B) expectedSize = 16384;
		else if (CartType == CART_TYPE_BOUNTY_BOB) expectedSize = 40960;
		else if (CartType == CART_TYPE_WILLIAMS_32K) expectedSize = 32768;
		else if (CartType == CART_TYPE_MEGACART_16K) expectedSize = 16384;
		else if (CartType == CART_TYPE_MEGACART_32K) expectedSize = 32768;
		else if (CartType == CART_TYPE_MEGACART_64K) expectedSize = 65536;
		else if (CartType == CART_TYPE_MEGACART_128K) expectedSize = 131072;
		else if (CartType == CART_TYPE_SW_XEGS_32K) expectedSize = 32768;
		else if (CartType == CART_TYPE_SW_XEGS_64K) expectedSize = 65536;
		else if (CartType == CART_TYPE_SW_XEGS_128K) expectedSize = 131072;
		else if (CartType == CART_TYPE_BLIZZARD_16K) expectedSize = 16384;
		else if (CartType == CART_TYPE_ATARIMAX_1MBIT) expectedSize = 131072;
		else if (CartType == CART_TYPE_SDX_128K) expectedSize = 131072;
		else if (CartType == CART_TYPE_OSS_8K) expectedSize = 8192;
		else if (CartType == CART_TYPE_OSS_16K_043M) expectedSize = 16384;
		else if (CartType == CART_TYPE_SIC_128K) expectedSize = 131072;
		else {
			return(2);
		}
		if (FileBytesRemaining != expectedSize) {
			return(2);
		}

	}
	else {	// if there's no header, try to intuit cart type
		if (FileBytesRemaining == 8192) CartType = CART_TYPE_8K;
		else if (FileBytesRemaining == 16384) CartType = CART_TYPE_16K;
		else if (FileBytesRemaining == 32768) CartType = CART_TYPE_XEGS_32K;
		else if (FileBytesRemaining == 65536) CartType = CART_TYPE_XEGS_64K;
		else if (FileBytesRemaining == 131072) CartType = CART_TYPE_XEGS_128K;
		else {
			return(2);
		}
	}

	if (FileBytesRemaining > 65536) {
		Size = ReadBuffer(cart_ram1,65536);
		if (Size == 65536) {
			Size = ReadBuffer(cart_ram2,FileBytesRemaining);
		}
	}
	else {
		Size = ReadBuffer(cart_ram1,FileBytesRemaining);
	}

	NewCart = CartType;
	return(1);
}





/*
 * Handle writes to open RAM window at 0xA000-BFFF.
 * Keeping this out of the main loop removes the need
 * for checking RW during normal emulation. We still
 * need to look after the HDD, though, although
 * it's unlikely we'll need to handle IDE writes
 * while the RAM is being filled.
 * Normal ROM emulation is completely disabled during
 * this phase.
 */

void LoadCartImage() {
	uint16_t addr, data, c;
	RD5_HIGH
	NewCart = RAMBase = 0;
	while (!NewCart)
	{
		// wait for phi2 high
		while (!((c = CONTROL_IN) & PHI2));
		addr = ADDR_IN;
		if (!(c & S5) && !(c & RW)) { // write to cart window
			data = DATA_IN;
			while (CONTROL_IN & PHI2) data = DATA_IN;
			if (RAMBank < 8) cart_ram1[RAMBase+addr] = data>>8;
			else cart_ram2[RAMBase+addr] = data>>8;
		}

		if (!(c & CCTL)) { // CCTL read/write
			addr &= 0xff;
			if (c & RW) {
				SET_DATA_MODE_OUT
				DATA_OUT = ((uint16_t)cart_d5xx[addr])<<8;
				while (CONTROL_IN & PHI2);
				SET_DATA_MODE_IN

				if(addr==IDE_DATA) {
					if (ATAByte<512) cart_d5xx[IDE_DATA] = ATASectorBuffer[ATAByte++];
					else {
						cart_d5xx[IDE_STAT] = 0x50;
						RED_LED_OFF
					}
				}
			}
			else {	// write to CCTL
				data = DATA_IN;
				// read data bus on falling edge of phi2
				while (CONTROL_IN & PHI2) data = DATA_IN;
				data >>= 8;
				if (addr>=IDE_DATA) {
					if (addr <= SIDE2_REMOVE_FLAG) cart_d5xx[addr] = data;	// don't allow writes to signature area
					if (addr == IDE_STAT) {	// we received an ATA command
						switch (data) {
							case ATA_CMD_READ:	// read a sector
								ReadSector();
								break;
/*							case ATA_CMD_WRITE:
								cart_d5xx[IDE_STAT] = 0x58;	// SKC, RDY and DRQ
								WriteActive = 1;	// we're now counting writes to the data register
								ATAByte = 0;
								break;
							case ATA_CMD_IDENTIFY_DEVICE:
								IdentifyDevice();
								break;
							case ATA_CMD_SET_FEATURES:	// acknowledge Set Features but do nothing
								cart_d5xx[IDE_STAT] = 0x50; // SKC and RDY
								break; */
							case CMD_SET_RAM_BANK:	// select a different RAM bank
								RAMBank = cart_d5xx[IDE_ERR];
								RAMBase = (RAMBank&0x07) * 8192;
								break;
							case CMD_SET_CART_TYPE:	// set the cartridge type
								NewCart = cart_d5xx[IDE_ERR];
								break;
						}
					}
//					if (addr == IDE_DATA) {	// write to data register
//						if(WriteActive) {
//							ATASectorBuffer[ATAByte++] = data;
//							if(ATAByte == 512) WriteSector();	// if this is the last byte, deassert DRQ and write sector
//						}
//					}
//					if (addr == IDE_CTRL) {	// write to reset register
//						if(!data) {	// assert reset
//						}
//					}
				}
			}
		}
	}
}






/*
 Theory of Operation
 -------------------
 Atari sends command to mcu on cart by writing to $D5DF ($D5E0-$D5FF = SDX)
 (extra paramters for the command in $D500-$D5DE)
 Atari must be running from RAM when it sends a command, since the mcu on the cart will
 go away at that point.
 Atari polls $D500 until it reads $11. At this point it knows the mcu is back
 and it is safe to rts back to code in cartridge ROM again.
 Results of the command are in $D501-$D5DF
*/

void EmulateNoCart() {
//	uint16_t addr, data, c;
	int BufferSize;
	while (!NewCart)
	{
		// wait for phi2 high
		while (!((c = CONTROL_IN) & PHI2)) ;
//		addr = ADDR_IN;

		if (!(c & CCTL)) {
//			addr &= 0xff;
			// CCTL low
			if (c & RW) {
				// read
				SET_DATA_MODE_OUT
				addr = ADDR_IN & 0xff;
				DATA_OUT = ((uint16_t)cart_d5xx[addr])<<8;
				while (CONTROL_IN & PHI2);
				SET_DATA_MODE_IN

				if(addr==IDE_DATA) {
					if (ATAByte<512) cart_d5xx[IDE_DATA] = ATASectorBuffer[ATAByte++];
					else {
						cart_d5xx[IDE_STAT] = 0x50;
						RED_LED_OFF
					}
				}
			}
			else {
				// write
				addr = ADDR_IN &0xff;
				data = DATA_IN;
				// read data bus on falling edge of phi2
				while (CONTROL_IN & PHI2) data = DATA_IN;
				data >>= 8;
				if (addr>=IDE_DATA) {
					if (addr <= SIDE2_REMOVE_FLAG) cart_d5xx[addr] = data;	// don't allow writes to signature area
					if (addr == IDE_STAT) {	// we received an ATA command
						switch (data) {
							case ATA_CMD_READ:	// read a sector
								ReadSector();
								break;
							case ATA_CMD_WRITE:
								cart_d5xx[IDE_STAT] = 0x58;	// SKC, RDY and DRQ
								WriteActive = 1;	// we're now counting writes to the data register
								ATAByte = 0;
								BufferSize = 512;
								break;
							case ATA_CMD_IDENTIFY_DEVICE:
								IdentifyDevice();
								break;
							case ATA_CMD_SET_FEATURES:	// acknowledge Set Features but do nothing
								cart_d5xx[IDE_STAT] = 0x50; // SKC and RDY
								break;
//							case CMD_SET_RAM_BANK:	// open a RAM window in the cart ROM area
//								LoadCartImage();
//								break;
							case CART_CMD_ACTIVATE_CART:
//								cart_d5xx[IDE_STAT] = 0x01;
								WriteActive = 1;
								ATAByte = 0;
								BufferSize = 19;
								break;
						}
					}
					if (addr == IDE_DATA) {	// write to data register
						if(WriteActive == 1) {
							ATASectorBuffer[ATAByte++] = data;
							if(ATAByte == BufferSize) {
								if (BufferSize == 512) WriteSector();	// if this is the last byte, deassert DRQ and write sector
								else cart_d5xx[IDE_STAT] = LoadCartridge();
							}
						}
					}
					if (addr == IDE_CTRL) {	// write to reset register
						if(!data) {	// assert reset
						}
					}
				}
			}
		}
	}
}




void EmulateStandard8K() {
//	uint16_t addr, data, c;
	RD5_HIGH
	while (!NewCart)
	{
		// wait for phi2 high
		while (!((c = CONTROL_IN) & PHI2)) ;
		addr = ADDR_IN;
		if (!(c & S5)) {
			SET_DATA_MODE_OUT
			DATA_OUT = ((uint16_t)cart_ram1[addr])<<8;
			while (CONTROL_IN & PHI2);
			SET_DATA_MODE_IN
		}
		if (!(c & CCTL)) {
			addr &= 0xff;
			// CCTL low
			if (c & RW) {
				// read
				SET_DATA_MODE_OUT
				DATA_OUT = ((uint16_t)cart_d5xx[addr])<<8;
				while (CONTROL_IN & PHI2);
				SET_DATA_MODE_IN

				if(addr==IDE_DATA) {
					if (ATAByte<512) cart_d5xx[IDE_DATA] = ATASectorBuffer[ATAByte++];
					else {
						cart_d5xx[IDE_STAT] = 0x50;
						RED_LED_OFF
					}
				}
			}
			else {
				// write
//				addr = ADDR_IN &0xff;
				data = DATA_IN;
				// read data bus on falling edge of phi2
				while (CONTROL_IN & PHI2) data = DATA_IN;
				data >>= 8;
				if (addr>=IDE_DATA) {
					if (addr <= SIDE2_REMOVE_FLAG) cart_d5xx[addr] = data;	// don't allow writes to signature area
					if (addr == IDE_STAT) {	// we received an ATA command
						switch (data) {
							case ATA_CMD_READ:	// read a sector
								ReadSector();
								break;
							case ATA_CMD_WRITE:
								cart_d5xx[IDE_STAT] = 0x58;	// SKC, RDY and DRQ
								WriteActive = 1;	// we're now counting writes to the data register
								ATAByte = 0;
								break;
							case ATA_CMD_IDENTIFY_DEVICE:
								IdentifyDevice();
								break;
							case ATA_CMD_SET_FEATURES:	// acknowledge Set Features but do nothing
								cart_d5xx[IDE_STAT] = 0x50; // SKC and RDY
								break;
							case CMD_SET_RAM_BANK:	// open a RAM window in the cart ROM area
								LoadCartImage();
								break;
						}
					}
					if (addr == IDE_DATA) {	// write to data register
						if(WriteActive) {
							ATASectorBuffer[ATAByte++] = data;
							if(ATAByte == 512) WriteSector();	// if this is the last byte, deassert DRQ and write sector
						}
					}
					if (addr == IDE_CTRL) {	// write to reset register
						if(!data) {	// assert reset
						}
					}
				}

			}
		}

	}
}



void EmulateStandard16K() {
	RD4_HIGH
	RD5_HIGH
	uint16_t addr, data, c;
	while (!NewCart)
	{
		// wait for phi2 high
		while (!((c = CONTROL_IN) & PHI2)) ;
		addr = ADDR_IN;
		if (!(c & S4)) {
			SET_DATA_MODE_OUT
			while (!S4_RD) {
				addr = ADDR_IN;
				DATA_OUT = ((uint16_t)cart_ram1[addr])<<8;
			}
		}
		if (!(c & S5)) {
			SET_DATA_MODE_OUT
			while (!S5_RD) {
				addr = ADDR_IN;
				DATA_OUT = ((uint16_t)cart_ram1[0x2000|addr])<<8;
			}
		}
		if (!(c & CCTL)) {
			addr &= 0xff;
			// CCTL low
			if (c & RW) {
				// read
				SET_DATA_MODE_OUT
				DATA_OUT = ((uint16_t)cart_d5xx[addr])<<8;
				while (CONTROL_IN & PHI2);
				SET_DATA_MODE_IN

				if(addr==IDE_DATA) {
					if (ATAByte<512) cart_d5xx[IDE_DATA] = ATASectorBuffer[ATAByte++];
					else {
						cart_d5xx[IDE_STAT] = 0x50;
						RED_LED_OFF
					}
				}
			}
			else {
				// write
//				addr = ADDR_IN &0xff;
				data = DATA_IN;
				// read data bus on falling edge of phi2
				while (CONTROL_IN & PHI2) data = DATA_IN;
				data >>= 8;
				if (addr>=IDE_DATA) {
					if (addr <= SIDE2_REMOVE_FLAG) cart_d5xx[addr] = data;	// don't allow writes to signature area
					if (addr == IDE_STAT) {	// we received an ATA command
						switch (data) {
							case ATA_CMD_READ:	// read a sector
								ReadSector();
								break;
							case ATA_CMD_WRITE:
								cart_d5xx[IDE_STAT] = 0x58;	// SKC, RDY and DRQ
								WriteActive = 1;	// we're now counting writes to the data register
								ATAByte = 0;
								break;
							case ATA_CMD_IDENTIFY_DEVICE:
								IdentifyDevice();
								break;
							case ATA_CMD_SET_FEATURES:	// acknowledge Set Features but do nothing
								cart_d5xx[IDE_STAT] = 0x50; // SKC and RDY
								break;
							case CMD_SET_RAM_BANK:	// open a RAM window in the cart ROM area
								LoadCartImage();
								break;
						}
					}
					if (addr == IDE_DATA) {	// write to data register
						if(WriteActive) {
							ATASectorBuffer[ATAByte++] = data;
							if(ATAByte == 512) WriteSector();	// if this is the last byte, deassert DRQ and write sector
						}
					}
					if (addr == IDE_CTRL) {	// write to reset register
						if(!data) {	// assert reset
						}
					}
				}

			}
		}

	}
}



void EmulateOSSB() {
	uint16_t addr, data, c;
	uint32_t bank = 0;
	int a0, a3;
	unsigned char *bankPtr;
	RD5_HIGH
	while (!NewCart)
	{
		bankPtr = &cart_ram1[0] + (4096*bank);
		// wait for phi2 high
		while (!((c = CONTROL_IN) & PHI2)) ;
		addr = ADDR_IN;

		if (!(c & S5)) {	// normal cart read
			SET_DATA_MODE_OUT
			if (addr & 0x1000)
				DATA_OUT = ((uint16_t)cart_ram1[addr&0xFFF])<<8;
			else
				DATA_OUT = ((uint16_t)(*(bankPtr+addr)))<<8;
			while (CONTROL_IN & PHI2);
			SET_DATA_MODE_IN
		}

		if (!(c & CCTL)) {	// CCTL access
			addr &= 0xff;
//			addr = ADDR_IN & 0xff;

			if(addr < IDE_DATA)	// handle OSS banking (R/W)
			{
				a0 = addr &0x01, a3 = addr & 0x08;
				if (a3 && !a0) {
					RD5_LOW
					GREEN_LED_OFF
				}
				else {
					RD5_HIGH
					GREEN_LED_ON
					if (!a3 && !a0) bank = 1;
					else if (!a3 && a0) bank = 3;
					else if (a3 && a0) bank = 2;
				}
//			while (CONTROL_IN & PHI2);
			}

			else	// we're addressing the IDE registers
			{
				if (c & RW)	{ // IDE read

					SET_DATA_MODE_OUT
					DATA_OUT = ((uint16_t)cart_d5xx[addr])<<8;
					while (CONTROL_IN & PHI2);
					SET_DATA_MODE_IN

					if(addr==IDE_DATA) {
						if (ATAByte<512) cart_d5xx[IDE_DATA] = ATASectorBuffer[ATAByte++];
						else {
							cart_d5xx[IDE_STAT] = 0x50;
							RED_LED_OFF
						}
					}
				}

				else {	// IDE write
					data = DATA_IN;
					// read data bus on falling edge of phi2
					while (CONTROL_IN & PHI2) data = DATA_IN;
					data >>= 8;
					if (addr>=IDE_DATA) {
						if (addr <= SIDE2_REMOVE_FLAG) cart_d5xx[addr] = data;	// don't allow writes to signature area
						if (addr == IDE_STAT) {	// we received an ATA command
							switch (data) {
								case ATA_CMD_READ:	// read a sector
									ReadSector();
									break;
								case ATA_CMD_WRITE:
									cart_d5xx[IDE_STAT] = 0x58;	// SKC, RDY and DRQ
									WriteActive = 1;	// we're now counting writes to the data register
									ATAByte = 0;
									break;
								case ATA_CMD_IDENTIFY_DEVICE:
									IdentifyDevice();
									break;
								case ATA_CMD_SET_FEATURES:	// acknowledge Set Features but do nothing
									cart_d5xx[IDE_STAT] = 0x50; // SKC and RDY
									break;
								case CMD_SET_RAM_BANK:	// open a RAM window in the cart ROM area
									LoadCartImage();
									break;
							}
						}
						if (addr == IDE_DATA) {	// write to data register
							if(WriteActive) {
								ATASectorBuffer[ATAByte++] = data;
								if(ATAByte == 512) WriteSector();	// if this is the last byte, deassert DRQ and write sector
							}
						}
						if (addr == IDE_CTRL) {	// write to reset register
							if(!data) {	// assert reset
							}
						}
					}
				}
			}
		}
//		while (CONTROL_IN & PHI2);
//		SET_DATA_MODE_IN
	}
}



void EmulateOSSA(char is034M) {
	uint16_t addr, data, c;
	uint32_t bank = 0;
	unsigned char *bankPtr;
	RD4_LOW
	RD5_HIGH
	while (!NewCart)
	{
		bankPtr = &cart_ram1[0] + (4096*bank);
		// wait for phi2 high
		while (!((c = CONTROL_IN) & PHI2)) ;
		addr = ADDR_IN;

		if (!(c & S5)) { // normal cart read
			SET_DATA_MODE_OUT
//			addr = ADDR_IN;
			if (addr & 0x1000)
				DATA_OUT = ((uint16_t)cart_ram1[addr|0x2000])<<8;	// 4k bank #3 always mapped to $Bxxx
			else
				DATA_OUT = ((uint16_t)(*(bankPtr+addr)))<<8;
			while (CONTROL_IN & PHI2);
			SET_DATA_MODE_IN
		}

		if (!(c & CCTL)) {	// CCTL access
			addr &= 0xff;
//			addr = ADDR_IN & 0xff;

			if(addr < IDE_DATA)	// handle OSS banking (R/W)
			{
				addr &= 0x0f;
				if (addr & 0x8) RD5_LOW
				else {
					RD5_HIGH
					if (addr == 0x0) bank = 0;
					if (addr == 0x3 || addr == 0x7) bank = is034M ? 1 : 2;
					if (addr == 0x4) bank = is034M ? 2 : 1;
//					bankPtr = &cart_ram1[0] + (4096*bank);
				}
//			while (CONTROL_IN & PHI2);
			}

			else	// we're addressing the IDE registers
			{
				if (c & RW)	{ // IDE read

					SET_DATA_MODE_OUT
					DATA_OUT = ((uint16_t)cart_d5xx[addr])<<8;
					while (CONTROL_IN & PHI2);
					SET_DATA_MODE_IN

					if(addr==IDE_DATA) {
						if (ATAByte<512) cart_d5xx[IDE_DATA] = ATASectorBuffer[ATAByte++];
						else {
							cart_d5xx[IDE_STAT] = 0x50;
							RED_LED_OFF
						}
					}
				}

				else {	// IDE write
					data = DATA_IN;
					// read data bus on falling edge of phi2
					while (CONTROL_IN & PHI2) data = DATA_IN;
					data >>= 8;
					if (addr>=IDE_DATA) {
						if (addr <= SIDE2_REMOVE_FLAG) cart_d5xx[addr] = data;	// don't allow writes to signature area
						if (addr == IDE_STAT) {	// we received an ATA command
							switch (data) {
								case ATA_CMD_READ:	// read a sector
									ReadSector();
									break;
								case ATA_CMD_WRITE:
									cart_d5xx[IDE_STAT] = 0x58;	// SKC, RDY and DRQ
									WriteActive = 1;	// we're now counting writes to the data register
									ATAByte = 0;
									break;
								case ATA_CMD_IDENTIFY_DEVICE:
									IdentifyDevice();
									break;
								case ATA_CMD_SET_FEATURES:	// acknowledge Set Features but do nothing
									cart_d5xx[IDE_STAT] = 0x50; // SKC and RDY
									break;
								case CMD_SET_RAM_BANK:	// open a RAM window in the cart ROM area
									LoadCartImage();
									break;
							}
						}
						if (addr == IDE_DATA) {	// write to data register
							if(WriteActive) {
								ATASectorBuffer[ATAByte++] = data;
								if(ATAByte == 512) WriteSector();	// if this is the last byte, deassert DRQ and write sector
							}
						}
						if (addr == IDE_CTRL) {	// write to reset register
							if(!data) {	// assert reset
							}
						}
					}
				}
			}
		}
//		while (CONTROL_IN & PHI2);
//		SET_DATA_MODE_IN
	}
}


/*

void EmulateOSS16K043M() {
	uint16_t addr, data, c;
	uint32_t bank = 0;
	unsigned char *bankPtr;
	while (!NewCart)
	{
		bankPtr = &cart_ram1[0] + (4096*bank);
		// wait for phi2 high
		while (!((c = CONTROL_IN) & PHI2)) ;
		addr = ADDR_IN;

		if (!(c & S5)) { // normal cart read
			SET_DATA_MODE_OUT
//			addr = ADDR_IN;
			if (addr & 0x1000)
				DATA_OUT = ((uint16_t)cart_ram1[addr|0x2000])<<8;	// 4k bank #3 always mapped to $Bxxx
			else
				DATA_OUT = ((uint16_t)(*(bankPtr+addr)))<<8;
			while (CONTROL_IN & PHI2);
			SET_DATA_MODE_IN
		}

		if (!(c & CCTL)) {	// CCTL access
			addr &= 0xff;
//			addr = ADDR_IN & 0xff;

			if(addr < IDE_DATA)	// handle OSS banking (R/W)
			{
				addr &= 0x0f;
				if (addr & 0x8) RD5_LOW
				else {
					RD5_HIGH
					if (addr == 0x0) bank = 0;
					if (addr == 0x3 || addr == 0x7) bank = 2;
					if (addr == 0x4) bank = 1;
//					bankPtr = &cart_ram1[0] + (4096*bank);
				}
//			while (CONTROL_IN & PHI2);
			}

			else	// we're addressing the IDE registers
			{
				if (c & RW)	{ // IDE read

					SET_DATA_MODE_OUT
					DATA_OUT = ((uint16_t)cart_d5xx[addr])<<8;
					while (CONTROL_IN & PHI2);
					SET_DATA_MODE_IN

					if(addr==IDE_DATA) {
						if (ATAByte<512) cart_d5xx[IDE_DATA] = ATASectorBuffer[ATAByte++];
						else {
							cart_d5xx[IDE_STAT] = 0x50;
							RED_LED_OFF
						}
					}
				}

				else {	// IDE write
					data = DATA_IN;
					// read data bus on falling edge of phi2
					while (CONTROL_IN & PHI2) data = DATA_IN;
					data >>= 8;
					if (addr>=IDE_DATA) {
						if (addr <= SIDE2_REMOVE_FLAG) cart_d5xx[addr] = data;	// don't allow writes to signature area
						if (addr == IDE_STAT) {	// we received an ATA command
							switch (data) {
								case ATA_CMD_READ:	// read a sector
									ReadSector();
									break;
								case ATA_CMD_WRITE:
									cart_d5xx[IDE_STAT] = 0x58;	// SKC, RDY and DRQ
									WriteActive = 1;	// we're now counting writes to the data register
									ATAByte = 0;
									break;
								case ATA_CMD_IDENTIFY_DEVICE:
									IdentifyDevice();
									break;
								case ATA_CMD_SET_FEATURES:	// acknowledge Set Features but do nothing
									cart_d5xx[IDE_STAT] = 0x50; // SKC and RDY
									break;
								case CMD_SET_RAM_BANK:	// open a RAM window in the cart ROM area
									LoadCartImage();
									break;
							}
						}
						if (addr == IDE_DATA) {	// write to data register
							if(WriteActive) {
								ATASectorBuffer[ATAByte++] = data;
								if(ATAByte == 512) WriteSector();	// if this is the last byte, deassert DRQ and write sector
							}
						}
						if (addr == IDE_CTRL) {	// write to reset register
							if(!data) {	// assert reset
							}
						}
					}
				}
			}
		}
//		while (CONTROL_IN & PHI2);
//		SET_DATA_MODE_IN
	}
}

*/



void EmulateSDX(int size) {
	RD4_LOW
	uint16_t addr, c;
	unsigned char *ramPtr = &cart_ram1[0];
	while (!NewCart)
	{
		// wait for phi2 high
		while (!((c = CONTROL_IN) & PHI2)) ;
		addr = ADDR_IN;

		if (!(c & S5)) { // normal cart read
			SET_DATA_MODE_OUT
//			addr = ADDR_IN;
			DATA_OUT = ((uint16_t)(*(ramPtr+addr)))<<8;
			SET_DATA_MODE_IN
		}

		if (!(c & CCTL)) {	// CCTL access
			addr &= 0xff;
//			addr = ADDR_IN & 0xff;

			if(addr < IDE_DATA)	// handle OSS banking (R/W)
			{
				addr &= 0x0f;
				if ((addr & 0xF0) == 0xE0) {
					// 64k & 128k versions
					if (size == 64) ramPtr = &cart_ram1[0]; else ramPtr = &cart_ram2[0];
					ramPtr += ((~addr) & 0x7) * 8192;
					if (addr & 0x8) RD5_LOW
					else RD5_HIGH
				}
				if (size == 128 && (addr & 0xF0) == 0xF0) {
					// 128k version only
					ramPtr = &cart_ram1[0] + ((~addr) & 0x7) * 8192;
					if (addr & 0x8) RD5_LOW
					else RD5_HIGH
				}
//				while (CONTROL_IN & PHI2);
			}

			else	// we're addressing the IDE registers
			{
				if (c & RW)	{ // IDE read

					SET_DATA_MODE_OUT
					DATA_OUT = ((uint16_t)cart_d5xx[addr])<<8;
					while (CONTROL_IN & PHI2);
					SET_DATA_MODE_IN

					if(addr==IDE_DATA) {
						if (ATAByte<512) cart_d5xx[IDE_DATA] = ATASectorBuffer[ATAByte++];
						else {
							cart_d5xx[IDE_STAT] = 0x50;
							RED_LED_OFF
						}
					}
				}

				else {	// IDE write
					data = DATA_IN;
					// read data bus on falling edge of phi2
					while (CONTROL_IN & PHI2) data = DATA_IN;
					data >>= 8;
					if (addr>=IDE_DATA) {
						if (addr <= SIDE2_REMOVE_FLAG) cart_d5xx[addr] = data;	// don't allow writes to signature area
						if (addr == IDE_STAT) {	// we received an ATA command
							switch (data) {
								case ATA_CMD_READ:	// read a sector
									ReadSector();
									break;
								case ATA_CMD_WRITE:
									cart_d5xx[IDE_STAT] = 0x58;	// SKC, RDY and DRQ
									WriteActive = 1;	// we're now counting writes to the data register
									ATAByte = 0;
									break;
								case ATA_CMD_IDENTIFY_DEVICE:
									IdentifyDevice();
									break;
								case ATA_CMD_SET_FEATURES:	// acknowledge Set Features but do nothing
									cart_d5xx[IDE_STAT] = 0x50; // SKC and RDY
									break;
								case CMD_SET_RAM_BANK:	// open a RAM window in the cart ROM area
									LoadCartImage();
									break;
							}
						}
						if (addr == IDE_DATA) {	// write to data register
							if(WriteActive) {
								ATASectorBuffer[ATAByte++] = data;
								if(ATAByte == 512) WriteSector();	// if this is the last byte, deassert DRQ and write sector
							}
						}
						if (addr == IDE_CTRL) {	// write to reset register
							if(!data) {	// assert reset
							}
						}
					}
				}
			}
		}
//		while (CONTROL_IN & PHI2);
//		SET_DATA_MODE_IN
	}
}




void EmulateAtariMax128K() {
	RD4_LOW
	RD5_HIGH
	uint16_t addr, c;
	uint32_t bank = 0;
	unsigned char *ramPtr;
	while (!NewCart)
	{
		if (bank & 0x8) ramPtr = &cart_ram2[0];
		else ramPtr = &cart_ram1[0];
		ramPtr += 8192 * (bank & 0x7);

		// wait for phi2 high
		while (!((c = CONTROL_IN) & PHI2)) ;
		addr = ADDR_IN;

		if (!(c & S5)) { // normal cart read
			SET_DATA_MODE_OUT
//			addr = ADDR_IN;
			DATA_OUT = ((uint16_t)(*(ramPtr+addr)))<<8;
			while (CONTROL_IN & PHI2);
			SET_DATA_MODE_IN
		}

		if (!(c & CCTL)) {	// CCTL access
			addr &= 0xff;
//			addr = ADDR_IN & 0xff;

			if(addr < IDE_DATA)	// handle OSS banking (R/W)
			{
				if ((addr & 0xE0) == 0) {
					bank = addr & 0x0f;
					if (addr & 0x10) RD5_LOW
					else RD5_HIGH
				}
//				while (CONTROL_IN & PHI2);
			}

			else	// we're addressing the IDE registers
			{
				if (c & RW)	{ // IDE read

					SET_DATA_MODE_OUT
					DATA_OUT = ((uint16_t)cart_d5xx[addr])<<8;
					while (CONTROL_IN & PHI2);
					SET_DATA_MODE_IN

					if(addr==IDE_DATA) {
						if (ATAByte<512) cart_d5xx[IDE_DATA] = ATASectorBuffer[ATAByte++];
						else {
							cart_d5xx[IDE_STAT] = 0x50;
							RED_LED_OFF
						}
					}
				}

				else {	// IDE write
					data = DATA_IN;
					// read data bus on falling edge of phi2
					while (CONTROL_IN & PHI2) data = DATA_IN;
					data >>= 8;
					if (addr>=IDE_DATA) {
						if (addr <= SIDE2_REMOVE_FLAG) cart_d5xx[addr] = data;	// don't allow writes to signature area
						if (addr == IDE_STAT) {	// we received an ATA command
							switch (data) {
								case ATA_CMD_READ:	// read a sector
									ReadSector();
									break;
								case ATA_CMD_WRITE:
									cart_d5xx[IDE_STAT] = 0x58;	// SKC, RDY and DRQ
									WriteActive = 1;	// we're now counting writes to the data register
									ATAByte = 0;
									break;
								case ATA_CMD_IDENTIFY_DEVICE:
									IdentifyDevice();
									break;
								case ATA_CMD_SET_FEATURES:	// acknowledge Set Features but do nothing
									cart_d5xx[IDE_STAT] = 0x50; // SKC and RDY
									break;
								case CMD_SET_RAM_BANK:	// open a RAM window in the cart ROM area
									LoadCartImage();
									break;
							}
						}
						if (addr == IDE_DATA) {	// write to data register
							if(WriteActive) {
								ATASectorBuffer[ATAByte++] = data;
								if(ATAByte == 512) WriteSector();	// if this is the last byte, deassert DRQ and write sector
							}
						}
						if (addr == IDE_CTRL) {	// write to reset register
							if(!data) {	// assert reset
							}
						}
					}
				}
			}
		}
//		while (CONTROL_IN & PHI2);
//		SET_DATA_MODE_IN
	}
}





void EmulateLoaderROM() {
	RD5_HIGH
	uint16_t addr, data, c;
	uint32_t bank = 0;
	unsigned char *bankPtr;
	while (!NewCart)
	{
		bankPtr = &loader_rom[0] + (8192*bank);
		// wait for phi2 high
		while (!((c = CONTROL_IN) & PHI2)) ;
		addr = ADDR_IN;

		if (!(c & S5)) {
			SET_DATA_MODE_OUT
			DATA_OUT = ((uint16_t)(*(bankPtr+addr)))<<8;
//			DATA_OUT = ((uint16_t)loader_rom[addr+(bank*8192)])<<8;
			while (CONTROL_IN & PHI2);
			SET_DATA_MODE_IN
		}

		if (!(c & CCTL)) {	// CCTL access
			addr &= 0xff;
//			addr = ADDR_IN & 0xff;

			if(addr == 0xe4)	// emulate SIDE cart banking
			{
				data = DATA_IN;
				while (CONTROL_IN & PHI2) data = DATA_IN;
				bank = data >>= 8;
				if (bank & 0x80) {
					bank = 0;
					RD5_LOW
				}
				else {
					RD5_HIGH
					bank&=1;
				}
			}

			else	// we're addressing the IDE registers
			{
				if (c & RW)	{ // IDE read

					SET_DATA_MODE_OUT
					DATA_OUT = ((uint16_t)cart_d5xx[addr])<<8;
					while (CONTROL_IN & PHI2);
					SET_DATA_MODE_IN

					if(addr==IDE_DATA) {
						if (ATAByte<512) cart_d5xx[IDE_DATA] = ATASectorBuffer[ATAByte++];
						else {
							cart_d5xx[IDE_STAT] = 0x50;
							RED_LED_OFF
						}
					}
				}

				else {	// IDE write
					data = DATA_IN;
					// read data bus on falling edge of phi2
					while (CONTROL_IN & PHI2) data = DATA_IN;
					data >>= 8;
					if (addr>=IDE_DATA) {
						if (addr <= SIDE2_REMOVE_FLAG) cart_d5xx[addr] = data;	// don't allow writes to signature area
						if (addr == IDE_STAT) {	// we received an ATA command
							switch (data) {
								case ATA_CMD_READ:	// read a sector
									ReadSector();
									break;
								case ATA_CMD_WRITE:
									cart_d5xx[IDE_STAT] = 0x58;	// SKC, RDY and DRQ
									WriteActive = 1;	// we're now counting writes to the data register
									ATAByte = 0;
									break;
								case ATA_CMD_IDENTIFY_DEVICE:
									IdentifyDevice();
									break;
								case ATA_CMD_SET_FEATURES:	// acknowledge Set Features but do nothing
									cart_d5xx[IDE_STAT] = 0x50; // SKC and RDY
									break;
								case CMD_SET_RAM_BANK:	// open a RAM window in the cart ROM area
									LoadCartImage();
									break;
							}
						}
						if (addr == IDE_DATA) {	// write to data register
							if(WriteActive) {
								ATASectorBuffer[ATAByte++] = data;
								if(ATAByte == 512) WriteSector();	// if this is the last byte, deassert DRQ and write sector
							}
						}
						if (addr == IDE_CTRL) {	// write to reset register
							if(!data) {	// assert reset
								cart_d5xx[IDE_STAT] = 0x50;
								RED_LED_OFF
							}
						}
					}
				}
			}
		}
//		while (CONTROL_IN & PHI2);
//		SET_DATA_MODE_IN
	}
}




void EmulateCarts() {
	__disable_irq();	// Disable interrupts
	RD5_LOW
	RD4_LOW
	CartType = NewCart = 0; // CART_TYPE_LOADER;
	while (1) {
		if (!CartType)
			EmulateNoCart();
		else if (CartType ==  CART_TYPE_8K)
			EmulateStandard8K();
		else if (CartType == CART_TYPE_16K)
			EmulateStandard16K();
		else if (CartType == CART_TYPE_OSS_8K)
			EmulateOSSB();
		else if (CartType == CART_TYPE_OSS_16K_034M)
			EmulateOSSA(1);
		else if (CartType == CART_TYPE_OSS_16K_043M)
			EmulateOSSA(0);
		else if (CartType == CART_TYPE_OSS_16K_TYPE_B)
			EmulateOSSB();
		else if (CartType == CART_TYPE_SDX_64K)
			EmulateSDX(64);
		else if (CartType == CART_TYPE_SDX_128K)
			EmulateSDX(128);
		else if (CartType == CART_TYPE_ATARIMAX_1MBIT)
			EmulateAtariMax128K();
		else if (CartType == CART_TYPE_LOADER)
			EmulateLoaderROM();
		/*		else if (CartType == CART_TYPE_WILLIAMS_64K)
			EmulateWilliams64K();
		else if (CartType == CART_TYPE_EXPRESS_64K)
			EmulateDiamondExpress(0x70);
		else if (CartType == CART_TYPE_DIAMOND_64K)
			EmulateDiamondExpress(0xd0);
		else if (CartType == CART_TYPE_XEGS_32K)
			EmulateXEGS32K(0);
		else if (CartType == CART_TYPE_XEGS_64K)
			EmulateXEGS64K(0);
		else if (CartType == CART_TYPE_XEGS_128K)
			EmulateXEGS128K(0);
		else if (CartType == CART_TYPE_BOUNTY_BOB)
			EmulateBountyBob();
		else if (CartType == CART_TYPE_WILLIAMS_64K)
			EmulateWilliams64K();
		else if (CartType == CART_TYPE_MEGACART_16K)
			EmulateMegaCart(16);
		else if (CartType == CART_TYPE_MEGACART_32K)
			EmulateMegaCart(32);
		else if (CartType == CART_TYPE_MEGACART_64K)
			EmulateMegaCart(64);
		else if (CartType == CART_TYPE_MEGACART_128K)
			EmulateMegaCart(128);
		else if (CartType == CART_TYPE_SW_XEGS_32K)
			EmulateXEGS32K(1);
		else if (CartType == CART_TYPE_SW_XEGS_64K)
			EmulateXEGS64K(1);
		else if (CartType == CART_TYPE_SW_XEGS_128K)
			EmulateXEGS128K(1);
		else if (CartType == CART_TYPE_BLIZZARD_16K)
			EmulateBlizzard16K();
		else if (CartType == CART_TYPE_SIC_128K)
			EmulateSIC128K(); */
		CartType = NewCart;
		NewCart = 0;
	}
}





int main(void) {
	/* Ouptut: LEDS - PB{0..1}, RD5 - PB2, RD4 - PB4 */
	config_gpio_leds_RD45();
	/* InOut: Data - PE{8..15} */
	config_gpio_data();
	/* In: Address - PD{0..15} */
	config_gpio_addr();
	/* In: Other Cart Input Sigs - PC{0..2, 4..5} */
	config_gpio_sig();

	cart_d5xx[0xfd] = 'I';
	cart_d5xx[0xfe] = 'D';
	cart_d5xx[0xff] = 'E';

	FATFS FatFs;
	TM_DELAY_Init();
//	if (f_mount(&FatFs, "", 1) == FR_OK)
	f_mount(&FatFs, "", 1);

	EmulateCarts();
}


