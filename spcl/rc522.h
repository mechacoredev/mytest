/*
 * rc522deneme.h
 *
 *  Created on: Aug 16, 2025
 *      Author: Enes
 */

#ifndef INC_RC522DENEME_H_
#define INC_RC522DENEME_H_

#include "stm32f4xx_hal.h"
#include "stdbool.h"

// Commands sent to the PICC.
#define PICC_REQIDL           0x26               // REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
#define PICC_REQALL           0x52               // Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
#define PICC_ANTICOLL         0x93               // Anti collision/Select, Cascade Level 1
#define PICC_SElECTTAG        0x93               // Anti collision/Select, Cascade Level 2
#define PICC_AUTHENT1A        0x60               // Perform authentication with Key A
#define PICC_AUTHENT1B        0x61               // Perform authentication with Key B
#define PICC_READ             0x30               // Reads one 16 byte block from the authenticated sector of the PICC. Also used for MIFARE Ultralight.
#define PICC_WRITE            0xA0               // Writes one 16 byte block to the authenticated sector of the PICC. Called "COMPATIBILITY WRITE" for MIFARE Ultralight.
#define PICC_DECREMENT        0xC0               // Decrements the contents of a block and stores the result in the internal data register.
#define PICC_INCREMENT        0xC1               // Increments the contents of a block and stores the result in the internal data register
#define PICC_RESTORE          0xC2               // Reads the contents of a block into the internal data register.
#define PICC_TRANSFER         0xB0               // Writes the contents of the internal data register to a block.
#define PICC_HALT             0x50               // HaLT command, Type A. Instructs an ACTIVE PICC to go to state HALT.

typedef enum{
	MI_OK=0,
	MI_NOTAGERR,
	MI_ERROR,
}RC522_Return_Status_t;

typedef enum{
	PCD_IDLE=0,
	PCD_MEM,
	PCD_RNG,
	PCD_CALC_CRC,
	PCD_TRANSMIT,
	PCD_NO_CMD_CHANGE=7,
	PCD_RECEIVE,
	PCD_TRANSCEIVE=12,
	PCD_MFAUTHENT=14,
	PCD_SOFTRESET=15,
}RC522_Commands_t;

typedef enum{
	// Page 0: Command and status
	REG_Reserved0=0,
	REG_CommandReg,
	REG_ComlEnReg,
	REG_DivlEnReg,
	REG_ComIrqReg,
	REG_DivIrqReg,
	REG_ErrorReg,
	REG_Status1Reg,
	REG_Status2Reg,
	REG_FIFODataReg,
	REG_FIFOLevelReg,
	REG_WaterLevelReg,
	REG_ControlReg,
	REG_BitFramingReg,
	REG_CollReg,
	REG_Reserved1,
	// Page 1: Command
	REG_Reserved2,
	REG_ModeReg,
	REG_TxModeReg,
	REG_RxModeReg,
	REG_TxControlReg,
	REG_TxASKReg,
	REG_TxSelReg,
	REG_RxSelReg,
	REG_RxThresholdReg,
	REG_DemodReg,
	REG_Reserved3,
	REG_Reserved4,
	REG_MfTxReg,
	REG_MfRxReg,
	REG_Reserved5,
	REG_SerialSpeedReg,
	// Page 2: Configuration
	REG_Reserved6,
	REG_CRCResultReg_MSB,
	REG_CRCResultReg_LSB,
	REG_Reserved7,
	REG_ModWidthReg,
	REG_Reserved8,
	REG_RFCfgReg,
	REG_GsNReg,
	REG_CWGsPReg,
	REG_ModGsPReg,
	REG_TModeReg,
	REG_TPrescalerReg,
	REG_TReloadReg_MSB,
	REG_TReloadReg_LSB,
	REG_TCounterValReg_MSB,
	REG_TCounterValReg_LSB,
	// Page 3: Test register
	REG_Reserved9,
	REG_TestSel1Reg,
	REG_TestSel2Reg,
	REG_TestPinEnReg,
	REG_TestPinValueReg,
	REG_TestBusReg,
	REG_AutoTestReg,
	REG_VersionReg,
	REG_AnalogTestReg,
	REG_TestDAC1Reg,
	REG_TestDAC2Reg,
	REG_TestADCReg
}RC522_Register_Addr_t;

typedef struct{
	SPI_HandleTypeDef* spi_handler;
	GPIO_TypeDef* rst_port;
	GPIO_TypeDef* cs_port;
	uint16_t rst_pin;
	uint16_t cs_pin;
	uint32_t max_delay;
}RC522_t;

bool _RC522_Init(RC522_t* dev, SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin, GPIO_TypeDef* rst_port, uint16_t rst_pin, uint32_t max_delay);

uint8_t _RC522_Request(RC522_t* dev, uint8_t reqmode, uint8_t* tagtype);

uint8_t _RC522_Anticoll(RC522_t* dev, uint8_t* psernum);

void _RC522_CalculateCRC(RC522_t* dev, uint8_t* pin, uint8_t sendlen, uint8_t* pout);

uint8_t _RC522_SelectTag(RC522_t* dev, uint8_t* sernum);

uint8_t _RC522_Auth(RC522_t* dev, uint8_t authMode, uint8_t blockAddr, uint8_t* pKey, uint8_t* pSerNum);

uint8_t _RC522_Read(RC522_t* dev, uint8_t blockAddr, uint8_t* recvdata);

uint8_t _RC522_Write(RC522_t* dev, uint8_t blockAddr, uint8_t* writedata);

void _RC522_Halt(RC522_t* dev);

void _RC522_StopCrpyo1(RC522_t* dev);

#endif /* INC_RC522DENEME_H_ */

