

/*
 * rc522deneme.c
 *
 *  Created on: Aug 16, 2025
 *      Author: Enes
 */

#include "rc522deneme.h"

static uint8_t _RC522SpiTransfer(RC522_t* dev, uint8_t txdata){
	uint8_t rxdata;
	HAL_SPI_TransmitReceive(dev->spi_handler, &txdata, &rxdata, 1, dev->max_delay);
	return rxdata;
}

static void _RC522WriteData(RC522_t* dev, uint8_t registeraddress ,uint8_t txdata){
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
	_RC522SpiTransfer(dev, (registeraddress<<1)&0x7E);
	_RC522SpiTransfer(dev, txdata);
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

static uint8_t _RC522ReadData(RC522_t* dev, uint8_t registeraddress){
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
	_RC522SpiTransfer(dev, ((registeraddress<<1)&0x7E)|0x80);
	uint8_t rxdata =_RC522SpiTransfer(dev, 0x00);
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
	return rxdata;
}

static void _RC522SetBitMask(RC522_t* dev, uint8_t registeraddress, uint8_t mask){
	uint8_t temp = _RC522ReadData(dev, registeraddress);
	_RC522WriteData(dev, registeraddress, temp | mask);
}

static void _RC522ClearBitMask(RC522_t* dev, uint8_t registeraddress, uint8_t mask){
	uint8_t temp = _RC522ReadData(dev, registeraddress);
	_RC522WriteData(dev, registeraddress, temp & (~mask));
}

static void _RC522AntennaOn(RC522_t* dev){
	_RC522SetBitMask(dev, REG_TxControlReg, 0x03);
}

static inline void _RC522AntennaOff(RC522_t* dev){
	_RC522ClearBitMask(dev, REG_TxControlReg, 0x03);
}

static void _RC522Reset(RC522_t* dev){
	_RC522WriteData(dev, REG_CommandReg, PCD_SOFTRESET);
}

bool _RC522_Init(RC522_t* dev, SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin, GPIO_TypeDef* rst_port, uint16_t rst_pin, uint32_t max_delay){
	dev->spi_handler=hspi;
	dev->rst_port=rst_port;
	dev->cs_port=cs_port;
	dev->cs_pin=cs_pin;
	dev->rst_pin=rst_pin;
	dev->max_delay=max_delay;
	HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(dev->rst_port, dev->rst_pin, GPIO_PIN_RESET); // 1. Adım: Pini LOW'a çek
	HAL_Delay(10);                                                 // 2. Adım: Kısa bir süre bekle (10ms yeterli)
	HAL_GPIO_WritePin(dev->rst_port, dev->rst_pin, GPIO_PIN_SET);  // 3. Adım: Pini HIGH'a bırak
	HAL_Delay(50);
	_RC522Reset(dev);
	_RC522WriteData(dev, REG_TModeReg, 0x8D);
	_RC522WriteData(dev, REG_TPrescalerReg, 0x3E);
	_RC522WriteData(dev, REG_TReloadReg_LSB, 30);
	_RC522WriteData(dev, REG_TReloadReg_MSB, 0);
	_RC522WriteData(dev, REG_TxASKReg, 0x40);
	_RC522WriteData(dev, REG_ModeReg, 0x3D);
	_RC522AntennaOn(dev);
	return true;
}

static uint8_t _RC522ToCard(RC522_t* dev, uint8_t command, uint8_t* senddata, uint8_t sendlen, uint8_t* backdata, uint32_t* backlen){
	uint8_t status = MI_ERROR;
	uint8_t n;
	uint8_t lastbits;
	uint8_t waitirq=0;
	uint8_t irqen=0;
	uint32_t i;
	switch(command){
		case PCD_MFAUTHENT:
		{
			irqen=0x12;
			waitirq=0x10;
			break;
		}
		case PCD_TRANSCEIVE:
		{
			irqen=0x77;
			waitirq=0x30;
			break;
		}
		default:
			break;
	}
	_RC522WriteData(dev, REG_ComlEnReg, irqen|0x80);
	_RC522ClearBitMask(dev, REG_ComIrqReg, 0x80);
	_RC522SetBitMask(dev, REG_FIFOLevelReg, 0x80);
	_RC522WriteData(dev, REG_CommandReg, PCD_IDLE);
	for(i=0;i<sendlen;i++){
		_RC522WriteData(dev, REG_FIFODataReg, senddata[i]);
	}
	_RC522WriteData(dev, REG_CommandReg, command);
	if(command==PCD_TRANSCEIVE){
		_RC522SetBitMask(dev, REG_BitFramingReg, 0x80);
	}
	i=2000;
	do
	{
		n=_RC522ReadData(dev, REG_ComIrqReg);
		i--;
	} while((i!=0) && !(n&0x01) && !(n&waitirq));
	_RC522ClearBitMask(dev, REG_BitFramingReg, 0x80);
	if(i!=0)
	{
		if(!(_RC522ReadData(dev, REG_ErrorReg) & 0x1B))
		{
			if(n&0x01&irqen)
			{
				status=MI_NOTAGERR;
				return status;
			}
			status=MI_OK;
			if(command==PCD_TRANSCEIVE){
				n=_RC522ReadData(dev, REG_FIFOLevelReg);
				lastbits=_RC522ReadData(dev, REG_ControlReg)&0x07;
				if(lastbits)
				{
					*backlen=(n-1)*8+lastbits;
				}
				else
				{
					*backlen=n*8;
				}
				if(n==0) { n=1; }
				if(n>16) { n=16;}
				for(i=0;i<n;i++){
					backdata[i]=_RC522ReadData(dev, REG_FIFODataReg);
				}
			}
		}
	}
	return status;
}

uint8_t _RC522_Request(RC522_t* dev, uint8_t reqmode, uint8_t* tagtype){
	uint32_t backlen;
	_RC522WriteData(dev, REG_BitFramingReg, 0x07);
	uint8_t status = _RC522ToCard(dev, PCD_TRANSCEIVE, &reqmode, 1, tagtype, &backlen);
	if((status!=MI_OK)||(backlen!=0x10))
	{
		status=MI_ERROR;
	}
	return status;
}

uint8_t _RC522_Anticoll(RC522_t* dev, uint8_t* psernum){
	uint8_t sernumcheck=0;
	uint32_t backlen_bits;
	uint8_t buffer[2];
	buffer[0]=PICC_ANTICOLL;
	buffer[1]=0x20;
	_RC522WriteData(dev, REG_BitFramingReg, 0x00);
	uint8_t status = _RC522ToCard(dev, PCD_TRANSCEIVE, buffer, 2, psernum, &backlen_bits);
	if((status==MI_OK) && backlen_bits==40){
		for(uint8_t i=0;i<4;i++){
			sernumcheck^=psernum[i];
		}
		if(sernumcheck!=psernum[4]){
			status=MI_ERROR;
		}
	}
	else
	{
		status=MI_ERROR;
	}
	return status;
}

void _RC522_CalculateCRC(RC522_t* dev, uint8_t* pin, uint8_t sendlen, uint8_t* pout){
	_RC522ClearBitMask(dev, REG_DivIrqReg, 0x04);
	_RC522SetBitMask(dev, REG_FIFOLevelReg, 0x80);
	uint8_t i,n;
	for(i=0;i<sendlen;i++){
		_RC522WriteData(dev, REG_FIFODataReg, pin[i]);
	}
	_RC522WriteData(dev, REG_CommandReg, PCD_CALC_CRC);
	i=255;
	do
	{
		n=_RC522ReadData(dev, REG_DivIrqReg);
		i--;
	} while((i!=0) && !(n&0x04));
	pout[0]=_RC522ReadData(dev, REG_CRCResultReg_LSB);
	pout[1]=_RC522ReadData(dev, REG_CRCResultReg_MSB);
}

uint8_t _RC522_SelectTag(RC522_t* dev, uint8_t* sernum){
	uint8_t i;
	uint8_t buffer[9];
	uint8_t size;
	uint8_t status;
	uint32_t receive_bits;
	buffer[0]=PICC_SElECTTAG;
	buffer[1]=0x70;
	for(i=0;i<5;i++){
		buffer[i+2]=sernum[i];
	}
	_RC522_CalculateCRC(dev, buffer, 7, &buffer[7]);
	status=_RC522ToCard(dev, PCD_TRANSCEIVE, buffer, 9, buffer, &receive_bits);
	if(status==MI_OK && receive_bits==24){
		size=buffer[0];
	}
	else
	{
		size=0;
	}
	return size;
}

uint8_t _RC522_Auth(RC522_t* dev, uint8_t authMode, uint8_t blockAddr, uint8_t* pKey, uint8_t* pSerNum){
	uint8_t status;
	uint8_t buffer[12];
	buffer[0]=authMode;
	buffer[1]=blockAddr;
	uint8_t i;
	uint8_t n;
	uint32_t backbits;
	for(i=0; i<6; i++){
		buffer[i+2]=pKey[i];
	}
	for(i=0; i<4; i++){
		buffer[i+8]=pSerNum[i];
	}
	status=_RC522ToCard(dev, PCD_MFAUTHENT, buffer, 12, buffer, &backbits);
	n=_RC522ReadData(dev, REG_Status2Reg) & 0x08; // MFRC522_MF_CRYPTO1_ON
	if(n!=8 || status!=MI_OK){
		status=MI_ERROR;
	}
	return status;
}

void _RC522_StopCrpyo1(RC522_t* dev){ // MFRC522_MF_CRYPTO1_OFF
	_RC522ClearBitMask(dev, REG_Status2Reg, 0x08);
}

uint8_t _RC522_Read(RC522_t* dev, uint8_t blockAddr, uint8_t* recvdata){
	uint8_t status;
	uint8_t buffer[4];
	uint32_t backlen;
	buffer[0]=PICC_READ;
	buffer[1]=blockAddr;
	_RC522_CalculateCRC(dev, buffer, 2, &buffer[2]); // buffer'ın 2. byte adresini yolluyorum çünkü crc hesaplaması 3. ve 4. byte'ta yer alsın
	status=_RC522ToCard(dev, PCD_TRANSCEIVE, buffer, 4, recvdata, &backlen);
	if(status!=MI_OK || backlen!=0x90){
		status=MI_ERROR;
	}
	return status;
}

uint8_t _RC522_Write(RC522_t* dev, uint8_t blockAddr, uint8_t* writedata){
	uint8_t status;
	uint8_t sendbuffer[18];
	uint8_t receivebuffer[4];
	uint32_t backlen;
	uint8_t i;
	sendbuffer[0]=PICC_WRITE;
	sendbuffer[1]=blockAddr;
	_RC522_CalculateCRC(dev, sendbuffer, 2, &sendbuffer[2]);
	status=_RC522ToCard(dev, PCD_TRANSCEIVE, sendbuffer, 4, receivebuffer, &backlen);
	if((receivebuffer[0] & 0x0F)!=0x0A || backlen!=4 || status!=MI_OK) // receivebuffer 4 bit olduğu için, ilk 4 biti 0'layarak işi garantiye almak istedim.
	{
		status=MI_ERROR;
		return status;
	}
	receivebuffer[0]=0; // burayı bilerek bırakıyorum. receive buffer'a sanki reset atmışım gibi düşünebilirsin. yoksa 2. if döngüsünde belki bir ihtimal 1. tocard'dan kalan sonuç sayesinde girebilir. önlem amaçlı yaptım yani.
	for(i=0; i<16; i++){
		sendbuffer[i]=writedata[i];
	}
	_RC522_CalculateCRC(dev, sendbuffer, 16, &sendbuffer[16]);
	status=_RC522ToCard(dev, PCD_TRANSCEIVE, sendbuffer, 18, receivebuffer, &backlen);
	if((receivebuffer[0] & 0x0F)!=0x0A || backlen!=4 || status!=MI_OK){
		status=MI_ERROR;
	}
	return status;
}

void _RC522_Halt(RC522_t* dev){
	uint8_t buffer[4]; // 5. byte geri dönüş değerini tutuyor. ne olur ne olmaz diye
	uint32_t backlen;
	buffer[0]=PICC_HALT;
	buffer[1]=0;
	_RC522_CalculateCRC(dev, buffer, 2, &buffer[2]);
	_RC522ToCard(dev, PCD_TRANSCEIVE, buffer, 4, &buffer[4], &backlen);
}
