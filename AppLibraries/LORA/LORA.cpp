/*
 * LORA.cpp
 *
 *  Created on: Dec 8, 2022
 *      Author: anh
 */

#include "LORA.h"



LoRa::LoRa(GPIO_TypeDef *CSPort, uint16_t CSPin, GPIO_TypeDef *RSTPort, uint16_t RSTPin, GPIO_TypeDef *INTPort, uint16_t INTPin){
	_csport = CSPort;
	_rstport = RSTPort;
	_intport = INTPort;

	_cs = CSPin;
	_rst = RSTPin;
	_int = INTPin;
}

bool LoRa::Init(SPI<uint8_t> *spi, long frequency, uint8_t power, uint32_t interruptpriority){
	_spi = spi;

	GPIO_Init(_csport, _cs, GPIO_OUTPUT_PUSHPULL);
	GPIO_Set(_csport, _cs);

	GPIO_Init(_rstport, _rst, GPIO_OUTPUT_PUSHPULL);
	GPIO_Reset(_rstport, _rst);
    TickDelay_ms(50);
    GPIO_Set(_rstport, _rst);
    TickDelay_ms(50);

    EXTI_Init(_intport, _int, EXTI_RISING_EDGE, interruptpriority);
    GPIO_Pulldown(_intport, _int);

	uint8_t version = readRegister(REG_VERSION);
	if(version != 0x12) return false;

	sleep();

	setFrequency(frequency);

	writeRegister(REG_FIFO_TX_BASE_ADDR, 0);

	writeRegister(REG_FIFO_RX_BASE_ADDR, 0);

	writeRegister(REG_LNA, readRegister(REG_LNA) | 0x03);

	writeRegister(REG_MODEM_CONFIG_3, 0x04);

	setTxPower(power);

	enableCrc();

	idle();

	return true;
}

void LoRa::Event_Register_Handler(void (*TxHandler)(void *arg), void (*RxHandler)(void *arg, uint8_t len)){
	TxDoneHandler = TxHandler;
	RxDoneHandler = RxHandler;
}

bool LoRa::beginPacket(bool implicitHeader){
	if (isTransmitting()) return false;

	idle();

	if (implicitHeader) implicitHeaderMode();
	else explicitHeaderMode();

	writeRegister(REG_FIFO_ADDR_PTR, 0);
	writeRegister(REG_PAYLOAD_LENGTH, 0);

	return true;
}

bool LoRa::endPacket(bool async){
	if(async && (TxDoneHandler)) writeRegister(REG_DIO_MAPPING_1, 0x40);

	writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

	if(!async){
		while ((readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0);

		writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
	}

	return true;
}

bool LoRa::isTransmitting(void){
	if ((readRegister(REG_OP_MODE) & MODE_TX) == MODE_TX) return true;

	if (readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);

	return false;
}

uint8_t LoRa::parsePacket(uint8_t size){
	uint8_t packetLength = 0;
	uint8_t irqFlags = readRegister(REG_IRQ_FLAGS);

	if(size > 0) {
		implicitHeaderMode();
		writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
	}
	else
		explicitHeaderMode();

	writeRegister(REG_IRQ_FLAGS, irqFlags);

	if((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0){
		_packetIndex = 0;

		if (_implicitHeaderMode) packetLength = readRegister(REG_PAYLOAD_LENGTH);
		else packetLength = readRegister(REG_RX_NB_BYTES);

		writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

		idle();
	}
	else if(readRegister(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)){
		writeRegister(REG_FIFO_ADDR_PTR, 0);

		writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
	}

	return packetLength;
}

uint8_t LoRa::packetRssi(void){
	return (readRegister(REG_PKT_RSSI_VALUE) - (_frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

float LoRa::packetSnr(void){
	return ((int8_t)readRegister(REG_PKT_SNR_VALUE)) * 0.25;
}

long LoRa::packetFrequencyError(void){
	int32_t freqError = 0;
	freqError = static_cast<int32_t>(readRegister(REG_FREQ_ERROR_MSB) & 0b111);
	freqError <<= 8L;
	freqError += static_cast<int32_t>(readRegister(REG_FREQ_ERROR_MID));
	freqError <<= 8L;
	freqError += static_cast<int32_t>(readRegister(REG_FREQ_ERROR_LSB));

	if (readRegister(REG_FREQ_ERROR_MSB) & 0b1000) freqError -= 524288;

	const float fXtal = 32E6;
	const float fError = ((static_cast<float>(freqError) * (1L << 24)) / fXtal) * (getSignalBandwidth() / 500000.0f); // p. 37

	return static_cast<long>(fError);
}

int16_t LoRa::rssi(void){
    return (readRegister(REG_RSSI_VALUE) - (_frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

size_t LoRa::write(uint8_t byte){
  return write(&byte, sizeof(byte));
}

size_t LoRa::write(const uint8_t *buffer, size_t size){
	int currentLength = readRegister(REG_PAYLOAD_LENGTH);

	if ((currentLength + size) > MAX_PKT_LENGTH) size = MAX_PKT_LENGTH - currentLength;

	for (size_t i = 0; i < size; i++) writeRegister(REG_FIFO, buffer[i]);

	writeRegister(REG_PAYLOAD_LENGTH, currentLength + size);

	return size;
}

uint8_t LoRa::available(void){
	return (readRegister(REG_RX_NB_BYTES) - _packetIndex);
}

uint8_t LoRa::read(void){
	if(!available()) return -1;

	_packetIndex++;

	return readRegister(REG_FIFO);
}

uint8_t LoRa::peek(void){
	if(!available()) return -1;

	int currentAddress = readRegister(REG_FIFO_ADDR_PTR);

	uint8_t b = readRegister(REG_FIFO);

	writeRegister(REG_FIFO_ADDR_PTR, currentAddress);

	return b;
}

void LoRa::Receive(uint8_t size){
	if(RxDoneHandler)writeRegister(REG_DIO_MAPPING_1, 0x00); // DIO0 => RXDONE

	if (size > 0) {
		implicitHeaderMode();

		writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
	}
	else {
		explicitHeaderMode();
	}

	writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

void LoRa::idle(void){
	writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void LoRa::sleep(void){
	writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void LoRa::setTxPower(uint8_t level, uint8_t outputPin){
	if(PA_OUTPUT_RFO_PIN == outputPin) {
		if(level < 0) level = 0;
		else if(level > 14) level = 14;

		writeRegister(REG_PA_CONFIG, 0x70 | level);
	}
	else{
		if(level > 17){
			if (level > 20) level = 20;
			level -= 3;

			writeRegister(REG_PA_DAC, 0x87);
			setOCP(140);
		}
		else {
			if (level < 2) level = 2;
			writeRegister(REG_PA_DAC, 0x84);
			setOCP(100);
		}

		writeRegister(REG_PA_CONFIG, PA_BOOST | (level - 2));
	}
}

void LoRa::setFrequency(long frequency){
	_frequency = frequency;

	uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

	writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
	writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
	writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

uint8_t LoRa::getSpreadingFactor(void){
	return readRegister(REG_MODEM_CONFIG_2) >> 4;
}

void LoRa::setSpreadingFactor(uint8_t sf){
	if (sf < 6) sf = 6;
	else if (sf > 12) sf = 12;


	if (sf == 6) {
		writeRegister(REG_DETECTION_OPTIMIZE, 0xc5);
		writeRegister(REG_DETECTION_THRESHOLD, 0x0c);
	}
	else {
		writeRegister(REG_DETECTION_OPTIMIZE, 0xc3);
		writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
	}

	writeRegister(REG_MODEM_CONFIG_2, (readRegister(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
	setLdoFlag();
}

long LoRa::getSignalBandwidth(void){
	uint8_t bw = (readRegister(REG_MODEM_CONFIG_1) >> 4);

	switch (bw) {
		case 0: return 7.8E3;
		case 1: return 10.4E3;
		case 2: return 15.6E3;
		case 3: return 20.8E3;
		case 4: return 31.25E3;
		case 5: return 41.7E3;
		case 6: return 62.5E3;
		case 7: return 125E3;
		case 8: return 250E3;
		case 9: return 500E3;
	}

	return -1;
}

void LoRa::setSignalBandwidth(long sbw){
	int bw;

	if (sbw <= 7.8E3)        bw = 0;
	else if (sbw <= 10.4E3)  bw = 1;
	else if (sbw <= 15.6E3)  bw = 2;
	else if (sbw <= 20.8E3)  bw = 3;
	else if (sbw <= 31.25E3) bw = 4;
	else if (sbw <= 41.7E3)  bw = 5;
	else if (sbw <= 62.5E3)  bw = 6;
	else if (sbw <= 125E3)   bw = 7;
	else if (sbw <= 250E3)   bw = 8;
	else/*if (sbw <= 250E3)*/bw = 9;

	writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
	setLdoFlag();
}

void LoRa::setLdoFlag(void){
	long symbolDuration = 1000 / ( getSignalBandwidth() / (1L << getSpreadingFactor()) ) ;

	bool ldoOn = symbolDuration > 16;

	uint8_t config3 = readRegister(REG_MODEM_CONFIG_3);
	if(ldoOn) config3 |= (1<<3);
	else config3 &=~ (1<<3);
	writeRegister(REG_MODEM_CONFIG_3, config3);
}

void LoRa::setCodingRate4(uint8_t denominator){
	if (denominator < 5) denominator = 5;
	else if (denominator > 8) denominator = 8;

	uint8_t cr = denominator - 4;

	writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void LoRa::setPreambleLength(long length){
	writeRegister(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
	writeRegister(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

void LoRa::setSyncWord(uint8_t sw){
	writeRegister(REG_SYNC_WORD, sw);
}

void LoRa::enableCrc(void){
	writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) | 0x04);
}

void LoRa::disableCrc(void){
	writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) & 0xfb);
}

void LoRa::enableInvertIQ(void){
	writeRegister(REG_INVERTIQ,  0x66);
	writeRegister(REG_INVERTIQ2, 0x19);
}

void LoRa::disableInvertIQ(void){
	writeRegister(REG_INVERTIQ,  0x27);
	writeRegister(REG_INVERTIQ2, 0x1d);
}

void LoRa::setOCP(uint8_t mA){
	uint8_t ocpTrim = 27;

	if (mA <= 120) ocpTrim = (mA - 45) / 5;
	else if (mA <=240) ocpTrim = (mA + 30) / 10;

	writeRegister(REG_OCP, 0x20 | (0x1F & ocpTrim));
}

void LoRa::setGain(uint8_t gain){
	if (gain > 6) gain = 6;

	idle();

	if (gain == 0) writeRegister(REG_MODEM_CONFIG_3, 0x04);
	else {
		writeRegister(REG_MODEM_CONFIG_3, 0x00);

		writeRegister(REG_LNA, 0x03);

		writeRegister(REG_LNA, readRegister(REG_LNA) | (gain << 5));
	}
}

uint8_t LoRa::random(void){
	return readRegister(REG_RSSI_WIDEBAND);
}


void LoRa::explicitHeaderMode(void){
	_implicitHeaderMode = 0;

	writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) & 0xfe);
}

void LoRa::implicitHeaderMode(void){
	_implicitHeaderMode = 1;

	writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) | 0x01);
}

void LoRa::handleDio0Rise(void){
	uint8_t irqFlags = readRegister(REG_IRQ_FLAGS);

	writeRegister(REG_IRQ_FLAGS, irqFlags);

	if ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
		if ((irqFlags & IRQ_RX_DONE_MASK) != 0) {
			_packetIndex = 0;

			uint8_t packetLength = _implicitHeaderMode ? readRegister(REG_PAYLOAD_LENGTH) : readRegister(REG_RX_NB_BYTES);

			writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

			if (RxDoneHandler) {
				RxDoneHandler(this, packetLength);
			}
		}
		else if ((irqFlags & IRQ_TX_DONE_MASK) != 0) {
			if (TxDoneHandler) {
				TxDoneHandler(this);
			}
		}
	}
}

uint8_t LoRa::readRegister(uint8_t address){
	return singleTransfer(address & 0x7f, 0x00);
}

void LoRa::writeRegister(uint8_t address, uint8_t value){
	singleTransfer(address | 0x80, value);
}

uint8_t LoRa::singleTransfer(uint8_t address, uint8_t value){
  uint8_t response;

  GPIO_Reset(_csport, _cs);
  _spi -> Transmit(address);
  _spi -> Transmit_Receive(value, &response);

  GPIO_Set(_csport, _cs);

  return response;
}


