

#include "FT6236.h"
#include "STM_LOG.h"


static const char *TAG = "FT6236";

FT6236::FT6236(void) {touches = 0; }

bool FT6236::Init(I2C *i2c, uint8_t thresh){
	_i2c = i2c;

    writeRegister8(FT6236_REG_THRESHHOLD, thresh);
    uint8_t tmp = readRegister8(FT6236_REG_VENDID);
    STM_LOG(BOLD_WHITE, TAG, "FT6236: %d, line: %d, function: %s", tmp, __LINE__, __func__);
    if (tmp != FT6236_VENDID){
        return false;
    }
    uint8_t id = readRegister8(FT6236_REG_CHIPID);
    STM_LOG(BOLD_WHITE, TAG, "FT6236: %d, line: %d, function: %s", id, __LINE__, __func__);
    if ((id != FT6236_CHIPID) && (id != FT6236U_CHIPID) && (id != FT6206_CHIPID)){
        return false;
    }

    return true;
}

uint8_t FT6236::touched(void){
    uint8_t n = readRegister8(FT6236_REG_NUMTOUCHES);
    if (n > 2){
        n = 0;
    }
    return n;
}

TS_Point FT6236::getPoint(uint8_t n){
    readData();
    if ((touches == 0) || (n > 1)){
        return TS_Point(0, 0, 0);
    }
    else{
        return TS_Point(touchX[n], touchY[n], 1);
    }
}

void FT6236::readData(void){

    uint8_t i2cdat[16];
    _i2c -> SendStart();
    _i2c -> SendSlaveAddr((FT6236_ADDR << 1), I2C_WRITE);
    _i2c -> Transmit(0);
    _i2c -> SendStop();

    _i2c -> SendStart();
    _i2c -> SendSlaveAddr((FT6236_ADDR << 1), I2C_READ);
    _i2c -> Receive(i2cdat, 16);
    _i2c -> SendStop();

    touches = i2cdat[0x02];
    if ((touches > 2) || (touches == 0)){
        touches = 0;
    }

    for (uint8_t i = 0; i < 2; i++){
        touchX[i] = i2cdat[0x03 + i * 6] & 0x0F;
        touchX[i] <<= 8;
        touchX[i] |= i2cdat[0x04 + i * 6];
        touchY[i] = i2cdat[0x05 + i * 6] & 0x0F;
        touchY[i] <<= 8;
        touchY[i] |= i2cdat[0x06 + i * 6];
        touchID[i] = i2cdat[0x05 + i * 6] >> 4;
    }
}

uint8_t FT6236::readRegister8(uint8_t reg){
    uint8_t x[2];
    Result_t res = {
    	.Status = OKE,
    };

    res = _i2c -> SendStart();
    res = _i2c -> SendSlaveAddr((FT6236_ADDR << 1), I2C_WRITE);
    res = _i2c -> Transmit(reg);
    res = _i2c -> SendStop();

    res = _i2c -> SendStart();
    res = _i2c -> SendSlaveAddr((FT6236_ADDR << 1), I2C_READ);
    res = _i2c -> Receive(x, 1);
    res = _i2c -> SendStop();

    return x[0];
}

void FT6236::writeRegister8(uint8_t reg, uint8_t val){
    Result_t res = {
    	.Status = OKE,
    };
	uint8_t x[2] = {reg, val};
	res = _i2c -> SendStart();
	res = _i2c -> SendSlaveAddr((FT6236_ADDR << 1), I2C_WRITE);
	res = _i2c -> Transmit(x, 2);
	res = _i2c -> SendStop();
}

void FT6236::debug(void){
	STM_LOG(BOLD_WHITE, TAG, "Vend ID: 0x%02x", readRegister8(FT6236_REG_VENDID));
	STM_LOG(BOLD_WHITE, TAG, "Chip ID: 0x%02x", readRegister8(FT6236_REG_CHIPID));
	STM_LOG(BOLD_WHITE, TAG, "Firm V:  %02d", readRegister8(FT6236_REG_FIRMVERS));
	STM_LOG(BOLD_WHITE, TAG, "Point Rate Hz: %02d", readRegister8(FT6236_REG_POINTRATE));
	STM_LOG(BOLD_WHITE, TAG, "Thresh: %02d", readRegister8(FT6236_REG_THRESHHOLD));
}

TS_Point::TS_Point(void) { x = y = z = 0; }

TS_Point::TS_Point(int16_t _x, int16_t _y, int16_t _z){
    x = _x;
    y = _y;
    z = _z;
}

bool TS_Point::operator==(TS_Point p1){
    return ((p1.x == x) && (p1.y == y) && (p1.z == z));
}

bool TS_Point::operator!=(TS_Point p1){
    return ((p1.x != x) || (p1.y != y) || (p1.z != z));
}
