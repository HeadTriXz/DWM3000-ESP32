#include <DWM3000.h>
#include <bitset>

const SPISettings SPI_FAST = SPISettings(8000000, MSBFIRST, SPI_MODE0);
const SPISettings SPI_SLOW = SPISettings(2000000, MSBFIRST, SPI_MODE0);
SPISettings _SPISettings;

// DWM3000
DWM3000::DWM3000(int csn): csn(csn) {};

// Private
void DWM3000::writeToSPI(byte address, uint16_t offset, uint16_t length, byte* buffer, uint16_t mode) {
    byte header[2];
    byte headerLength = 0;

    uint16_t addr = (address << 9) | (offset << 2);

    header[0] = (byte)((mode | addr) >> 8);
    header[1] = (byte)(addr | (mode & 0x03));

    if (length == 0) {
        header[0] = (byte)(address << 1) | 0x81;
        headerLength = 1;
    } else if (offset == 0 && (mode == SPI_WR_BIT || mode == SPI_RD_BIT)) {
        headerLength = 1;
    } else {
        header[0] |= 0x40;
        headerLength = 2;
    }

    SPI.beginTransaction(_SPISettings);
    digitalWrite(csn, LOW);

    for (int i = 0; i < headerLength; i++) {
        SPI.transfer(header[i]);
    }

    switch (mode) {
        case SPI_AND_OR_8:
        case SPI_AND_OR_16:
        case SPI_AND_OR_32:
        case SPI_WR_BIT: {
            for (int i = 0; i < length; i++) {
                SPI.transfer(buffer[i]);
            }
            break;
        }
        case SPI_RD_BIT: {
            for (int i = 0; i < length; i++) {
                buffer[i] = SPI.transfer(0x00);
            }
            break;
        }
    }

    delayMicroseconds(5);
    digitalWrite(csn, HIGH);
    SPI.endTransaction();    
}

// Public
void DWM3000::and8bitReg(byte address, int offset, const uint8_t value) {
    modify8bitReg(address, offset, value, 0);
}

void DWM3000::and16bitReg(byte address, int offset, const uint16_t value) {
    modify16bitReg(address, offset, value, 0);
}

void DWM3000::and32bitReg(byte address, int offset, const uint32_t value) {
    modify32bitReg(address, offset, value, 0);
}

byte DWM3000::calcBandwidth(uint16_t targetCount, int channel) {
    write16bitReg(PMSC, CLK_CTRL, 0x1822);
    enableRFTX(channel, 0);
    enableRFTXBlocks(channel);

    write16bitReg(TX_CAL, PG_CAL_TARGET, targetCount & 0xFFF);
    or8bitReg(TX_CAL, PGC_CTRL, 0x03);
    while (read8bitReg(TX_CAL, PGC_CTRL) & 0x01);

    disableRFTXBlocks();
    disableRFTX(0);
    write16bitReg(PMSC, CLK_CTRL, 0x200);

    return (read8bitReg(RF_CONF, RF_TX_CTRL_2) & 0x3F);
}

void DWM3000::clearReceiveStatus() {
    write32bitReg(GEN_CFG_AES_0, SYS_STATUS, 0x2427FF00);
}

void DWM3000::clearTransmitStatus() {
    or8bitReg(GEN_CFG_AES_0, SYS_STATUS, 0xF8);
}

void DWM3000::configure(DWM3000Config *config) {
    byte mode = (config->phrMode == 0x01) ? 0x10 : 0;

    modify32bitReg(GEN_CFG_AES_0, SYS_CFG, 0xFFFC4FCF,
        (((uint32_t)config->phrRate << 5) & 0x20) | mode); // SYS_CFG

    // OTP_CFG
    uint16_t preambleLen;
    switch (config->txPreambLength) {
        case PLEN_32:
            preambleLen = 32;
            break;
        case PLEN_64:
            preambleLen = 64;
            break;
        case PLEN_72:
            preambleLen = 72;
            break;
        case PLEN_128:
            preambleLen = 128;
            break;
        default:
            preambleLen = 256;
            break;
    }

    if (preambleLen >= 256) {
        modify32bitReg(OTP_IF, OTP_CFG, 0xFFFFE7FF, 0x0400);
    } else {
        modify32bitReg(OTP_IF, OTP_CFG, 0xFFFFE7FF, 0x1400);
    }

    modify8bitReg(DRX, DTUNE0, 0xFC, config->rxPAC); // PAC
    write8bitReg(STS_CONFIG, STS_CFG, 0x03); // Disable STS
    and16bitReg(GEN_CFG_AES_0, TX_FCTRL_HI, 0x00FF); // Clear FINE_PLEN
    write32bitReg(DRX, DTUNE3, 0xAF5F584C); // Default DTUNE3

    // CHAN_CTRL
    modify32bitReg(GEN_CFG_AES_1, CHAN_CTRL, 0xFFFFE000, 
        (config->channel == 9) |                        // Channel 5 or 9
        (0x1F00 & ((uint32_t)config->rxCode << 8)) |    // RX code
        (0x00F8 & ((uint32_t)config->txCode << 3)) |    // TX code
        (0x0006 & ((uint32_t)config->sfdType << 1)));   // SFD type

    // TX_FCTRL
    modify32bitReg(GEN_CFG_AES_0, TX_FCTRL, 0xFFFF0BFF, 
        ((uint32_t)config->dataRate << 10) | 
        ((uint32_t)config->txPreambLength) << 12);
    
    // DTUNE (SFD timeout)
    if (!config->sfdTO) {
        config->sfdTO = 0x81;
    }

    write16bitReg(DRX, RX_SFD_TOC, config->sfdTO);

    if (config->channel == 9) {
        write32bitReg(RF_CONF, RF_TX_CTRL_2, 0x1C010034);
        write16bitReg(FS_CTRL, PLL_CFG, 0x0F3C);
        write32bitReg(RF_CONF, RF_RX_CTRL, 0x08B5A833);
    } else {
        write32bitReg(RF_CONF, RF_TX_CTRL_2, 0x1C071134);
        write16bitReg(FS_CTRL, PLL_CFG, 0x1F3C);
    }

    write8bitReg(RF_CONF, LDO_RLOAD, 0x14);
    write8bitReg(RF_CONF, RF_TX_CTRL_1, 0x0E);
    write8bitReg(FS_CTRL, PLL_CAL, 0x81);
    write8bitReg(GEN_CFG_AES_0, SYS_STATUS, 0x02);
    
    // Set state to IDLE
    write16bitReg(PMSC, CLK_CTRL, 0x200);
    or16bitReg(PMSC, SEQ_CTRL, 0x100);

    byte flag = 1;
    for (byte i = 0; i < 6; i++) {
        delayMicroseconds(20);
        if (read8bitReg(GEN_CFG_AES_0, SYS_STATUS) & 0x02) {
            flag = 0;
            break;
        }

        Serial.println("PLL not locked, retrying...");
    }

    if (flag) {
        Serial.println("Exceeded \"MAX_RETRIES_FOR_PLL\"");
        while(1) {}
    }

    if ((config->rxCode >= 9) && (config->rxCode <= 24)) {
        if (dgcOTP) {
            modify16bitReg(OTP_IF, OTP_CFG, 0xDFFF, 
                config->channel == 5 ? 0x0040 : 0x2040);
        } else {
            configurelut(config->channel);
        }

        modify16bitReg(RX_TUNE, DGC_CFG, 0x81FF, 0x6400);
    } else {
        and8bitReg(RX_TUNE, DGC_CFG, 0xFE);
    }

    pgfCal();
}

void DWM3000::configurelut(int channel) {
    if (channel == 5) {
        write32bitReg(RX_TUNE, DGC_LUT_0, (uint32_t)CH5_DGC_LUT_0);
        write32bitReg(RX_TUNE, DGC_LUT_1, (uint32_t)CH5_DGC_LUT_1);
        write32bitReg(RX_TUNE, DGC_LUT_2, (uint32_t)CH5_DGC_LUT_2);
        write32bitReg(RX_TUNE, DGC_LUT_3, (uint32_t)CH5_DGC_LUT_3);
        write32bitReg(RX_TUNE, DGC_LUT_4, (uint32_t)CH5_DGC_LUT_4);
        write32bitReg(RX_TUNE, DGC_LUT_5, (uint32_t)CH5_DGC_LUT_5);
        write32bitReg(RX_TUNE, DGC_LUT_6, (uint32_t)CH5_DGC_LUT_6);
    } else {
        write32bitReg(RX_TUNE, DGC_LUT_0, (uint32_t)CH9_DGC_LUT_0);
        write32bitReg(RX_TUNE, DGC_LUT_1, (uint32_t)CH9_DGC_LUT_1);
        write32bitReg(RX_TUNE, DGC_LUT_2, (uint32_t)CH9_DGC_LUT_2);
        write32bitReg(RX_TUNE, DGC_LUT_3, (uint32_t)CH9_DGC_LUT_3);
        write32bitReg(RX_TUNE, DGC_LUT_4, (uint32_t)CH9_DGC_LUT_4);
        write32bitReg(RX_TUNE, DGC_LUT_5, (uint32_t)CH9_DGC_LUT_5);
        write32bitReg(RX_TUNE, DGC_LUT_6, (uint32_t)CH9_DGC_LUT_6);
    }

    write32bitReg(RX_TUNE, DGC_CFG0, 0x10000240);
    write32bitReg(RX_TUNE, DGC_CFG1, 0x1B6DA489);
}

void DWM3000::configureRFTX(DWM3000TXConfig *config) {
    if (config->PGcount == 0) {
        write8bitReg(RF_CONF, RF_TX_CTRL_2, config->PGdly);
    } else {
        uint8_t channel = 5;
        if (read8bitReg(GEN_CFG_AES_1, CHAN_CTRL) & 0x01) {
            channel = 9;
        }
    
        calcBandwidth(config->PGcount, channel);
    }

    write32bitReg(GEN_CFG_AES_1, TX_POWER, config->power);
}

void DWM3000::disableRFTX(byte switchConfig) {
    write32bitReg(RF_CONF, LDO_CTRL, 0x00000000);
    write32bitReg(RF_CONF, RF_ENABLE, 0x00000000);

    if (switchConfig) {
        write32bitReg(RF_CONF, RF_SWITCH, 0x1C000000);
    }
}

void DWM3000::disableRFTXBlocks() {
    write32bitReg(RF_CONF, RF_CTRL_MASK, 0x00000000);
}

void DWM3000::enableClock(byte clock) {
    byte pmscCtrl[4];
    readReg(PMSC, CLK_CTRL, 2, pmscCtrl);

    pmscCtrl[0] &= 0xFC;
    pmscCtrl[0] |= clock;
    
	writeReg(PMSC, CLK_CTRL, 2, pmscCtrl);
    delay(5);
}

void DWM3000::enableRFTX(uint32_t channel, byte switchControl) {
    or32bitReg(RF_CONF, LDO_CTRL, 0x08000800);
    or32bitReg(RF_CONF, LDO_CTRL, 0x00600060);

    if (channel == 5) {
        or32bitReg(RF_CONF, RF_ENABLE, 0x02003C00);
    } else {
        or32bitReg(RF_CONF, RF_ENABLE, 0x02001C00);
    }

    if (switchControl) {
        write32bitReg(RF_CONF, RF_SWITCH, 0x01011100);
    }
}

void DWM3000::enableRFTXBlocks(uint32_t channel) {
    if (channel == 5) {
        or32bitReg(RF_CONF, RF_CTRL_MASK, 0x02003C00);
    } else {
        or32bitReg(RF_CONF, RF_CTRL_MASK, 0x02001C00);
    }
}

void DWM3000::endAccMemRead() {
    or16bitReg(PMSC, CLK_CTRL, 0x7FBF);
}

void DWM3000::getPrintableDevID(char msgBuffer[]) {
    byte data[4];
    readReg(GEN_CFG_AES_0, DEV_ID, 4, data);
	sprintf(msgBuffer, "%02X - model: %d, version: %d, revision: %d",
		(uint16_t)((data[3] << 8) | data[2]), data[1], (data[0] >> 4) & 0x0F, data[0] & 0x0F);
}

void DWM3000::getReceiveData(int length, byte* data) {
    readReg(RX_BUFFER_0, 0, length, data);
}

uint64_t DWM3000::getReceiveTimestamp() {
    return read64bitReg(GEN_CFG_AES_0, RX_TIME);
}

uint32_t DWM3000::getTimestamp() {
    return read32bitReg(GEN_CFG_AES_0, SYS_TIME);
}

uint64_t DWM3000::getTransmitTimestamp() {
    return read64bitReg(GEN_CFG_AES_0, TX_TIME);
}

void DWM3000::initialize(int rstPin) {
    _SPISettings = SPI_FAST;

    pinMode(csn, OUTPUT);
    digitalWrite(csn, HIGH);
    reset(rstPin);

    delay(2);
    while (!isIdleRC()) {
        delay(2);
    }

    // Validate device ID
    uint32_t devID = read32bitReg(GEN_CFG_AES_0, DEV_ID);
    if (devID != 0xDECA0302 && devID != 0xDECA0312) {
        return;
    }

    uint32_t ldoTuneLo = readOTP(0x04);
    uint32_t ldoTuneHi = readOTP(0x05);
    uint32_t biasTune = (readOTP(0x0A) >> 16) & 0x1F;
    if (ldoTuneLo && ldoTuneHi && biasTune) {
        or16bitReg(OTP_IF, OTP_CFG, 0x180);
        modify16bitReg(PMSC, BIAS_CTRL, 0xFFE0, biasTune);
    }

    dgcOTP = readOTP(0x20) == 0x10000240;

    byte fsXtalt = readOTP(0x1E) & 0x7F;
    if (!fsXtalt) {
        fsXtalt = 0x2E;
    }

    write8bitReg(FS_CTRL, XTAL, fsXtalt);
}

bool DWM3000::isIdleRC() {
    uint32_t val = read32bitReg(GEN_CFG_AES_0, SYS_STATUS);
    return (val & 0x01000000) == 0x01000000;
}

bool DWM3000::isReceiveDone() {
    uint16_t val = read16bitReg(GEN_CFG_AES_0, SYS_STATUS);
    return (val & 0x6000) == 0x6000;
}

bool DWM3000::isReceiveError() {
    uint32_t val = read32bitReg(GEN_CFG_AES_0, SYS_STATUS);
    return (val & 0x50000) | (val & 0x9000);
}

bool DWM3000::isReceiveTimestampAvailable() {
    uint16_t val = read16bitReg(GEN_CFG_AES_0, SYS_STATUS);
    return (val & 0x400) == 0x400;
}

bool DWM3000::isTransmitDone() {
    byte val = read8bitReg(GEN_CFG_AES_0, SYS_STATUS);
    return (val >> 7);
}

void DWM3000::modify8bitReg(byte address, int offset, const uint8_t _and, const uint8_t _or) {
    byte buffer[8];
    buffer[0] = (byte)_and;
    buffer[1] = (byte)_or;

    writeToSPI(address, offset, 8, buffer, SPI_AND_OR_8);
}

void DWM3000::modify16bitReg(byte address, int offset, const uint16_t _and, const uint16_t _or) {
    byte buffer[4];
    buffer[0] = (byte)_and;
    buffer[1] = (byte)(_and >> 8);

    buffer[2] = (byte)_or;
    buffer[3] = (byte)(_or >> 8);

    writeToSPI(address, offset, 4, buffer, SPI_AND_OR_16);
}

void DWM3000::modify32bitReg(byte address, int offset, const uint32_t _and, const uint32_t _or) {
    byte buffer[8];
    buffer[0] = (byte)_and;
    buffer[1] = (byte)(_and >> 8);
    buffer[2] = (byte)(_and >> 16);
    buffer[3] = (byte)(_and >> 24);

    buffer[4] = (byte)_or;
    buffer[5] = (byte)(_or >> 8);
    buffer[6] = (byte)(_or >> 16);
    buffer[7] = (byte)(_or >> 24);

    writeToSPI(address, offset, 8, buffer, SPI_AND_OR_32);
}

void DWM3000::or8bitReg(byte address, int offset, const uint8_t value) {
    modify8bitReg(address, offset, -1, value);
}

void DWM3000::or16bitReg(byte address, int offset, const uint16_t value) {
    modify16bitReg(address, offset, -1, value);
}

void DWM3000::or32bitReg(byte address, int offset, const uint32_t value) {
    modify32bitReg(address, offset, -1, value);
}

void DWM3000::pgfCal() {
    uint16_t temp = read16bitReg(RF_CONF, LDO_CTRL);
    or16bitReg(RF_CONF, LDO_CTRL, 0x105);

    write32bitReg(EXT_SYNC, RX_CAL, 0x20001);
    or8bitReg(EXT_SYNC, RX_CAL, 0x10);

    byte flag = 1;
    for (int i = 0; i < 3; i++) {
        delayMicroseconds(20);
        if (read8bitReg(EXT_SYNC, RX_CAL_STS) == 1) {
            flag = 0;
            break;
        }
    }

    if (flag) {
        Serial.println("Exceeded \"MAX_RETRIES_FOR_PGF\"");
        while(1) {}
    }

    write8bitReg(EXT_SYNC, RX_CAL, 0);
    write8bitReg(EXT_SYNC, RX_CAL_STS, 1);
    or32bitReg(EXT_SYNC, RX_CAL, 0x10000);

    // Throw error if failed
    uint32_t val = read32bitReg(EXT_SYNC, RX_CAL_RESI);
    if (val == 0x1FFFFFFF) {
        Serial.println("RX calibration result I has failed");
        while(1) {}
    }

    val = read32bitReg(EXT_SYNC, RX_CAL_RESQ);
    if (val == 0x1FFFFFFF) {
        Serial.println("RX calibration result Q has failed");
        while(1) {}
    }

    and16bitReg(RF_CONF, LDO_CTRL, temp);
}

void DWM3000::readAccMem(int offset, int length, byte* buffer) {
    readReg(ACC_MEM, offset, length, buffer);
}

uint32_t DWM3000::readOTP(uint16_t address) {
    write16bitReg(OTP_IF, OTP_CFG, 0x0001);
    write16bitReg(OTP_IF, OTP_ADDR, address);
    write16bitReg(OTP_IF, OTP_CFG, 0x0002);

    return read32bitReg(OTP_IF, OTP_RDATA);
}

void DWM3000::readReg(byte address, int offset, int length, byte* buffer) {
    writeToSPI(address, offset, length, buffer, SPI_RD_BIT);
}

uint8_t DWM3000::read8bitReg(byte address, int offset) {
    uint8_t value;
    readReg(address, offset, 1, &value);

    return value;
}

uint16_t DWM3000::read16bitReg(byte address, int offset) {
    byte buffer[2];

    readReg(address, offset, 2, buffer);
    return (buffer[1] << 8) + buffer[0];
}

uint32_t DWM3000::read32bitReg(byte address, int offset) {
    uint32_t value = 0;
    byte buffer[4];

    readReg(address, offset, 4, buffer);
    for (int i = 3; i >= 0; i--) {
        value = (value << 8) + buffer[i];
    }

    return value;
}

uint64_t DWM3000::read64bitReg(byte address, int offset) {
    uint64_t value = 0;
    byte buffer[8];

    readReg(address, offset, 8, buffer);
    for (int i = 7; i >= 0; i--) {
        value = (value << 8) + buffer[i];
    }

    return value;
}

void DWM3000::reset(int rstPin) {
    digitalWrite(rstPin, LOW);
    pinMode(rstPin, OUTPUT_OPEN_DRAIN);
    delay(2);
    digitalWrite(rstPin, HIGH);
    delay(5);
}

void DWM3000::resetReceiver() {
    // Switch to SYS_FAST_RC_4_CLOCK
    enableClock(SYS_FAST_RC_4_CLOCK);

    // Soft-reset receiver
    write8bitReg(PMSC, SOFT_RST, 0xEF);
    write8bitReg(PMSC, SOFT_RST, 0xFF);
}

void DWM3000::setDelayedTime(uint32_t delayedTime) {
    write32bitReg(GEN_CFG_AES_0, DX_TIME, delayedTime);
}

void DWM3000::setTransmitData(int length, byte* buffer, int ranging, int fcs) {
    writeReg(TX_BUFFER, 0, length, buffer);
    if (fcs) {
        length += 2;
    }

    modify32bitReg(GEN_CFG_AES_0, TX_FCTRL_HI, 0xFC00F400, length | ranging << 11);
}

void DWM3000::setPAN(uint16_t panID, uint16_t shortAddr) {
    uint32_t value = (panID << 16) | shortAddr;
    write32bitReg(GEN_CFG_AES_0, PANADR, value);
}

void DWM3000::startAccMemRead() {
    or16bitReg(PMSC, CLK_CTRL, 0x8040);
}

void DWM3000::startReceive(bool delayed) {
    if (delayed) {
        return writeFastCMD(CMD_DRX);
    }

    writeFastCMD(CMD_RX);
}

void DWM3000::startTransmit(bool delayed, bool wait4resp) {
    if (delayed) {
        if (wait4resp) {
            return writeFastCMD(CMD_DTX_W4R);
        }

        return writeFastCMD(CMD_DTX);
    } else if (wait4resp) {
        return writeFastCMD(CMD_TX_W4R);
    }

    writeFastCMD(CMD_TX);
}

void DWM3000::writeFastCMD(int cmd) {
    writeReg(cmd, 0, 0, 0);
}

void DWM3000::writeReg(byte address, int offset, int length, byte* buffer) {
    writeToSPI(address, offset, length, buffer, SPI_WR_BIT);
}

void DWM3000::write8bitReg(byte address, int offset, uint8_t value) {
    writeReg(address, offset, 1, &value);
}

void DWM3000::write16bitReg(byte address, int offset, uint16_t value) {
    uint8_t buffer[2];
    buffer[0] = (uint8_t)value;
    buffer[1] = (uint8_t)(value >> 8);

    writeReg(address, offset, 2, buffer);
}

void DWM3000::write32bitReg(byte address, int offset, uint32_t value) {
    uint8_t buffer[4];
    buffer[0] = (uint8_t)value;
    buffer[1] = (uint8_t)(value >> 8);
    buffer[2] = (uint8_t)(value >> 16);
    buffer[3] = (uint8_t)(value >> 24);

    writeReg(address, offset, 4, buffer);
}
