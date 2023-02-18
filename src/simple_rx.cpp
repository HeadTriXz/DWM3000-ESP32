#include <DWM3000.h>
#include <iostream>
#include <bitset>

const int PIN_CSN = 5;
const int PIN_RST = 25;

uint16_t frameLen;
uint32_t status;
byte rxBuffer[127];

DWM3000 dw(PIN_CSN);
DWM3000Config config = {
    5,             /* Channel number. */
    PLEN_256,      /* Preamble length. Used in TX only. */
    PAC16,         /* Preamble acquisition chunk size. Used in RX only. */
    9,             /* TX preamble code. Used in TX only. */
    9,             /* RX preamble code. Used in RX only. */
    1,             /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    BR_850K,       /* Data rate. */
    PHRMODE_STD,   /* PHY header mode. */
    PHRRATE_STD,   /* PHY header rate. */
    (257 + 8 - 16) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

void setup() {
    Serial.begin(9600);
    SPI.begin();

    dw.initialize(PIN_RST);
    dw.configure(&config);

    // LED
    dw.modify32bitReg(GPIO_CTRL, GPIO_MODE, 0xFFFFF03F, 0x240);
    dw.or32bitReg(PMSC, CLK_CTRL, 0x840000);
    dw.write32bitReg(PMSC, 0x16, 0xF0110);
    dw.write32bitReg(PMSC, 0x16, 0x00110);

    // Print DEV ID
    char devString[64];
    dw.getPrintableDevID(devString);
    Serial.println(devString);
}

void loop() {
    memset(rxBuffer, 0, sizeof(rxBuffer));
    dw.startReceive(false);

    while (!((status = dw.read32bitReg(GEN_CFG_AES_0, SYS_STATUS)) & 0x2405D000)) {};

    if (status & 0x4000) {
        frameLen = dw.read32bitReg(GEN_CFG_AES_0, RX_FINFO) & 0x3FF;
        if (frameLen <= 127) {
            dw.getReceiveData(frameLen - 2, rxBuffer);
        }

        dw.write32bitReg(GEN_CFG_AES_0, SYS_STATUS, 0x4000);
    } else {
        dw.write32bitReg(GEN_CFG_AES_0, SYS_STATUS, 0x24059000);
    }
}
