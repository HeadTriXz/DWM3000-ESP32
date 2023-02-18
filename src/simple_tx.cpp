#include <DWM3000.h>
#include <iostream>
#include <bitset>

const int PIN_CSN = 5;
const int PIN_RST = 25;

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

DWM3000TXConfig txConfig = {
    0x34,       /* PGdly value */
    0xFFFFFFFF, /* Power value */
    0x00        /* PG count */
};

static byte txMsg[10] = {0xC5, 0, 'D', 'E', 'C', 'A', 'W', 'A', 'V', 'E'};

#define BLINK_FRAME_SN_IDX 1
#define FRAME_LENGTH 10
#define TX_DELAY_MS 500

void setup() {
    Serial.begin(9600);
    SPI.begin();

    dw.initialize(PIN_RST);
    dw.configure(&config);
    dw.configureRFTX(&txConfig);

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
    dw.setTransmitData(FRAME_LENGTH, txMsg, false);
    dw.startTransmit(false, false);

    while (!(dw.read32bitReg(GEN_CFG_AES_0, SYS_STATUS) & 0x80)) {};

    dw.write32bitReg(GEN_CFG_AES_0, SYS_STATUS, 0x80);

    delay(TX_DELAY_MS);
    txMsg[BLINK_FRAME_SN_IDX]++;
}
