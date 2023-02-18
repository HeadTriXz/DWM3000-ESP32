#include <DWM3000.h>
#include <iostream>
#include <bitset>

const int PIN_CSN = 5;
const int PIN_RST = 25;

/* Frames used in the ranging process. */
byte txPollMsg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21};
byte rxRespMsg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0};
byte txFinalMsg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

uint32_t status;
byte frameSeq = 0;
byte rxBuffer[20];

uint64_t finalTXTimestamp;
uint64_t pollTXTimestamp;
uint64_t respRXTimestamp;

DWM3000 dw(PIN_CSN);
DWM3000Config config = {
    5,             /* Channel number. */
    PLEN_128,      /* Preamble length. Used in TX only. */
    PAC8,          /* Preamble acquisition chunk size. Used in RX only. */
    9,             /* TX preamble code. Used in TX only. */
    9,             /* RX preamble code. Used in RX only. */
    1,             /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    BR_6M8,        /* Data rate. */
    PHRMODE_STD,   /* PHY header mode. */
    PHRRATE_STD,   /* PHY header rate. */
    (128 + 8 - 8)  /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

DWM3000TXConfig rftxConfig = {
    0x34,       /* PGdly value */
    0xFFFFFFFF, /* Power value */
    0x00        /* PG count */
};

void setup() {
    Serial.begin(9600);
    SPI.begin();

    dw.initialize(PIN_RST);
    dw.configure(&config);
    dw.configureRFTX(&rftxConfig);

    dw.write16bitReg(CIA_2, CIA_CONF, 0x4001);
    dw.write16bitReg(GEN_CFG_AES_1, TX_ANTD, 0x4001);

    dw.modify32bitReg(GEN_CFG_AES_1, ACK_RESP_T, 0xFFF00000, 700);
    dw.write32bitReg(GEN_CFG_AES_0, RX_FWTO, 300);
    dw.or16bitReg(GEN_CFG_AES_0, SYS_CFG, 0x200);
    dw.write16bitReg(DRX, DTUNE1, 5);

    dw.modify32bitReg(GPIO_CTRL, GPIO_MODE, 0xFFE00FC0, 0x49000);

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
    txPollMsg[2] = frameSeq;
    dw.setTransmitData(sizeof(txPollMsg), txPollMsg, 1, 1);
    dw.startTransmit(false, true);

    while (!((status = dw.read32bitReg(GEN_CFG_AES_0, SYS_STATUS)) & 0x2427D000)) {};

    frameSeq++;
    if (status & 0x4000) {
        dw.write32bitReg(GEN_CFG_AES_0, SYS_STATUS, 0x4080);

        uint32_t frameLength = dw.read32bitReg(GEN_CFG_AES_0, RX_FINFO) & 0x3FF;
        if (frameLength <= 20) {
            dw.getReceiveData(frameLength, rxBuffer);
        }

        rxBuffer[2] = 0;
        if (memcmp(rxBuffer, rxRespMsg, 10) == 0) {
            uint32_t finalTXTime;

            pollTXTimestamp = dw.getTransmitTimestamp();
            respRXTimestamp = dw.getReceiveTimestamp();
            finalTXTime = (respRXTimestamp + 0x02AA8118) >> 8;
            dw.setDelayedTime(finalTXTime);

            /* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
            finalTXTimestamp = (((uint64_t)(finalTXTime & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

            for (byte i = 0; i < 4; i++) {
                (&txFinalMsg[10])[i] = (byte)pollTXTimestamp;
                (&txFinalMsg[14])[i] = (byte)respRXTimestamp;
                (&txFinalMsg[18])[i] = (byte)finalTXTimestamp;

                pollTXTimestamp >>= 8;
                respRXTimestamp >>= 8;
                finalTXTimestamp >>= 8;
            }

            txFinalMsg[2] = frameSeq;
            dw.setTransmitData(sizeof(txFinalMsg), txFinalMsg, 1, 1);
            dw.startTransmit(true, false);

            while (!dw.isTransmitDone()) {};
            dw.write32bitReg(GEN_CFG_AES_0, SYS_STATUS, 0x80);
            frameSeq++;
        }
    } else {
        dw.write32bitReg(GEN_CFG_AES_0, SYS_STATUS, 0x24279080);
    }

    delay(1000);
}
