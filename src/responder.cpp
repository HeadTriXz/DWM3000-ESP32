#include <DWM3000.h>
#include <iostream>
#include <bitset>

const int PIN_CSN = 5;
const int PIN_RST = 25;

/* Frames used in the ranging process. */
byte rxPollMsg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
byte txRespMsg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
byte rxFinalMsg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

uint32_t status;
byte frameSeq = 0;
byte rxBuffer[24];

uint64_t pollRXTimestamp;
uint64_t respTXTimestamp;
uint64_t finalRXTimestamp;

double tof;
double distance;

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
    dw.write16bitReg(DRX, DTUNE1, 0x00);
    dw.and16bitReg(GEN_CFG_AES_0, SYS_CFG, 0xFDFF);
    dw.startReceive(false);

    while (!((status = dw.read32bitReg(GEN_CFG_AES_0, SYS_STATUS)) & 0x2427D000)) {};

    if (status & 0x4000) {
        dw.write32bitReg(GEN_CFG_AES_0, SYS_STATUS, 0x4000);

        uint32_t frameLength = dw.read32bitReg(GEN_CFG_AES_0, RX_FINFO) & 0x3FF;
        if (frameLength <= 24) {
            dw.getReceiveData(frameLength, rxBuffer);
        }

        rxBuffer[2] = 0;
        if (memcmp(rxBuffer, rxPollMsg, 10) == 0) {
            uint32_t respTXTime;

            pollRXTimestamp = dw.getReceiveTimestamp();
            respTXTime = (pollRXTimestamp + 0x036D8168) >> 8;
            dw.setDelayedTime(respTXTime);

            dw.modify32bitReg(GEN_CFG_AES_1, ACK_RESP_T, 0xFFF00000, 500);
            dw.write32bitReg(GEN_CFG_AES_0, RX_FWTO, 220);
            dw.or16bitReg(GEN_CFG_AES_0, SYS_CFG, 0x200);
            dw.write16bitReg(DRX, DTUNE1, 5);

            txRespMsg[2] = frameSeq;
            dw.setTransmitData(sizeof(txRespMsg), txRespMsg, 1, 0);
            dw.startTransmit(true, true);

            while (!((status = dw.read32bitReg(GEN_CFG_AES_0, SYS_STATUS)) & 0x2427D000)) {};

            frameSeq++;
            if (status & 0x4000) {
                dw.write32bitReg(GEN_CFG_AES_0, SYS_STATUS, 0x4080);

                frameLength = dw.read32bitReg(GEN_CFG_AES_0, RX_FINFO) & 0x3FF;
                if (frameLength <= 24) {
                    dw.getReceiveData(frameLength, rxBuffer);
                }

                rxBuffer[2] = 0;
                if (memcmp(rxBuffer, rxFinalMsg, 10) == 0) {
                    uint32_t pollTXTimestamp, respRXTimestamp, finalTXTimestamp;
                    uint32_t pollRXTimestamp32, respTXTimestamp32, finalRXTimestamp32;
                    double Ra, Rb, Da, Db;
                    int64_t tofDtu;

                    respTXTimestamp = dw.getTransmitTimestamp();
                    finalRXTimestamp = dw.getReceiveTimestamp();

                    for (byte i = 0; i < 4; i++) {
                        (&rxBuffer[10])[i] = (uint8_t)pollTXTimestamp;
                        (&rxBuffer[14])[i] = (uint8_t)respRXTimestamp;
                        (&rxBuffer[18])[i] = (uint8_t)finalTXTimestamp;

                        pollTXTimestamp >>= 8;
                        respRXTimestamp >>= 8;
                        finalTXTimestamp >>= 8;
                    }

                    pollRXTimestamp32 = (uint32_t)pollRXTimestamp;
                    respTXTimestamp32 = (uint32_t)respTXTimestamp;
                    finalRXTimestamp32 = (uint32_t)finalRXTimestamp;
                    Ra = (double)(respRXTimestamp - pollTXTimestamp);
                    Rb = (double)(finalRXTimestamp32 - respTXTimestamp32);
                    Da = (double)(finalTXTimestamp - respRXTimestamp);
                    Db = (double)(respTXTimestamp32 - pollRXTimestamp32);

                    tofDtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

                    tof = tofDtu * DWT_TIME_UNITS;
                    distance = tof * SPEED_OF_LIGHT;

                    char *distStr;
                    sprintf(distStr, "DIST: %3.2f m", distance);
                    Serial.println(distStr);

                    delay(990);
                }
            } else {
                dw.write32bitReg(GEN_CFG_AES_0, SYS_STATUS, 0x24279000);
            }
        }
    } else {
        dw.write32bitReg(GEN_CFG_AES_0, SYS_STATUS, 0x24279000);
    }
}
