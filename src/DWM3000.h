#ifndef DWM3000_H
#define DWM3000_H

#include <Arduino.h>
#include <SPI.h>

typedef struct {
    uint8_t channel;                // Channel number (5 or 9)
    uint8_t txPreambLength;         // DWT_PLEN_64..DWT_PLEN_4096
    uint8_t rxPAC;                  // Acquisition Chunk Size (Relates to RX preamble length)
    uint8_t txCode;                 // TX preamble code (the code configures the PRF, e.g. 9 -> PRF of 64 MHz)
    uint8_t rxCode;                 // RX preamble code (the code configures the PRF, e.g. 9 -> PRF of 64 MHz)
    uint8_t sfdType;                // SFD type (0 for short IEEE 8-bit standard, 1 for DW 8-bit, 2 for DW 16-bit, 3 for 4z BPRF)
    uint8_t dataRate;               // Data rate {DWT_BR_850K or DWT_BR_6M8}
    uint8_t phrMode;                // PHR mode {0x0 - standard DWT_PHRMODE_STD, 0x3 - extended frames DWT_PHRMODE_EXT}
    uint8_t phrRate;                // PHR rate {0x0 - standard DWT_PHRRATE_STD, 0x1 - at datarate DWT_PHRRATE_DTA}
    uint16_t sfdTO;                 // SFD timeout value (in symbols)
} DWM3000Config;

typedef enum {
    CH5_DGC_LUT_0 = 0x1C0FD,
    CH5_DGC_LUT_1 = 0x1C43E,
    CH5_DGC_LUT_2 = 0x1C6BE,
    CH5_DGC_LUT_3 = 0x1C77E,
    CH5_DGC_LUT_4 = 0x1CF36,
    CH5_DGC_LUT_5 = 0x1CFB5,
    CH5_DGC_LUT_6 = 0x1CFF5
} DWM3000lutCh5;

typedef enum {
    CH9_DGC_LUT_0 = 0x2A8FE,
    CH9_DGC_LUT_1 = 0x2AC36,
    CH9_DGC_LUT_2 = 0x2A5FE,
    CH9_DGC_LUT_3 = 0x2AF3E,
    CH9_DGC_LUT_4 = 0x2AF7D,
    CH9_DGC_LUT_5 = 0x2AFB5,
    CH9_DGC_LUT_6 = 0x2AFB5
} DWM3000lutCh9;

typedef struct {
    uint8_t   PGdly;
    uint32_t  power;
    uint16_t  PGcount;
} DWM3000TXConfig;

// Main class
class DWM3000 {
    private:
        int csn;
        bool dgcOTP;
        void writeToSPI(byte address, uint16_t offset, uint16_t length, byte* buffer, uint16_t mode);

    public:
        DWM3000(int csn);

        void and8bitReg(byte address, int offset, const uint8_t value);
        void and16bitReg(byte address, int offset, const uint16_t value);
        void and32bitReg(byte address, int offset, const uint32_t value);

        byte calcBandwidth(uint16_t targetCount, int channel);
        void clearReceiveStatus();
        void clearTransmitStatus();
        void configure(DWM3000Config *config);
        void configurelut(int channel);
        void configureRFTX(DWM3000TXConfig *config);
        void disableRFTX(byte switchConfig);
        void disableRFTXBlocks();
        void enableClock(byte clock);
        void enableRFTX(uint32_t channel, byte switchControl);
        void enableRFTXBlocks(uint32_t channel);
        void endAccMemRead();
        void getPrintableDevID(char msgBuffer[]);
        void getReceiveData(int length, byte* buffer);
        uint64_t getReceiveTimestamp();
        uint32_t getTimestamp();
        uint64_t getTransmitTimestamp();
        void initialize(int rstPin);
        bool isIdleRC();
        bool isReceiveDone();
        bool isReceiveError();
        bool isReceiveTimestampAvailable();
        bool isTransmitDone();

        void modify8bitReg(byte address, int offset, const uint8_t _and, const uint8_t _or);
        void modify16bitReg(byte address, int offset, const uint16_t _and, const uint16_t _or);
        void modify32bitReg(byte address, int offset, const uint32_t _and, const uint32_t _or);

        void or8bitReg(byte address, int offset, const uint8_t value);
        void or16bitReg(byte address, int offset, const uint16_t value);
        void or32bitReg(byte address, int offset, const uint32_t value);

        void pgfCal();
        void readAccMem(int offset, int length, byte* buffer);
        uint32_t readOTP(uint16_t address);
    
        void readReg(byte address, int offset, int length, byte* buffer);
        uint8_t read8bitReg(byte address, int offset);
        uint16_t read16bitReg(byte address, int offset);
        uint32_t read32bitReg(byte address, int offset);
        uint64_t read64bitReg(byte address, int offset);
    
        void reset(int rstPin);
        void resetReceiver();
        void setDelayedTime(uint32_t delayedTime);
        void setTransmitData(int length, byte* buffer, int ranging, int fcs);
        void setPAN(uint16_t panID, uint16_t shortAddr);
        void startAccMemRead();
        void startReceive(bool delayed);
        void startTransmit(bool delayed, bool wait4resp);
        void writeFastCMD(int cmd);

        void writeReg(byte address, int offset, int length, byte* buffer);
        void write8bitReg(byte address, int offset, uint8_t value);
        void write16bitReg(byte address, int offset, uint16_t value);
        void write32bitReg(byte address, int offset, uint32_t value);
};

// SPI Modes
constexpr uint16_t SPI_RD_BIT =     0x0000U;
constexpr uint16_t SPI_WR_BIT =     0x8000U;
constexpr uint16_t SPI_AND_OR_8 =   0x8001U;
constexpr uint16_t SPI_AND_OR_16 =  0x8002U;
constexpr uint16_t SPI_AND_OR_32 =  0x8003U;

// Clock values
constexpr byte SYS_AUTO_CLOCK =         0x00;
constexpr byte SYS_FAST_RC_4_CLOCK =    0x01;
constexpr byte SYS_PLL_CLOCK =          0x02;
constexpr byte SYS_FAST_RC_CLOCK =      0x03;

// Registers
/**
 * @brief General configuration registers and AES 0
 */
constexpr byte GEN_CFG_AES_0 =  0x00;
constexpr byte DEV_ID =         0x00;
constexpr byte PANADR =         0x0C;
constexpr byte SYS_CFG =        0x10;
constexpr byte SYS_TIME =       0x1C;
constexpr byte TX_FCTRL =       0x24;
constexpr byte TX_FCTRL_HI =    0x28;
constexpr byte DX_TIME =        0x2C;
constexpr byte RX_FWTO =        0x34;
constexpr byte SYS_CTRL =       0x38;
constexpr byte SYS_STATUS =     0x44;
constexpr byte RX_FINFO =       0x4C;
constexpr byte RX_TIME =        0x64;
constexpr byte TX_TIME =        0x74;

/**
 * @brief General configuration registers and AES 1
 */
constexpr byte GEN_CFG_AES_1 =  0x01;
constexpr byte TX_ANTD =        0x04;
constexpr byte ACK_RESP_T =     0x08;
constexpr byte TX_POWER =       0x0C;
constexpr byte CHAN_CTRL =      0x14;

/**
 * @brief Scrambled Timestamp Sequence configuration and status registers
 */
constexpr byte STS_CONFIG =     0x02;
constexpr byte STS_CFG =        0x00;

/**
 * @brief RX tuning register
 */
constexpr byte RX_TUNE =        0x03;
constexpr byte DGC_CFG =        0x18;
constexpr byte DGC_CFG0 =       0x1C;
constexpr byte DGC_CFG1 =       0x20;
constexpr byte DGC_LUT_0 =      0x38;
constexpr byte DGC_LUT_1 =      0x3C;
constexpr byte DGC_LUT_2 =      0x40;
constexpr byte DGC_LUT_3 =      0x44;
constexpr byte DGC_LUT_4 =      0x48;
constexpr byte DGC_LUT_5 =      0x4C;
constexpr byte DGC_LUT_6 =      0x50;

/**
 * @brief External sync control and RX calibration
 */
constexpr byte EXT_SYNC =       0x04;
constexpr byte EC_CTRL =        0x00;
constexpr byte RX_CAL =         0x0C;
constexpr byte RX_CAL_RESI =    0x14;
constexpr byte RX_CAL_RESQ =    0x1C;
constexpr byte RX_CAL_STS =     0x20;

/**
 * @brief General Purpose Input-Output control registers
 */
constexpr byte GPIO_CTRL =      0x05;
constexpr byte GPIO_MODE =      0x00;

/**
 * @brief Digital receiver tuning and configuration
 */
constexpr byte DRX =            0x06;
constexpr byte DTUNE0 =         0x00;
constexpr byte DTUNE1 =         0x04;
constexpr byte RX_SFD_TOC =     0x02;
constexpr byte DTUNE3 =         0x0C;

/**
 * @brief Analog RF configuration block
 */
constexpr byte RF_CONF =        0x07;
constexpr byte RF_ENABLE =      0x00;
constexpr byte RF_CTRL_MASK =   0x04;
constexpr byte RF_RX_CTRL =     0x10;
constexpr byte RF_SWITCH =      0x14;
constexpr byte RF_TX_CTRL_1 =   0x1A;
constexpr byte RF_TX_CTRL_2 =   0x1C;
constexpr byte LDO_CTRL =       0x48;
constexpr byte LDO_RLOAD =      0x51;

/**
 * @brief Transmitter calibration block
 */
constexpr byte TX_CAL =         0x08;
constexpr byte PGC_CTRL =       0x10;
constexpr byte PG_CAL_TARGET =  0x1C;

/**
 * @brief Frequency synthesiser control block
 */
constexpr byte FS_CTRL =        0x09;
constexpr byte PLL_CFG =        0x00;
constexpr byte PLL_CAL =        0x08;
constexpr byte XTAL =           0x14;

/**
 * @brief OTP memory interface
 */
constexpr byte OTP_IF =         0x0B;
constexpr byte OTP_ADDR =       0x04;
constexpr byte OTP_CFG =        0x08;
constexpr byte OTP_RDATA =      0x10;

/**
 * @brief CIA
 */
constexpr byte CIA_2 =          0x0E;
constexpr byte CIA_CONF =       0x00;

/**
 * @brief PMSC control and status
 */
constexpr byte PMSC =           0x11;
constexpr byte SOFT_RST =       0x00;
constexpr byte CLK_CTRL =       0x04;
constexpr byte SEQ_CTRL =       0x08;
constexpr byte BIAS_CTRL =      0x1F;

/**
 * @brief RX frame data buffer 0
 */
constexpr byte RX_BUFFER_0 =    0x12;

/**
 * @brief Transmit data buffer
 */
constexpr byte TX_BUFFER =      0x14;

/**
 * @brief Read access to accumulator data memory
 */
constexpr byte ACC_MEM =        0x15;


/**
 * @brief Fast commands
 */
constexpr byte CMD_TXRXOFF =        0x00;
constexpr byte CMD_TX =             0x01;
constexpr byte CMD_RX =             0x02;
constexpr byte CMD_DTX =            0x03;
constexpr byte CMD_DRX =            0x04;
constexpr byte CMD_DTX_TS =         0x05;
constexpr byte CMD_DRX_TS =         0x06;
constexpr byte CMD_DTX_RS =         0x07;
constexpr byte CMD_DRX_RS =         0x08;
constexpr byte CMD_DTX_REF =        0x09;
constexpr byte CMD_DRX_REF =        0x0A;
constexpr byte CMD_CCA_TX =         0x0B;
constexpr byte CMD_TX_W4R =         0x0C;
constexpr byte CMD_DTX_W4R =        0x0D;
constexpr byte CMD_DTX_TS_W4R =     0x0E;
constexpr byte CMD_DTX_RS_W4R =     0x0F;
constexpr byte CMD_DTX_REF_W4R =    0x10;
constexpr byte CMD_CCA_TX_W4R =     0x11;
constexpr byte CMD_CLR_IRQS =       0x12;
constexpr byte CMD_DB_TOGGLE =      0x13;


// Antenna delay - this is best determined by testing range measurements
#define TX_ANT_DLY                      (16445)
#define TX_ANT_DLY_MSB                  ((TX_ANT_DLY>>8) & 0xFF)
#define TX_ANT_DLY_LSB                  (TX_ANT_DLY & 0xFF)

// Distance calculation params
#define DWT_TIME_UNITS                  (1.0/499.2e6/128.0)
#define SPEED_OF_LIGHT                  299702547
#define DISTANCE_PER_DWT_TIME_UNIT      ((double)DWT_TIME_UNITS * (double)SPEED_OF_LIGHT)

// Configuration options
#define PAC8        0       // PAC  8 (recommended for RX of preamble length  128 and below
#define PAC16       1       // PAC 16 (recommended for RX of preamble length  256
#define PAC32       2       // PAC 32 (recommended for RX of preamble length  512
#define PAC4        3       // PAC  4 (recommended for RX of preamble length  < 127

#define PLEN_4096   0x03    // Standard preamble length 4096 symbols
#define PLEN_2048   0x0A    // Non-standard preamble length 2048 symbols
#define PLEN_1536   0x06    // Non-standard preamble length 1536 symbols
#define PLEN_1024   0x02    // Standard preamble length 1024 symbols
#define PLEN_512    0x0D    // Non-standard preamble length 512 symbols
#define PLEN_256    0x09    // Non-standard preamble length 256 symbols
#define PLEN_128    0x05    // Non-standard preamble length 128 symbols
#define PLEN_64     0x01    // Standard preamble length 64 symbols
#define PLEN_32     0x04    // Non-standard length 32
#define PLEN_72     0x07    // Non-standard length 72

#define BR_850K     0       // UWB bit rate 850 kbits/s
#define BR_6M8      1       // UWB bit rate 6.8 Mbits/s
#define BR_NODATA   2       // No data (SP3 packet format)

#define PHRMODE_STD 0x0     // Standard PHR mode
#define PHRMODE_EXT 0x1     // DW proprietary extended frames PHR mode

#define PHRRATE_STD 0x0     // Standard PHR rate
#define PHRRATE_DTA 0x1     // PHR at data rate (6M81)

#endif // DWM3000_H