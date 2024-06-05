#ifndef USE_RFM95
#pragma once

#include "pico/stdlib.h"

// Process depends on loraState; it can be
#define LORA_TO_INIT 0              // device must be initialized
#define LORA_IN_SLEEP 1             // wait a delay and set lora in recieve continous mode
#define LORA_IN_RECEIVE 2           // wait that a package has been received or a max delay; if package has been received,Tx power changes, update Tx power, change mode to LORA_TO_TRANSMIT
#define LORA_TO_TRANSMIT 3          // fill lora with data to be send and ask for sending (but do not wait), change mode to LORA_WAIT_END_OF_TRANSMIT
#define LORA_WAIT_END_OF_TRANSMIT 4 // wait that pakage has been sent (or wait max x sec)
#define LORA_IN_ERROR 5             // state when init fail



void loraHandle() ;
void initSpi();
bool loraSetup() ;         // parameters that are set only once
void loraTransmit(uint8_t *message, uint8_t length, uint32_t timeout);
void loraReceiveOn(uint8_t length, uint32_t timeout);
void loraReadPacket() ;           // read a packet with 2 bytes ; PacketType and PacketTxPower
void loraFillTxPacket();

//++++++++++ here copy of code from https://github.com/chandrawi/LoRaRF-Arduino/tree/main/src +++++++++++++
// Still SPI read and write have been rewritten for RP2040 without use of Arduino lib

// SX126X commands
#define SX126X_SetSleep 0x84 
#define SX126X_SetStandby 0x80
#define SX126X_SetFs 0xC1
#define SX126X_SetTx 0x83
#define SX126X_SetRx 0x82
#define SX126X_StopTimerOnPreamble 0x9F
#define SX126X_SetRxDutyCycle 0x94
#define SX126X_SetCad 0xC5
#define SX126X_SetTxContinuousWave 0xD1
#define SX126X_SetTxInfinitePreamble 0xD2
#define SX126X_SetRegulatorMode 0x96
#define SX126X_Calibrate 0x89
#define SX126X_CalibrateImage 0x98
#define SX126X_SetPaConfig 0x95
#define SX126X_SetRxTxFallbackMode 0x93 fallbackMode Defines into which mode the chip goes 
#define SX126X_WriteRegister 0x0D
#define SX126X_ReadRegister 0x1D
#define SX126X_WriteBuffer 0x0E
#define SX126X_ReadBuffer 0x1E


//******************   here original driver 
// SX126X register map
#define SX126X_REG_FSK_WHITENING_INITIAL_MSB    0x06B8
#define SX126X_REG_FSK_CRC_INITIAL_MSB          0x06BC
#define SX126X_REG_FSK_SYNC_WORD_0              0x06C0
#define SX126X_REG_FSK_NODE_ADDRESS             0x06CD
#define SX126X_REG_IQ_POLARITY_SETUP            0x0736
#define SX126X_REG_LORA_SYNC_WORD_MSB           0x0740
#define SX126X_REG_RANDOM_NUMBER_GEN            0x0819
#define SX126X_REG_TX_MODULATION                0x0889
#define SX126X_REG_RX_GAIN                      0x08AC
#define SX126X_REG_TX_CLAMP_CONFIG              0x08D8
#define SX126X_REG_OCP_CONFIGURATION            0x08E7
#define SX126X_REG_RTC_CONTROL                  0x0902
#define SX126X_REG_XTA_TRIM                     0x0911
#define SX126X_REG_XTB_TRIM                     0x0912
#define SX126X_REG_EVENT_MASK                   0x0944

// SetSleep
#define SX126X_SLEEP_COLD_START                 0x00        // sleep mode: cold start, configuration is lost (default)
#define SX126X_SLEEP_WARM_START                 0x04        //             warm start, configuration is retained
#define SX126X_SLEEP_COLD_START_RTC             0x01        //             cold start and wake on RTC timeout
#define SX126X_SLEEP_WARM_START_RTC             0x05        //             warm start and wake on RTC timeout

// SetStandby
#define SX126X_STANDBY_RC                       0x00        // standby mode: using 13 MHz RC oscillator
#define SX126X_STANDBY_XOSC                     0x01        //               using 32 MHz crystal oscillator

// SetTx
#define SX126X_TX_SINGLE                        0x000000    // Tx timeout duration: no timeout (Rx single mode)

// SetRx
#define SX126X_RX_SINGLE                        0x000000    // Rx timeout duration: no timeout (Rx single mode)
#define SX126X_RX_CONTINUOUS                    0xFFFFFF    //                      infinite (Rx continuous mode)

// SetRegulatorMode
#define SX126X_REGULATOR_LDO                    0x00        // set regulator mode: LDO (default)
#define SX126X_REGULATOR_DC_DC                  0x01        //                     DC-DC

// CalibrateImage
#define SX126X_CAL_IMG_430                      0x6B        // ISM band: 430-440 Mhz Freq1
#define SX126X_CAL_IMG_440                      0x6F        //           430-440 Mhz Freq2
#define SX126X_CAL_IMG_470                      0x75        //           470-510 Mhz Freq1
#define SX126X_CAL_IMG_510                      0x81        //           470-510 Mhz Freq2
#define SX126X_CAL_IMG_779                      0xC1        //           779-787 Mhz Freq1
#define SX126X_CAL_IMG_787                      0xC5        //           779-787 Mhz Freq2
#define SX126X_CAL_IMG_863                      0xD7        //           863-870 Mhz Freq1
#define SX126X_CAL_IMG_870                      0xDB        //           863-870 Mhz Freq2
#define SX126X_CAL_IMG_902                      0xE1        //           902-928 Mhz Freq1
#define SX126X_CAL_IMG_928                      0xE9        //           902-928 Mhz Freq2

// SetPaConfig
#define SX126X_TX_POWER_SX1261                  0x01        // device version for TX power: SX1261
#define SX126X_TX_POWER_SX1262                  0x02        //                            : SX1262
#define SX126X_TX_POWER_SX1268                  0x08        //                            : SX1268

// SetRxTxFallbackMode
#define SX126X_FALLBACK_FS                      0x40        // after Rx/Tx go to: FS mode
#define SX126X_FALLBACK_STDBY_XOSC              0x30        //                    standby mode with crystal oscillator
#define SX126X_FALLBACK_STDBY_RC                0x20        //                    standby mode with RC oscillator (default)

// SetDioIrqParams
#define SX126X_IRQ_TX_DONE                      0x0001      // packet transmission completed
#define SX126X_IRQ_RX_DONE                      0x0002      // packet received
#define SX126X_IRQ_PREAMBLE_DETECTED            0x0004      // preamble detected
#define SX126X_IRQ_SYNC_WORD_VALID              0x0008      // valid sync word detected
#define SX126X_IRQ_HEADER_VALID                 0x0010      // valid LoRa header received
#define SX126X_IRQ_HEADER_ERR                   0x0020      // LoRa header CRC error
#define SX126X_IRQ_CRC_ERR                      0x0040      // wrong CRC received
#define SX126X_IRQ_CAD_DONE                     0x0080      // channel activity detection finished
#define SX126X_IRQ_CAD_DETECTED                 0x0100      // channel activity detected
#define SX126X_IRQ_TIMEOUT                      0x0200      // Rx or Tx timeout
#define SX126X_IRQ_ALL                          0x03FF      // all interrupts
#define SX126X_IRQ_NONE                         0x0000      // no interrupts

// SetDio2AsRfSwitch
#define SX126X_DIO2_AS_IRQ                      0x00        // DIO2 configuration: IRQ
#define SX126X_DIO2_AS_RF_SWITCH                0x01        //                     RF switch control

// SetDio3AsTcxoCtrl
#define SX126X_DIO3_OUTPUT_1_6                  0x00        // DIO3 voltage output for TCXO: 1.6 V
#define SX126X_DIO3_OUTPUT_1_7                  0x01        //                               1.7 V
#define SX126X_DIO3_OUTPUT_1_8                  0x02        //                               1.8 V
#define SX126X_DIO3_OUTPUT_2_2                  0x03        //                               2.2 V
#define SX126X_DIO3_OUTPUT_2_4                  0x04        //                               2.4 V
#define SX126X_DIO3_OUTPUT_2_7                  0x05        //                               2.7 V
#define SX126X_DIO3_OUTPUT_3_0                  0x06        //                               3.0 V
#define SX126X_DIO3_OUTPUT_3_3                  0x07        //                               3.3 V
#define SX126X_TCXO_DELAY_1                     0x0040      // TCXO delay time: 1 ms
#define SX126X_TCXO_DELAY_2                     0x0080      //                  2 ms
#define SX126X_TCXO_DELAY_5                     0x0140      //                  5 ms
#define SX126X_TCXO_DELAY_10                    0x0280      //                  10 ms

// SetRfFrequency
#define SX126X_RF_FREQUENCY_XTAL                32000000    // XTAL frequency used for RF frequency calculation
#define SX126X_RF_FREQUENCY_SHIFT               25          // RfFreq = Frequency * 2^25 / 32000000

// SetPacketType
#define SX126X_FSK_MODEM                        0x00        // GFSK packet type
#define SX126X_LORA_MODEM                       0x01        // LoRa packet type

// SetTxParams
#define SX126X_PA_RAMP_10U                      0x00        // ramp time: 10 us
#define SX126X_PA_RAMP_20U                      0x01        //            20 us
#define SX126X_PA_RAMP_40U                      0x02        //            40 us
#define SX126X_PA_RAMP_80U                      0x03        //            80 us
#define SX126X_PA_RAMP_200U                     0x04        //            200 us
#define SX126X_PA_RAMP_800U                     0x05        //            800 us
#define SX126X_PA_RAMP_1700U                    0x06        //            1700 us
#define SX126X_PA_RAMP_3400U                    0x07        //            3400 us

// SetModulationParams for LoRa packet type
#define SX126X_BW_7800                          0x00        // LoRa bandwidth: 7.8 kHz
#define SX126X_BW_10400                         0x08        //                 10.4 kHz
#define SX126X_BW_15600                         0x01        //                 15.6 kHz
#define SX126X_BW_20800                         0x09        //                 20.8 kHz
#define SX126X_BW_31250                         0x02        //                 31.25 kHz
#define SX126X_BW_41700                         0x0A        //                 41.7 kHz
#define SX126X_BW_62500                         0x03        //                 62.5 kHz
#define SX126X_BW_125000                        0x04        //                 125 kHz
#define SX126X_BW_250000                        0x05        //                 250 kHz
#define SX126X_BW_500000                        0x06        //                 500 kHz
#define SX126X_CR_4_4                           0x00        // LoRa coding rate: 4/4 (no coding rate)
#define SX126X_CR_4_5                           0x01        //                   4/5
#define SX126X_CR_4_6                           0x02        //                   4/6
#define SX126X_CR_4_7                           0x03        //                   4/7
#define SX126X_CR_4_8                           0x04        //                   4/8
#define SX126X_LDRO_OFF                         0x00        // LoRa low data rate optimization: disabled
#define SX126X_LDRO_ON                          0x00        //                                  enabled

// SetModulationParams for FSK packet type
#define SX126X_PULSE_NO_FILTER                  0x00        // FSK pulse shape: no filter applied
#define SX126X_PULSE_GAUSSIAN_BT_0_3            0x08        //                  Gaussian BT 0.3
#define SX126X_PULSE_GAUSSIAN_BT_0_5            0x09        //                  Gaussian BT 0.5
#define SX126X_PULSE_GAUSSIAN_BT_0_7            0x0A        //                  Gaussian BT 0.7
#define SX126X_PULSE_GAUSSIAN_BT_1              0x0B        //                  Gaussian BT 1
#define SX126X_BW_4800                          0x1F        // FSK bandwidth: 4.8 kHz DSB
#define SX126X_BW_5800                          0x17        //                5.8 kHz DSB
#define SX126X_BW_7300                          0x0F        //                7.3 kHz DSB
#define SX126X_BW_9700                          0x1E        //                9.7 kHz DSB
#define SX126X_BW_11700                         0x16        //                11.7 kHz DSB
#define SX126X_BW_14600                         0x0E        //                14.6 kHz DSB
#define SX126X_BW_19500                         0x1D        //                19.5 kHz DSB
#define SX126X_BW_23400                         0x15        //                23.4 kHz DSB
#define SX126X_BW_29300                         0x0D        //                29.3 kHz DSB
#define SX126X_BW_39000                         0x1C        //                39 kHz DSB
#define SX126X_BW_46900                         0x14        //                46.9 kHz DSB
#define SX126X_BW_58600                         0x0C        //                58.6 kHz DSB
#define SX126X_BW_78200                         0x1B        //                78.2 kHz DSB
#define SX126X_BW_93800                         0x13        //                93.8 kHz DSB
#define SX126X_BW_117300                        0x0B        //                117.3 kHz DSB
#define SX126X_BW_156200                        0x1A        //                156.2 kHz DSB
#define SX126X_BW_187200                        0x12        //                187.2 kHz DSB
#define SX126X_BW_234300                        0x0A        //                232.3 kHz DSB
#define SX126X_BW_312000                        0x19        //                312 kHz DSB
#define SX126X_BW_373600                        0x11        //                373.6 kHz DSB
#define SX126X_BW_467000                        0x09        //                476 kHz DSB

// SetPacketParams for LoRa packet type
#define SX126X_HEADER_EXPLICIT                  0x00        // LoRa header mode: explicit
#define SX126X_HEADER_IMPLICIT                  0x01        //                   implicit
#define SX126X_CRC_OFF                          0x00        // LoRa CRC mode: disabled
#define SX126X_CRC_ON                           0x01        //                enabled
#define SX126X_IQ_STANDARD                      0x00        // LoRa IQ setup: standard
#define SX126X_IQ_INVERTED                      0x01        //                inverted

// SetPacketParams for FSK packet type
#define SX126X_PREAMBLE_DET_LEN_OFF             0x00        // FSK preamble detector length: off
#define SX126X_PREAMBLE_DET_LEN_8               0x04        //                               8-bit
#define SX126X_PREAMBLE_DET_LEN_16              0x05        //                               16-bit
#define SX126X_PREAMBLE_DET_LEN_24              0x06        //                               24-bit
#define SX126X_PREAMBLE_DET_LEN_32              0x07        //                               32-bit
#define SX126X_ADDR_COMP_OFF                    0x00        // FSK address filtering: off
#define SX126X_ADDR_COMP_NODE                   0x01        //                        filtering on node address
#define SX126X_ADDR_COMP_ALL                    0x02        //                        filtering on node and broadcast address
#define SX126X_PACKET_KNOWN                     0x00        // FSK packet type: the packet length known on both side
#define SX126X_PACKET_VARIABLE                  0x01        //                  the packet length on variable size
#define SX126X_CRC_0                            0x01        // FSK CRC type: no CRC
#define SX126X_CRC_1                            0x00        //               CRC computed on 1 byte
#define SX126X_CRC_2                            0x02        //               CRC computed on 2 byte
#define SX126X_CRC_1_INV                        0x04        //               CRC computed on 1 byte and inverted
#define SX126X_CRC_2_INV                        0x06        //               CRC computed on 2 byte and inverted
#define SX126X_WHITENING_OFF                    0x00        // FSK whitening: no encoding
#define SX126X_WHITENING_ON                     0x01        //                whitening enable

// SetCadParams
#define SX126X_CAD_ON_1_SYMB                    0x00        // number of symbols used for CAD: 1
#define SX126X_CAD_ON_2_SYMB                    0x01        //                                 2
#define SX126X_CAD_ON_4_SYMB                    0x02        //                                 4
#define SX126X_CAD_ON_8_SYMB                    0x03        //                                 8
#define SX126X_CAD_ON_16_SYMB                   0x04        //                                 16
#define SX126X_CAD_EXIT_STDBY                   0x00        // after CAD is done, always exit to STDBY_RC mode
#define SX126X_CAD_EXIT_RX                      0x01        // after CAD is done, exit to Rx mode if activity is detected

// GetStatus
#define SX126X_STATUS_DATA_AVAILABLE            0x04        // command status: packet received and data can be retrieved
#define SX126X_STATUS_CMD_TIMEOUT               0x06        //                 SPI command timed out
#define SX126X_STATUS_CMD_ERROR                 0x08        //                 invalid SPI command
#define SX126X_STATUS_CMD_FAILED                0x0A        //                 SPI command failed to execute
#define SX126X_STATUS_CMD_TX_DONE               0x0C        //                 packet transmission done
#define SX126X_STATUS_MODE_STDBY_RC             0x20        // current chip mode: STDBY_RC
#define SX126X_STATUS_MODE_STDBY_XOSC           0x30        //                    STDBY_XOSC
#define SX126X_STATUS_MODE_FS                   0x40        //                    FS
#define SX126X_STATUS_MODE_RX                   0x50        //                    RX
#define SX126X_STATUS_MODE_TX                   0x60        //                    TX

// GetDeviceErrors
#define SX126X_RC64K_CALIB_ERR                  0x0001      // device errors: RC64K calibration failed
#define SX126X_RC13M_CALIB_ERR                  0x0002      //                RC13M calibration failed
#define SX126X_PLL_CALIB_ERR                    0x0004      //                PLL calibration failed
#define SX126X_ADC_CALIB_ERR                    0x0008      //                ADC calibration failed
#define SX126X_IMG_CALIB_ERR                    0x0010      //                image calibration failed
#define SX126X_XOSC_START_ERR                   0x0020      //                crystal oscillator failed to start
#define SX126X_PLL_LOCK_ERR                     0x0040      //                PLL failed to lock
#define SX126X_PA_RAMP_ERR                      0x0100      //                PA ramping failed

// LoraSyncWord
#define SX126X_LORA_SYNC_WORD_PUBLIC            0x3444      // LoRa SyncWord for public network
#define SX126X_LORA_SYNC_WORD_PRIVATE           0x0741      // LoRa SyncWord for private network (default)

// RxGain
#define SX126X_RX_GAIN_POWER_SAVING             0x00        // gain used in Rx mode: power saving gain (default)
#define SX126X_RX_GAIN_BOOSTED                  0x01        //                       boosted gain
#define SX126X_POWER_SAVING_GAIN                0x94        // power saving gain register value
#define SX126X_BOOSTED_GAIN                     0x96        // boosted gain register value

// Default Hardware Configuration
#define SX126X_PIN_NSS                          10
#define SX126X_PIN_RESET                        4
#define SX126X_PIN_BUSY                         5
#define SX126X_SPI                              SPI
#ifdef __AVR__
    #define SX126X_SPI_FREQUENCY                F_CPU / 2   // SPI speed set to half of CPU frequency for AVR processor
#else
    #define SX126X_SPI_FREQUENCY                16000000    // Maximum LoRa SPI frequency
#endif
#define SX126X_BUSY_TIMEOUT                     5000        // Default timeout for checking busy pin

//void sx126x_setSPI(SPIClass &SpiObject, uint32_t frequency);
//void sx126x_setPins(int8_t nss, int8_t busy);
void sx126x_reset(int8_t reset);
void sx126x_begin();
bool sx126x_busyCheck(uint32_t timeout=SX126X_BUSY_TIMEOUT);

// SX126x driver: Operational Modes Commands
void sx126x_setSleep(uint8_t sleepConfig);
void sx126x_setStandby(uint8_t standbyMode);
void sx126x_setFs();
void sx126x_setTx(uint32_t timeout);
void sx126x_setRx(uint32_t timeout);
void sx126x_stopTimerOnPreamble(uint8_t enable);
void sx126x_setRxDutyCycle(uint32_t rxPeriod, uint32_t sleepPeriod);
void sx126x_setCad();
void sx126x_setTxContinuousWave();
void sx126x_setTxInfinitePreamble();
void sx126x_setRegulatorMode(uint8_t modeParam);
void sx126x_calibrate(uint8_t calibParam);
void sx126x_calibrateImage(uint8_t freq1, uint8_t freq2);
void sx126x_setPaConfig(uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut);
void sx126x_setRxTxFallbackMode(uint8_t fallbackMode);

// SX126x driver: Register and Buffer Access Commands
void sx126x_writeRegister(uint16_t address, uint8_t* data, uint8_t nData);
void sx126x_readRegister(uint16_t address, uint8_t* data, uint8_t nData);
void sx126x_writeBuffer(uint8_t offset, uint8_t* data, uint8_t nData);
void sx126x_readBuffer(uint8_t offset, uint8_t* data, uint8_t nData);

// SX126x driver: DIO and IRQ Control
void sx126x_setDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask);
void sx126x_getIrqStatus(uint16_t* irqStatus);
void sx126x_clearIrqStatus(uint16_t clearIrqParam);
void sx126x_setDio2AsRfSwitchCtrl(uint8_t enable);
void sx126x_setDio3AsTcxoCtrl(uint8_t tcxoVoltage, uint32_t delay);

// SX126x driver: RF, Modulation and Packet Commands
void sx126x_setRfFrequency(uint32_t rfFrequency);
void sx126x_setPacketType(uint8_t packetType);
void sx126x_getPacketType(uint8_t* packetType);
void sx126x_setTxParams(uint8_t power, uint8_t rampTime);
void sx126x_setModulationParamsLoRa(uint8_t sf, uint8_t bw, uint8_t cr, uint8_t ldro);
void sx126x_setModulationParamsFSK(uint32_t br, uint8_t pulseShape, uint8_t bandwidth, uint32_t Fdev);
void sx126x_setPacketParamsLoRa(uint16_t preambleLength, uint8_t headerType, uint8_t payloadLength, uint8_t crcType, uint8_t invertIq);
void sx126x_setPacketParamsFSK(uint16_t preambleLength, uint8_t preambleDetector, uint8_t syncWordLength, uint8_t addrComp, uint8_t packetType, uint8_t payloadLength, uint8_t crcType, uint8_t whitening);
void sx126x_setCadParams(uint8_t cadSymbolNum, uint8_t cadDetPeak, uint8_t cadDetMin, uint8_t cadExitMode, uint32_t cadTimeout);
void sx126x_setBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress);
void sx126x_setLoRaSymbNumTimeout(uint8_t symbnum);

// SX126x driver: Status Commands
void sx126x_getStatus(uint8_t* status);
void sx126x_getRxBufferStatus(uint8_t* payloadLengthRx, uint8_t* rxStartBufferPointer);
void sx126x_getPacketStatus(uint8_t* rssiPkt, uint8_t* snrPkt, uint8_t* signalRssiPkt);
void sx126x_getRssiInst(uint8_t* rssiInst);
void sx126x_getStats(uint16_t* nbPktReceived, uint16_t* nbPktCrcError, uint16_t* nbPktHeaderErr);
void sx126x_resetStats();
void sx126x_getDeviceErrors(uint16_t* opError);
void sx126x_clearDeviceErrors();

// SX126x driver: Workaround functions
void sx126x_fixLoRaBw500(uint32_t bw);
void sx126x_fixResistanceAntenna();
void sx126x_fixRxTimeout();
void sx126x_fixInvertedIq(uint8_t invertIq);

// Utilities
void sx126x_transfer(uint8_t opCode, uint8_t* data, uint8_t nData);
void sx126x_transfer(uint8_t opCode, uint8_t* data, uint8_t nData, uint8_t* address, uint8_t nAddress, bool read);

#endif