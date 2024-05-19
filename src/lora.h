#ifdef NOTUSED
#pragma once


#include "pico/stdlib.h"

void initSpi() ; // initialise SPI (with only 1 slave)
uint8_t spiSend(uint8_t value); 
uint8_t loraSingleTransfer(uint8_t reg, uint8_t value) ;  // only for internal use; Write and read one LORA register
void loraWriteRegister(uint8_t reg, uint8_t value) ;   // write a LORA register, discard the read value
uint8_t loraReadRegister(uint8_t reg) ;                // Read a LORA register; send a dummy value because it is just a read
void loraReadRegisterBurst( uint8_t reg, uint8_t* dataIn, uint8_t numBytes) ;
void loraWriteRegisterBurst( uint8_t reg, uint8_t* dataOut, uint8_t numBytes) ;

void loraHandle() ;
void loraSetup() ;         // parameters that are set only once
void loraTxOn(uint8_t txPower);
void loraRxOn() ;
void loraInSleep() ;
void loraReadPacket() ;           // read a packet with 2 bytes ; packetType and TxPower
void loraFillTxPacket() ;


// SX127x series common LoRa registers
#define LORA_REG_FIFO                               0x00  //oXsTx     Receiver
#define LORA_REG_OP_MODE                            0x01  // should be 0b10000xxx
                                                          // bit7=1=Lora mode
                                                          // bit6=0 = do not access FSK registers
                                                          // bit3= LowfrequencyModeOn; 0=access to HF test reg; 1=access to LF test register
                                                          // bits2-0=Mode; 000=sleep, 001=standby, 011=Transmit, 101=Receive continous, 110=Receive single
#define LORA_REG_FRF_MSB                            0x06  //frequency (in steps of 61.035 Hz)
#define LORA_REG_FRF_MID                            0x07  //frequency
#define LORA_REG_FRF_LSB                            0x08  //frequency
#define LORA_REG_PA_CONFIG                          0x09  //power amplifier source and value eg. 0x8F for 17dbm on PA_boost
                                                          // bit 7 = 1 means PA_BOOST is used; min +2,max 17dBm (or 20 with 1%); 0= FRO pin = min -4,max 14 dBm
                                                          // bits 6-4 = MaxPowerPmax = 10.8 + 0.6MaxPower
                                                          // bits 0-3 = OutputPower
                                                          // Pout= 10.8 + 0.6MaxPowerPmax - 15 + OutputPower (if bit7=0)
                                                          // Pout= 2 + OutputPower (if bit7=1)
#define LORA_REG_PA_RAMP                            0x0A  //power amplifier ramp: only bits 3-0; ex: 1000= 62usec; default = 0x09 = 125usec
#define LORA_REG_OCP                                0x0B  // current protection: could be 0b00111011 (enabled at 240 ma)
                                                          // bit 5 = 1 = enabled
                                                          // bits 4-0 = OcpTrim
                                                          // Imax = 45+5*OcpTrim (if OcpTrim <=15 so 120mA)
                                                          //      = -30+10*OcpTrim (sinon et <=27 so 240mA)
                                                          // default 0x0B = 100mA; max value = OcpTrim=27 (décimal) = 11011
#define LORA_REG_LNA                                0x0C  // gain in reception ex: 0b00100011max gain, no boost on LNA)
                                                          // bits 7-5 = LnaGain
                                                          //       000 = not used
                                                          //       001 = max gain   // 110 = min gain
                                                          // bits 4-3 Low frequency LNA current adjustment; must be 00
                                                          // bits 1-0 High frequency LNA current adjustment
                                                          //       00 = Default LNA current
                                                          //       11 = Boost on, 150% LNA current
#define LORA_REG_FIFO_ADDR_PTR                      0x0D  // address of current byte to be read/writen in Fifo 
#define LORA_REG_FIFO_TX_BASE_ADDR                  0x0E  // base of Tx fifo; default 0x80
#define LORA_REG_FIFO_RX_BASE_ADDR                  0x0F  // base of Rx fifo; default 0x00
#define LORA_REG_FIFO_RX_CURRENT_ADDR               0x10  // adress of start of last Rx packet received (can't be written)
#define LORA_REG_IRQ_FLAGS_MASK                     0x11  // Irq flag mask
#define LORA_REG_IRQ_FLAGS                          0x12  // Irq flags (write 1 to bit to clear); write 0xFF clear all flags
                                                          // bit 7=RxTimeout, 6=RxDone, 5=CrcError, 4= validHeader
                                                          //     3=TxDone, 2=CadDone, 1=FhssChange, 0= Cad detected
#define LORA_REG_RX_NB_BYTES                        0x13  // Number of received bytes in payload of last packet received
#define LORA_REG_RX_HEADER_CNT_VALUE_MSB            0x14  // count nr of header received
#define LORA_REG_RX_HEADER_CNT_VALUE_LSB            0x15
#define LORA_REG_RX_PACKET_CNT_VALUE_MSB            0x16  // count nr of packet received
#define LORA_REG_RX_PACKET_CNT_VALUE_LSB            0x17
#define LORA_REG_MODEM_STAT                         0x18  // Live LoRaTM modem status (see page 111 of datasheet)
#define LORA_REG_PKT_SNR_VALUE                      0x19  // SNR of last packet received
                                                          // SNR = (bit 7-0 in 2 complement)/4
#define LORA_REG_PKT_RSSI_VALUE                     0x1A  // RSSI of last packet received
                                                          // see 5.5.5 of datasheet
#define LORA_REG_RSSI_VALUE                         0x1B  // current RSSI (not used)
#define LORA_REG_HOP_CHANNEL                        0x1C  // start of hop channel (not used)
#define LORA_REG_MODEM_CONFIG_1                     0x1D  // config of modem part 1  // e.g. BW=125,CR=4/5 , no Header => 0b01110011
                                                          // bits 7-4 = BW; 0110 = 62.5Khz; 0111=125Khz
                                                          // bits 3-1 = CR ; 001 = 4/5; 100=4/8
                                                          //bit 0: 0=Explicit Header; 1=no Header
#define LORA_REG_MODEM_CONFIG_2                     0x1E  // config of modem part 2 //e.g.SF=10,1 packet,CRCon,=> Ob10100100
                                                          // bits 7-4=SF ; from 6 up to 12
                                                          // bit 3=TxContinous mode;0=one packet only
                                                          // bit2=RxPayloadCrcON ; 1=Enable
                                                          // bits1-0= SymbTimeOut(9:8) = MSB
#define LORA_REG_SYMB_TIMEOUT_LSB                   0x1F  // Receiver timeout value LSB (in single mode) in number of symbols
#define LORA_REG_PREAMBLE_MSB                       0x20  // size of preamble; e.g. 0x0006 (default 000C)
#define LORA_REG_PREAMBLE_LSB                       0x21
#define LORA_REG_PAYLOAD_LENGTH                     0x22  // Payload length; has to be defined when no header 
                                                          // e.g. 0x02 (for Receiver => 150msec) and 0x06 (for oXs => 190msec) 
#define LORA_REG_MAX_PAYLOAD_LENGTH                 0x23  // max payload length (not used??? when no header); for safety, set to 0x06
#define LORA_REG_HOP_PERIOD                         0x24  // frequency hop period (not used)
#define LORA_REG_FIFO_RX_BYTE_ADDR                  0x25  // adress of last byte written in Fifo in receive mode
#define LORA_REG_MODEM_CONFIG_3                     0x26  // config of modem part 3 ; e.g. 0b00001100
                                                          // bit3=LowDataRateOptimize; 1=Enabled mandated when symbol length exceeds 16ms
                                                          // bit2=AgcAutoOn; 1=LNA gain set by internal AGCloop instead of by register LnaGain
#define LORA_REG_FEI_MSB                            0x28  // estimated frequency error
#define LORA_REG_FEI_MID                            0x29
#define LORA_REG_FEI_LSB                            0x2A
#define LORA_REG_RSSI_WIDEBAND                      0x2C  //wideband RSSI measurement (= average) (not used)
#define LORA_REG_DETECT_OPTIMIZE                    0x31  // lora detection optimize ;e.g. 0x03 (for sf10)
                                                          // bit 2-0 = 011 for SF7 to 12; 0101 for sf6 only; default 011
#define LORA_REG_INVERT_IQ                          0x33  // Invert lora I and Q signals
#define LORA_REG_DETECTION_THRESHOLD                0x37  // lora detection threshold default 0X0A is ok for SF7-12; 0x0C for sf6
#define LORA_REG_SYNC_WORD                          0x39  // lora Sync Word (default 0x12)
#define LORA_REG_DIO_MAPPING_1                      0x40  // DIO mapping
#define LORA_REG_DIO_MAPPING_2                      0x41
#define LORA_REG_VERSION                            0x42  // lora version
#define LORA_REG_PA_DAC                             0x4D  // 0x84 = normal power (up to 17 dBm); 0x87= boost (20dBm)

// SX127x common LoRa modem settings
// LORA_REG_OP_MODE                                                 MSB   LSB   DESCRIPTION
#define LORA_SLEEP                                  0b00000000  //  2     0     sleep
#define LORA_STANDBY                                0b00000001  //  2     0     standby
#define LORA_TX                                     0b00000011  //  2     0     transmit
#define LORA_RXCONTINUOUS                           0b00000101  //  2     0     receive continuous
#define LORA_RXSINGLE                               0b00000110  //  2     0     receive single


// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40

#endif