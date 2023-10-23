#pragma once


#include "config.h"


class LOGGER
{
public:
    explicit LOGGER(void);
    void begin();
    
    void logByteNoStuff(uint8_t c);                 // write one byte in the buffer and start dma when buffer is full and switch the writing buffer
    void logBytewithStuff(uint8_t c);               // convert byte with stuffing and log 1 or 2 bytes 
    void logint32withStuff(uint8_t type ,int32_t value);  // log the type and value of a data
    void logAllRcChannels();                           // log all RC channels
    void logTimestampMs(uint32_t value);               // log the timestamp in msec
private:
};

