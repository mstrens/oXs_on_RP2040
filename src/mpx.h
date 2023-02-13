#pragma once

//#include "Arduino.h"

void setupMpx(void);
void setupListMpxFieldsToReply(); // fill a table with the mpx Index oXs should reply (true = to reply; false do not reply)

void mpxPioRxHandlerIrq() ;   // when a byte is received on the Sport, read the pio Sport fifo and push the data to a queue (to be processed in the main loop)

void handleMpxRxTx(void);
bool sendMpxFrame(uint8_t data_id) ; // search for the next data to be sent
