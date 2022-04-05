/*
 * Copyright (C) ExpressLRS_relay
 *
 *
 * License GPLv3: http://www.gnu.org/licenses/gpl-3.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#pragma once

#include "Arduino.h"


void setupSport(void);

void sportPioRxHandlerIrq() ;   // when a byte is received on the Sport, read the pio Sport fifo and push the data to a queue (to be processed in the main loop)

void handleSportRxTx(void);
void sendNextSportFrame(uint8_t data_id) ; // search for the next data to be sent
void sendOneSport(uint8_t idx); 

