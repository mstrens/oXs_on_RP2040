#pragma once

#include "crsf_frames.h"


void on_crsf_uart_rx();
void on_crsf2_uart_rx();
void setupCrsfIn();
void setupCrsf2In();
void handleCrsfIn(void);
void handleCrsf2In(void);
