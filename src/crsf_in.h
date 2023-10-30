#pragma once

#include "crsf_frames.h"

#define CRSF_UART_ID uart1 // used by primary receiver
#define CRSF2_UART_ID uart0 // used by secondary receiver


void on_crsf_uart_rx();
void on_crsf2_uart_rx();
void setupCrsfIn();
void setupCrsf2In();
void handleCrsfIn(void);
void handleCrsf2In(void);
