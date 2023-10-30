#pragma once

#define SBUS_UART_ID uart1
#define SBUS2_UART_ID uart0


void setupSbusIn();
void setupSbus2In();
void handleSbusIn();
void handleSbus2In();
void storeSbusFrame();
void storeSbus2Frame();