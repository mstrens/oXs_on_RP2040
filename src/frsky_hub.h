#pragma once


void setupFrskyHub();
void handleFrskyHubFrames();
void sendFrskyHubFrame1();
void sendFrskyHubFrame2();
void sendFrskyHubFrame3();
void sendHubValue(uint8_t ID, uint16_t Value) ;
void sendHubByte( uint8_t byte );

