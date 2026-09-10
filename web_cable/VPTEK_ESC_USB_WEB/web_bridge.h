#pragma once

#include <Arduino.h>

void setupWebBridge();
void webBridgeLoop();

bool webBridgeLocksEsc();

void startWebRadio();
void stopWebRadio();
bool webRadioIsRunning();
