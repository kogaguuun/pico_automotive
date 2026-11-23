#ifndef SERIAL_OUTPUT_H
#define SERIAL_OUTPUT_H

#include <Arduino.h>

// Initialize serial output task. interval_ms defaults to 10ms.
void serialOutputInit(unsigned long interval_ms = 10);
// Call from main loop to run periodic serial output.
void serialOutputLoop();

#endif // SERIAL_OUTPUT_H
